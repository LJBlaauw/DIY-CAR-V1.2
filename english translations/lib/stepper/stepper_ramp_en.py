# ================================================================
# Dual Stepper Controller WITH ramp (PIO + DMA) and heading correction
# ================================================================
#
# PLEASE NOTE — this is a REPLACEMENT for stepper.py, not an addition.
# Both claim PIO0 SM0..SM3 and the same GPIOs. Always import only one of them.
#
# It is NOT a drop-in replacement. The names mov/s1/s2/rotate/stop/enable/
# disable/status/distance/pio_pos1/pio_pos2/reset_PIO_distance still exist with
# the same meaning, but:
#   - there are no more global sm0..sm3 (code waiting on sm0.active() breaks;
#     the generator SMs remain permanently active here and stall on an empty FIFO);
#   - distance() prints nothing and returns a single signed center distance instead of two
#     motor distances;
#   - the movement functions return True/False instead of None;
#   - stopping and "ready" have different semantics (see halt/brake/busy).
#
# OPERATING PRINCIPLE
# ------------------
# The step generator in the PIO reads 32-bit words from its TX-FIFO. Each word
# encodes an entire SEGMENT instead of a single step:
#
#     bits 15..0   = number of steps in this segment - 1   (max 65536 steps)
#     bits 31..16  = delay per step in PIO cycles        (max 65535)
#
# As a result, a complete ramp of thousands of steps costs only a few hundred
# words. A ramp of 256 segments is 1 kB; one word per step would be 36 kB and
# that does not fit reliably in the MicroPython heap.
#
# Phases of a movement:
#
#   ramp up    -> DMA table (256 words + 1 bridge segment). Segments of ~2 ms
#                 are too fast for MicroPython, so this MUST go via DMA. Immune
#                 to GC pauses.
#   cruise     -> without correction: one word, so part of the same
#                 DMA transfer -> zero CPU overhead.
#                 with correction: the CPU pushes one word per SLICE_MS per motor
#                 (~50 put()'s per second per motor, approx. 0.1% CPU).
#   ramp down  -> DMA table, triggered as soon as the number of committed steps
#                 reaches the deceleration point.
#
# DMA AND CPU MUST NEVER WRITE TO THE SAME FIFO SIMULTANEOUSLY. The sequence
# would then get mixed up and the two motors could get different segment
# orderings. Therefore:
#   - during _RAMP_UP, the CPU pushes nothing; it waits until dma.active()
#     of BOTH motors is False (the DMA has then stopped WRITING, while
#     the FIFO still contains data -- exactly the head start the CPU needs);
#   - the ramp-down DMA is only started after the CPU has stopped pushing.
#
# Waiting until the ramp steps are also EXECUTED would be wrong: the FIFO
# would run empty and the motor would pause between ramp and cruise phase.
#
# BRIDGE SEGMENT
# --------------
# When the ramp-up DMA finishes writing, there are still at most 4 words left
# in the FIFO. At the end of the ramp we are at top speed, so those 4 words
# together are only ~3 ms. Servicing every 10 ms would still drain the FIFO.
# Therefore, the ramp-up table ends with one BRIDGE SEGMENT at cruise speed that
# lasts BRIDGE_SLICES * SLICE_MS long. These steps belong to the cruise phase but
# are not corrected - a correction 40 ms earlier or later makes no difference.
#
# EXACT DISTANCE
# --------------
# Correcting changes WHEN steps arrive, not HOW MANY. We track `committed` per motor:
# the sum of all repeat values we have written out.
# The deceleration point is determined based on `committed`, not on a measured position, so
# the final distance is exact - independent of when the control loop invokes it.
#
# ODOMETRY
# --------
# The counter SMs count STEP FLANKS and know nothing about the DIR pin. A raw
# pulse counter is therefore not a position: during a rotation both counters increase
# positively while the wheels rotate in opposite directions. Therefore, each motor tracks a
# SIGNED position (`travel()`) alongside the monotonic pulse counter, which carries
# the sign over at each direction change. `pulses()` remains the monotonic counter and
# is used for `busy()`, where unsigned counting is required.
#
# WHAT THE ODOMETER IS AND IS NOT
# ------------------------------
# The counter counts COMMANDED wheel steps. The commanded average
# step total is exact; the physical distance is not, because microstepping, slip,
# tyre deformation, and the calibration of WHEEL_CIRC will still affect it.
# CM_PER_STEP is a nominal RESOLUTION of 14.9 um, not an accuracy.
#
#   pulse counter -> commanded wheel steps
#   gyro-Z        -> actual angular velocity and relative rotation in the short term
#   magnetometer  -> absolute orientation, provided it is not magnetically disturbed
#   actual linear position -> requires an EXTERNAL reference (wheel encoder,
#                    optical flow, beacon, map observation). Double integration of
#                    the accelerometer drifts too fast for that.
#
# TRANSACTIONS
# ------------
# Replacing an active movement and chaining a profile BEHIND an existing FIFO
# are two different operations, and mixing them up results in stationary carts:
#   start_table()   chains something behind and REFUSES as long as the DMA is still writing;
#   replace_table() clears DMA and FIFO first and resynchronizes `committed`.
# Each new command (mov, Move, ...) therefore begins with halt() on both motors.
#
# HEADING CORRECTION
# ------------------
# Heading change comes from a DIFFERENCE IN STEP COUNT between the wheels, not
# from a difference in speed if both wheels are bound to the same total.
# In the cruise phase, that total is not fixed, so there a speed difference does
# integrate into a real heading change. See HeadingController.
#
# The clock divider alternative (changing SMn_CLKDIV during runtime) was intentionally
# NOT used: that sits outside the data path, is not synchronous with segment boundaries,
# and would rescale the ramp-down tables on the fly (acceleration scales with f^2).
# ================================================================

from array import array
from math import sqrt
import rp2
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio


# ----------------------------------------------------------------
# PIO: step generator with (repeat, delay) words
# ----------------------------------------------------------------
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT)
def ramp_stepper():
    # 7 instructions. Falls through after the last jmp and automatically wraps to 0.
    pull(block)             .side(0)        # type: ignore  stalls here with STEP low
    out(y, 16)              .side(0)        # type: ignore  y = repeat-1
    out(x, 16)              .side(0)        # type: ignore  x = delay
    mov(isr, x)             .side(0)        # type: ignore  preserve delay (ISR = scratch register)
    label("pulse")                          # type: ignore
    mov(x, isr)             .side(1)  [2]   # type: ignore  STEP high 3 cycles = 200 ns @15 MHz
    label("wait")                           # type: ignore
    jmp(x_dec, "wait")      .side(0)        # type: ignore  STEP low, run out delay
    jmp(y_dec, "pulse")                     # type: ignore  next step in this segment


# ----------------------------------------------------------------
# PIO: step counter (hardware odometer, no CPU)
# ----------------------------------------------------------------
@asm_pio()
def step_counter():
    # Y decrements from 0xFFFFFFFF. Pulses = 0xFFFFFFFF - Y.
    # in_base = the STEP pin, so pin index 0. Does NOT know the DIR pin; the sign
    # is tracked in software (see _Motor.travel).
    label("loop")                           # type: ignore
    wait(0, pin, 0)                         # type: ignore  wait until STEP low
    wait(1, pin, 0)                         # type: ignore  wait for rising edge
    jmp(y_dec, "loop")                      # type: ignore


# ----------------------------------------------------------------
# Constants
# ----------------------------------------------------------------
F_PIO         = 15_000_000   # 150 MHz sysclk / 10 -> integer clock divider, no fractional jitter
CYCLES_FIXED  = 5            # fixed cycles PER STEP within a segment:
                             #   mov(x, isr)[2]      = 3
                             #   jmp(x_dec, "wait")  = delay + 1  (when x == 0 the
                             #                         instruction is executed one more time
                             #                         without jumping)
                             #   jmp(y_dec, "pulse") = 1
                             #   -> step period = delay + 5 cycles
                             # NOT counted: 4 cycles PER SEGMENT for pull/out/out/mov.
                             # The effective overhead is therefore 5 + 4/repeat. With 256
                             # ramp segments (~22 steps) that is 0.18 cycle out of ~1000
                             # (0.02%); with segments of 1 step 0.4%.
                             # CALCULATED, not yet measured. TO BE VERIFIED with a logic
                             # analyzer on both long segments and segments of
                             # 1, 2, 8 and 256 steps (see meet_frequentie()).

WHEEL_CIRC    = 19.1         # cm — measured wheel circumference
TRACK_WIDTH   = 13.6         # cm — track width center-to-center
STEPS_REV     = 12800        # 1/64 microstepping (TMC2209, 200 full steps x 64)
CM_PER_STEP   = WHEEL_CIRC / STEPS_REV                 # ~14.9 um/step
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH / CM_PER_STEP   # ~159 steps difference per degree

RAD_TO_DEG    = 57.29577951308232

# Velocity limits (motor 17HS8401, TMC2209 stealthChop, VREF 1 V = 0.71 A RMS)
V_MAX_CM_S    = 19.1         # 1.0 rev/s — stealthChop reaches ~300 rpm, we are at 60
V_START_CM_S  = 1.91         # 0.1 rev/s — safe starting speed, to be verified empirically
