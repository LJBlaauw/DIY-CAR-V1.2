# ================================================================
# Dual Stepper Controller without ramp, with PIO counters (optimized)
# ================================================================
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

# -----------------------------
# PIO stepper generator (delay via OSR -> X/Y)
# -----------------------------

@asm_pio(sideset_init=PIO.OUT_LOW)
def stepper():
    pull(noblock) .side(1)  # type: ignore
    nop()        [7]        # type: ignore
    mov(x, osr)             # type: ignore  recycle last delay in X
    mov(y, x)               # type: ignore
    label("delay")          # type: ignore
    nop()         .side(0)  # type: ignore
    jmp(y_dec, "delay")     # type: ignore

# === NEW TARGET COUNTER PIO ===
# STEP = pin 1, DIR = pin 0 (set in_base at StateMachine, but DIR is no longer used here)
@asm_pio()
def stepper_counter():
    # X contains the number of remaining steps (loaded from Python)
    label("loop")           # type: ignore
    wait(0, pin, 1)         # type: ignore  Wait for STEP low (pin index 1)
    wait(1, pin, 1)         # type: ignore  Wait for STEP high
    jmp(y_dec, "dec_x")     # type: ignore  Distance counter counts down from 0xFFFFFFFF to 0
    label("dec_x")          # type: ignore
    jmp(x_dec, "loop")      # type: ignore  X--, as long as X != 0 stay in loop
    irq(rel(0))                  # type: ignore  Target reached → one IRQ to Python
    jmp("loop")             # type: ignore  Wait until the motor starts running again.


# -----------------------------------------
# Constants and derivation
# -----------------------------------------
F_PIO = 15_000_000         # 150 MHz sysclk / 10 → integer clock divider (no fractional jitter).
                           # STEP pulse remains 11 cycles = 733 ns ≥ 100 ns (TMC2209 minimum).
WHEEL_CIRC   = 19.1        # cm — measured circumference (was 20.94; which gave 8.8% too short distances)
TRACK_WIDTH  = 13.6        # cm — track width center-to-center, for rotation/heading calculation
STEPS_REV    = 12800       # 1/64 microstepping (TMC2209: 200 full steps × 64; MS1→GND, MS2→+5V)
CM_PER_STEP  = WHEEL_CIRC / STEPS_REV   # ≈ 14.9 µm/step
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH / CM_PER_STEP   # ≈ 159 steps difference per degree heading

# -----------------------------------------
# Pin definition
# -----------------------------------------
ENA1  = Pin(18, Pin.OUT); ENA1.value(0)
STEP1 = Pin(17, Pin.OUT)
DIR1  = Pin(16, Pin.OUT); DIR1.value(0)

ENA2  = Pin(14, Pin.OUT); ENA2.value(0)
STEP2 = Pin(13, Pin.OUT)
DIR2  = Pin(12, Pin.OUT); DIR2.value(1)

# -----------------------------------------
# State machines for stepping
# -----------------------------------------
sm0 = StateMachine(0, stepper, freq=F_PIO, sideset_base=STEP1)
sm1 = StateMachine(1, stepper, freq=F_PIO, sideset_base=STEP2)

# -----------------------------------------
# PIO counters for target detection
# -----------------------------------------
# Note: in_base chosen so that STEP is at pin index 1 (DIR at index 0).
sm2 = StateMachine(2, stepper_counter, in_base=DIR1, freq=F_PIO)
sm3 = StateMachine(3, stepper_counter, in_base=DIR2, freq=F_PIO)

# -----------------------------------------
# PIO counters for distance measurement are set to 0 at boot
# -----------------------------------------
sm2.active(1); sm2.exec("mov(y, 0xFFFFFFFF)")
sm3.active(1); sm3.exec("mov(y, 0xFFFFFFFF)")
# -----------------------------------------
# Positions (software counter) and targets
# -----------------------------------------
pos1 = 0
pos2 = 0
target_pos1 = 0
target_pos2 = 0

# -----------------------------------------
# Helper functions
# -----------------------------------------
def speed_to_delay(speed_cm_s):
    # Overhead matched to this PIO code path (without irq in the PIO)
    OVERHEAD = 9
    f_step = speed_cm_s / CM_PER_STEP
    y = int(F_PIO / (2 * f_step) - OVERHEAD)
    return max(5, y)

def enable():
    ENA1.value(0); ENA2.value(0)

def disable():
    ENA1.value(1); ENA2.value(1)

def stop():
    sm0.active(0)
    sm1.active(0)

def status():
    print("=== STATUS ===")
    print("Motor A:", "running" if sm0.active() else "stopped",
          f"Start_Pos: {pos1*CM_PER_STEP:.2f} cm",
          f"Target_Pos: {target_pos1*CM_PER_STEP:.2f} cm",
          f"Distance: {pio_pos1()*CM_PER_STEP:.2f} cm",)
    print("Motor B:", "running" if sm1.active() else "stopped",
          f"Start_Pos: {pos2*CM_PER_STEP:.2f} cm",
          f"Target_Pos: {target_pos2*CM_PER_STEP:.2f} cm",
          f"Distance: {pio_pos2()*CM_PER_STEP:.2f} cm",)
    print("================")
# -----------------------------------------
# PIO counter helpers for distance (hardware counter)
# -----------------------------------------
def reset_PIO_distance():
    """Reset the counter value in the PIO counters to -1 (0xFFFFFFFF)."""
    for sm in (sm2, sm3):
        sm.put(0xFFFFFFFF); sm.exec("pull()"); sm.exec("mov(y, osr)")

def pio_pos1():
    """Number of step pulses motor A"""
    sm2.exec("mov(isr, y)"); sm2.exec("push()"); y_val = sm2.get()
    return 0xFFFFFFFF - y_val

def pio_pos2():
    """Number of step pulses motor B"""
    sm3.exec("mov(isr, y)"); sm3.exec("push()"); y_val = sm3.get()
    return 0xFFFFFFFF - y_val

def distance():
    p1 = pio_pos1(); p2 = pio_pos2()
    d1 = p1 * CM_PER_STEP
    d2 = p2 * CM_PER_STEP
    print("Distance (PIO) Motor A:", d1, "cm")
    print("Distance (PIO) Motor B:", d2, "cm")
    return d1, d2
# -----------------------------------------
# PIO counter IRQ handlers – target reached
# -----------------------------------------
def counter_irq0(sm):
    # Motor A: target reached → pos1 = target_pos1, stop sm0
    global pos1
    sm0.active(0)
#    sm2.active(0)
    pos1 += target_pos1

def counter_irq1(sm):
    # Motor B: target reached → pos2 = target_pos2, stop sm1
    global pos2
    sm1.active(0)
#    sm3.active(0)
    pos2 += target_pos2

# -----------------------------------------
# bind interrupt handlers
# -----------------------------------------
sm2.irq(handler=counter_irq0); sm3.irq(handler=counter_irq1)

# -----------------------------------------
# Start helper – now also sends target steps to sm2/sm3
# -----------------------------------------
def _start_motor(sm, delay, sm_counter,steps):
    # Load the number of steps into X of the counter SM (target count)
    sm_counter.put(steps)
    sm_counter.exec("pull()")
    sm_counter.exec("mov(x, osr)")
    sm_counter.active(1)
    # Start the stepper SM with the desired delay
    sm.put(delay)
    sm.active(1)
    enable()

# -----------------------------------------
# High-level commands (dir format: f,b,l,r)
# -----------------------------------------
def s1(dir, speed, distance):
    """Motor A: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos1
    forward = (dir.lower() == 'f')
    DIR1.value(0 if forward else 1)
    steps = int(distance / CM_PER_STEP)
    # Track target position in software
    target_pos1 = (steps if forward else -steps)
    _start_motor(sm0, speed_to_delay(abs(speed)), sm2, steps)

def s2(dir, speed, distance):
    """Motor B: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos2
    forward = (dir.lower() == 'f')
    DIR2.value(1 if forward else 0)
    steps = int(distance / CM_PER_STEP)
    target_pos2 = (steps if forward else -steps)
    _start_motor(sm1, speed_to_delay(abs(speed)), sm3, steps)

def mov(dir, speed, distance):
    """Both motors in the same direction and distance."""
    s1(dir, speed, distance)
    s2(dir, speed, distance)

def rotate(dir, speed, distance):
    """Rotation: 'l' (left) or 'r' (right)."""
    if dir.lower() == 'r':
        s1('b', speed, distance)
        s2('f', speed, distance)
    else:
        s1('f', speed, distance)
        s2('b', speed, distance)
