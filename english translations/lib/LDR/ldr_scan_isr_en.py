# ldr_scan_isr.py
# LDR scan with PIO-IRQ
# - RP2350 has 3 PIO blocks (SM 0-11):
#     SM0-SM3 : stepper (PIO0)
#     SM4     : ultrasonic (PIO1)
#     SM8     : LDR scan clock (PIO2) — separate block, no IRQ conflict with SM4
# - call: scan('l', 5.0, 100.0, start_graden=10.0, ...)
# - computes degrees -> cm via WHEEL_BASE_CM
# - CSV uses ; as delimiter (Dutch Excel format)
# - procedural, no class

from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio
from array import array
import math

# =========================
#  CONFIG / CONSTANTS
# =========================

LDR_SM_ID            = 8              # PIO2 SM0 — separate from stepper (PIO0) and ultrasonic (PIO1)
LDR_PIO_FREQ_HZ      = 100_000

LDR_PIN_A            = 26
LDR_PIN_B            = 27
LDR_TIMER_PIN        = 9              # sideset output of PIO clock (NC on PCB, internal use)

# geometry
# PLEASE NOTE: this must be the same track width as TRACK_WIDTH in
# lib/stepper/stepper_ramp.py. Was previously set to 18.5, while the measured
# track width center-to-center is 13.6 cm -> a commanded 370° scan turned
# 503° in reality. Empirical correction (tyre-scrub, backlash) belongs in
# ROT_SCALE, not in this value.
WHEEL_BASE_CM        = 13.6
ROT_SCALE            = 1.0

# LDR settings
#
# Topology: R_FIXED is the PULL-UP to 3V3, the LDR is the pull-down to GND
# (see hardware/gpio_pinout.md, R29/R30). Bright light -> low LDR resistance ->
# low ADC value. _adc_to_res_ohm() relies on this.
#
# R29/R30 have been changed from 10 kΩ to 1 kΩ: with an LDR of 100-200 Ω,
# 10 kΩ only used 0.97 % of the ADC scale (~40 actual 12-bit codes), and with 1 kΩ
# that is 7.6 % — a factor of 7.8. This is necessary for beam axis determination,
# which needs to see a Q-drop of ~34 LSBs; with 10 kΩ that would be 4.4 LSBs and
# thus disappear in the noise. Across 20 Ω .. 20 kΩ the resolution remains
# 79-1024 actual 12-bit codes per e-fold, so the entire working range from 5 m to 5 cm is usable.
LDR_R_FIXED_OHM      = 1_000
# Lower limit of the percentage scale. Was 60 Ω, but on the beam axis close by
# the cell drops below that -> the scale then clamped firmly at 100 % and the final phase
# had no more information. 20 Ω provides margin and still keeps ~79 codes per e-fold.
LDR_R_MIN_OHM        = 20.0
LDR_R_MAX_OHM        = 20_000.0
# PLEASE NOTE: LDR_GAIN_B also compensated for the tolerance of the old 10 kΩ pair.
# After swapping R29/R30, this factor MUST be calibrated RE-AGAIN.
LDR_GAIN_A           = 1.0
LDR_GAIN_B           = 1.136

LDR_SAMPLES_PER_TICK = 8
TARGET_SAMPLES_PER_DEG = 3

ACCEL_MM_S2          = 500.0
BACKTRACK_SPEED_CM_S = 3.0
CSV_DEFAULT          = "/scan.csv"

# =========================
#  PIO TIMER (on PIO2)
# =========================

@asm_pio(sideset_init=PIO.OUT_LOW)
def clk_var():
    pull(noblock)      .side(1)
    mov(x, osr)
    nop()              [7]
    mov(y, x)
    irq(rel(0))        .side(0)
    label("delay")
    jmp(y_dec, "delay")

# =========================
#  GLOBAL STATE
# =========================

_adc_a = None
_adc_b = None
_tick_pin = None
_sm = None

_raw_sum_a = None
_raw_sum_b = None
_step_s1   = None

_idx = 0
_tick_counter = 0
_segment_done = False
_mode = "idle"

def _stepper_pos_fallback():
    return 0

_stepper_pos = _stepper_pos_fallback

# =========================
#  HW INIT
# =========================

def _init_hw():
    global _adc_a, _adc_b, _tick_pin
    if _adc_a is None:
        _adc_a = ADC(Pin(LDR_PIN_A, Pin.IN))
    if _adc_b is None:
        _adc_b = ADC(Pin(LDR_PIN_B, Pin.IN))
    if _tick_pin is None:
        _tick_pin = Pin(LDR_TIMER_PIN, Pin.OUT)

def _y_from_tick_ms(ms):
    y = int(round(LDR_PIO_FREQ_HZ * (ms / 1000.0))) - 12
    if y < 1:
        y = 1
    return y

def _start_pio(tick_ms):
    global _sm
    _init_hw()
    if _sm is not None:
        try:
            _sm.irq(None)
            _sm.active(0)
        except Exception:
            pass
    _sm = StateMachine(LDR_SM_ID, clk_var,
                       freq=LDR_PIO_FREQ_HZ,
                       sideset_base=_tick_pin)
    _sm.put(_y_from_tick_ms(tick_ms))
    _sm.irq(_on_pio_irq)
    _sm.active(1)

def _stop_pio():
    global _sm
    if _sm is not None:
        try:
            _sm.irq(None)
            _sm.active(0)
        except Exception:
            pass

# =========================
#  HELPERS
# =========================

def _deg_to_distance_cm(deg):
    return math.pi * WHEEL_BASE_CM * (deg / 360.0) * ROT_SCALE

def _opposite_dir(d):
    return 'l' if d == 'r' else 'r'

def _estimate_time_from_degrees(total_deg, speed_cm_s):
    dist_cm = _deg_to_distance_cm(total_deg)
    s_mm = dist_cm * 10.0
    v_mm_s = speed_cm_s * 10.0
    a = ACCEL_MM_S2
    if s_mm <= 0 or v_mm_s <= 0:
        return 0.0, dist_cm
    t_acc = v_mm_s / a
    s_acc = 0.5 * a * t_acc * t_acc
    if s_mm >= 2.0 * s_acc:
        t_cruise = (s_mm - 2.0 * s_acc) / v_mm_s
        T = 2.0 * t_acc + t_cruise
    else:
        T = 2.0 * math.sqrt(s_mm / a)
    return T, dist_cm

_STEPPER_DONE_TIMEOUT_MS = 30_000

def _wait_stepper_done(stepper_mod):
    """Block until both motor state machines are finished, max 30 s."""
    import time
    t0 = time.ticks_ms()
    while stepper_mod.sm0.active() or stepper_mod.sm1.active():
        if time.ticks_diff(time.ticks_ms(), t0) > _STEPPER_DONE_TIMEOUT_MS:
            stepper_mod.stop()
            raise RuntimeError("stepper timeout in _wait_stepper_done")

# =========================
#  ISR (with guard)
# =========================

def _on_pio_irq(sm):
    global _idx, _tick_counter, _segment_done

    if _tick_counter <= 0:
        return

    read_a = _adc_a.read_u16
    read_b = _adc_b.read_u16

    sA = 0
    sB = 0
    n = LDR_SAMPLES_PER_TICK
    while n:
        sA += read_a()
        sB += read_b()
        n -= 1

    i = _idx
    _raw_sum_a[i] = sA
    _raw_sum_b[i] = sB
    _step_s1[i]   = _stepper_pos()

    _idx = i + 1
    _tick_counter -= 1
    if _tick_counter == 0:
        _segment_done = True

# =========================
#  LINKAGES + TEST MEASUREMENT
# =========================

def attach_stepper_reader(fn):
    """E.g. attach_stepper_reader(stepper.pio_pos1)."""
    global _stepper_pos
    if callable(fn):
        _stepper_pos = fn

def measure_now(n=8):
    """Read both LDR values directly (%, tuple A/B)."""
    _init_hw()
    acc_a = 0
    acc_b = 0
    m = n
    while m:
        acc_a += _adc_a.read_u16()
        acc_b += _adc_b.read_u16()
        m -= 1
    adc_a = acc_a // n
    adc_b = acc_b // n
    ra = _adc_to_res_ohm(adc_a)
    rb = _adc_to_res_ohm(adc_b)
    pa = round(_res_to_percent_log(ra, LDR_GAIN_A), 1)
    pb = round(_res_to_percent_log(rb, LDR_GAIN_B), 1)
    return (pa, pb)

# =========================
#  ADC → %
# =========================

def _adc_to_res_ohm(adc_u16):
    """ADC value -> LDR resistance in ohm.

    Divider: R_FIXED as pull-up to 3V3, LDR as pull-down to GND. Then:
        adc/FS = R_ldr / (R_ldr + R_FIXED)   ->   R_ldr = R_FIXED * adc/(FS-adc)
    If this assumption is incorrect, the entire resistance and percentage scale runs
    inverted. Test: shine light on LDR A and read the raw ADC. If it goes to
    ZERO, then the LDR is the pull-down and this formula is correct.
    """
    if adc_u16 <= 0:
        adc_u16 = 1
    if adc_u16 >= 65535:
        adc_u16 = 65534
    return (LDR_R_FIXED_OHM * adc_u16) / (65535.0 - adc_u16)

def _res_to_percent_log(r_ohm, gain=1.0):
    r = r_ohm / (gain if gain > 0 else 1.0)
    if r < LDR_R_MIN_OHM: r = LDR_R_MIN_OHM
    if r > LDR_R_MAX_OHM: r = LDR_R_MAX_OHM
    ln_min = math.log(LDR_R_MIN_OHM)
    ln_max = math.log(LDR_R_MAX_OHM)
    p = 100.0 * (ln_max - math.log(r)) / (ln_max - ln_min)
    if p < 0.0:   p = 0.0
    if p > 100.0: p = 100.0
    return p

def _postprocess_to_percent():
    N = len(_raw_sum_a)
    pct_a = [0.0] * N
    pct_b = [0.0] * N
    avg   = [0.0] * N
    for i in range(N):
        adc_a = _raw_sum_a[i] // LDR_SAMPLES_PER_TICK
        adc_b = _raw_sum_b[i] // LDR_SAMPLES_PER_TICK
        ra = _adc_to_res_ohm(adc_a)
        rb = _adc_to_res_ohm(adc_b)
        pa = _res_to_percent_log(ra, LDR_GAIN_A)
        pb = _res_to_percent_log(rb, LDR_GAIN_B)
        pct_a[i] = round(pa, 1)
        pct_b[i] = round(pb, 1)
        avg[i]   = round((pa + pb) * 0.5, 1)
    return pct_a, pct_b, avg, _step_s1

def _find_peak(avg):
    if not avg:
        return -1, None
    i_max = 0
    v_max = avg[0]
    for i in range(1, len(avg)):
        if avg[i] > v_max:
            v_max = avg[i]
            i_max = i
    return i_max, v_max

def _write_csv(path, PCT_A, PCT_B, AVG, STEP):
    """Write CSV with ; as delimiter (Dutch Excel format)."""
    try:
        with open(path, "w") as f:
            f.write("index;ldr_a_pct;ldr_b_pct;avg_pct;stepper_steps\n")
            for i in range(len(AVG)):
                f.write("{};{:.1f};{:.1f};{:.1f};{}\n".format(
                    i + 1, PCT_A[i], PCT_B[i], AVG[i], STEP[i]))
        return True, None
    except Exception as e:
        return False, e

# =========================
#  MAIN: scan(...)
# =========================

def scan(dir_char,
         speed_cm_s,
         graden,
         start_graden=0.0,
         go_max=True,
         excel=True,
         out_csv=CSV_DEFAULT):
    """
    Example:
      res = scan('l', 5.0, 100.0, start_graden=10.0, go_max=True, excel=True)
    """
    import time
    global _raw_sum_a, _raw_sum_b, _step_s1
    global _idx, _tick_counter, _segment_done, _mode

    if _mode != "idle":
        raise RuntimeError("LDR scan busy")

    import stepper

    if hasattr(stepper, "pio_pos1"):
        attach_stepper_reader(stepper.pio_pos1)

    # 1) PRE-ROLL (blocking)
    if start_graden and start_graden > 0.0:
        pre_dist_cm = _deg_to_distance_cm(start_graden)
        stepper.rotate(_opposite_dir(dir_char), speed_cm_s, pre_dist_cm)
        _wait_stepper_done(stepper)

