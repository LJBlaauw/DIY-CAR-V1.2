# ldr_scan_isr.py
# LDR-scan met PIO-IRQ
# - aanroepen met graden (scan('l', 5.0, 100.0, start_graden=10.0, ...))
# - rekent zelf graden -> cm gebaseerd op WHEEL_BASE_CM
# - gebruikt ALTIJD PIO state machine 5 (stepper gebruikt SM0..SM4)
# - procedureel, geen klasse
# - wacht blokkerend op pre-roll
# - ISR heeft guard tegen array index out of range

from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio
from array import array
import math

# =========================
#  CONFIG / CONSTANTEN
# =========================

LDR_SM_ID            = 5              # LDR gebruikt SM5
LDR_PIO_FREQ_HZ      = 100_000

LDR_PIN_A            = 26
LDR_PIN_B            = 27
LDR_TIMER_PIN        = 9

# geometrie
WHEEL_BASE_CM        = 18.5           # afstand tussen de wielen
ROT_SCALE            = 1.0            # tuning-factor indien praktijk ≠ theorie

# LDR instellingen
LDR_R_FIXED_OHM      = 10_000
LDR_R_MIN_OHM        = 60.0
LDR_R_MAX_OHM        = 20_000.0
LDR_GAIN_A           = 1.0
LDR_GAIN_B           = 1.136

LDR_SAMPLES_PER_TICK = 8
TARGET_SAMPLES_PER_DEG = 3            # richtwaarde ~3 samples per graad

ACCEL_MM_S2          = 500.0

BACKTRACK_SPEED_CM_S = 3.0
CSV_DEFAULT          = "/scan.csv"

# =========================
#  PIO TIMER
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
#  GLOBALE STATE
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
        except:
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
        except:
            pass

# =========================
#  HELPERS
# =========================

def _deg_to_distance_cm(deg):
    dist = math.pi * WHEEL_BASE_CM * (deg / 360.0)
    return dist * ROT_SCALE

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

def _wait_stepper_done(stepper_mod):
    """Blokkeer tot beide motor state machines klaar zijn."""
    while stepper_mod.sm0.active() or stepper_mod.sm1.active():
        pass

# =========================
#  ISR (met guard)
# =========================

def _on_pio_irq(sm):
    global _idx, _tick_counter, _segment_done

    # guard tegen extra IRQ's na einde buffer
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
#  KOPPELINGEN + TESTMETING
# =========================

def attach_stepper_reader(fn):
    """Bv. attach_stepper_reader(stepper.pio_pos1)."""
    global _stepper_pos
    if callable(fn):
        _stepper_pos = fn

def measure_now(n=8):
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
    if p < 0.0: p = 0.0
    if p > 100.0: p = 100.0
    return p

def _postprocess_to_percent():
    N = len(_raw_sum_a)
    AVG = [0.0] * N
    for i in range(N):
        adc_a = _raw_sum_a[i] // LDR_SAMPLES_PER_TICK
        adc_b = _raw_sum_b[i] // LDR_SAMPLES_PER_TICK
        ra = _adc_to_res_ohm(adc_a)
        rb = _adc_to_res_ohm(adc_b)
        pa = _res_to_percent_log(ra, LDR_GAIN_A)
        pb = _res_to_percent_log(rb, LDR_GAIN_B)
        AVG[i] = round((pa + pb) * 0.5, 1)
    return AVG, _step_s1

def _find_peak(AVG):
    if not AVG:
        return -1, None
    i_max = 0
    v_max = AVG[0]
    for i in range(1, len(AVG)):
        if AVG[i] > v_max:
            v_max = AVG[i]
            i_max = i
    return i_max, v_max

def _write_csv_small(path, AVG, STEP):
    try:
        with open(path, "w") as f:
            f.write("index,avg_percent,stepper_s1\n")
            for i in range(len(AVG)):
                f.write("{},{:.1f},{}\n".format(i+1, AVG[i], STEP[i]))
        return True, None
    except Exception as e:
        return False, e

# =========================
#  HOOFD: scan(...)
# =========================

def scan(dir_char,
         speed_cm_s,
         graden,
         start_graden=0.0,
         go_max=True,
         exell=True,
         out_csv=CSV_DEFAULT):
    """
    Voorbeeld:
      res = scan('l', 5.0, 100.0, start_graden=10.0, go_max=True, exell=True)
    """
    global _raw_sum_a, _raw_sum_b, _step_s1
    global _idx, _tick_counter, _segment_done, _mode

    if _mode != "idle":
        raise RuntimeError("LDR scan busy")

    import stepper

    # automatisch teller koppelen
    if hasattr(stepper, "pio_pos1"):
        attach_stepper_reader(stepper.pio_pos1)

    # 1) PRE-ROLL (blokkerend)
    if start_graden and start_graden > 0.0:
        pre_dist_cm = _deg_to_distance_cm(start_graden)
        stepper.rotate(_opposite_dir(dir_char), speed_cm_s, pre_dist_cm)
        _wait_stepper_done(stepper)
    else:
        pre_dist_cm = 0.0  # wordt verder niet gebruikt; alleen voor volledigheid

    # 2) totale graden (voor sampleplanning)
    total_deg = graden + max(0.0, start_graden)

    # 3) tijd + afstand schatten
    T_s, dist_cm = _estimate_time_from_degrees(total_deg, speed_cm_s)
    T_s *= 1.05   # kleine marge

    # 4) aantal samples + tick_ms
    target_samples = int(round(TARGET_SAMPLES_PER_DEG * total_deg))
    if target_samples < 1:
        target_samples = 1
    tick_ms = int(round((T_s * 1000.0) / target_samples))
    if tick_ms < 1: tick_ms = 1
    if tick_ms > 100: tick_ms = 100

    N = target_samples

    # 5) buffers
    _raw_sum_a = array('I', [0] * N)
    _raw_sum_b = array('I', [0] * N)
    _step_s1   = array('i', [0] * N)
    _idx = 0
    _tick_counter = N
    _segment_done = False

    # 6) PIO starten
    _start_pio(tick_ms)
    _mode = "scan"

    # 7) ECHTE scan-rotatie
    stepper.rotate(dir_char, speed_cm_s, dist_cm)

    # 8) wachten tot alle samples binnen zijn
    while not _segment_done:
        pass

    # 9) PIO stoppen
    _stop_pio()
    _mode = "idle"

    # 10) naverwerken
    AVG, STEP = _postprocess_to_percent()
    i_max, v_max = _find_peak(AVG)

    # 11) terug naar max (altijd tegenrichting van scan)
    back_info = None
    if go_max and i_max >= 0:
        s1_start = STEP[0]
        s1_end   = STEP[-1]
        s1_peak  = STEP[i_max]
        delta_steps = s1_peak - s1_end

        if s1_end != s1_start:
            frac   = delta_steps / (s1_end - s1_start)
            back_cm = abs(dist_cm * frac)
        else:
            back_cm = 0.0

        back_dir = _opposite_dir(dir_char)
        stepper.rotate(back_dir,
                       min(speed_cm_s, BACKTRACK_SPEED_CM_S),
                       back_cm)
        back_info = dict(back_dir=back_dir,
                         back_cm=back_cm,
                         delta_steps=delta_steps)

    # 12) CSV
    csv_path = None
    csv_err = None
    if exell:
        ok, err = _write_csv_small(out_csv, AVG, STEP)
        if ok:
            csv_path = out_csv
        else:
            csv_err = str(err)

    return dict(
        samples=len(AVG),
        tick_ms=tick_ms,
        est_time_s=T_s,
        total_deg=total_deg,
        dist_cm=dist_cm,
        peak_index=i_max,
        peak_percent=v_max,
        s1_at_peak=STEP[i_max] if i_max >= 0 else None,
        backtrack=back_info,
        csv_path=csv_path,
        csv_error=csv_err,
    )
