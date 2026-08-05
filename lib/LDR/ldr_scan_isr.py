# ldr_scan_isr.py
# LDR-scan met PIO-IRQ
# - RP2350 heeft 3 PIO-blokken (SM 0-11):
#     SM0-SM3 : stepper (PIO0)
#     SM4     : ultrasoon (PIO1)
#     SM8     : LDR-scan klok (PIO2) — apart blok, geen IRQ-conflict met SM4
# - aanroepen: scan('l', 5.0, 100.0, start_graden=10.0, ...)
# - rekent graden -> cm via WHEEL_BASE_CM
# - CSV gebruikt ; als scheidingsteken (Nederlands Excel-formaat)
# - procedureel, geen klasse

from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio
from array import array
import math

# =========================
#  CONFIG / CONSTANTEN
# =========================

LDR_SM_ID            = 8              # PIO2 SM0 — los van stepper (PIO0) en ultrasoon (PIO1)
LDR_PIO_FREQ_HZ      = 100_000

LDR_PIN_A            = 26
LDR_PIN_B            = 27
LDR_TIMER_PIN        = 9              # sideset-uitgang van PIO-klok (NC op PCB, intern gebruik)

# geometrie
# LET OP: dit moet dezelfde spoorbreedte zijn als TRACK_WIDTH in
# lib/stepper/stepper_ramp.py. Stond eerder op 18.5, terwijl de gemeten
# spoorbreedte hart-op-hart 13.6 cm is -> een gecommandeerde 370°-scan draaide
# in werkelijkheid 503°. Empirische correctie (tyre-scrub, backlash) hoort in
# ROT_SCALE, niet in deze waarde.
WHEEL_BASE_CM        = 13.6
ROT_SCALE            = 1.0

# LDR instellingen
#
# Topologie: R_FIXED is de PULL-UP naar 3V3, de LDR is de pull-down naar GND
# (zie hardware/gpio_pinout.md, R29/R30). Fel licht -> lage LDR-weerstand ->
# lage ADC-waarde. _adc_to_res_ohm() rekent daarop.
#
# R29/R30 zijn van 10 kΩ naar 1 kΩ gebracht: bij een LDR van 100-200 Ω gebruikte
# 10 kΩ maar 0,97 % van de ADC-schaal (~40 werkelijke 12-bit codes), en met 1 kΩ
# is dat 7,6 % — een factor 7,8. Dat is nodig voor de bundelas-bepaling, die een
# Q-daling van ~34 LSB's moet zien; met 10 kΩ zou dat 4,4 LSB's zijn en dus in
# de ruis verdwijnen. Over 20 Ω .. 20 kΩ blijft de resolutie 79-1024 werkelijke
# 12-bit codes per e-voud, dus het hele werkbereik van 5 m tot 5 cm is bruikbaar.
LDR_R_FIXED_OHM      = 1_000
# Ondergrens van de procentschaal. Stond op 60 Ω, maar op de bundelas dichtbij
# komt de cel daaronder -> de schaal klemde dan vast op 100 % en de eindfase had
# geen informatie meer. 20 Ω geeft marge en houdt nog ~79 codes per e-voud.
LDR_R_MIN_OHM        = 20.0
LDR_R_MAX_OHM        = 20_000.0
# LET OP: LDR_GAIN_B compenseerde ook de tolerantie van het oude 10 kΩ-paar.
# Na het verwisselen van R29/R30 moet deze factor OPNIEUW gekalibreerd worden.
LDR_GAIN_A           = 1.0
LDR_GAIN_B           = 1.136

LDR_SAMPLES_PER_TICK = 8
TARGET_SAMPLES_PER_DEG = 3

ACCEL_MM_S2          = 500.0
BACKTRACK_SPEED_CM_S = 3.0
CSV_DEFAULT          = "/scan.csv"

# =========================
#  PIO TIMER (op PIO2)
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
    """Blokkeer tot beide motor state machines klaar zijn, max 30 s."""
    import time
    t0 = time.ticks_ms()
    while stepper_mod.sm0.active() or stepper_mod.sm1.active():
        if time.ticks_diff(time.ticks_ms(), t0) > _STEPPER_DONE_TIMEOUT_MS:
            stepper_mod.stop()
            raise RuntimeError("stepper timeout in _wait_stepper_done")

# =========================
#  ISR (met guard)
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
#  KOPPELINGEN + TESTMETING
# =========================

def attach_stepper_reader(fn):
    """Bv. attach_stepper_reader(stepper.pio_pos1)."""
    global _stepper_pos
    if callable(fn):
        _stepper_pos = fn

def measure_now(n=8):
    """Direct beide LDR-waarden lezen (%, tuple A/B)."""
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
    """ADC-waarde -> LDR-weerstand in ohm.

    Deler: R_FIXED als pull-up naar 3V3, LDR als pull-down naar GND. Dan is
        adc/FS = R_ldr / (R_ldr + R_FIXED)   ->   R_ldr = R_FIXED * adc/(FS-adc)
    Klopt deze aanname niet, dan loopt de hele weerstands- en procentschaal
    omgekeerd. Test: schijn licht op LDR A en lees de ruwe ADC. Gaat die naar
    NUL, dan is de LDR de pull-down en is deze formule juist.
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
    """Schrijf CSV met ; als scheidingsteken (Nederlands Excel-formaat)."""
    try:
        with open(path, "w") as f:
            f.write("index;ldr_a_pct;ldr_b_pct;avg_pct;stepper_stappen\n")
            for i in range(len(AVG)):
                f.write("{};{:.1f};{:.1f};{:.1f};{}\n".format(
                    i + 1, PCT_A[i], PCT_B[i], AVG[i], STEP[i]))
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
         excel=True,
         out_csv=CSV_DEFAULT):
    """
    Voorbeeld:
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

    # 1) PRE-ROLL (blokkerend)
    if start_graden and start_graden > 0.0:
        pre_dist_cm = _deg_to_distance_cm(start_graden)
        stepper.rotate(_opposite_dir(dir_char), speed_cm_s, pre_dist_cm)
        _wait_stepper_done(stepper)

    # 2) scan planning (pre-roll + scan samen voor tijdschatting)
    total_deg = graden + max(0.0, start_graden)
    T_s, dist_cm = _estimate_time_from_degrees(total_deg, speed_cm_s)
    T_s *= 1.05

    # 3) aantal samples + tick_ms
    target_samples = int(round(TARGET_SAMPLES_PER_DEG * total_deg))
    if target_samples < 1:
        target_samples = 1
    tick_ms = int(round((T_s * 1000.0) / target_samples))
    if tick_ms < 1:   tick_ms = 1
    if tick_ms > 100: tick_ms = 100

    N = target_samples

    # 4) buffers
    _raw_sum_a = array('I', [0] * N)
    _raw_sum_b = array('I', [0] * N)
    _step_s1   = array('i', [0] * N)
    _idx = 0
    _tick_counter = N
    _segment_done = False

    # 5) PIO starten, dan scan-rotatie
    _start_pio(tick_ms)
    _mode = "scan"
    stepper.rotate(dir_char, speed_cm_s, dist_cm)

    # 6) wachten tot alle samples binnen zijn (max T_s * 2 + 5 s veiligheidsmarge)
    scan_timeout_ms = int((T_s * 2.0 + 5.0) * 1000)
    t0 = time.ticks_ms()
    try:
        while not _segment_done:
            if time.ticks_diff(time.ticks_ms(), t0) > scan_timeout_ms:
                raise RuntimeError("LDR scan timeout: samples niet compleet")
    finally:
        # 7) PIO altijd stoppen (ook bij fout of Ctrl+C)
        _stop_pio()
        _mode = "idle"

    # 7b) motor afwachten zodat backtrack niet conflicteert
    _wait_stepper_done(stepper)

    # 8) naverwerken
    PCT_A, PCT_B, AVG, STEP = _postprocess_to_percent()
    i_max, v_max = _find_peak(AVG)

    # 9) terug naar piekpositie
    back_info = None
    if go_max and i_max >= 0:
        s1_start = STEP[0]
        s1_end   = STEP[-1]
        s1_peak  = STEP[i_max]
        delta_steps = s1_peak - s1_end

        if s1_end != s1_start:
            frac    = delta_steps / (s1_end - s1_start)
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

    # 10) CSV schrijven
    csv_path = None
    csv_err  = None
    if excel:
        ok, err = _write_csv(out_csv, PCT_A, PCT_B, AVG, STEP)
        csv_path = out_csv if ok else None
        csv_err  = str(err) if not ok else None

    return dict(
        samples      = len(AVG),
        tick_ms      = tick_ms,
        est_time_s   = T_s,
        total_deg    = total_deg,
        dist_cm      = dist_cm,
        peak_index   = i_max,
        peak_percent = v_max,
        s1_at_peak   = STEP[i_max] if i_max >= 0 else None,
        backtrack    = back_info,
        csv_path     = csv_path,
        csv_error    = csv_err,
    )
