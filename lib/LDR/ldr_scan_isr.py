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
# R29/R30: 10 kΩ -> 1 kΩ -> 10 kΩ -> nu 47 kΩ (zie git-historie en
# Meting_LDR_afstand_hoek.md). Met de huidige lamp loopt de LDR van ~4,7 kΩ
# (15 cm) tot ~450 kΩ (4 m). De resolutie per e-voud weerstand is maximaal
# (ADC_FS/4) als R_ldr = R_FIXED; 47 kΩ ligt precies op het geometrisch
# midden van dat bereik en geeft aan BEIDE uiteinden ~350-470 12-bit codes per
# e-voud. 200 kΩ zou boven ~2 m winnen, maar bij 13 cm (grijpafstand) zakken
# van ~350 naar ~96 codes per e-voud.
LDR_R_FIXED_OHM      = 47_000
# Grenzen van de log-procentschaal. Onder: gemeten minimum ~4,7 kΩ (15 cm);
# 100 Ω laat ruim marge voor een fellere bundelas. Boven: stond op 20 kΩ,
# waardoor de procentschaal al vlakbij vastklemde. 5 MΩ ligt ruim boven 4 m
# (~450 kΩ, extrapolatie ~15 m) en onder de donkerwaarde (~8-15 MΩ).
LDR_R_MIN_OHM        = 100.0
LDR_R_MAX_OHM        = 5_000_000.0

# Correctie LDR B -> LDR A (machtsverband, zie Meting_LDR_afstand_hoek.md):
#     ln R_B = GAMMA * ln R_A + LN_C      ->   ln R_B' = (ln R_B - LN_C) / GAMMA
# De twee cellen verschillen niet in een vaste factor maar in hun helling
# (gamma); een vaste gain liet op 0,15-4 m tot 0,20 restfout in ln(R) over
# (~5° koersfout), deze correctie 0,036 (~1°). Kost per meting één aftrekking
# en één vermenigvuldiging op de log die toch al berekend wordt.
# Bron: config.json, sectie "ldr"; deze waarden zijn de fallback.
CONFIG_PATH          = "config.json"
LDR_B_GAMMA          = 1.0772
LDR_B_LN_C           = -0.6005


def _load_ldr_config():
    """Lees de "ldr"-sectie uit config.json; {} als die ontbreekt of corrupt is."""
    try:
        import json
        with open(CONFIG_PATH) as f:
            return json.load(f).get("ldr", {})
    except Exception:
        return {}


_cfg = _load_ldr_config()
LDR_R_FIXED_OHM = float(_cfg.get("r_fixed_ohm", LDR_R_FIXED_OHM))
LDR_B_GAMMA     = float(_cfg.get("b_gamma", LDR_B_GAMMA))
LDR_B_LN_C      = float(_cfg.get("b_ln_c", LDR_B_LN_C))
del _cfg
_INV_B_GAMMA    = 1.0 / LDR_B_GAMMA
_LN_R_MIN       = math.log(LDR_R_MIN_OHM)
_LN_R_MAX       = math.log(LDR_R_MAX_OHM)

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

# --- Fase-gesynchroniseerde achtergrond-sampling (zie start_synced_sampling) ---
SYNC_BURST_N   = 8            # samples per servo-PWM-tick (20 ms), back-to-back
_sync_buf_a    = [0.0, 0.0]   # ping-pong: laatst-klare gemiddelde adc_u16 per buffer
_sync_buf_b    = [0.0, 0.0]
_sync_ready    = 0            # index van de buffer die klaar is om te LEZEN
_sync_acc_a    = 0
_sync_acc_b    = 0
_sync_n        = 0
_sync_active   = False

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
    """LET OP: de LDR_SAMPLES_PER_TICK samples hieronder volgen elkaar zonder
    delay op (microseconden totaal), dus dit middelt GEEN 100Hz-netrimpel weg
    zoals measure_now() dat doet — dit is een hardware-IRQ en mag niet
    blokkeren. De rimpel komt hier terug als sample-tot-sample ruis over de
    scan; de piekbepaling over een hele sweep is daar in de praktijk ongevoelig
    voor, maar voor een losse, nauwkeurige meting gebruik measure_now().
    """
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

def measure_now(n=10, interval_ms=1):
    """Direct beide LDR-waarden lezen (%, tuple A/B).

    Samples 1 ms uit elkaar (default n=10 -> 10 ms = één 100Hz-netperiode),
    zodat de netrimpel van de lichtbron zichzelf opheft in het gemiddelde
    (zelfde truc als tests/test_ldr.py en tests/test_ldr_beam.py). Dit is GEEN
    real-time context zoals _on_pio_irq() hieronder, dus blokkeren mag hier.
    """
    import time
    _init_hw()
    acc_a = 0
    acc_b = 0
    for i in range(n):
        acc_a += _adc_a.read_u16()
        acc_b += _adc_b.read_u16()
        if i < n - 1:
            time.sleep_ms(interval_ms)
    pa, pb = _adc_to_percent_ab(acc_a // n, acc_b // n)
    return (round(pa, 1), round(pb, 1))

# =========================
#  FASE-GESYNCHRONISEERDE ACHTERGROND-SAMPLING (servo-PWM-tick)
# =========================
#
# Alternatief voor measure_now(): i.p.v. blokkerend 10 samples over 10 ms uit
# te middelen om de 100Hz-netrimpel WEG te middelen, wordt hier bewust altijd
# op DEZELFDE fase t.o.v. die rimpel gesampled — getriggerd door de rising
# edge van servo1's 50Hz-PWM (zie lib/servo/servo_crl.py, GPIO2->GPIO5). Dat
# geeft geen ruis meer (de fase ligt vast), maar wel een STATISCHE bias ter
# grootte van de rimpelwaarde op die fase. Voor een differentieel gebruik
# (A-B, of Q-verschil over een rit) valt die bias grotendeels weg omdat beide
# kanalen in dezelfde IRQ-call, op vrijwel hetzelfde moment, dezelfde bias
# zien. Voor een absolute meting verdwijnt de bias in de gefitte constanten
# (LDR_B_GAMMA, LDR_B_LN_C).
#
# LET OP: de servo-PWM-klok (RP2350) en het 230V-net lopen op onafhankelijke
# oscillators, dus de fase driftlangzaam weg (orde seconden-tot-minuten voor
# een volledige cyclus). "Statisch" geldt dus op de tijdschaal van één meting
# of één rijsegment, niet voor iets dat minutenlang loopt.
#
# _sync_tick() draait IN DE HARDE PWM-IRQ van ServoController (zie
# attach_tick_callback in servo_crl.py): geen allocaties, geen sleep, geen
# exception op het normale pad. De SYNC_BURST_N samples per tick zijn back-
# to-back (geen 1 ms-spacing zoals measure_now) en middelen alleen ADC-/
# thermische ruis uit, niet de netrimpel — dat is hier al opgelost door de
# vaste fase.
#
# Slaat over tijdens een actieve scan() (_mode == "scan"): scan() heeft zijn
# eigen ISR (_on_pio_irq) op dezelfde _adc_a/_adc_b, en twee IRQ's die
# tegelijk een ADC-kanaal omschakelen kan de mux van elkaar laten conflicteren.

def _sync_tick():
    """Aan te roepen vanuit ServoController._pwm_irq via attach_tick_callback.
    Puur mechanisch: ADC-burst + optellen, wisselt de ping-pong-buffer om de
    SYNC_BURST_N samples. Doet GEEN %-omrekening (te duur voor een hard-IRQ)."""
    global _sync_acc_a, _sync_acc_b, _sync_n, _sync_ready
    if _mode == "scan" or _adc_a is None:
        return
    _sync_acc_a += _adc_a.read_u16()
    _sync_acc_b += _adc_b.read_u16()
    _sync_n += 1
    if _sync_n >= SYNC_BURST_N:
        w = 1 - _sync_ready
        _sync_buf_a[w] = _sync_acc_a / _sync_n
        _sync_buf_b[w] = _sync_acc_b / _sync_n
        _sync_ready = w
        _sync_acc_a = 0
        _sync_acc_b = 0
        _sync_n = 0

def start_synced_sampling(servo_controller):
    """Koppel de LDR-sampling aan de 50Hz-servo-PWM-tick van `servo_controller`
    (een lib.servo.servo_crl.ServoController). Vereist dat GPIO2 fysiek naar
    GPIO5 is verbonden (zie hardware/gpio_pinout.md). Na deze aanroep geeft
    read_synced() een doorlopend ververste waarde (max ~20 ms oud, één
    servo-PWM-frame) zonder dat de aanroeper hoeft te blokkeren."""
    global _sync_active
    _init_hw()
    servo_controller.attach_tick_callback(_sync_tick)
    _sync_active = True

def stop_synced_sampling(servo_controller):
    """Ontkoppel weer; read_synced() geeft daarna de laatst gemeten waarde."""
    global _sync_active
    servo_controller.attach_tick_callback(None)
    _sync_active = False

def read_synced():
    """Laatste klare, fase-gesynchroniseerde LDR-waarde (%, tuple A/B).

    Niet-blokkerend: leest alleen de al-klare ping-pong-buffer. Vereist een
    voorafgaande start_synced_sampling(); zonder actieve tick-bron geeft dit
    de laatst berekende waarde (of (0.0, 0.0) als er nog nooit gesampled is)."""
    pa, pb = _adc_to_percent_ab(int(_sync_buf_a[_sync_ready]),
                                int(_sync_buf_b[_sync_ready]))
    return (round(pa, 1), round(pb, 1))

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

def _ln_ab(adc_a, adc_b):
    """ADC-waarden -> (ln R_A, ln R_B'), met R_B' gecorrigeerd naar cel A."""
    ln_a = math.log(_adc_to_res_ohm(adc_a))
    ln_b = (math.log(_adc_to_res_ohm(adc_b)) - LDR_B_LN_C) * _INV_B_GAMMA
    return ln_a, ln_b

def _ln_to_percent(ln_r):
    """ln(weerstand) -> 0..100 % (log-schaal, 100 % = LDR_R_MIN_OHM = fel)."""
    p = 100.0 * (_LN_R_MAX - ln_r) / (_LN_R_MAX - _LN_R_MIN)
    if p < 0.0:   p = 0.0
    if p > 100.0: p = 100.0
    return p

def _adc_to_percent_ab(adc_a, adc_b):
    """ADC-waarden -> (%A, %B), B gecorrigeerd naar cel A."""
    ln_a, ln_b = _ln_ab(adc_a, adc_b)
    return _ln_to_percent(ln_a), _ln_to_percent(ln_b)

# =========================
#  RICHTINGSSIGNAAL
# =========================
#
# ldr_diff = ln R_A - ln R_B'  (B gecorrigeerd naar cel A, zie LDR_B_GAMMA)
#
# Positief = A donkerder dan B = bron ligt RECHTS (B is de rechter cel). Dat
# is de publieke conventie van stepper_ramp, dus daar LDR_DIFF_SIGN = +1.
# Gemeten op 0,5 m (Meting_LDR_afstand_hoek.md), kar gedraaid t.o.v. de bron:
#
#     bron rechts  1°: +0,02   3°: +0,11   5°: +0,22   10°: +0,79   15°: +1,39
#     bron links   1°: -0,02   3°: -0,09   5°: -0,19   10°: -0,73   15°: -1,45
#
# Monotoon tot ±15°; daarna kijken beide cellen langs de bron en zakt de
# waarde in (20°: ~0,3-0,4), maar het TEKEN blijft tot ±25° goed. Bewust geen
# (A-B)/(A+B): die explodeert als beide cellen donker worden.

def diff_from_adc(adc_a, adc_b):
    ln_a, ln_b = _ln_ab(adc_a, adc_b)
    return ln_a - ln_b

def measure_diff(n=10, interval_ms=1):
    """Blokkerend: ldr_diff over één 100Hz-netperiode (zie measure_now)."""
    import time
    _init_hw()
    acc_a = 0
    acc_b = 0
    for i in range(n):
        acc_a += _adc_a.read_u16()
        acc_b += _adc_b.read_u16()
        if i < n - 1:
            time.sleep_ms(interval_ms)
    return diff_from_adc(acc_a // n, acc_b // n)

def read_synced_diff():
    """Niet-blokkerend: ldr_diff uit de fase-gesynchroniseerde buffer.
    Geschikt als ldr_diff-callable voor stepper_ramp.HeadingController."""
    return diff_from_adc(int(_sync_buf_a[_sync_ready]),
                         int(_sync_buf_b[_sync_ready]))

def _postprocess_to_percent():
    N = len(_raw_sum_a)
    pct_a = [0.0] * N
    pct_b = [0.0] * N
    avg   = [0.0] * N
    for i in range(N):
        adc_a = _raw_sum_a[i] // LDR_SAMPLES_PER_TICK
        adc_b = _raw_sum_b[i] // LDR_SAMPLES_PER_TICK
        pa, pb = _adc_to_percent_ab(adc_a, adc_b)
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
