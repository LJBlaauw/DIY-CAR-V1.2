# ldr_scan_isr.py
# LDR-scan met PIO-IRQ
# - volledige scan van 370 graden, zonder pre-roll
# - RP2350 heeft 3 PIO-blokken (SM 0-11):
#     SM0-SM3 : stepper (PIO0)
#     SM4     : ultrasoon (PIO1)
#     SM8     : LDR-scan klok (PIO2) — apart blok, geen IRQ-conflict met SM4
# - aanroepen: scan('l', 5.0, ...)
# - rekent graden -> cm via WHEEL_BASE_CM
# - CSV gebruikt ; als scheidingsteken (Nederlands Excel-formaat)
# - procedureel, geen klasse

from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio
from array import array
import math
import time

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

LDR_RING_N = 10
LDR_SAMPLE_MS = 1
TARGET_SAMPLES_PER_DEG = 3

ACCEL_MM_S2          = 500.0
SCAN_DEG             = 370.0
BACKTRACK_SPEED_CM_S = 3.0
NULL_WINDOW_DEG      = 15.0
NULL_STEP_DEG        = 1.0
NULL_DEADBAND        = 0.03
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

# 10 ms ringbuffer: één A/B/positie-sample per milliseconde.
_ring_a = array('H', [0] * LDR_RING_N)
_ring_b = array('H', [0] * LDR_RING_N)
_ring_pos = array('i', [0] * LDR_RING_N)
_ring_index = 0
_ring_count = 0
_ring_sum_a = 0
_ring_sum_b = 0
_ring_sum_pos = 0
_store_interval_us = 1000
_store_phase_us = 0

# Optionele meting van de Python ISR-duur. Standaard uit: de twee ticks_us()-
# aanroepen zouden anders zelf iedere milliseconde extra overhead toevoegen.
_irq_timing_enabled = False
_irq_last_us = 0
_irq_max_us = 0
_irq_total_us = 0
_irq_count = 0
# Tijd TUSSEN opeenvolgende ISR-starts. De PIO-tick is exact 1 ms, maar de
# handler is soft (via micropython.schedule): bezette hoofdlus, GC of andere
# geschedulede IRQ's laten hem later starten (gap_max_us) of ticks wegvallen.
# Gemiste ticks = verstreken tijd in ms + 1 - aantal ISR-aanroepen; een tick
# die alleen laat komt (gat 1,6 ms, daarna 0,4 ms) telt dus niet als gemist.
_irq_first_us = -1
_irq_prev_us = -1
_gap_max_us = 0

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
    """Blokkeer tot de stepper klaar is, max 30 s."""
    t0 = time.ticks_ms()
    if hasattr(stepper_mod, "busy"):
        is_busy = stepper_mod.busy
    else:
        is_busy = lambda: (stepper_mod.sm0.active() or
                           stepper_mod.sm1.active())
    while is_busy():
        if time.ticks_diff(time.ticks_ms(), t0) > _STEPPER_DONE_TIMEOUT_MS:
            stepper_mod.stop()
            raise RuntimeError("stepper timeout in _wait_stepper_done")
        time.sleep_ms(1)


def _on_pio_irq(sm):
    """1kHz-sampler: update 10ms-ringgemiddelde en decimeer naar scanpunten.

    Er vindt geen allocatie en geen blokkering plaats. De opgeslagen
    stepperpositie is het gemiddelde over exact hetzelfde venster als A en B;
    daardoor wordt de groepsvertraging van circa 4,5 ms geen hoekverschuiving.
    """
    global _ring_index, _ring_count
    global _ring_sum_a, _ring_sum_b, _ring_sum_pos
    global _store_interval_us, _store_phase_us, _idx, _tick_counter, _segment_done
    global _irq_last_us, _irq_max_us, _irq_total_us, _irq_count
    global _irq_first_us, _irq_prev_us, _gap_max_us

    if _tick_counter <= 0:
        return

    t0 = 0
    if _irq_timing_enabled:
        t0 = time.ticks_us()
        if _irq_prev_us >= 0:
            gap = time.ticks_diff(t0, _irq_prev_us)
            if gap > _gap_max_us:
                _gap_max_us = gap
        else:
            _irq_first_us = t0
        _irq_prev_us = t0
    i = _ring_index
    new_a = _adc_a.read_u16()
    new_b = _adc_b.read_u16()
    new_pos = _stepper_pos()

    if _ring_count < LDR_RING_N:
        _ring_count += 1
    else:
        _ring_sum_a -= _ring_a[i]
        _ring_sum_b -= _ring_b[i]
        _ring_sum_pos -= _ring_pos[i]

    _ring_a[i] = new_a
    _ring_b[i] = new_b
    _ring_pos[i] = new_pos
    _ring_sum_a += new_a
    _ring_sum_b += new_b
    _ring_sum_pos += new_pos
    _ring_index = (i + 1) % LDR_RING_N

    if _ring_count >= LDR_RING_N:
        _store_phase_us += LDR_SAMPLE_MS * 1000
        if _store_phase_us >= _store_interval_us:
            _store_phase_us -= _store_interval_us
            out = _idx
            _raw_sum_a[out] = _ring_sum_a // LDR_RING_N
            _raw_sum_b[out] = _ring_sum_b // LDR_RING_N
            _step_s1[out] = _ring_sum_pos // LDR_RING_N
            _idx = out + 1
            _tick_counter -= 1
            if _tick_counter == 0:
                _segment_done = True

    if _irq_timing_enabled:
        elapsed = time.ticks_diff(time.ticks_us(), t0)
        _irq_last_us = elapsed
        if elapsed > _irq_max_us:
            _irq_max_us = elapsed
        _irq_total_us += elapsed
        _irq_count += 1


def enable_irq_timing(enabled=True):
    """Schakel ISR-duurmeting in/uit; reset statistiek bij inschakelen."""
    global _irq_timing_enabled, _irq_last_us, _irq_max_us
    global _irq_total_us, _irq_count
    global _irq_first_us, _irq_prev_us, _gap_max_us
    _irq_timing_enabled = bool(enabled)
    if enabled:
        _irq_last_us = 0
        _irq_max_us = 0
        _irq_total_us = 0
        _irq_count = 0
        _irq_first_us = -1
        _irq_prev_us = -1
        _gap_max_us = 0


def irq_timing_stats():
    """Retourneer ISR-duur (last/max/avg in us), CPU-aandeel bij een 1ms-tick,
    het grootste gat tussen twee ISR-starts en het aantal gemiste ticks."""
    avg = (_irq_total_us / _irq_count) if _irq_count else None
    missed = None
    if _irq_count:
        span_ms = time.ticks_diff(_irq_prev_us, _irq_first_us) / 1000.0
        missed = max(0, int(span_ms + 0.5) + 1 - _irq_count)
    return dict(enabled=_irq_timing_enabled, count=_irq_count,
                last_us=_irq_last_us, max_us=_irq_max_us, avg_us=avg,
                cpu_percent=(avg / 10.0) if avg is not None else None,
                gap_max_us=_gap_max_us, missed_ticks=missed)

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
    diff  = [0.0] * N
    for i in range(N):
        adc_a = _raw_sum_a[i]
        adc_b = _raw_sum_b[i]
        ln_a, ln_b = _ln_ab(adc_a, adc_b)
        pa, pb = _ln_to_percent(ln_a), _ln_to_percent(ln_b)
        pct_a[i] = round(pa, 1)
        pct_b[i] = round(pb, 1)
        avg[i]   = round((pa + pb) * 0.5, 1)
        diff[i]  = ln_a - ln_b
    return pct_a, pct_b, avg, diff, _step_s1


def _find_peak(values):
    if not values:
        return -1, None
    i_max = 0
    v_max = values[0]
    for i in range(1, len(values)):
        if values[i] > v_max:
            v_max = values[i]
            i_max = i
    return i_max, v_max


def _sample_angle(i, steps, total_deg=SCAN_DEG):
    """Hoek van sample i uit de werkelijk getelde stappen (fallback: index)."""
    if not steps:
        return 0.0
    span = steps[-1] - steps[0]
    if span:
        return total_deg * (steps[i] - steps[0]) / span
    return total_deg * i / max(1, len(steps) - 1)


def _find_alignment(pct_a, pct_b, avg, diff, steps, total_deg=SCAN_DEG):
    """Vind de helderste nuldoorgang van het gekalibreerde A-B-signaal."""
    peak_a, _ = _find_peak(pct_a)
    peak_b, _ = _find_peak(pct_b)
    best = None
    for i in range(1, len(diff)):
        d0, d1 = diff[i - 1], diff[i]
        if d0 == 0.0 or d1 == 0.0 or (d0 < 0.0 < d1) or (d1 < 0.0 < d0):
            denom = abs(d0) + abs(d1)
            frac = abs(d0) / denom if denom else 0.0
            brightness = avg[i - 1] + frac * (avg[i] - avg[i - 1])
            angle0 = _sample_angle(i - 1, steps, total_deg)
            angle1 = _sample_angle(i, steps, total_deg)
            angle = angle0 + frac * (angle1 - angle0)
            candidate = (brightness, angle, i - 1, i)
            if best is None or candidate[0] > best[0]:
                best = candidate

    if best is not None:
        return dict(angle_deg=best[1] % 360.0,
                    source="zero_crossing", brightness=best[0],
                    bracket=(best[2], best[3]),
                    peak_a_index=peak_a, peak_b_index=peak_b)

    # Geen duidelijke nuldoorgang: gebruik het circulaire midden van de twee
    # afzonderlijke LDR-pieken als grove, stappen-gebaseerde fallback.
    if peak_a >= 0 and peak_b >= 0:
        a = _sample_angle(peak_a, steps, total_deg) % 360.0
        b = _sample_angle(peak_b, steps, total_deg) % 360.0
        delta = ((b - a + 180.0) % 360.0) - 180.0
        angle = (a + 0.5 * delta) % 360.0
        return dict(angle_deg=angle, source="peak_midpoint", brightness=None,
                    bracket=None, peak_a_index=peak_a, peak_b_index=peak_b)
    return None


def _rotate_degrees(stepper_mod, direction, degrees, speed_cm_s):
    if degrees <= 0.0:
        return False
    if hasattr(stepper_mod, "rotate_deg"):
        signed = degrees if direction == "r" else -degrees
        started = stepper_mod.rotate_deg(signed, speed_cm_s)
    else:
        started = stepper_mod.rotate(direction, speed_cm_s,
                                     _deg_to_distance_cm(degrees))
    if started is not False:
        _wait_stepper_done(stepper_mod)
    return started


def _go_to_alignment(stepper_mod, scan_dir, angle_deg, speed_cm_s):
    """Ga vanaf eindhoek 370 graden via de kortste weg naar angle_deg."""
    end_angle = SCAN_DEG % 360.0
    forward_deg = (angle_deg - end_angle) % 360.0
    backward_deg = (end_angle - angle_deg) % 360.0
    speed = min(speed_cm_s, BACKTRACK_SPEED_CM_S)
    if forward_deg <= backward_deg:
        _rotate_degrees(stepper_mod, scan_dir, forward_deg, speed)
        return dict(direction=scan_dir, degrees=forward_deg,
                    method="forward_shortest", backlash_deg=0.0)

    # Bij terugdraaien iets voorbij het doel en eindigen in scanrichting, zodat
    # de mechanische speling steeds vanaf dezelfde kant wordt opgenomen.
    backlash = min(2.0, max(0.0, 0.5 * backward_deg))
    _rotate_degrees(stepper_mod, _opposite_dir(scan_dir),
                    backward_deg + backlash, speed)
    _rotate_degrees(stepper_mod, scan_dir, backlash, speed)
    return dict(direction=_opposite_dir(scan_dir), degrees=backward_deg,
                method="backward_with_backlash", backlash_deg=backlash)


def _fine_null_seek(stepper_mod, speed_cm_s):
    """Zoek A-B=0 binnen een begrensd venster; herstel positie bij mislukken."""
    d = measure_diff()
    start_diff = d
    moved_right_deg = 0.0
    samples = [d]
    if abs(d) <= NULL_DEADBAND:
        return dict(success=True, reason="deadband", start_diff=d,
                    final_diff=d, moved_deg=0.0, samples=samples)

    while abs(moved_right_deg) < NULL_WINDOW_DEG:
        direction = "r" if d > 0.0 else "l"
        step_deg = min(NULL_STEP_DEG,
                       NULL_WINDOW_DEG - abs(moved_right_deg))
        _rotate_degrees(stepper_mod, direction, step_deg,
                        min(speed_cm_s, BACKTRACK_SPEED_CM_S))
        moved_right_deg += step_deg if direction == "r" else -step_deg
        new_d = measure_diff()
        samples.append(new_d)
        if abs(new_d) <= NULL_DEADBAND or d * new_d < 0.0:
            return dict(success=True, reason="zero_crossing", start_diff=start_diff,
                        final_diff=new_d, moved_deg=moved_right_deg,
                        samples=samples)
        d = new_d

    # Geen nul gevonden: terug naar de grove stappen-target.
    if moved_right_deg:
        restore_dir = "l" if moved_right_deg > 0.0 else "r"
        _rotate_degrees(stepper_mod, restore_dir, abs(moved_right_deg),
                        min(speed_cm_s, BACKTRACK_SPEED_CM_S))
    return dict(success=False, reason="window_exhausted", start_diff=start_diff,
                final_diff=d, moved_deg=0.0, samples=samples)


def _write_csv(path, PCT_A, PCT_B, AVG, DIFF, STEP):
    """Schrijf CSV met ; als scheidingsteken (Nederlands Excel-formaat).

    Na een scan staan er ~1100 floats per lijst op de heap. Twee keer (van ~25
    pogingen, 2026-10-21) gaf het schrijven toen "memory allocation failed,
    allocating 1148 bytes" bij >250 kB vrij; oorzaak niet gevonden, vermoedelijk
    fragmentatie. Daarom eerst gc.collect() en bij MemoryError één herkansing.
    """
    import gc
    err = None
    for _ in range(2):
        gc.collect()
        try:
            with open(path, "w") as f:
                f.write("index;ldr_a_pct;ldr_b_pct;avg_pct;ldr_diff;stepper_stappen\n")
                for i in range(len(AVG)):
                    f.write("{};{:.1f};{:.1f};{:.1f};{:.6f};{}\n".format(
                        i + 1, PCT_A[i], PCT_B[i], AVG[i], DIFF[i], STEP[i]))
            return True, None
        except MemoryError as e:
            err = e
        except Exception as e:
            return False, e
    return False, err

# =========================
#  HOOFD: scan(...)
# =========================

def scan(dir_char,
         speed_cm_s,
         graden=SCAN_DEG,
         start_graden=0.0,
         go_max=True,
         excel=True,
         out_csv=CSV_DEFAULT):
    """Scan 370 graden met een 1kHz/10ms-ringgemiddelde.

    `graden` en `start_graden` blijven tijdelijk in de signatuur om oude
    aanroepen met een duidelijke foutmelding af te vangen. Een scan korter dan
    370 graden of met pre-roll kan het maximum rond 0/360 graden missen.
    """
    global _raw_sum_a, _raw_sum_b, _step_s1
    global _idx, _tick_counter, _segment_done, _mode
    global _ring_a, _ring_b, _ring_pos, _ring_index, _ring_count
    global _ring_sum_a, _ring_sum_b, _ring_sum_pos
    global _store_interval_us, _store_phase_us

    if _mode != "idle":
        raise RuntimeError("LDR scan busy")
    dir_char = str(dir_char).lower()
    if dir_char not in ("l", "r"):
        raise ValueError("dir_char moet 'l' of 'r' zijn")
    if abs(float(graden) - SCAN_DEG) > 1e-6:
        raise ValueError("LDR scan moet %.0f graden zijn" % SCAN_DEG)
    if start_graden not in (None, 0, 0.0):
        raise ValueError("start_graden/pre-roll is vervallen; gebruik 0")
    if speed_cm_s <= 0:
        raise ValueError("speed_cm_s moet groter dan nul zijn")

    import stepper
    if hasattr(stepper, "pio_pos1"):
        attach_stepper_reader(stepper.pio_pos1)

    total_deg = SCAN_DEG
    T_s, dist_cm = _estimate_time_from_degrees(total_deg, speed_cm_s)
    T_s *= 1.05
    target_samples = max(1, int(round(TARGET_SAMPLES_PER_DEG * total_deg)))
    output_interval_ms = (T_s * 1000.0) / target_samples

    _raw_sum_a = array('I', [0] * target_samples)
    _raw_sum_b = array('I', [0] * target_samples)
    _step_s1 = array('i', [0] * target_samples)
    _ring_a = array('H', [0] * LDR_RING_N)
    _ring_b = array('H', [0] * LDR_RING_N)
    _ring_pos = array('i', [0] * LDR_RING_N)
    _ring_index = 0
    _ring_count = 0
    _ring_sum_a = _ring_sum_b = _ring_sum_pos = 0
    _store_interval_us = max(1000, int(round(output_interval_ms * 1000.0)))
    # Eerste uitvoer zodra de ring na 10 ms volledig gevuld is.
    _store_phase_us = _store_interval_us - (LDR_SAMPLE_MS * 1000)
    _idx = 0
    _tick_counter = target_samples
    _segment_done = False

    # SM8 samplet vast op 1 kHz; scanpunten worden met een fase-accumulator
    # gedecimeerd naar TARGET_SAMPLES_PER_DEG zonder afrondingsdrift.
    _mode = "scan"
    _start_pio(LDR_SAMPLE_MS)
    stepper.rotate(dir_char, speed_cm_s, dist_cm)

    scan_timeout_ms = int((T_s * 2.0 + 5.0) * 1000)
    t0 = time.ticks_ms()
    try:
        while not _segment_done:
            if time.ticks_diff(time.ticks_ms(), t0) > scan_timeout_ms:
                stepper.stop()
                raise RuntimeError("LDR scan timeout: samples niet compleet")
            time.sleep_ms(1)
    finally:
        _stop_pio()
        _mode = "idle"

    _wait_stepper_done(stepper)
    PCT_A, PCT_B, AVG, DIFF, STEP = _postprocess_to_percent()
    alignment = _find_alignment(PCT_A, PCT_B, AVG, DIFF, STEP, total_deg)
    avg_peak_index, avg_peak_percent = _find_peak(AVG)

    return_path = None
    null_seek = None
    if go_max and alignment is not None:
        return_path = _go_to_alignment(stepper, dir_char,
                                       alignment["angle_deg"], speed_cm_s)
        null_seek = _fine_null_seek(stepper, speed_cm_s)

    csv_path = None
    csv_err = None
    if excel:
        ok, err = _write_csv(out_csv, PCT_A, PCT_B, AVG, DIFF, STEP)
        csv_path = out_csv if ok else None
        csv_err = str(err) if not ok else None

    return dict(
        samples=len(AVG),
        sample_ms=LDR_SAMPLE_MS,
        average_samples=LDR_RING_N,
        output_interval_ms=output_interval_ms,
        irq_timing=irq_timing_stats(),
        est_time_s=T_s,
        total_deg=total_deg,
        dist_cm=dist_cm,
        alignment=alignment,
        alignment_angle_deg=(alignment["angle_deg"]
                             if alignment is not None else None),
        peak_a_index=(alignment["peak_a_index"]
                      if alignment is not None else None),
        peak_b_index=(alignment["peak_b_index"]
                      if alignment is not None else None),
        peak_index=avg_peak_index,
        peak_percent=avg_peak_percent,
        s1_at_peak=(STEP[avg_peak_index] if avg_peak_index >= 0 else None),
        backtrack=return_path,
        null_seek=null_seek,
        csv_path=csv_path,
        csv_error=csv_err,
    )
