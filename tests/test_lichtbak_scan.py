# ================================================================
# tests/test_lichtbak_scan.py
#
# Meetscript voor het "scheve lichtbak"-probleem: hoe zien de ToF, de
# ultrasoon en de LDR's de lichtbak als die onder een hoek alfa t.o.v. de
# aanrijlijn van de kar staat, op korte afstand (20-50 cm)?
#
# Opzet (per positie één "sweep"):
#   - De kar staat op de gradenboog rond de lichtbron, op een bekende afstand
#     (lampfront -> voorkant ultrasoon) en onder boog-hoek alfa, met de laser
#     precies op de lamp gericht. Voor de kar is "kar op de boog onder alfa"
#     hetzelfde als "lichtbak over alfa gedraaid".
#   - De kar draait op zijn as stapsgewijs van -BEREIK tot +BEREIK graden
#     (gyro-gestuurd, stepper_ramp.rotate_deg_gyro) en meet op elke stap:
#       * ToF VL53L1X met meerdere ROI's (vol 16x16, klein 4x4 in het midden,
#         en klein 4x4 hoger/lager in het beeld, zie ROIS)
#       * ultrasoon (mediaan van US_N metingen)
#       * LDR A/B (ruwe ADC, R_A, R_B' gecorrigeerd naar cel A, ldr_diff, Q)
#     en draait daarna terug naar de laserstand (theta = 0).
#   - Elke stap wordt meteen als regel aan CSV_PAD toegevoegd.
#
# Hoekconventies (zelfde als de rest van het project):
#   theta  positief = kar naar RECHTS gedraaid (stepper_ramp.rotate_deg)
#   alfa   positief = kar staat, kijkend naar de lichtbak, RECHTS van de
#          bundelas (de loodlijn op het voorvlak door de lamp)
#   ldr_diff positief = bron ligt rechts (B is de rechter cel)
#
# Geometrie van de kar (globale_specificatie_NL.md -> Afmetingen ->
# Afstandssensoren t.o.v. de wielas): ultrasoon x=-3,5 mm (links) y=33 mm z=85 mm,
# ToF x=+32 mm (rechts, gezien vanuit de kar) y=36 mm z=73 mm, LDR's op de middellijn y~25 mm. De lichtbak
# is 100 mm hoog, dus beide afstandssensoren kijken dicht bij de bovenrand;
# daarom de hoger/lager verschoven ROI's.
#
# Referentie: Meting_LDR_afstand_hoek.md (2026-10-19) mat op 0,5 m langs de
# boog met de laser op de bron een LDR-nulafwijking van ~2-3 graden bij 30
# graden naast de as en ~7 graden bij 50 graden. sweep() print per positie
# waar ldr_diff door nul gaat, zodat je dat direct kunt vergelijken.
#
# Gebruik (REPL, na ./upload_to_pico.sh):
#   >>> import sys; sys.path.append('/tests')
#   >>> from test_lichtbak_scan import *
#   >>> roi_test()                     # eenmalig: welke ROI kijkt omhoog?
#   >>> sweep(30, 20)                  # één positie: 30 cm, alfa = +20 graden
#   >>> reeks()                        # het volledige meetprogramma
#   Daarna op de pc:  mpremote cp :/lichtbak_scan.csv .
#
# Vereist: de kar vrij om zijn as kunnen draaien (+/-BEREIK), USB-kabel
# zonder trek. De laser staat alleen aan tijdens het richten.
# ================================================================

import math
import time
import machine
from machine import ADC, Pin

import stepper_ramp as sr
import ldr_scan_isr as ldr
import ultrasoon
from servo_crl import ServoController
from icm20948 import ICM20948, load_mag_calibration
from vl53l1x import VL53L1X, LONG, RANGE_VALID, status_name, spad_nummer

# ---------------- Instellingen ----------------
BEREIK_GRAD = 40.0           # sweep van -BEREIK tot +BEREIK
STAP_GRAD = 2.0              # hoekstap van de sweep
STAP_SNELHEID_CM_S = 3.0     # langzaam: remhoek ~0,4 graden bij 55 cm/s2
GROTE_DRAAI_CM_S = 5.0       # naar het begin van de sweep en terug naar 0
TOL_GRAD = 0.3               # tolerantie per stap (rotate_deg_gyro)
SETTLE_MS = 200              # CdS-LDR's en mechaniek laten uitdempen

AFSTANDEN_CM = (50, 30, 20)  # lampfront -> voorkant ultrasoon
ALFAS_GRAD = (0, 20, 30, 45, -45)   # 30 = herhaling van de 0,5 m-meting

US_N = 5                     # ultrasoonmetingen per stap (mediaan)
US_INTERVAL_MS = 60          # ultrasoon meet elke 50 ms in de achtergrond

LDR_N = 20                   # 20 x 1 ms = twee 100Hz-netperioden
US_TOT_LDR_CM = 0.8          # LDR's zitten ~8 mm achter de voorkant ultrasoon
                             # (y = 25 mm tegen 33 mm vóór de wielas)

TOF_TIMING_BUDGET_US = 33_000
TOF_PERIOD_MS = 50
# Kleine (4x4) ROI's hebben 16x minder SPADs: met 33 ms gaven ze op 50 cm
# bijna nooit een geldige meting (sweep 2026-11-24). Langer integreren.
TOF_BUDGET_KLEIN_US = 100_000
TOF_PERIOD_KLEIN_MS = 110
TOF_N = 4                    # geldige metingen per ROI per stap
TOF_WEGGOOI = 1              # eerste meting na een ROI-wissel weggooien
                             # (stop_continuous() reset de kalibratie)
TOF_TIMEOUT_MS = 400

# ROI's: (naam, breedte, hoogte, centrum-SPAD). Rij 7/kolom 8 (= 199) is het
# midden; rij 3 en rij 12 liggen ~4,5 SPAD-rijen (~6 graden) daarboven/
# daaronder in ST's tabel. Welke fysiek OMHOOG kijkt hangt van de lens en de
# montage af. GEMETEN 2026-11-24 op de kar (karton op 15 cm, dicht vanaf
# 7,3 cm = ToF-hoogte, wand op 30 cm): rij 12 zag het karton (13,9 cm), rij 3
# keek eronderdoor naar de wand (29,9 cm) -> rij 12 = boven, rij 3 = onder.
ROIS = (
    ("vol",   16, 16, 199),
    ("m4",     4,  4, spad_nummer(7, 8)),
    ("boven",  4,  4, spad_nummer(12, 8)),
    ("onder",  4,  4, spad_nummer(3, 8)),
)

# Links/rechts naast elkaar (zelfde rij als het midden, kolom 3 en 12, ~12
# graden uit elkaar): het afstandsverschil geeft de schuinte van het
# voorvlak zonder de kar te draaien. GEMETEN 2026-11-24 (karton op 15 cm
# links van de ToF-as, wand op 30 cm): kolom 3 zag het karton (15,4 cm),
# kolom 12 de wand (~27 cm) -> kolom 3 = links, kolom 12 = rechts.
ROIS_LR = (
    ("links",  4,  4, spad_nummer(7, 3)),
    ("rechts", 4,  4, spad_nummer(7, 12)),
)
CSV_LR_PAD = "/lichtbak_lr.csv"

CSV_PAD = "/lichtbak_scan.csv"

# ---------------- Hardware (lui, één keer) ----------------
_sc = None
_tof = None
_gyro = None
_adc_a = None
_adc_b = None
_run = 0


def _init():
    global _sc, _tof, _gyro, _adc_a, _adc_b
    if _sc is not None:
        return
    _sc = ServoController()
    i2c0 = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                       freq=400000, timeout=200_000)
    _tof = VL53L1X(i2c0)
    if not _tof.init():
        raise RuntimeError("VL53L1X.init() mislukt")
    _tof.set_distance_mode(LONG)
    _tof.set_timing_budget_us(TOF_TIMING_BUDGET_US)
    i2c1 = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
    sensor = ICM20948(i2c1, **load_mag_calibration())
    _gyro = sr.gyro_z_deg_s(sensor)
    _adc_a = ADC(Pin(ldr.LDR_PIN_A, Pin.IN))
    _adc_b = ADC(Pin(ldr.LDR_PIN_B, Pin.IN))
    print("Hardware klaar. ROI's:", ", ".join(
        "%s=%dx%d@%d" % r for r in ROIS))


# ---------------- Metingen ----------------
def _mediaan(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return None
    if n % 2:
        return xs[n // 2]
    return 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def meet_tof(roi):
    """Eén ROI: (mediaan_mm of None, n_geldig, signaal_mcps, ambient_mcps,
    laatste_status)."""
    naam, w, h, c = roi
    klein = w * h < 256
    _tof.stop_continuous()
    _tof.set_roi(w, h, c)
    _tof.set_timing_budget_us(TOF_BUDGET_KLEIN_US if klein
                              else TOF_TIMING_BUDGET_US)
    _tof.start_continuous(TOF_PERIOD_KLEIN_MS if klein else TOF_PERIOD_MS)
    mms, sigs, ambs = [], [], []
    status = ""
    pogingen = TOF_N + TOF_WEGGOOI + 4      # wat ruimte voor ongeldige metingen
    for i in range(pogingen):
        t0 = time.ticks_ms()
        while not _tof.data_ready():
            if time.ticks_diff(time.ticks_ms(), t0) > TOF_TIMEOUT_MS:
                break
            time.sleep_ms(2)
        mm = _tof.read(blocking=False)
        if i < TOF_WEGGOOI:
            continue
        status = status_name(_tof.range_status)
        if _tof.range_status == RANGE_VALID:
            mms.append(mm)
            sigs.append(_tof.signal_mcps)
            ambs.append(_tof.ambient_mcps)
            if len(mms) >= TOF_N:
                break
    _tof.stop_continuous()
    if not mms:
        return None, 0, 0.0, 0.0, status
    return (_mediaan(mms), len(mms), sum(sigs) / len(sigs),
            sum(ambs) / len(ambs), status)


def meet_us():
    """Mediaan van US_N geldige ultrasoonmetingen in cm (of None), en n."""
    xs = []
    for _ in range(US_N):
        cm, kind = ultrasoon.read_cm()
        if kind == 'ok' and cm:
            xs.append(cm)
        time.sleep_ms(US_INTERVAL_MS)
    return _mediaan(xs), len(xs)


def meet_ldr(d_ldr_cm):
    """(adc_a, adc_b, R_A, R_B', ldr_diff, lnE, Q). R_B' is naar cel A
    omgerekend (LDR_B_GAMMA/LDR_B_LN_C), Q zoals in tests/test_ldr_beam.py
    maar met de gecorrigeerde B: Q = ln(R_A^(-1/g) + R_B'^(-1/g)) + 2 ln d."""
    acc_a = 0
    acc_b = 0
    for i in range(LDR_N):
        acc_a += _adc_a.read_u16()
        acc_b += _adc_b.read_u16()
        if i < LDR_N - 1:
            time.sleep_ms(1)
    a = acc_a // LDR_N
    b = acc_b // LDR_N
    ln_a, ln_b = ldr._ln_ab(a, b)
    g = ldr.GAMMA
    ln_e = math.log(math.exp(-ln_a / g) + math.exp(-ln_b / g))
    q = ln_e + 2.0 * math.log(d_ldr_cm)
    return a, b, math.exp(ln_a), math.exp(ln_b), ln_a - ln_b, ln_e, q


# ---------------- CSV ----------------
def _kop():
    k = ["run", "t_ms", "afstand_cm", "alfa_deg", "theta_cmd", "theta_gyro",
         "theta_stap", "us_cm", "us_n"]
    for naam, _, _, _ in ROIS:
        k += ["tof_%s_mm" % naam, "tof_%s_n" % naam,
              "tof_%s_sig" % naam, "tof_%s_amb" % naam]
    k += ["adc_a", "adc_b", "R_A", "R_Bc", "ldr_diff", "lnE", "Q"]
    return ",".join(k)


def _open_csv():
    try:
        with open(CSV_PAD) as f:
            nieuw = not f.readline()
    except OSError:
        nieuw = True
    f = open(CSV_PAD, "a")
    if nieuw:
        f.write(_kop() + "\n")
    return f


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return "%.4f" % v
    return str(v)


# ---------------- Draaien ----------------
def _draai_naar(doel, theta, bias, snelheid):
    """Draai van gyrohoek theta naar doel; geeft de nieuwe gyrohoek."""
    delta = doel - theta
    if abs(delta) < 0.05:
        return theta
    r = sr.rotate_deg_gyro(delta, _gyro, speed=snelheid, tol_deg=TOL_GRAD,
                           bias=bias)
    return theta + r["gyro"]


def _gyro_bias(ms=500):
    som, n, t0 = 0.0, 0, time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        som += _gyro()
        n += 1
        time.sleep_ms(3)
    return som / n


def _ldr_nul(rijen):
    """theta's waar ldr_diff van teken wisselt (lineair geïnterpoleerd)."""
    nullen = []
    for i in range(len(rijen) - 1):
        t1, d1 = rijen[i]
        t2, d2 = rijen[i + 1]
        if d1 == 0.0:
            nullen.append(t1)
        elif d1 * d2 < 0.0:
            nullen.append(t1 + (t2 - t1) * d1 / (d1 - d2))
    return nullen


# ---------------- Publieke functies ----------------
def laser_aan():
    """Laser aan om te richten, zonder input() -- voor aansturing op afstand
    (mpremote exec); sweep(..., vraag=False) zet hem daarna weer uit."""
    _init()
    _sc.laser_power(100.0)


def richt(afstand_cm, alfa_deg):
    """Laser aan, laat de gebruiker de kar plaatsen, laser uit."""
    _init()
    _sc.laser_power(100.0)
    print("\nZet de kar op de boog: %d cm (lampfront -> voorkant ultrasoon),"
          % afstand_cm)
    print("alfa = %+d graden (kar RECHTS van de bundelas = positief)."
          % alfa_deg)
    antw = input("Laser precies op de lamp, dan Enter (s = overslaan, "
                 "q = stoppen): ").strip().lower()
    _sc.laser_off()
    return antw


def sweep(afstand_cm, alfa_deg, bereik=BEREIK_GRAD, stap=STAP_GRAD,
          vraag=True):
    """Eén positie meten. Geeft de lijst theta's waar ldr_diff door nul gaat,
    of "s"/"q" als de gebruiker bij het richten oversloeg/stopte."""
    global _run
    _init()
    if vraag:
        antw = richt(afstand_cm, alfa_deg)
        if antw in ("s", "q"):
            return antw
    else:
        _sc.laser_off()        # na laser_aan() via een losse aanroep
    _run += 1
    run_id = "%d_%d_%+d" % (time.ticks_ms() // 1000, afstand_cm, alfa_deg)
    d_ldr = afstand_cm + US_TOT_LDR_CM

    time.sleep_ms(300)
    bias = _gyro_bias()
    h0 = sr.heading()
    theta = 0.0
    n_stap = int(round(bereik / stap))
    doelen = [-bereik + i * stap for i in range(2 * n_stap + 1)]

    print("\n=== sweep %s: %d cm, alfa %+d, theta %+.0f..%+.0f per %.1f ==="
          % (run_id, afstand_cm, alfa_deg, -bereik, bereik, stap))
    print(" theta    US  " + "  ".join("%6s" % r[0] for r in ROIS)
          + "   diff      Q")
    diffs = []
    f = _open_csv()
    try:
        theta = _draai_naar(doelen[0], theta, bias, GROTE_DRAAI_CM_S)
        for doel in doelen:
            theta = _draai_naar(doel, theta, bias, STAP_SNELHEID_CM_S)
            time.sleep_ms(SETTLE_MS)
            ldr_v = meet_ldr(d_ldr)          # eerst: na de rust, vóór ToF-I2C
            us_cm, us_n = meet_us()
            tofs = [meet_tof(r) for r in ROIS]
            rij = [run_id, time.ticks_ms(), afstand_cm, alfa_deg, doel,
                   theta, sr.heading() - h0, us_cm, us_n]
            for mm, n, sig, amb, _ in tofs:
                rij += [mm, n, sig, amb]
            rij += list(ldr_v)
            f.write(",".join(_fmt(v) for v in rij) + "\n")
            diffs.append((theta, ldr_v[4]))
            print("%+6.1f %5s  " % (theta, "%.1f" % us_cm if us_cm else "-")
                  + "  ".join("%6s" % ("%.1f" % (t[0] / 10) if t[0] else "-")
                              for t in tofs)
                  + "  %+.3f  %.3f" % (ldr_v[4], ldr_v[6]))
    finally:
        f.close()
        try:
            _draai_naar(0.0, theta, bias, GROTE_DRAAI_CM_S)
        except Exception as e:
            print("! terugdraaien naar 0 mislukt:", e)

    nullen = _ldr_nul(diffs)
    if nullen:
        print("LDR-nul (ldr_diff = 0) bij theta = %s graden"
              % ", ".join("%+.1f" % t for t in nullen))
    else:
        print("LDR-nul: geen tekenwissel binnen het bereik")
    print("CSV bijgewerkt: %s" % CSV_PAD)
    return nullen


def lr_meting(afstand_cm, alfa_deg, herhaal=3):
    """Stilstaand meten (GEEN rotatie): alle ROI's incl. links/rechts,
    ultrasoon en LDR, `herhaal` keer, naar CSV_LR_PAD. Het verschil
    rechts - links is de maat voor de schuinte van het voorvlak."""
    _init()
    _sc.laser_off()
    time.sleep_ms(SETTLE_MS)
    roi_alle = ROIS + ROIS_LR
    run_id = "%d_%d_%+d" % (time.ticks_ms() // 1000, afstand_cm, alfa_deg)
    try:
        with open(CSV_LR_PAD) as f:
            nieuw = not f.readline()
    except OSError:
        nieuw = True
    with open(CSV_LR_PAD, "a") as f:
        if nieuw:
            k = ["run", "afstand_cm", "alfa_deg", "us_cm"]
            for r in roi_alle:
                k += ["tof_%s_mm" % r[0], "tof_%s_n" % r[0], "tof_%s_sig" % r[0]]
            k += ["ldr_diff", "Q"]
            f.write(",".join(k) + "\n")
        print("\n=== lr_meting %s: %d cm, alfa %+d ===" % (run_id, afstand_cm, alfa_deg))
        print("   US  " + "  ".join("%6s" % r[0] for r in roi_alle) + "   R-L    diff")
        for _ in range(herhaal):
            ldr_v = meet_ldr(afstand_cm + US_TOT_LDR_CM)
            us_cm, _n = meet_us()
            tofs = [meet_tof(r) for r in roi_alle]
            rij = [run_id, afstand_cm, alfa_deg, us_cm]
            for mm, n, sig, _amb, _st in tofs:
                rij += [mm, n, sig]
            rij += [ldr_v[4], ldr_v[6]]
            f.write(",".join(_fmt(v) for v in rij) + "\n")
            l, r = tofs[-2][0], tofs[-1][0]
            print("%5s  " % ("%.1f" % us_cm if us_cm else "-")
                  + "  ".join("%6s" % ("%.1f" % (t[0] / 10) if t[0] else "-")
                              for t in tofs)
                  + "  %5s  %+.3f" % ("%+.1f" % ((r - l) / 10) if l and r else "-",
                                      ldr_v[4]))


def reeks(afstanden=AFSTANDEN_CM, alfas=ALFAS_GRAD):
    """Het volledige meetprogramma: per alfa alle afstanden (ver -> dichtbij,
    dus de kar alleen langs dezelfde straal van de boog naar binnen
    schuiven). Per positie: 's' slaat over, 'q' stopt de reeks."""
    _init()
    samenvatting = []
    for alfa in alfas:
        for d in afstanden:
            r = sweep(d, alfa)
            if r == "q":
                print("Reeks gestopt.")
                _print_samenvatting(samenvatting)
                return samenvatting
            if r == "s":
                continue
            samenvatting.append((d, alfa, r))
    _print_samenvatting(samenvatting)
    return samenvatting


def _print_samenvatting(s):
    print("\n=== Samenvatting LDR-nul ===")
    print(" afstand  alfa   theta(ldr_diff=0)")
    for d, alfa, nullen in s:
        print("  %3d cm  %+4d   %s" % (d, alfa, ", ".join(
            "%+.1f" % t for t in nullen) if nullen else "-"))
    print("Referentie 0,5 m (2026-10-19): ~2-3 graden bij alfa 30,"
          " ~7 graden bij alfa 50.")


def roi_test(seconden=30):
    """Live ToF-afstand per ROI, om te bepalen welke kleine ROI fysiek
    omhoog kijkt. Zet de kar ~30 cm voor een vlakke wand en houd een kaart
    BOVEN de denkbeeldige middellijn van de ToF in beeld (op ~15 cm): de
    ROI die dan het eerst korter wordt, kijkt omhoog."""
    _init()
    t0 = time.ticks_ms()
    print("  " + "  ".join("%6s" % r[0] for r in ROIS) + "   (cm)")
    while time.ticks_diff(time.ticks_ms(), t0) < seconden * 1000:
        tofs = [meet_tof(r) for r in ROIS]
        print("  " + "  ".join("%6s" % ("%.1f" % (t[0] / 10) if t[0] else "-")
                               for t in tofs))


def help():
    print("""
test_lichtbak_scan -- zie de kop van het bestand voor de opzet.

  roi_test()             welke kleine ROI kijkt omhoog (eenmalig)
  sweep(afstand, alfa)   één positie: draai -%.0f..+%.0f graden, log naar CSV
  reeks()                alle posities: alfa %s, afstand %s cm
  richt(afstand, alfa)   alleen laser aan + plaatsingsinstructie

CSV: %s  (ophalen: mpremote cp :%s .)
""" % (BEREIK_GRAD, BEREIK_GRAD, ALFAS_GRAD, AFSTANDEN_CM, CSV_PAD, CSV_PAD))


print("test_lichtbak_scan geladen. Typ help().")
