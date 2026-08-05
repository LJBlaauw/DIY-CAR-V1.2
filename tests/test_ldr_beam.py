# ================================================================
# tests/test_ldr_beam.py
#
# Meetscript voor de LDR-karakteristiek, het bundelprofiel van de lichtbron,
# en daarmee de laterale afwijking y van de kar t.o.v. de bundelas.
#
# Drie metingen, los aan te roepen vanuit de REPL:
#
#   gamma()    - bepaalt de LDR-exponent gamma. Bankmeting: bron op de as,
#                een reeks afstanden. R ~ E^-gamma, dus ln R = 2*gamma*ln d + c.
#   bundel()   - bepaalt het bundelprofiel I(phi). Bankmeting: LDR op een boog
#                met VASTE radius rond de bron, steeds NAAR de bron gericht.
#                Zo staan tunnelhoek en afstand vast en meet je puur de bundel.
#   meet_y()   - op de kar: rijd een recht stuk en bereken uit de daling van de
#                genormaliseerde helderheid hoe ver je naast de bundelas zit.
#
# WAAROM DIT WERKT
# ----------------
# Ontvangen licht E = I(phi) / d^2, met phi = atan(y/d) de hoek waaronder de
# bron je ziet. Normaliseer de afstand eruit:
#
#     Q = -(1/gamma)*ln(R) + 2*ln(d)      (= ln I(phi) op een constante na)
#
# Q blijft CONSTANT als je op de bundelas zit (phi = 0 op elke afstand), en
# DAALT als je ernaast zit, want phi groeit terwijl je nadert. Uit de daling
# over een rechte rit volgt y. Geen zijstappen nodig voor de grootte; alleen
# het teken kost nog een dither.
#
# LET OP - eenmalig te controleren:
#   Na het verlagen van R29/R30 naar 1 kOhm MOET LDR_R_FIXED_OHM in
#   lib/LDR/ldr_scan_isr.py ook op 1000 staan, anders zijn alle
#   weerstandswaarden een factor 10 fout. Dit script controleert dat.
# ================================================================

import math
import time
import sys
from machine import ADC, Pin

sys.path.append("/lib/LDR")
sys.path.append("/lib/stepper")
sys.path.append("/lib/ultrasoon")

# ----------------------------------------------------------------
# Configuratie
# ----------------------------------------------------------------
LDR_PIN_A = 26
LDR_PIN_B = 27

R_FIXED_VERWACHT = 1000.0     # pull-up naar 3V3, na de hardwarewijziging
ADC_MAX = 65535.0

N_SAMPLES = 32                # gemiddelde per meting; onderdrukt ADC-ruis
CSV_DIR = "/"

# Startwaarden; worden overschreven door gamma() en bundel()
GAMMA = 0.7                   # LDR-exponent, R ~ E^-GAMMA
BEAM_W_DEG = 33.0             # 1/e-halfhoek van de bundel in graden

# Aannames voor de foutschatting in meet_y()
US_FOUT_CM = 0.3              # nauwkeurigheid ultrasoon
DRIFT_Q = 0.01                # drift in Q over de duur van één meetbeen

_adc_a = ADC(Pin(LDR_PIN_A, Pin.IN))
_adc_b = ADC(Pin(LDR_PIN_B, Pin.IN))

_R_FIXED = R_FIXED_VERWACHT
try:
    import ldr_scan_isr
    _R_FIXED = float(ldr_scan_isr.LDR_R_FIXED_OHM)
except Exception:
    print("! ldr_scan_isr niet importeerbaar; val terug op R_FIXED =",
          R_FIXED_VERWACHT)


def controleer_config():
    """Waarschuw als de code niet bij de gewijzigde hardware past."""
    ok = True
    if abs(_R_FIXED - R_FIXED_VERWACHT) > 1.0:
        print("! LDR_R_FIXED_OHM = %.0f, verwacht %.0f." % (_R_FIXED,
                                                            R_FIXED_VERWACHT))
        print("  Pas lib/LDR/ldr_scan_isr.py aan, anders is elke weerstand")
        print("  een factor %.1f fout." % (_R_FIXED / R_FIXED_VERWACHT))
        ok = False
    try:
        if ldr_scan_isr.LDR_R_MIN_OHM > 20.0:
            print("! LDR_R_MIN_OHM = %.0f klemt de procentschaal dichtbij de"
                  % ldr_scan_isr.LDR_R_MIN_OHM)
            print("  bron vast op 100%%. Zet hem op ~10 voor de eindfase.")
            ok = False
    except Exception:
        pass
    if ok:
        print("Configuratie OK. R_FIXED = %.0f ohm (pull-up naar 3V3)."
              % _R_FIXED)
    return ok


# ----------------------------------------------------------------
# Basismeting
# ----------------------------------------------------------------
def _raw(adc, n=N_SAMPLES):
    acc = 0
    for _ in range(n):
        acc += adc.read_u16()
    return acc / n


def _res(adc, n=N_SAMPLES):
    """LDR-weerstand in ohm. Topologie: R_FIXED naar 3V3, LDR naar GND."""
    a = _raw(adc, n)
    if a < 1.0:
        a = 1.0
    elif a > ADC_MAX - 1.0:
        a = ADC_MAX - 1.0
    return _R_FIXED * a / (ADC_MAX - a)


def lees(n=N_SAMPLES):
    """Geef (R_A, R_B, ruwe_A, ruwe_B)."""
    ra = _res(_adc_a, n)
    rb = _res(_adc_b, n)
    return ra, rb, _raw(_adc_a, 4), _raw(_adc_b, 4)


def _ln_E(ra, rb, gamma=None):
    """ln van de gecombineerde lichtsterkte van beide LDR's, op een constante na.

    E ~ R^(-1/gamma), dus de SOM van de lichtsterktes is
    R_A^(-1/g) + R_B^(-1/g). Niet de weerstanden optellen: die schaal is
    logaritmisch en niet-lineair in licht.
    """
    g = GAMMA if gamma is None else gamma
    e = ra ** (-1.0 / g) + rb ** (-1.0 / g)
    return math.log(e)


def Q(d_cm, ra=None, rb=None, gamma=None):
    """Genormaliseerde helderheid: ln I(phi) op een constante na.

    Constant over de afstand als je op de bundelas zit; dalend als je ernaast
    zit. Dit is de grootheid waar meet_y() op rekent.
    """
    if ra is None:
        ra, rb, _, _ = lees()
    return _ln_E(ra, rb, gamma) + 2.0 * math.log(d_cm)


def meting(label=""):
    """Print één momentopname; handig om los te controleren."""
    ra, rb, xa, xb = lees()
    print("%-12s R_A=%8.1f  R_B=%8.1f  adc=%6.0f/%6.0f  A-B=%+.3f"
          % (label, ra, rb, xa, xb, math.log(rb / ra)))
    return ra, rb


# ----------------------------------------------------------------
# 1. GAMMA — LDR-exponent
# ----------------------------------------------------------------
def gamma(afstanden=(15, 20, 30, 40, 60), csv=None):
    """Bepaal de LDR-exponent gamma uit een reeks afstanden OP DE BUNDELAS.

    Op de as is I(phi) constant, dus E ~ 1/d^2 en geldt
        ln R = 2*gamma*ln d + c
    De helling van die rechte is 2*gamma. Meerdere punten in plaats van twee,
    zodat je ook ziet OF de kwadratenwet opgaat: een spleetbron is dichtbij
    geen puntbron, en dan buigt de lijn af.

    Richt de LDR steeds recht op de bron en houd hem exact op de as.
    """
    controleer_config()
    print("\n=== GAMMA-meting ===")
    print("Houd de LDR steeds RECHT op de bron en EXACT op de bundelas.")
    xs, ys, rijen = [], [], []
    for d in afstanden:
        input("  Zet de bron op %d cm en druk op Enter... " % d)
        ra, rb, xa, xb = lees()
        r = math.sqrt(ra * rb)          # geometrisch gemiddelde van beide LDR's
        xs.append(math.log(d))
        ys.append(math.log(r))
        rijen.append((d, ra, rb, r, xa, xb))
        print("    d=%3d cm  R_A=%8.1f  R_B=%8.1f  R_gem=%8.1f"
              % (d, ra, rb, r))
        if xa > ADC_MAX * 0.97 or xb > ADC_MAX * 0.97:
            print("    ! ADC bijna vol -> te donker, of R_FIXED te hoog")
        if xa < ADC_MAX * 0.03 or xb < ADC_MAX * 0.03:
            print("    ! ADC bijna nul -> mogelijk LDR-verzadiging; dimp de bron")

    helling, offset, r2 = _fit(xs, ys)
    g = helling / 2.0
    print("\n  helling ln R vs ln d = %.4f   ->  gamma = %.4f" % (helling, g))
    print("  R^2 = %.5f %s" % (r2, "(goed)" if r2 > 0.99 else
                               "(LET OP: kwadratenwet gaat niet op)"))
    if not (0.3 < g < 1.2):
        print("  ! gamma buiten het gebruikelijke bereik 0,4-1,0 voor CdS.")
    print("\n  Zet in dit script:  GAMMA = %.3f" % g)
    if csv:
        _csv(csv, "d_cm,R_A,R_B,R_gem,adc_A,adc_B", rijen)
    return g


# ----------------------------------------------------------------
# 2. BUNDEL — profiel I(phi)
# ----------------------------------------------------------------
def bundel(radius_cm=40, hoeken=(0, 5, 10, 15, 20, 25, 30, 40, 50), csv=None):
    """Bepaal het bundelprofiel I(phi) op een boog met VASTE radius.

    Houd de LDR steeds NAAR de bron gericht en op een vaste afstand. Dan zijn
    de tunnelhoek en de afstand constant, en meet je uitsluitend hoeveel licht
    de bron in die richting uitstraalt.

    Fit daarna een gaussisch profiel  I(phi)/I(0) = exp(-(phi/w)^2)  en geef de
    1/e-halfhoek w. Die w zit in elke y-berekening.

    Tip: zet een touwtje van radius_cm aan de bron en markeer de hoeken op de
    vloer; dat is nauwkeurig genoeg en veel sneller dan meten per punt.
    """
    controleer_config()
    print("\n=== BUNDELPROFIEL ===")
    print("Radius %d cm. LDR steeds NAAR de bron gericht." % radius_cm)
    ln_e0 = None
    rijen, ws = [], []
    for phi in hoeken:
        input("  Zet de LDR op %d graden uit de as en druk op Enter... " % phi)
        ra, rb, xa, xb = lees()
        ln_e = _ln_E(ra, rb)
        if ln_e0 is None:
            ln_e0 = ln_e
        rel = math.exp(ln_e - ln_e0)     # I(phi)/I(0)
        w = None
        if phi > 0 and 0.0 < rel < 1.0:
            w = phi / math.sqrt(-math.log(rel))
            ws.append(w)
        rijen.append((phi, ra, rb, rel, w if w else 0.0))
        print("    phi=%3d  R_A=%8.1f  R_B=%8.1f  I/I0=%.4f%s"
              % (phi, ra, rb, rel,
                 "  w=%.1f graden" % w if w else ""))

    if ws:
        w_gem = sum(ws) / len(ws)
        spreiding = max(ws) - min(ws)
        print("\n  1/e-halfhoek w = %.1f graden  (spreiding %.1f)"
              % (w_gem, spreiding))
        print("  halfwaarde-halfhoek = %.1f graden" % (w_gem * 0.8326))
        if spreiding > 0.3 * w_gem:
            print("  ! grote spreiding -> het profiel is niet gaussisch;")
            print("    gebruik dan de tabel i.p.v. de fit.")
        print("\n  Zet in dit script:  BEAM_W_DEG = %.1f" % w_gem)
    else:
        print("\n  ! geen bruikbare punten; staat de bron wel aan?")
        w_gem = None
    if csv:
        _csv(csv, "phi_deg,R_A,R_B,I_rel,w_deg", rijen)
    return w_gem


# ----------------------------------------------------------------
# 3. MEET_Y — laterale afwijking van de bundelas
# ----------------------------------------------------------------
def _y_uit_dQ(dQ, d1, d2, w_rad):
    """Los y op uit  dQ = (atan(y/d2)/w)^2 - (atan(y/d1)/w)^2.

    Monotoon stijgend in y, dus bisectie. Exact, dus ook geldig bij grote
    hoeken waar de kleine-hoekbenadering y ~ w*sqrt(dQ/(1/d2^2-1/d1^2)) afwijkt.
    """
    if dQ <= 0.0:
        return 0.0

    def f(y):
        a2 = math.atan(y / d2) / w_rad
        a1 = math.atan(y / d1) / w_rad
        return a2 * a2 - a1 * a1

    lo, hi = 0.0, 200.0
    if f(hi) < dQ:
        return hi
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if f(mid) < dQ:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def y_uit_twee_punten(d1, Q1, d2, Q2, w_deg=None):
    """Bereken y uit twee (afstand, Q)-metingen. d2 < d1."""
    w = math.radians(BEAM_W_DEG if w_deg is None else w_deg)
    dQ = Q1 - Q2
    y = _y_uit_dQ(dQ, d1, d2, w)

    # Foutschatting: de ultrasoon domineert. Gebruik je de odometer voor de
    # BEENLENGTE en de ultrasoon alleen voor de absolute afstand, dan werkt
    # de afstandsfout maar één keer door i.p.v. twee keer.
    sig = 2.0 * US_FOUT_CM * abs(1.0 / d1 - 1.0 / d2) + DRIFT_Q
    y_hi = _y_uit_dQ(dQ + sig, d1, d2, w)
    y_lo = _y_uit_dQ(max(0.0, dQ - sig), d1, d2, w)
    return y, y_lo, y_hi, dQ


def meet_y(been_cm=15.0, snelheid=8.0, d_start=None, w_deg=None):
    """Meet op de kar de laterale afwijking van de bundelas.

    VOORWAARDE: de koers moet eerst genulled zijn op A-B (recht naar de bron
    kijken), anders vignetteert de tunnel en meet je die in plaats van de
    bundel. Dit script controleert dat en waarschuwt.

    Rijdt `been_cm` recht vooruit en vergelijkt Q voor en na. De beenlengte komt
    uit de ODOMETER (exact tot 15 um), niet uit de ultrasoon; die wordt alleen
    gebruikt voor de absolute startafstand. Dat scheelt een factor ~3 in de fout.
    """
    import stepper_ramp as sr
    import ultrasoon

    print("\n=== MEET_Y ===")
    ra, rb, _, _ = lees()
    scheef = abs(math.log(rb / ra))
    if scheef > 0.05:
        print("! A en B verschillen %.3f in ln R -> koers niet genulled." % scheef)
        print("  Nul eerst op A-B, anders meet je tunnelvignettering.")

    d1 = ultrasoon.read_cm() if d_start is None else d_start
    if not d1 or d1 <= 0:
        print("! geen geldige ultrasoonmeting")
        return None
    Q1 = Q(d1, ra, rb)
    p0 = sr.MA.travel()
    print("  start: d=%.1f cm  Q=%.4f" % (d1, Q1))

    sr.mov('f', snelheid, been_cm)
    while sr.busy():
        time.sleep_ms(10)
    time.sleep_ms(150)                       # LDR's laten settelen (CdS is traag)

    gereden = sr.steps_to_cm(sr.MA.travel() - p0)
    d2 = d1 - gereden
    ra2, rb2, _, _ = lees()
    Q2 = Q(d2, ra2, rb2)
    print("  eind : d=%.1f cm  Q=%.4f  (odometer: %.2f cm gereden)"
          % (d2, Q2, gereden))
    print("  ultrasoon nu: %.1f cm (controle)" % ultrasoon.read_cm())

    y, y_lo, y_hi, dQ = y_uit_twee_punten(d1, Q1, d2, Q2, w_deg)
    print("\n  dQ = %+.4f" % dQ)
    if dQ <= 0.0:
        print("  Q is niet gedaald -> je zit binnen de ruis op de bundelas.")
    print("  y = %.1f cm   (%.1f .. %.1f cm)" % (y, y_lo, y_hi))
    print("  Teken nog onbekend: doe één dither (arc links/rechts) en kijk")
    print("  welke kant Q verhoogt.")
    return y, y_lo, y_hi


def dither_teken(hoek=8.0, been_cm=6.0, snelheid=8.0):
    """Bepaal aan WELKE kant van de bundelas je zit.

    Maakt een kleine zijstap naar links, meet Q, keert terug, en doet hetzelfde
    naar rechts. De kant met de hoogste Q is de kant waar de bundelas ligt.
    Geeft -1 (links), +1 (rechts) of 0 (geen verschil boven de ruis).
    """
    import stepper_ramp as sr
    import ultrasoon

    def _zijstap(teken):
        sr.rotate_deg(teken * hoek, snelheid)
        while sr.busy():
            time.sleep_ms(10)
        sr.mov('f', snelheid, been_cm)
        while sr.busy():
            time.sleep_ms(10)
        sr.rotate_deg(-teken * hoek, snelheid)
        while sr.busy():
            time.sleep_ms(10)
        time.sleep_ms(150)
        return Q(ultrasoon.read_cm())

    print("\n=== DITHER ===")
    q_l = _zijstap(-1)
    print("  links : Q=%.4f" % q_l)
    q_r = _zijstap(+1)                       # netto 2x been_cm naar rechts
    print("  rechts: Q=%.4f" % q_r)
    verschil = q_r - q_l
    if abs(verschil) < DRIFT_Q:
        print("  geen verschil boven de ruis -> vergroot been_cm")
        return 0
    teken = 1 if verschil > 0 else -1
    print("  bundelas ligt %s (dQ=%+.4f)"
          % ("rechts" if teken > 0 else "links", verschil))
    return teken


# ----------------------------------------------------------------
# Verwachte nauwkeurigheid — puur rekenwerk, geen hardware
# ----------------------------------------------------------------
def nauwkeurigheid(benen=((60, 45), (45, 30), (30, 20), (20, 14)),
                   w_deg=None):
    """Print de haalbare y-resolutie per meetbeen. Geen hardware nodig.

    Toont ook wat het halveren van de spleet voor de lichtbron oplevert.
    """
    w0 = BEAM_W_DEG if w_deg is None else w_deg
    print("\n=== HAALBARE y-RESOLUTIE ===")
    print("Aannames: ultrasoon +/-%.1f cm, drift %.3f in Q, beenlengte uit"
          % (US_FOUT_CM, DRIFT_Q))
    print("de odometer (dus de afstandsfout werkt maar 1x door).\n")
    print("  been (cm)   w=%.0f graden   w=%.0f graden (spleet gehalveerd)"
          % (w0, w0 / 2))
    for d1, d2 in benen:
        sig = 2.0 * US_FOUT_CM * abs(1.0 / d1 - 1.0 / d2) + DRIFT_Q
        a = _y_uit_dQ(sig, d1, d2, math.radians(w0))
        b = _y_uit_dQ(sig, d1, d2, math.radians(w0 / 2))
        print("  %3d -> %-3d      %5.2f            %5.2f" % (d1, d2, a, b))
    n = len(benen)
    print("\n  Bij %d onafhankelijke benen verbetert dit met sqrt(%d) = %.2fx"
          % (n, n, math.sqrt(n)))
    print("  De ultrasoon en de lichtdrift zijn de beperking, niet de LDR")
    print("  of de ADC. Halveren van de spleet wint een factor 2; halveren")
    print("  van de ultrasoonfout maar sqrt(2).")


# ----------------------------------------------------------------
# Hulpfuncties
# ----------------------------------------------------------------
def _fit(xs, ys):
    """Kleinste-kwadraten rechte; geeft (helling, offset, R^2)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    helling = sxy / sxx if sxx else 0.0
    r2 = (sxy * sxy) / (sxx * syy) if sxx and syy else 0.0
    return helling, my - helling * mx, r2


def _csv(naam, kop, rijen):
    pad = naam if naam.startswith("/") else CSV_DIR + naam
    try:
        with open(pad, "w") as f:
            f.write(kop + "\n")
            for r in rijen:
                f.write(",".join("%.4f" % v if isinstance(v, float) else str(v)
                                 for v in r) + "\n")
        print("  CSV geschreven: %s" % pad)
    except Exception as e:
        print("  ! CSV mislukt: %s" % e)


def help():
    print(__doc__ if __doc__ else "")
    print("""
Aanbevolen volgorde:

  1. controleer_config()          hardware en code kloppen bij elkaar
  2. gamma()                      bankmeting, LDR-exponent
  3. bundel()                     bankmeting, 1/e-halfhoek van de bundel
     -> zet GAMMA en BEAM_W_DEG bovenaan dit bestand
  4. nauwkeurigheid()             wat is er haalbaar (geen hardware)
  5. meet_y()                     op de kar, na nullen op A-B
  6. dither_teken()               aan welke kant ligt de bundelas
""")


controleer_config()
print("test_ldr_beam geladen. Typ help() voor de meetvolgorde.")
