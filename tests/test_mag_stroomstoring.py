import machine
import time
import math
from icm20948 import ICM20948
import stepper_ramp as sr
from testscherm import led, groot, toon, aftellen, oled, GROEN, ROOD, ORANJE, UIT

# --- Testscript: magnetische storing door STROOM (motoren aan vs. uit) ---
# Til de kar op zodat de wielen VRIJ hangen. Drie fasen, aangegeven op de
# OLED; alle aanwijzingen en resultaten staan op de OLED en in de uitvoer:
#   A  drivers UIT              (referentie, geen motorstroom)
#   B  drivers AAN, stilstand   (houdstroom door de spoelen)
#   C  motoren draaien          (wielen vrij, 5 cm/s)
# De ICM-20948 meet op zijn eigen plek (ruw, ongekalibreerd); het VERSCHIL
# tussen de fasen is de storing door stroom. Een paar uT is acceptabel; meer
# betekent: voedingsdraden (plus en min samen) in elkaar draaien, te beginnen
# met het stuk dat het dichtst langs de sensor loopt.
#
# Optische synchronisatie met de RGB-LED (zie lib/display/testscherm.py), per fase:
#   ORANJE  pauze: lees op de OLED welke fase komt   (PAUZE_S)
#   GROEN   meting loopt, niets aanraken             (FASE_S)
#   ROOD    meting gestopt, resultaat op de OLED      (RESULTAAT_S)

PAUZE_S = 20
FASE_S = 15
RESULTAAT_S = 10

imu = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000))
mag = imu.ak09916


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def meet_fase(naam, uitleg, actie=None):
    aftellen(PAUZE_S, ORANJE, "VOLGT " + naam, (uitleg, "wacht op GROEN"))

    led(GROEN)
    sx = sy = sz = 0.0
    n = 0
    vorig = None
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < FASE_S * 1000:
        if actie is not None and not sr.busy():
            actie()
        x, y, z = mag.magnetic
        sx += x; sy += y; sz += z; n += 1
        over = FASE_S - time.ticks_diff(time.ticks_ms(), t0) // 1000
        if over != vorig:           # OLED-push kost ~25 ms: eens per seconde
            vorig = over
            toon("%s  %ds" % (naam, over), (uitleg, "METEN"),
                 "%.1fuT" % norm((x, y, z)))
        time.sleep_ms(50)
    if actie is not None:
        sr.halt()                   # motoren stil tijdens het aflezen
    gem = (sx / n, sy / n, sz / n)
    print("fase %s (%s): X %.2f  Y %.2f  Z %.2f  |B| %.2f uT" % (
        naam, uitleg, gem[0], gem[1], gem[2], norm(gem)))
    return gem


def resultaat(naam, gem, ref=None):
    led(ROOD)
    klein = ["gemiddeld |B|"]
    if ref is not None:
        klein.append("storing t.o.v. A:")
        onder = "d%.1fuT" % norm([gem[i] - ref[i] for i in range(3)])
    else:
        klein.append("= referentie")
        onder = ""
    oled.fill(0)
    groot("%s %.1f" % (naam, norm(gem)), 0)
    for i, t in enumerate(klein):
        oled.text(t, 0, 20 + i * 10)
    if onder:
        groot(onder, 44)
    oled.show()
    time.sleep(RESULTAAT_S)


try:
    sr.disable()
    a = meet_fase("A", "drivers UIT")
    resultaat("A", a)

    sr.enable()                     # tijdens de oranje pauze: veld kan zich zetten
    b = meet_fase("B", "houdstroom")
    resultaat("B", b, a)

    c = meet_fase("C", "motoren draaien",
                  actie=lambda: sr.rotate_deg(90.0, 5.0))
    resultaat("C", c, a)
finally:
    sr.halt()
    sr.disable()
    led(UIT)

for naam, f in (("B", b), ("C", c)):
    d = [f[i] - a[i] for i in range(3)]
    print("storing %s t.o.v. A: dX %+.2f  dY %+.2f  dZ %+.2f  |d| %.2f uT" % (
        naam, d[0], d[1], d[2], norm(d)))
toon("KLAAR", ("zie uitvoer", "drivers uit"))
