import machine
import time
import math
from icm20948 import ICM20948, load_mag_calibration
from testscherm import led, toon, aftellen, GROEN, ROOD, ORANJE, UIT

# --- Testscript: kompaskoers in vier richtingen (papieren kompasroos) ---
# Kar plat op de vloer op een kompastekening, uit de buurt van metaal.
# Start met de neus EXACT naar het zuiden. Per richting:
#   ORANJE  draai de kar rechtsom naar de richting op de OLED  (PAUZE_S)
#   GROEN   stilhouden, meten                                  (MEET_S)
#   ROOD    gemeten koers en afwijking op de OLED              (RESULTAAT_S)
# De laatste stap is weer zuid: het verschil met de eerste meting laat zien
# of de koers na een volle draai terugkomt (hysterese / verplaatsing).
# Gebruikt kalibratie + "mount" uit config.json: koers rechtsom vanaf noord.

PAUZE_S = 15
MEET_S = 10
RESULTAAT_S = 8

RICHTINGEN = (("ZUID", 180), ("WEST", 270), ("NOORD", 0), ("OOST", 90), ("ZUID", 180))

sensor = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000),
                  **load_mag_calibration())


def hoekverschil(a, b):
    """a - b in graden, in het bereik -180..180."""
    return (a - b + 540) % 360 - 180


def meet(seconden):
    # Middelen als vector (mx, my): geen sprong rond 0/360 graden
    sx = sy = 0.0
    n = 0
    t0 = time.ticks_ms()
    vorig = None
    while time.ticks_diff(time.ticks_ms(), t0) < seconden * 1000:
        mx, my, _mz = sensor.magnetic
        sx += mx; sy += my; n += 1
        over = seconden - time.ticks_diff(time.ticks_ms(), t0) // 1000
        if over != vorig:           # OLED-push kost ~25 ms: eens per seconde
            vorig = over
            toon("METEN", ("stilhouden", ""), "%ds" % over)
        time.sleep_ms(20)
    return math.degrees(math.atan2(sy, sx)) % 360, math.sqrt(sx * sx + sy * sy) / n


uitslag = []
try:
    for nr, (naam, doel) in enumerate(RICHTINGEN):
        if nr == 0:
            aftellen(PAUZE_S, ORANJE, naam, ("neus naar", "%s (%d gr)" % (naam.lower(), doel)))
        else:
            aftellen(PAUZE_S, ORANJE, naam, ("draai 90 rechtsom", "naar %s" % naam.lower()))
        led(GROEN)
        koers, bh = meet(MEET_S)
        fout = hoekverschil(koers, doel)
        uitslag.append((naam, doel, koers, fout, bh))
        print("%-5s doel %3d  koers %5.1f  afwijking %+5.1f  Bh %.1f uT" % (naam, doel, koers, fout, bh))
        led(ROOD)
        toon("%s %.0f" % (naam[0], koers), ("afwijking", "Bh %.1f uT" % bh), "%+.0f gr" % fout)
        time.sleep(RESULTAAT_S)
finally:
    led(UIT)

if len(uitslag) == len(RICHTINGEN):
    fouten = [u[3] for u in uitslag[:4]]
    gem = sum(fouten) / 4
    spreid = max(fouten) - min(fouten)
    terug = hoekverschil(uitslag[4][2], uitslag[0][2])
    print("\nvaste verschuiving (gem. afwijking): %+.1f gr" % gem)
    print("spreiding van de afwijking over 4 richtingen: %.1f gr" % spreid)
    print("zuid aan het eind t.o.v. het begin: %+.1f gr" % terug)
    toon("KLAAR", ("gem afwijking", "spreiding %.0f gr" % spreid), "%+.0f gr" % gem)
