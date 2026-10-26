import machine
import time
import math
from array import array
from icm20948 import ICM20948
import stepper_ramp as sr
from testscherm import led, toon, aftellen, GROEN, ROOD, ORANJE, UIT

# --- Testscript: begeleide magnetometer-kalibratie (handmatig draaien) ---
# ICM-20948:        GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
# OLED + RGB-LED:   zie lib/display/testscherm.py
#
# NIET op of bij het bureau kalibreren (ijzeren balk eronder): op de vloer,
# vrij van metaal. Motordrivers staan UIT. Servoarm in rustpositie.
#
# Vier stappen, elk ORANJE (lees wat komt) -> GROEN (draaien) -> ROOD
# (bereik per as tot nu toe). Draai rustig: een rondje in ~10 s.
#   1  Z-as:   kar plat op de vloer, 2x rond om de verticale as
#   2  X-as:   neus naar OOST of WEST, kar 1x over de zijkant rollen
#   3  Y-as:   neus naar NOORD of ZUID, kar 1x voorover rollen
# (richting met het kompas van een telefoon). Waarom: min/max per as werkt
# alleen als elke as een keer MET en een keer TEGEN het aardveld in wijst. Het
# veld helt hier 66 graden naar beneden, richting noord. Rollen om de lengteas
# met de neus naar O/W draait Y en Z door het volle veld; voorover rollen met
# de neus naar N/Z doet dat voor X en Z. Plat ronddraaien (stap 1) haalt
# alleen de horizontale ~20 uT.
#   4  achtje: vrije 8-beweging in alle richtingen
# Het bereik per as moet uitkomen op ~2x het aardveld (~100 uT bij 50 uT).
#
# Hard-iron offset en soft-iron scale via min/max per as, plus een
# kwaliteitsmaat: de spreiding van |B| over alle metingen NA kalibratie.

PAUZE_S = 20
STAP_S = 25
RESULTAAT_S = 10
INTERVAL_MS = 50        # 20 Hz; de AK09916 meet op 100 Hz

STAPPEN = (
    ("Z-as", ("plat op vloer,", "2x rond draaien")),
    ("X-as", ("neus naar O of W", "rol over zijkant")),
    ("Y-as", ("neus naar N of Z", "voorover rollen")),
    ("ACHTJE", ("8-beweging, alle", "richtingen")),
)

sensor = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000))
mag = sensor.ak09916            # ongekalibreerd: offset (0,0,0), scale (1,1,1)

N_MAX = len(STAPPEN) * STAP_S * 1000 // INTERVAL_MS + 10
xs = array('f', [0.0] * N_MAX)
ys = array('f', [0.0] * N_MAX)
zs = array('f', [0.0] * N_MAX)
n = 0
lo = [1e9, 1e9, 1e9]
hi = [-1e9, -1e9, -1e9]


def bereik():
    return [hi[i] - lo[i] if hi[i] > lo[i] else 0.0 for i in range(3)]


try:
    sr.disable()
    for nr, (naam, uitleg) in enumerate(STAPPEN, 1):
        aftellen(PAUZE_S, ORANJE, "%d %s" % (nr, naam), uitleg)

        led(GROEN)
        t0 = time.ticks_ms()
        vorig = None
        while time.ticks_diff(time.ticks_ms(), t0) < STAP_S * 1000 and n < N_MAX:
            m = mag.magnetic
            xs[n], ys[n], zs[n] = m
            n += 1
            for i in range(3):
                if m[i] < lo[i]: lo[i] = m[i]
                if m[i] > hi[i]: hi[i] = m[i]
            over = STAP_S - time.ticks_diff(time.ticks_ms(), t0) // 1000
            if over != vorig:       # OLED-push kost ~25 ms: eens per seconde
                vorig = over
                toon(naam, uitleg, "%ds" % over)
            time.sleep_ms(INTERVAL_MS)

        b = bereik()
        print("stap %d (%s): bereik X %.1f  Y %.1f  Z %.1f uT" % (nr, naam, b[0], b[1], b[2]))
        led(ROOD)
        toon("%d KLAAR" % nr, ("bereik uT:", "X%.0f Y%.0f Z%.0f" % tuple(b)))
        time.sleep(RESULTAAT_S)
except KeyboardInterrupt:
    print("Onderbroken; resultaat met de tot nu toe verzamelde metingen.")
finally:
    sr.disable()
    led(UIT)

if n < 10:
    raise SystemExit("te weinig metingen")

# Hard-iron offsets (het verschoven middelpunt)
offset = [(hi[i] + lo[i]) / 2 for i in range(3)]
# Soft-iron schaling (de vervorming van de bol)
chord = [(hi[i] - lo[i]) / 2 or 1e-6 for i in range(3)]  # voorkom deling door nul
straal = sum(chord) / 3
scale = [straal / c for c in chord]

# Kwaliteit: |B| na kalibratie hoort overal gelijk te zijn
s = s2 = 0.0
for k in range(n):
    bx = (xs[k] - offset[0]) * scale[0]
    by = (ys[k] - offset[1]) * scale[1]
    bz = (zs[k] - offset[2]) * scale[2]
    v = math.sqrt(bx * bx + by * by + bz * bz)
    s += v; s2 += v * v
gem = s / n
sd_pct = 100.0 * math.sqrt(max(0.0, s2 / n - gem * gem)) / gem

toon("|B| %.0f" % gem, ("uT, spreiding:", "%d metingen" % n), "%.1f%%" % sd_pct)

print("\n--- KALIBRATIE VOLTOOID ---")
print("%d metingen; |B| na kalibratie %.1f uT, spreiding (1 sigma) %.1f %%" % (n, gem, sd_pct))
print("Zet deze waarden in config.json, sectie \"icm20948\":")
print('"mag_offset": [%.2f, %.2f, %.2f],' % tuple(offset))
print('"mag_scale": [%.4f, %.4f, %.4f],' % tuple(scale))
print("---------------------------")
