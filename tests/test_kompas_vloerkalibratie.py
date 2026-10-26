import machine
import time
import math
from array import array
from icm20948 import ICM20948, load_mag_calibration
import stepper_ramp as sr
from testscherm import led, toon, aftellen, GROEN, ROOD, ORANJE, UIT

# --- Testscript: automatische X/Y-kalibratie van het kompas OP DE VLOER ---
# De kar draait zelf langzaam ~370 graden (gemeten met de gyro) op de plaats; niemand raakt hem aan
# (geen horloge/ring in de buurt van de sensor, zoals bij de handkalibratie).
# Door de gemeten (mx, my) in het KAR-assenstelsel wordt een cirkel gepast
# (Kasa-fit, kleinste kwadraten). Het middelpunt is de REST-offset bovenop de
# huidige kalibratie uit config.json; de straal is het horizontale veld Bh.
# De Z-offset en de schaalfactoren uit de handkalibratie blijven staan.
#
#   ORANJE  handen weg, de kar gaat zo draaien   (PAUZE_S)
#   GROEN   kar draait, meten                    (tot de draai klaar is)
#   ROOD    resultaat op de OLED                 (blijft staan)
#
# Uitvoer: nieuwe "mag_offset" voor config.json (sensor-assenstelsel).

PAUZE_S = 15
DOEL_GRADEN = 370.0     # werkelijk gedraaid volgens de GYRO; de omrekening
                        # stappen -> graden hangt af van de ondergrond
                        # (rubber ~0,93), dus de stappen alleen zijn niet genoeg
SNELHEID_CM_S = 3.0     # wielsnelheid; ~13 s voor een rondje
INTERVAL_MS = 20
N_MAX = 2000

kal = load_mag_calibration()
sensor = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000), **kal)

xs = array('f', [0.0] * N_MAX)
ys = array('f', [0.0] * N_MAX)
n = 0


def los3(a, b):
    """Los het 3x3-stelsel a * x = b op (Gauss met pivoteren)."""
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for k in range(3):
        p = max(range(k, 3), key=lambda r: abs(m[r][k]))
        m[k], m[p] = m[p], m[k]
        for r in range(k + 1, 3):
            f = m[r][k] / m[k][k]
            for c in range(k, 4):
                m[r][c] -= f * m[k][c]
    x = [0.0, 0.0, 0.0]
    for k in (2, 1, 0):
        x[k] = (m[k][3] - sum(m[k][c] * x[c] for c in range(k + 1, 3))) / m[k][k]
    return x


gyro = sr.gyro_z_deg_s(sensor)   # graden/s, positief = rechts
yaw = 0.0

try:
    sr.disable()
    aftellen(PAUZE_S, ORANJE, "HANDEN", ("weg van de kar,", "hij gaat draaien"))
    # Gyro-bias meten terwijl de kar stilstaat
    som = 0.0
    for _ in range(100):
        som += gyro()
        time.sleep_ms(3)
    bias = som / 100

    led(GROEN)
    toon("DRAAIEN", ("niet aanraken", ""))
    sr.rotate_deg(DOEL_GRADEN * 1.5, SNELHEID_CM_S)   # ruim; we remmen op de gyro
    tp = time.ticks_us()
    while sr.busy() and n < N_MAX:
        mx, my, _mz = sensor.magnetic
        xs[n] = mx; ys[n] = my; n += 1
        nu = time.ticks_us()
        yaw += (gyro() - bias) * time.ticks_diff(nu, tp) * 1e-6
        tp = nu
        if yaw >= DOEL_GRADEN:
            sr.brake()          # nette afremramp; meten loopt door tot stilstand
            break
        time.sleep_ms(INTERVAL_MS)
    while sr.busy() and n < N_MAX:
        mx, my, _mz = sensor.magnetic
        xs[n] = mx; ys[n] = my; n += 1
        nu = time.ticks_us()
        yaw += (gyro() - bias) * time.ticks_diff(nu, tp) * 1e-6
        tp = nu
        time.sleep_ms(INTERVAL_MS)
finally:
    sr.halt()
    sr.disable()

# Kasa-fit: x^2 + y^2 + D x + E y + F = 0, middelpunt (-D/2, -E/2)
sxx = sxy = syy = sx = sy = 0.0
sxz = syz = sz = 0.0
for k in range(n):
    x = xs[k]; y = ys[k]; z = x * x + y * y
    sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y
    sxz += x * z; syz += y * z; sz += z
D, E, F = los3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]], [-sxz, -syz, -sz])
cx, cy = -D / 2, -E / 2
r = math.sqrt(max(0.0, cx * cx + cy * cy - F))

# Kwaliteit: afwijking van de cirkel (RMS), omgerekend naar koersfout
s2 = 0.0
for k in range(n):
    d = math.sqrt((xs[k] - cx) ** 2 + (ys[k] - cy) ** 2) - r
    s2 += d * d
rms = math.sqrt(s2 / n)
koersfout = math.degrees(rms / r) if r else 0.0

# Rest-offset (kar) terug naar het sensor-assenstelsel: kar = mount * sensor,
# en de offset wordt VOOR de schaal afgetrokken.
mount, off, sc = kal["mount"], kal["mag_offset"], kal["mag_scale"]
nieuw = [off[0] + mount[0] * cx / sc[0], off[1] + mount[1] * cy / sc[1], off[2]]

led(ROOD)
toon("dX%+.1f" % cx, ("dY%+.1f Bh %.1f" % (cy, r), "gedraaid %.0f gr" % yaw), "~%.1fgr" % koersfout)
print("%d metingen, gedraaid %.0f graden (gyro)" % (n, yaw))
if yaw < 360:
    print("LET OP: minder dan een volle cirkel -- middelpunt minder betrouwbaar")
print("rest-offset (kar): dX %+.2f  dY %+.2f uT, Bh %.2f uT" % (cx, cy, r))
print("afwijking van de cirkel: %.2f uT RMS, ~%.1f gr koersfout" % (rms, koersfout))
print("oude mag_offset: [%.2f, %.2f, %.2f]" % tuple(off))
print('nieuwe "mag_offset": [%.2f, %.2f, %.2f],' % tuple(nieuw))
time.sleep(2)
led(UIT)
