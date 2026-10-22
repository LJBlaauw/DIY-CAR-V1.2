import machine
import time
import math
from icm20948 import ICM20948
from ssd1306 import SSD1306_I2C
import stepper_ramp as sr

# --- Testscript: magnetische storing door STROOM (motoren aan vs. uit) ---
# Til de kar op zodat de wielen VRIJ hangen. Drie fasen van 15 s, aangegeven
# op de OLED; leg desgewenst een smartphone met kompas-/magnetometer-app op de
# kandidaat-sensorplek en noteer per fase de veldsterkte:
#   A  drivers UIT              (referentie, geen motorstroom)
#   B  drivers AAN, stilstand   (houdstroom door de spoelen)
#   C  motoren draaien          (wielen vrij, 5 cm/s)
# De ICM-20948 meet op zijn eigen plek mee (ruw, ongekalibreerd); het VERSCHIL
# tussen de fasen is de storing door stroom. Een paar uT is acceptabel; meer
# betekent: voedingsdraden (plus en min samen) in elkaar draaien, te beginnen
# met het stuk dat het dichtst langs de sensor loopt.

FASE_S = 15

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)
imu = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000))
mag = imu.ak09916


def toon(*regels):
    oled.fill(0)
    for i, t in enumerate(regels):
        oled.text(t, 0, i * 12)
    oled.show()


def meet_fase(naam, uitleg, actie=None):
    sx = sy = sz = 0.0
    n = 0
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < FASE_S * 1000:
        if actie is not None and not sr.busy():
            actie()
        x, y, z = mag.magnetic
        sx += x; sy += y; sz += z; n += 1
        over = FASE_S - time.ticks_diff(time.ticks_ms(), t0) // 1000
        toon("FASE " + naam, uitleg, "nog %ds" % over,
             "|B| %.1f uT" % math.sqrt(x * x + y * y + z * z))
        time.sleep_ms(50)
    gem = (sx / n, sy / n, sz / n)
    print("fase %s (%s): X %.2f  Y %.2f  Z %.2f  |B| %.2f uT" % (
        naam, uitleg, gem[0], gem[1], gem[2], math.sqrt(sum(v * v for v in gem))))
    return gem


try:
    for k in range(10, 0, -1):
        toon("STROOMSTORING", "wielen VRIJ?", "start over %ds" % k)
        time.sleep(1)

    sr.disable()
    a = meet_fase("A", "drivers UIT")

    sr.enable()
    b = meet_fase("B", "houdstroom")

    c = meet_fase("C", "motoren draaien",
                  actie=lambda: sr.rotate_deg(90.0, 5.0))
finally:
    sr.halt()
    sr.disable()

for naam, f in (("B", b), ("C", c)):
    d = [f[i] - a[i] for i in range(3)]
    print("storing %s t.o.v. A: dX %+.2f  dY %+.2f  dZ %+.2f  |d| %.2f uT" % (
        naam, d[0], d[1], d[2], math.sqrt(sum(v * v for v in d))))
toon("KLAAR", "zie uitvoer", "drivers uit")
