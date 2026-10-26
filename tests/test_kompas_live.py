import machine
import time
import math
from icm20948 import ICM20948, load_mag_calibration
from testscherm import led, toon, GROEN, UIT

# --- Testscript: live kompaskoers op de OLED (vergelijken met een telefoon) ---
# Gebruikt de kalibratie en "mount" uit config.json, dus de koers van de KAR:
# 0 = neus naar het noorden, 90 = oost (rechtsom, zoals een kompas).
# Kar plat op de vloer, uit de buurt van het bureau. Draai de kar met de hand
# in stappen van 90 graden rechtsom; de koers moet dan steeds ~90 OPLOPEN.
# Bh (horizontaal veld) hoort ~19 uT te zijn; wijkt dat sterk af, dan is het
# veld ter plekke verstoord (metaal in de buurt).

DUUR_S = 120

sensor = ICM20948(machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000),
                  **load_mag_calibration())

led(GROEN)
t0 = time.ticks_ms()
try:
    while time.ticks_diff(time.ticks_ms(), t0) < DUUR_S * 1000:
        # 10 metingen middelen (als vector, dus geen sprong rond 0/360)
        sx = sy = 0.0
        for _ in range(10):
            mx, my, _mz = sensor.magnetic
            sx += mx; sy += my
            time.sleep_ms(20)
        koers = math.degrees(math.atan2(sy, sx)) % 360
        bh = math.sqrt(sx * sx + sy * sy) / 10
        over = DUUR_S - time.ticks_diff(time.ticks_ms(), t0) // 1000
        toon("KOMPAS", ("Bh %.1f uT" % bh, "nog %ds" % over), "%3.0f gr" % koers)
        print("koers %3.0f  Bh %.1f uT" % (koers, bh))
finally:
    led(UIT)
toon("KLAAR")
