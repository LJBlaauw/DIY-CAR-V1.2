import time
import machine
from servo_crl import ServoController
from ssd1306 import SSD1306_I2C

# --- Testscript: Laser (vermogensramp) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
#
# Laat het laservermogen in STAPPEN stapjes oplopen van START_PCT naar
# EIND_PCT, en zet het daarna terug naar EIND_ACTIEF_PCT.
#
# LET OP — in tegenstelling tot de andere testscripts (test_servo.py,
# test_led.py, test_stepper.py) is er hier BEWUST geen `finally` die de
# laser uitzet: de laser moet aan het eind actief BLIJVEN staan op
# EIND_ACTIEF_PCT, ook na Ctrl-C. Zet 'm handmatig uit met
# sc.laser_off() als dat nodig is.
#
# ServoController claimt ook de 3 MG996R-servo's; dit script raakt die niet
# aan, maar ze gaan door __init__ wel meteen naar hun rustpositie (zie
# lib/servo/servo_crl.py).

START_PCT = 10.0
EIND_PCT = 100.0
STAPPEN = 10
EIND_ACTIEF_PCT = 50.0
STAP_VERTRAGING_MS = 1000

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Laser test", "start over 2s..."])
print(f"Laser test: {STAPPEN} stappen van {START_PCT:.0f}% naar {EIND_PCT:.0f}%, "
      f"dan terug naar {EIND_ACTIEF_PCT:.0f}% (blijft aan)")
time.sleep(2)

sc = ServoController()          # zet ook de servo's meteen naar hun rustpositie

print("Test gestart. Ctrl-C om te stoppen (laser blijft op het laatst ingestelde vermogen).")

stap_grootte = (EIND_PCT - START_PCT) / (STAPPEN - 1)

try:
    for i in range(STAPPEN):
        pct = START_PCT + i * stap_grootte
        sc.laser_power(pct)
        toon(["Laser test", "oplopend:", f"{pct:5.1f} %"])
        print(f"Laser vermogen: {pct:5.1f} %")
        time.sleep_ms(STAP_VERTRAGING_MS)

    sc.laser_power(EIND_ACTIEF_PCT)
    toon(["Laser test", "Klaar!", f"{EIND_ACTIEF_PCT:.0f} % actief"])
    print(f"Laser op {EIND_ACTIEF_PCT:.0f}% en blijft actief.")
except KeyboardInterrupt:
    toon(["Laser test", "Gestopt.", "laser blijft aan"])
    print("Test onderbroken; laser blijft op het laatst ingestelde vermogen (geen auto-uit).")
