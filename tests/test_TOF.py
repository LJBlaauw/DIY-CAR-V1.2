# ================================================================
# tests/test_TOF.py
#
# Testscript: VL53L1X time-of-flight-afstandssensor, interrupt-gestuurd.
#
# I2C: I2C0 (GPIO0 SDA / GPIO1 SCL) -- GEDEELD met de SSD1306-OLED (0x3C).
#      De VL53L1X zit op 0x29, geen adresconflict.
# Interrupt: GPIO1 van de sensor (open-drain, active-low) -> Pico GPIO8
#      (fysieke pin 11). Vereist een externe pull-up naar 3V3 (datasheet:
#      10 kOhm) -- de interne PULL_UP hieronder is een extra marge, geen
#      vervanging daarvoor.
#
# Timing: 33 ms timing budget, 50 ms inter-measurement period (20 Hz),
# zoals besproken: 17 ms marge boven de budget voor de I2C-afhandeling.
#
# VEILIGHEID: de IRQ-handler doet GEEN I2C (zie lib/VL53L1X/vl53l1x.py en
# lib/ultrasoon/ultrasoon.py voor dezelfde reden) -- hij zet alleen een vlag;
# het echte lezen gebeurt in de hoofdlus.
# ================================================================

import time
import machine
from ssd1306 import SSD1306_I2C
from servo_crl import ServoController
from vl53l1x import VL53L1X, LONG, RANGE_VALID, status_name

TIMING_BUDGET_US = 33_000
PERIOD_MS = 50
IRQ_PIN = 8   # GPIO8, fysieke pin 11
N_AVG = 20    # aantal metingen (~1s bij 50ms) per getoond gemiddelde

i2c = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                  freq=400000, timeout=200_000)
oled = SSD1306_I2C(128, 64, i2c)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["VL53L1X test", "start over 2s..."])
print("Test gestart. Ctrl-C om te stoppen.")

sc = ServoController()
sc.laser_power(100.0)   # laser aan, zodat je visueel kunt zien waar de sensor op gericht staat

time.sleep(2)

tof = VL53L1X(i2c)
if not tof.init():
    toon(["VL53L1X test", "init MISLUKT", "(model-id?)"])
    raise RuntimeError("VL53L1X.init() mislukt -- WHO_AM_I klopt niet of geen boot-ack")

tof.set_distance_mode(LONG)
tof.set_timing_budget_us(TIMING_BUDGET_US)
tof.start_continuous(PERIOD_MS)

_data_ready = False


def _on_irq(pin):
    global _data_ready
    _data_ready = True


irq_pin = machine.Pin(IRQ_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
irq_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=_on_irq)

print("VL53L1X geinitialiseerd, timing budget %d us, periode %d ms" % (TIMING_BUDGET_US, PERIOD_MS))

try:
    n = 0            # totaal aantal metingen sinds start (voor het logregel-nummer)
    n_in_batch = 0    # metingen in de huidige groep van N_AVG
    n_ok = 0          # waarvan status == RANGE_VALID
    som_mm = 0
    laatste_status = ""
    t_start = time.ticks_ms()
    while True:
        if _data_ready:
            _data_ready = False
            mm = tof.read(blocking=False)
            status_ok = (tof.range_status == RANGE_VALID)
            laatste_status = status_name(tof.range_status)
            n += 1
            n_in_batch += 1
            if status_ok:
                n_ok += 1
                som_mm += mm

            if n_in_batch >= N_AVG:
                gem_mm = (som_mm / n_ok) if n_ok else 0
                dt = time.ticks_diff(time.ticks_ms(), t_start)
                print("%6d ms  #%-4d  gem=%6.1f mm  (%d/%d geldig)  laatste status: %s"
                      % (dt, n, gem_mm, n_ok, n_in_batch, laatste_status))
                toon([
                    "VL53L1X gem %ds" % (N_AVG * PERIOD_MS // 1000),
                    "%6.1f mm" % gem_mm,
                    "%d/%d geldig" % (n_ok, n_in_batch),
                    laatste_status[:16],
                ])
                n_in_batch = 0
                n_ok = 0
                som_mm = 0
        else:
            time.sleep_ms(5)
except KeyboardInterrupt:
    print("Test gestopt.")
finally:
    irq_pin.irq(handler=None)
    tof.stop_continuous()
    sc.laser_off()
    toon(["VL53L1X test", "Gestopt."])
