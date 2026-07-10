import machine
import time
from mpu9250 import MPU9250
from ak8963 import AK8963
from stepper import rotate, stop, disable
from ssd1306 import SSD1306_I2C

# --- Testscript: automatische (gemotoriseerde) magnetometer-kalibratie op OLED ---
# GY9250 (MPU9250): GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
# OLED (SSD1306):   GPIO0  (SDA0) / GPIO1  (SCL0) -> I2C-bus 0
#
# Gebruikt de bestaande PIO-steppermotor-driver (lib/stepper/stepper.py) om
# de robot op zijn as te laten draaien tijdens het kalibreren.

DRAAI_SNELHEID_CM_S = 8       # rotatiesnelheid van elk wiel
DRAAI_AFSTAND_CM = 500        # ruim voldoende voor meerdere volle rondjes
AANTAL = 1000                 # ca. 15-20 seconden metingen

i2c_sensor = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)

oled = SSD1306_I2C(128, 64, i2c_oled)

dummy = MPU9250(i2c_sensor)  # opent de I2C-bypass naar de AK8963
ak8963 = AK8963(i2c_sensor)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Auto-kalibratie", "Zet de robot vrij", "op de grond...", "start over 3s..."])
print("Auto-kalibratie: zet de robot vrij op de grond...")
time.sleep(3)

toon(["Auto-kalibratie", "Motoren aan..."])
print("Motoren ingeschakeld. Kalibratie start nu!")
rotate('r', DRAAI_SNELHEID_CM_S, DRAAI_AFSTAND_CM)

reading = ak8963.magnetic
min_x = max_x = reading[0]
min_y = max_y = reading[1]
min_z = max_z = reading[2]

try:
    for i in range(AANTAL):
        time.sleep_ms(15)
        try:
            mx, my, mz = ak8963.magnetic
        except Exception:
            continue

        if mx < min_x: min_x = mx
        if mx > max_x: max_x = mx
        if my < min_y: min_y = my
        if my > max_y: max_y = my
        if mz < min_z: min_z = mz
        if mz > max_z: max_z = mz

        toon([
            "AUTO-KALIBRATIE",
            f"{i + 1}/{AANTAL}",
            f"X {min_x:6.1f}/{max_x:6.1f}",
            f"Y {min_y:6.1f}/{max_y:6.1f}",
            f"Z {min_z:6.1f}/{max_z:6.1f}",
        ])
        print(
            f"[{i + 1}/{AANTAL}] "
            f"x=({min_x:.1f},{max_x:.1f}) y=({min_y:.1f},{max_y:.1f}) z=({min_z:.1f},{max_z:.1f})"
        )
except KeyboardInterrupt:
    print("Onderbroken door gebruiker, resultaat wordt berekend met de tot nu toe verzamelde metingen.")
finally:
    # Motoren moeten altijd stoppen, ook bij een onderbreking
    stop()
    disable()

print("\n--- KALIBRATIE VOLTOOID ---")

# Hard-iron offsets (het verschoven middelpunt)
offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2

# Soft-iron schaling (de vervorming van de cirkel)
chord_x = (max_x - min_x) / 2 or 1e-6  # voorkom deling door nul
chord_y = (max_y - min_y) / 2 or 1e-6
chord_z = (max_z - min_z) / 2 or 1e-6
gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

scale_x = gemiddelde_straal / chord_x
scale_y = gemiddelde_straal / chord_y
scale_z = gemiddelde_straal / chord_z

toon([
    "KLAAR!",
    f"o({offset_x:.1f},{offset_y:.1f}",
    f" {offset_z:.1f})",
    f"s({scale_x:.2f},{scale_y:.2f}",
    f" {scale_z:.2f})",
])

print("Kopieer de onderstaande regels naar uw hoofdprogramma (AK8963-constructor):")
print(f"offset=({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f})")
print(f"scale=({scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f})")
print("---------------------------")
