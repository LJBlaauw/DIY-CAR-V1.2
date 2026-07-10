import machine
import time
from mpu9250 import MPU9250
from ak8963 import AK8963
from ssd1306 import SSD1306_I2C

# --- Testscript: handmatige magnetometer-kalibratie met live weergave op OLED ---
# GY9250 (MPU9250): GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
# OLED (SSD1306):   GPIO0  (SDA0) / GPIO1  (SCL0) -> I2C-bus 0
#
# Til de robot op en draai hem tijdens het meten langzaam en volledig rond
# alle assen (net als een 8-vormige beweging). Voor een gemotoriseerde
# variant zie test_gy9250_auto_calibrate.py

AANTAL = 256
VERTRAGING_MS = 200

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


toon(["Kalibratie test", "Draai de sensor", "rond alle assen", "start over 3s..."])
print("Kalibratietest: draai de sensor rond alle assen...")
time.sleep(3)

reading = ak8963.magnetic
min_x = max_x = reading[0]
min_y = max_y = reading[1]
min_z = max_z = reading[2]

print(f"Kalibratie gestart ({AANTAL} metingen, {VERTRAGING_MS} ms tussenpauze)...")

try:
    for i in range(AANTAL):
        time.sleep_ms(VERTRAGING_MS)
        mx, my, mz = ak8963.magnetic

        if mx < min_x: min_x = mx
        if mx > max_x: max_x = mx
        if my < min_y: min_y = my
        if my > max_y: max_y = my
        if mz < min_z: min_z = mz
        if mz > max_z: max_z = mz

        toon([
            "KALIBRATIE",
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

print("\n--- KALIBRATIE VOLTOOID ---")
print("Kopieer de onderstaande regels naar uw hoofdprogramma (AK8963-constructor):")
print(f"offset=({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f})")
print(f"scale=({scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f})")
print("---------------------------")
