import machine
import time
from mpu9250 import MPU9250
from ak8963 import AK8963
from stepper import rotate, stop, disable
from ssd1306 import SSD1306_I2C

# --- Test script: automatic (motorized) magnetometer calibration on OLED ---
# GY9250 (MPU9250): GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C bus 1
# OLED (SSD1306):   GPIO0  (SDA0) / GPIO1  (SCL0) -> I2C bus 0
#
# Uses the existing PIO stepper motor driver (lib/stepper/stepper.py) to
# rotate the robot on its axis during calibration.

DRAAI_SNELHEID_CM_S = 8       # rotation speed of each wheel
DRAAI_AFSTAND_CM = 800        # more than enough for multiple full rotations
AANTAL = 1000                 # number of magnetometer measurements
OLED_INTERVAL = 10            # update OLED every N measurements (faster sampling)

i2c_sensor = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)

oled = SSD1306_I2C(128, 64, i2c_oled)

dummy = MPU9250(i2c_sensor)  # opens the I2C bypass to the AK8963
ak8963 = AK8963(i2c_sensor)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Auto-calibration", "Clear the robot", "on the floor...", "start in 3s..."])
print("Auto-calibration: clear the robot on the floor...")
time.sleep(3)

toon(["Auto-calibration", "Motors on..."])
print("Motors enabled. Calibration starts now!")
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

        # Update OLED only every N measurements (I2C framebuffer push is slow)
        if (i + 1) % OLED_INTERVAL == 0 or i == AANTAL - 1:
            toon([
                "AUTO-CALIBRATION",
                f"{i + 1}/{AANTAL}",
                f"X{min_x:+.0f}/{max_x:+.0f}",
                f"Y{min_y:+.0f}/{max_y:+.0f}",
                f"Z{min_z:+.0f}/{max_z:+.0f}",
            ])
        print(
            f"[{i + 1}/{AANTAL}] "
            f"x=({min_x:.1f},{max_x:.1f}) y=({min_y:.1f},{max_y:.1f}) z=({min_z:.1f},{max_z:.1f})"
        )
except KeyboardInterrupt:
    print("Interrupted by user, result will be calculated with measurements gathered so far.")
finally:
    # Motors must always stop, even during an interruption
    stop()
    disable()

print("\n--- CALIBRATION COMPLETED ---")

# Hard-iron offsets (the shifted center point)
offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2

# Soft-iron scaling (the distortion of the circle)
chord_x = (max_x - min_x) / 2
chord_y = (max_y - min_y) / 2
chord_z = (max_z - min_z) / 2
if chord_x == 0: chord_x = 1e-6  # prevent division by zero
if chord_y == 0: chord_y = 1e-6
if chord_z == 0: chord_z = 1e-6
gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

scale_x = gemiddelde_straal / chord_x
scale_y = gemiddelde_straal / chord_y
scale_z = gemiddelde_straal / chord_z

toon([
    "DONE!",
    f"o({offset_x:.1f},{offset_y:.1f}",
    f" {offset_z:.1f})",
    f"s({scale_x:.2f},{scale_y:.2f}",
    f" {scale_z:.2f})",
])

print("Copy the lines below to your main program (AK8963 constructor):")
print(f"offset=({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f})")
print(f"scale=({scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f})")
print("---------------------------")
