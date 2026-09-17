import machine
import time
from mpu9250 import MPU9250

# --- Manual Magnetometer Calibration Script ---
# GY9250 is on GPIO10 (SDA1) and GPIO11 (SCL1) -> hardware I2C bus 1
#
# Instructions: Start this script and slowly rotate the sensor/robot in all
# possible directions (figure-8 motion in 3D space) so that the sensor
# registers all maximum and minimum magnetic fields.
# Press Ctrl-C to stop measuring and calculate the values.

i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
sensor = MPU9250(i2c)

print("Magnetometer calibration: rotate the sensor in all directions (3D space).")
print("Press Ctrl-C when you are done to calculate the offsets.\n")
time.sleep(2)

min_x = max_x = min_y = max_y = min_z = max_z = None
teller = 0

try:
    while True:
        time.sleep_ms(20)
        try:
            # Read the raw, uncalibrated magnetometer values
            mx, my, mz = sensor.magnetic
            teller += 1

            if min_x is None:
                min_x = max_x = mx
                min_y = max_y = my
                min_z = max_z = mz
            else:
                if mx < min_x: min_x = mx
                if mx > max_x: max_x = mx
                if my < min_y: min_y = my
                if my > max_y: max_y = my
                if mz < min_z: min_z = mz
                if mz > max_z: max_z = mz

            if teller % 25 == 0:
                print(f"[{teller}] X:({min_x:+.1f}/{max_x:+.1f})  Y:({min_y:+.1f}/{max_y:+.1f})  Z:({min_z:+.1f}/{max_z:+.1f})")
        except Exception:
            pass
except KeyboardInterrupt:
    print("\n--- CALIBRATION ENDED ---")

# 1. Hard-iron offsets (the shifted center point of the sphere)
offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2

# 2. Soft-iron scaling (correcting the squashed sphere into a perfect sphere)
chord_x = (max_x - min_x) / 2
chord_y = (max_y - min_y) / 2
chord_z = (max_z - min_z) / 2

chord_x = chord_x if chord_x != 0 else 1e-6
chord_y = chord_y if chord_y != 0 else 1e-6
chord_z = chord_z if chord_z != 1e-6 else 1e-6

gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

scale_x = gemiddelde_straal / chord_x
scale_y = gemiddelde_straal / chord_y
scale_z = gemiddelde_straal / chord_z

print("\nCopy the lines below to your main program (AK8963 constructor):")
print(f"offset=({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f})")
print(f"scale=({scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f})")
print("---------------------------")
