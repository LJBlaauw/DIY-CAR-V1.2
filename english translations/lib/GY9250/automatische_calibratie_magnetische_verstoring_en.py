import machine
import time
from mpu9250 import MPU9250
from stepper import rotate, stop, disable

# --- AUTOMATIC CALIBRATION (robot rotates by itself) ---
# Uses the existing PIO stepper motor driver (lib/stepper/stepper.py) to
# rotate the robot on its axis during calibration. This way you don't
# have to rotate the robot by hand.

DRAAI_SNELHEID_CM_S = 8      # rotation speed of each wheel
DRAAI_AFSTAND_CM = 500       # more than enough for multiple full turns

# GY9250 is on GPIO10 (SDA1) and GPIO11 (SCL1) -> hardware I2C bus 1
i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
sensor = MPU9250(i2c)

print("START COUNTDOWN: Clear the robot on the floor...")
time.sleep(3)

print("Motors enabled. Calibration starts now!")
rotate('r', DRAAI_SNELHEID_CM_S, DRAAI_AFSTAND_CM)

# Arrays to store the boundary values
min_x = max_x = min_y = max_y = min_z = max_z = None

# We take 1000 measurements while rotating (approx. 15-20 seconds)
for i in range(1000):
    time.sleep_ms(15)

    try:
        # Read the raw, uncalibrated magnetometer values
        mx, my, mz = sensor.magnetic

        # Initialize boundaries on the first measurement
        if min_x is None:
            min_x = max_x = mx
            min_y = max_y = my
            min_z = max_z = mz
        else:
            # Update minimum and maximum values
            if mx < min_x: min_x = mx
            if mx > max_x: max_x = mx
            if my < min_y: min_y = my
            if my > max_y: max_y = my
            if mz < min_z: min_z = mz
            if mz > max_z: max_z = mz

    except Exception:
        pass

# Stop the robot after measurements
stop()
disable()
print("\n--- CALIBRATION COMPLETED ---")

# 1. CALCULATE HARD-IRON OFFSETS (The shifted center point)
hard_iron_x = (max_x + min_x) / 2
hard_iron_y = (max_y + min_y) / 2
hard_iron_z = (max_z + min_z) / 2

# 2. CALCULATE SOFT-IRON SCALING (The distortion of the circle)
# Calculate the average radius per axis
chord_x = (max_x - min_x) / 2
chord_y = (max_y - min_y) / 2
chord_z = (max_z - min_z) / 2

# Prevent division by zero if an axis happens to show no variation
chord_x = chord_x or 1e-6
chord_y = chord_y or 1e-6
chord_z = chord_z or 1e-6

gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

soft_iron_x = gemiddelde_straal / chord_x
soft_iron_y = gemiddelde_straal / chord_y
soft_iron_z = gemiddelde_straal / chord_z

# Print the ready-to-use lines for your main program
print("\nCopy the lines below to your main program (AK8963 constructor):")
print(f"offset=({hard_iron_x:.2f}, {hard_iron_y:.2f}, {hard_iron_z:.2f})")
print(f"scale=({soft_iron_x:.4f}, {soft_iron_y:.4f}, {soft_iron_z:.4f})")
print("\n---------------------------")
