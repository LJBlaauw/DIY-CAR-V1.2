import machine
import time
import _thread
from mpu9250 import MPU9250
from ak8963 import AK8963
from fusion import Fusion

# Global variables for the current orientation
# Core 1 writes to these continuously, Core 0 reads them live
actuele_koers = 0.0
actuele_roll = 0.0
actuele_pitch = 0.0


def verwerk_fusion_core1():
    global actuele_koers, actuele_roll, actuele_pitch

    # 1. Always initialize I2C WITHIN the core function
    # GY9250 is on GPIO10 (SDA1) and GPIO11 (SCL1) -> hardware I2C bus 1
    i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)

    # 2. ENTER YOUR OWN CALIBRATION RESULTS HERE (from Calibrate_GY9250.py):
    dummy = MPU9250(i2c)  # opens the I2C bypass to the AK8963
    ak8963 = AK8963(
        i2c,
        offset=(-24.5, 12.3, -5.1),      # Hard-iron offsets (X, Y, Z)
        scale=(0.95, 1.02, 1.03),        # Soft-iron scaling (X, Y, Z)
    )
    sensor = MPU9250(i2c, ak8963=ak8963)

    # Initialize the Fusion library object for tilt-compensated filtering
    fuse = Fusion()

    print("Core 1: Advanced fusion task active at 200Hz")

    while True:
        try:
            # Read the corrected sensor data vectors
            accel = sensor.acceleration
            gyro = sensor.gyro
            mag = sensor.magnetic

            # Update the filter; returns the tilt-compensated heading
            heading = fuse.update(accel, gyro, mag)

            # Write results atomically to the global variables
            actuele_koers = heading
            actuele_roll = fuse.roll
            actuele_pitch = fuse.pitch

        except Exception:
            # Prevent Core 1 from crashing during an incidental I2C glitch caused by the motors
            pass

        time.sleep_ms(5)  # 200 Hz update frequency


# --- MAIN PROGRAM (Runs on Core 0) ---

# Start the sensor fusion calculation on Core 1
_thread.start_new_thread(verwerk_fusion_core1, ())

print("Core 0: Control system ready")
time.sleep(1)  # Give Core 1 a brief moment to boot up and stabilize

while True:
    # Retrieve the most recent, stable 3D orientation values
    robot_richting = actuele_koers
    robot_roll = actuele_roll
    robot_pitch = actuele_pitch

    # --- CONTROL LOGIC HERE ---
    # Use these stable orientation variables to guide your robot cart.
    # Pitch and roll can be used to detect slopes or prevent tipping over.

    print(f"Live robot orientation: Heading={robot_richting:.1f} Roll={robot_roll:.1f} Pitch={robot_pitch:.1f}")
    time.sleep_ms(100)  # Core 0 runs at its own processing pace
