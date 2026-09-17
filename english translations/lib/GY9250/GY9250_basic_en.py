import machine
import time
import _thread
import math
from mpu9250 import MPU9250
from ak8963 import AK8963

# Global variable for the current, corrected heading
# Core 1 writes to this continuously, Core 0 reads it live
actuele_koers = 0.0


def verwerk_kompas_core1():
    global actuele_koers

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

    # Variable for the complementary filter
    # We start at 0.0, the filter stabilizes itself quickly
    gefilterde_hoek = 0.0
    dt = 0.02  # Loop execution time (20 milliseconds = 50 Hz)

    print("Core 1: Calibrated compass task active at 50Hz")

    while True:
        try:
            # Read the corrected magnetometer and gyro values
            # The driver automatically applies your offsets above
            mx, my, mz = sensor.magnetic
            gx, gy, gz = sensor.gyro  # gz is the rotation speed around the Z-axis (rad/s)

            # Convert gyro rotation from radians to degrees per second
            gyro_z_graden = gz * (180.0 / math.pi)

            # Calculate the raw magnetic angle in degrees (0 to 360)
            ruwe_hoek = math.atan2(my, mx) * (180.0 / math.pi)
            if ruwe_hoek < 0:
                ruwe_hoek += 360.0

            # --- COMPLEMENTARY FILTER (Sensor Fusion) ---
            # This effectively filters out the PWM noise from your TMC2209 drivers.
            # 96% weight to the fast gyroscope, 4% to the correcting magnetic sensor.
            gefilterde_hoek = 0.96 * (gefilterde_hoek + gyro_z_graden * dt) + 0.04 * ruwe_hoek
            gefilterde_hoek = gefilterde_hoek % 360.0

            # Write the result atomically to the global variable
            actuele_koers = gefilterde_hoek

        except Exception:
            # Prevent Core 1 from crashing during an incidental I2C glitch caused by the motors
            pass

        time.sleep_ms(20)  # 50 Hz update frequency


# --- MAIN PROGRAM (Runs on Core 0) ---

# Start the compass calculation on Core 1
_thread.start_new_thread(verwerk_kompas_core1, ())

print("Core 0: Motor control ready")
time.sleep(1)  # Give Core 1 a brief moment to boot up

while True:
    # Retrieve the most recent, stable direction
    robot_richting = actuele_koers

    # --- MOTOR LOGIC HERE ---
    # Use 'robot_richting' to adjust your NEMA 17 motors while driving
    # If the direction deviates from your target heading, adjust the speed of the left or right motor.

    print("Live robot direction:", robot_richting)
    time.sleep_ms(100)  # Core 0 may run at its own pace
