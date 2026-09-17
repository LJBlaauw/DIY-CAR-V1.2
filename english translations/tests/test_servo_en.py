# ================================================================
# tests/test_servo.py
#
# Test script for the PCA9685 / I2C servo controller board and the gripper.
# Servos: GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C bus 0
#
# Tests continuous movement, angles, and checks current consumption limits.
# ================================================================

import time
import machine
from machine import Pin, I2C
from servo_crl import ServoController
from ssd1306 import SSD1306_I2C

# Configuration
SERVO_ID = 1
START_DEG = 10.0
END_DEG = 170.0
STEP_DEG = 5.0
DELAY_MS = 100

i2c_oled = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)

sc = ServoController()
sc.servo_cur_limit(300)  # Current limit in mA

def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()

toon(["Servo test", f"Servo ID: {SERVO_ID}", "start in 2s..."])
print(f"Servo test started on ID {SERVO_ID}. Current limit: 300mA")
time.sleep(2)

print("Test started. Ctrl-C to stop.")

try:
    while True:
        # Move forward
        print(f"Moving from {START_DEG} to {END_DEG} degrees...")
        hoek = START_DEG
        while hoek <= END_DEG:
            sc.servo_pos(SERVO_ID, hoek, 40)  # Move to position at 40 deg/s
            toon([
                "SERVO TEST",
                f"Servo ID: {SERVO_ID}",
                f"Angle: {hoek:.1f} deg",
                f"Current: {sc.read_current():.0f} mA"
            ])
            print(f"Angle: {hoek:.1f} -> Current: {sc.read_current()} mA")
            hoek += STEP_DEG
            time.sleep_ms(DELAY_MS)

        time.sleep(1)

        # Move backward
        print(f"Moving from {END_DEG} to {START_DEG} degrees...")
        hoek = END_DEG
        while hoek >= START_DEG:
            sc.servo_pos(SERVO_ID, hoek, 40)
            toon([
                "SERVO TEST",
                f"Servo ID: {SERVO_ID}",
                f"Angle: {hoek:.1f} deg",
                f"Current: {sc.read_current():.0f} mA"
            ])
            print(f"Angle: {hoek:.1f} -> Current: {sc.read_current()} mA")
            hoek -= STEP_DEG
            time.sleep_ms(DELAY_MS)

        time.sleep(1)

except KeyboardInterrupt:
    sc.servo_rest()  # Move all servos to rest position
    toon(["Servo test", "Stopped."])
    print("Test stopped.")
