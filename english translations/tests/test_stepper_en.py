import time
import machine
from machine import Pin, I2C
import stepper
from ssd1306 import SSD1306_I2C

# --- Test script: Stepper Motors (PIO State Machines) ---
# Stepper Left (M0):  GPIO2 (Step) / GPIO3 (Dir) -> PIO0 SM0
# Stepper Right (M1): GPIO4 (Step) / GPIO5 (Dir) -> PIO0 SM1
# OLED (SSD1306):     GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C bus 0

SPEED_CM_S = 10.0
DISTANCE_CM = 30.0

i2c_oled = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Stepper test", "Initializing...", "start in 2s..."])
print("Stepper test: initializing PIO blocks...")
stepper.reset_PIO_distance()
stepper.status()
stepper.disable()
time.sleep(2)

print("Test started. Ctrl-C to stop.")

try:
    # 1) Rotate Left
    print(f"Rotating left: speed={SPEED_CM_S}cm/s, distance={DISTANCE_CM}cm")
    toon(["STEPPER TEST", "Action: Rotate L", f"Speed: {SPEED_CM_S}", f"Dist:  {DISTANCE_CM}"])
    stepper.rotate('l', SPEED_CM_S, DISTANCE_CM)

    # Wait until the stepper rotation is finished
    while stepper.sm0.active() or stepper.sm1.active():
        pass

    time.sleep(1)

    # 2) Move Backwards
    print(f"Moving backwards: speed={SPEED_CM_S}cm/s, distance={DISTANCE_CM}cm")
    toon(["STEPPER TEST", "Action: Move B", f"Speed: {SPEED_CM_S}", f"Dist:  {DISTANCE_CM}"])
    stepper.mov('b', SPEED_CM_S, DISTANCE_CM)

    while stepper.sm0.active() or stepper.sm1.active():
        pass

    time.sleep(1)

    # 3) Single Motor Test (Stepper 1 Forward)
    print("Moving stepper 1 forward only: speed=5cm/s, distance=10cm")
    toon(["STEPPER TEST", "Action: S1 Forward", "Speed: 5.0", "Dist:  10.0"])
    stepper.s1('f', 5.0, 10.0)

    while stepper.sm0.active() or stepper.sm1.active():
        pass

    stepper.distance()
    toon(["STEPPER TEST", "Done!"])
    print("Test done.")

except KeyboardInterrupt:
    stepper.disable()
    toon(["Stepper test", "Stopped."])
    print("Test stopped.")
