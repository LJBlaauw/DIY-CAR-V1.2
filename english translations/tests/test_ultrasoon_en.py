import time
import machine
import ultrasoon
from ssd1306 import SSD1306_I2C

# --- Test script: Ultrasonic (measured distance) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C bus 0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Ultrasonic test", "start in 2s..."])
time.sleep(2)

print("Test started. Ctrl-C to stop.")

try:
    while True:
        cm, kind = ultrasoon.read_cm()
        if kind == 'ok':
            toon(["ULTRASONIC", f"{cm:6.1f} cm"])
            print(f"Distance: {cm:.1f} cm")
        elif kind == 'overflow':
            toon(["ULTRASONIC", "Max/overflow"])
            print("Max/overflow reached")
        else:
            toon(["ULTRASONIC", "Timeout", "no echo"])
            print("Timeout: no echo")
        time.sleep_ms(100)
except KeyboardInterrupt:
    ultrasoon.stop()
    toon(["Ultrasonic test", "Stopped."])
    print("Test stopped.")
