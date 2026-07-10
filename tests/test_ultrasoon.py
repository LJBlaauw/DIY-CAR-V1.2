import time
import machine
import ultrasoon
from ssd1306 import SSD1306_I2C

# --- Testscript: Ultrasoon (gemeten afstand) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["Ultrasoon test", "start over 2s..."])
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        cm, kind = ultrasoon.read_cm()
        if kind == 'ok':
            toon(["ULTRASOON", f"{cm:6.1f} cm"])
            print(f"Afstand: {cm:.1f} cm")
        elif kind == 'overflow':
            toon(["ULTRASOON", "Max/overflow"])
            print("Max/overflow bereikt")
        else:
            toon(["ULTRASOON", "Timeout", "geen echo"])
            print("Timeout: geen echo")
        time.sleep_ms(100)
except KeyboardInterrupt:
    ultrasoon.stop()
    toon(["Ultrasoon test", "Gestopt."])
    print("Test gestopt.")
