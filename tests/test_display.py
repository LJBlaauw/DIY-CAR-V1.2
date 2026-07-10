import machine
import time
from ssd1306 import SSD1306_I2C

# --- Testscript: OLED-display (simpele tekst) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


print("Display test: I2C-scan...")
adressen = i2c_oled.scan()
print("Gevonden I2C-adressen:", [hex(a) for a in adressen])

toon(["DISPLAY TEST", "Hallo DIY-CAR!", "128x64 SSD1306"])
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

teller = 0
try:
    while True:
        toon([
            "DISPLAY TEST",
            "Live teller:",
            f"{teller}",
        ])
        print("teller =", teller)
        teller += 1
        time.sleep_ms(500)
except KeyboardInterrupt:
    toon(["DISPLAY TEST", "Gestopt."])
    print("Test gestopt.")
