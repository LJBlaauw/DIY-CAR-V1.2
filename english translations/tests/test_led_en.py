import time
import machine
from machine import Pin
from neopixel import NeoPixel
from ssd1306 import SSD1306_I2C

# --- Test script: WS2812B RGB-LED (single-wire, GPIO6) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C bus 0
#
# Sequence: red, green, blue each at 50%, then white brightening up
# from 5% to 60% brightness.

LED_PIN = 6
AANTAL_PIXELS = 1

VASTE_HELDERHEID_PCT = 50.0
WIT_START_PCT = 5.0
WIT_EIND_PCT = 60.0
WIT_STAP_PCT = 1.0
WIT_STAP_VERTRAGING_MS = 50

VASTHOUD_SECONDEN = 2.0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)

np = NeoPixel(Pin(LED_PIN), AANTAL_PIXELS)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def pct_naar_255(pct):
    pct = max(0.0, min(100.0, pct))
    return round(pct / 100.0 * 255)


def zet_kleur(r_pct, g_pct, b_pct):
    kleur = (pct_naar_255(r_pct), pct_naar_255(g_pct), pct_naar_255(b_pct))
    np[0] = kleur
    np.write()
    return kleur


def uit():
    np[0] = (0, 0, 0)
    np.write()


toon(["RGB LED test", "start in 2s..."])
print(f"RGB-LED test on GPIO{LED_PIN} (WS2812B, {AANTAL_PIXELS} pixel)")
time.sleep(2)

print("Test started. Ctrl-C to stop.")

try:
    # 1) Red, green, blue each at 50%
    for naam, kleur_pct in (
        ("RED", (VASTE_HELDERHEID_PCT, 0.0, 0.0)),
        ("GREEN", (0.0, VASTE_HELDERHEID_PCT, 0.0)),
        ("BLUE", (0.0, 0.0, VASTE_HELDERHEID_PCT)),
    ):
        rgb = zet_kleur(*kleur_pct)
        toon([
            "RGB LED TEST",
            f"{naam} {VASTE_HELDERHEID_PCT:.0f}%",
            f"RGB={rgb}",
        ])
        print(f"{naam} at {VASTE_HELDERHEID_PCT:.0f}% -> RGB={rgb}")
        time.sleep(VASTHOUD_SECONDEN)

    # 2) White, increasing from 5% to 60%
    print(f"White: {WIT_START_PCT:.0f}% -> {WIT_EIND_PCT:.0f}%")
    pct = WIT_START_PCT
    while pct <= WIT_EIND_PCT:
        rgb = zet_kleur(pct, pct, pct)
        toon([
            "RGB LED TEST",
            "WHITE increasing:",
            f"{pct:5.1f} %",
            f"RGB={rgb}",
        ])
        print(f"WHITE {pct:.1f}% -> RGB={rgb}")
        pct += WIT_STAP_PCT
        time.sleep_ms(WIT_STAP_VERTRAGING_MS)

    toon(["RGB LED TEST", "Done!", f"White at {WIT_EIND_PCT:.0f}%"])
    print("Test done.")
    time.sleep(VASTHOUD_SECONDEN)
except KeyboardInterrupt:
    toon(["RGB LED test", "Stopped."])
    print("Test stopped.")
finally:
    uit()
