import time
from machine import ADC, Pin, I2C
from ssd1306 import SSD1306_I2C

# --- Testscript: LDR (gemeten spanningen) ---
# LDR A / LDR B: GPIO26 / GPIO27 (moet overeenkomen met ldr_scan_isr)
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
#
# De LED-lamp (230V AC) geeft 100Hz rimpel op de gemeten spanning. Om die
# weg te middelen wordt elke 1 ms een sample genomen en uitgemiddeld over
# 10 samples: 10 x 1 ms = 10 ms = precies één 100Hz-periode, dus de rimpel
# heft zichzelf op in het gemiddelde.

LDR_PIN_A = 26
LDR_PIN_B = 27

ADC_VREF = 3.3
ADC_MAX = 65535.0

SAMPLES = 10
SAMPLE_INTERVAL_MS = 1

i2c_oled = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)

adc_a = ADC(LDR_PIN_A)
adc_b = ADC(LDR_PIN_B)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def naar_volt(raw_u16):
    return (raw_u16 / ADC_MAX) * ADC_VREF


def lees_gemiddeld(adc_a, adc_b, n=SAMPLES, interval_ms=SAMPLE_INTERVAL_MS):
    """Middelt n samples, 1 ms uit elkaar, om 100Hz-netrimpel weg te middelen."""
    som_a = 0
    som_b = 0
    for i in range(n):
        som_a += adc_a.read_u16()
        som_b += adc_b.read_u16()
        if i < n - 1:
            time.sleep_ms(interval_ms)
    return som_a / n, som_b / n


toon(["LDR test", f"pin A={LDR_PIN_A} B={LDR_PIN_B}", "start over 2s..."])
print(f"LDR test: gemeten spanningen op GPIO{LDR_PIN_A} / GPIO{LDR_PIN_B}")
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        raw_a, raw_b = lees_gemiddeld(adc_a, adc_b)
        v_a = naar_volt(raw_a)
        v_b = naar_volt(raw_b)

        toon([
            "LDR TEST",
            "Gemeten spanning:",
            f"A: {v_a:5.2f} V",
            f"B: {v_b:5.2f} V",
        ])

        print(f"LDR A: {v_a:.3f} V   LDR B: {v_b:.3f} V")
        time.sleep_ms(200)
except KeyboardInterrupt:
    toon(["LDR test", "Gestopt."])
    print("Test gestopt.")
