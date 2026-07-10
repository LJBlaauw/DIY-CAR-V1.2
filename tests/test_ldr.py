import machine
import time
from machine import ADC, Pin
from ssd1306 import SSD1306_I2C
import ldr_scan_isr as ldr

# --- Testscript: LDR (gemeten spanningen) ---
# LDR A / LDR B: zie ldr_scan_isr.LDR_PIN_A / LDR_PIN_B (GPIO26 / GPIO27)
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0

ADC_VREF = 3.3
ADC_MAX = 65535.0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)

adc_a = ADC(Pin(ldr.LDR_PIN_A, Pin.IN))
adc_b = ADC(Pin(ldr.LDR_PIN_B, Pin.IN))


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def naar_volt(raw_u16):
    return (raw_u16 / ADC_MAX) * ADC_VREF


toon(["LDR test", f"pin A={ldr.LDR_PIN_A} B={ldr.LDR_PIN_B}", "start over 2s..."])
print(f"LDR test: gemeten spanningen op GPIO{ldr.LDR_PIN_A} / GPIO{ldr.LDR_PIN_B}")
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        v_a = naar_volt(adc_a.read_u16())
        v_b = naar_volt(adc_b.read_u16())

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
