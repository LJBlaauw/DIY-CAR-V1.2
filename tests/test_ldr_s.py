import sys
import time
from machine import ADC, Pin, I2C
from ssd1306 import SSD1306_I2C

sys.path.append("/lib/LDR")
sys.path.append("/lib/servo")

import ldr_scan_isr
from servo_crl import ServoController

# --- Testscript: LDR fase-gesynchroniseerde sampling (read_synced) ---
#
# Laat drie manieren om met de 100Hz-netrimpel (LED-lamp op 230V AC) om te
# gaan naast elkaar zien, zodat het effect direct af te lezen is:
#
#   RUW    - één losse ADC-sample per kanaal, GEEN correctie. Dit is het
#            probleem: springt zichtbaar mee met de netrimpel.
#   AVG    - ldr_scan_isr.measure_now(): blokkerend, 10 samples over 10 ms
#            uitgemiddeld (= exact één netperiode) -> de rimpel middelt weg.
#   SYNC   - ldr_scan_isr.read_synced(): niet-blokkerend, leest de achter-
#            grondbuffer die ServoController's 50Hz-PWM-tick (GPIO2->GPIO5,
#            rising edge) elke ~20 ms bijwerkt. Altijd dezelfde fase t.o.v.
#            de netrimpel -> geen jitter, wel een vaste (kleine) bias.
#
# Alle drie in dezelfde %-schaal (log-resistance, zie ldr_scan_isr.py), zodat
# ze rechtstreeks te vergelijken zijn. RUW hergebruikt daarvoor bewust de
# _adc_to_res_ohm()/_res_to_percent_log()-functies uit ldr_scan_isr in plaats
# van een derde kopie van die formule te maken (zie de LDR_R_FIXED_OHM-
# geschiedenis in globale_specificatie_NL.md voor waarom dat eerder al
# fout ging).
#
# VEREIST: GPIO2 (servo1 PWM) fysiek verbonden met GPIO5 (input) - zie
# hardware/gpio_pinout.md. Zonder die draad blijft SYNC hangen op de
# beginwaarde (0.0%, 0.0%) en verandert nooit.
#
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
# LDR A / LDR B: GPIO26 / GPIO27

LDR_PIN_A = 26
LDR_PIN_B = 27
ADC_VREF = 3.3
ADC_MAX = 65535.0

i2c_oled = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)

_adc_a = ADC(LDR_PIN_A)
_adc_b = ADC(LDR_PIN_B)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def naar_volt(raw_u16):
    return (raw_u16 / ADC_MAX) * ADC_VREF


def ruw_volt():
    """Eén losse sample per kanaal, in volt. Puur ter illustratie van de
    zichtbare netrimpel-jitter (vergelijk met tests/test_ldr.py)."""
    return naar_volt(_adc_a.read_u16()), naar_volt(_adc_b.read_u16())


def ruw_pct():
    """Eén losse sample per kanaal, in dezelfde %-schaal als measure_now()
    en read_synced()."""
    ra_ohm = ldr_scan_isr._adc_to_res_ohm(_adc_a.read_u16())
    rb_ohm = ldr_scan_isr._adc_to_res_ohm(_adc_b.read_u16())
    pa = round(ldr_scan_isr._res_to_percent_log(ra_ohm, ldr_scan_isr.LDR_GAIN_A), 1)
    pb = round(ldr_scan_isr._res_to_percent_log(rb_ohm, ldr_scan_isr.LDR_GAIN_B), 1)
    return pa, pb


toon(["LDR SYNC test", "check: GPIO2 naar", "GPIO5 verbonden?", "start over 2s..."])
print("LDR fase-sync test: RUW / AVG (measure_now) / SYNC (read_synced).")
print("Controleer eerst dat GPIO2 fysiek met GPIO5 verbonden is (PWM-tick).")
time.sleep(2)

sc = ServoController()
ldr_scan_isr.start_synced_sampling(sc)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        rv_a, rv_b = ruw_volt()
        rp_a, rp_b = ruw_pct()
        av_a, av_b = ldr_scan_isr.measure_now()
        sy_a, sy_b = ldr_scan_isr.read_synced()

        toon([
            "LDR SYNC TEST",
            f"raw A{rp_a:5.1f} B{rp_b:5.1f}",
            f"avg A{av_a:5.1f} B{av_b:5.1f}",
            f"syn A{sy_a:5.1f} B{sy_b:5.1f}",
        ])

        print("volt: A=%5.3f B=%5.3f   raw%%: A=%5.1f B=%5.1f   "
              "avg%%: A=%5.1f B=%5.1f   sync%%: A=%5.1f B=%5.1f"
              % (rv_a, rv_b, rp_a, rp_b, av_a, av_b, sy_a, sy_b))
        time.sleep_ms(200)
except KeyboardInterrupt:
    ldr_scan_isr.stop_synced_sampling(sc)
    toon(["LDR SYNC test", "Gestopt."])
    print("Test gestopt.")
