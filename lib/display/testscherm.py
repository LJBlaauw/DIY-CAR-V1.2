# ================================================================
# testscherm.py - OLED + RGB-LED voor begeleide hardwaretests
# ================================================================
#
# Optische synchronisatie, zodat je tijdens een test niet het kleine
# display hoeft te volgen:
#   ORANJE  pauze: lees op de OLED wat de volgende stap is
#   GROEN   stap loopt: nu de handeling uitvoeren (of: niets aanraken)
#   ROOD    stap gestopt, resultaat op de OLED
#
# Hardware: OLED (SSD1306) op I2C0 (SDA GPIO0 / SCL GPIO1),
#           WS2812B RGB-LED op GPIO6.

import machine
import time
import framebuf
from neopixel import NeoPixel
from ssd1306 import SSD1306_I2C

LED_PIN = 6
HELDERHEID = 0.3        # LED niet op vol vermogen: fel genoeg, verblindt niet

GROEN = (0, 255, 0)
ROOD = (255, 0, 0)
ORANJE = (255, 70, 0)   # WS2812B: rood + weinig groen oogt oranje
UIT = (0, 0, 0)

# timeout ruim boven de duur van een volledig scherm (1 KB: ~25 ms op 400 kHz,
# ~93 ms op 100 kHz). De default van 50 ms gaf op 100 kHz ETIMEDOUT, waarna de
# OLED halverwege een overdracht bleef hangen (2026-10-26).
oled = SSD1306_I2C(128, 64, machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                                        freq=400000, timeout=200_000))
_np = NeoPixel(machine.Pin(LED_PIN), 1)
_kl = bytearray(8 * 8)  # kladbuffer voor maximaal 8 tekens groot


def led(kleur):
    _np[0] = tuple(int(c * HELDERHEID) for c in kleur)
    _np.write()


def groot(t, y):
    """Tekst op dubbele grootte (16 px hoog, max. 8 tekens per regel)."""
    t = t[:8]
    fb = framebuf.FrameBuffer(_kl, len(t) * 8, 8, framebuf.MONO_VLSB)
    fb.fill(0)
    fb.text(t, 0, 0, 1)
    for px in range(len(t) * 8):
        for py in range(8):
            if fb.pixel(px, py):
                oled.fill_rect(px * 2, y + py * 2, 2, 2, 1)


def toon(boven, klein=(), onder=""):
    """Grote kop, max. twee kleine regels (16 tekens), en een grote onderregel."""
    oled.fill(0)
    groot(boven, 0)
    for i, t in enumerate(klein[:2]):
        oled.text(t, 0, 20 + i * 10)
    if onder:
        groot(onder, 44)
    oled.show()


def aftellen(seconden, kleur, boven, klein=()):
    """LED in `kleur` en een grote aftelling op de onderste regel."""
    led(kleur)
    for k in range(seconden, 0, -1):
        toon(boven, klein, "%ds" % k)
        time.sleep(1)
