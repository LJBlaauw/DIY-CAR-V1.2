import time
import machine
import stepper
from ssd1306 import SSD1306_I2C

# --- Testscript: Stepper (alle basisfuncties: setwaarden en afgelegde weg) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0

SNELHEID_CM_S = 5.0

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def wacht_tot_klaar(titel, doel_cm):
    while stepper.sm0.active() or stepper.sm1.active():
        d1 = stepper.pio_pos1() * stepper.CM_PER_STEP
        d2 = stepper.pio_pos2() * stepper.CM_PER_STEP
        toon([
            titel,
            f"doel: {doel_cm:6.1f} cm",
            f"A: {d1:6.1f} cm",
            f"B: {d2:6.1f} cm",
        ])
        print(f"{titel}: doel={doel_cm:.1f} cm  A={d1:.2f} cm  B={d2:.2f} cm")
        time.sleep_ms(100)

    d1 = stepper.pio_pos1() * stepper.CM_PER_STEP
    d2 = stepper.pio_pos2() * stepper.CM_PER_STEP
    toon([titel, f"doel: {doel_cm:6.1f} cm", f"A: {d1:6.1f} cm KLAAR", f"B: {d2:6.1f} cm KLAAR"])
    print(f"{titel} klaar: doel={doel_cm:.1f} cm  A={d1:.2f} cm  B={d2:.2f} cm")
    time.sleep(1)


toon(["Stepper test", "start over 2s..."])
print("Stepper test: basisfuncties (enable/disable/status/reset) + bewegingen")
time.sleep(2)

try:
    # Setwaarden/basisfuncties: teller resetten, motoren inschakelen, status tonen
    stepper.reset_PIO_distance()
    stepper.enable()
    stepper.status()

    print("--- s1: alleen motor A, 10 cm vooruit ---")
    stepper.s1('f', SNELHEID_CM_S, 10.0)
    wacht_tot_klaar("STEPPER s1(f)", 10.0)

    print("--- s2: alleen motor B, 10 cm vooruit ---")
    stepper.s2('f', SNELHEID_CM_S, 10.0)
    wacht_tot_klaar("STEPPER s2(f)", 10.0)

    print("--- mov: beide motoren, 10 cm achteruit ---")
    stepper.mov('b', SNELHEID_CM_S, 10.0)
    wacht_tot_klaar("STEPPER mov(b)", 10.0)

    print("--- rotate: 30 cm naar links ---")
    stepper.rotate('l', SNELHEID_CM_S, 30.0)
    wacht_tot_klaar("STEPPER rotate(l)", 30.0)

    stepper.status()
    stepper.distance()

    toon(["Stepper test", "Alle functies OK"])
    print("Alle stepperfuncties getest.")
except KeyboardInterrupt:
    stepper.stop()
    toon(["Stepper test", "Gestopt."])
    print("Test gestopt.")
finally:
    stepper.disable()
