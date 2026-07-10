import time
import machine
from servo_crl import ServoController
from ssd1306 import SSD1306_I2C

# --- Testscript: Servo (kleine beweging vanaf 50% van de slag) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
#
# Voor elke servo: ga naar 50% van de slag (90 graden = midden), en maak
# daarna een kleine extra beweging. Vereist de fysieke koppeling
# GPIO2 -> GPIO5 (PWM-tick) zoals beschreven in lib/servo/servo_crl.py.

# servo_crl.py mapt 0 graden -> 2.5% duty en 180 graden -> 12.5% duty,
# dus 90 graden (50% van de slag) = het midden van dat bereik.
MIDDEN_PCT = (2.5 + 12.5) / 2
KLEINE_STAP_GRADEN = 10.0
SNELHEID_GRAD_S = 20.0
TOLERANTIE_GRAD = 1.0
TIMEOUT_MS = 5000

i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c_oled)


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


def wacht_tot_klaar(sc, nr):
    t0 = time.ticks_ms()
    s = sc.servos[nr]
    while abs(s["pos_deg"] - s["target_deg"]) > TOLERANTIE_GRAD:
        if time.ticks_diff(time.ticks_ms(), t0) > TIMEOUT_MS:
            break
        toon([
            f"Servo {nr}",
            f"doel: {s['target_deg']:6.1f}",
            f"pos : {s['pos_deg']:6.1f}",
        ])
        time.sleep_ms(50)
    toon([
        f"Servo {nr} KLAAR",
        f"doel: {s['target_deg']:6.1f}",
        f"pos : {s['pos_deg']:6.1f}",
    ])


toon(["Servo test", "start over 2s..."])
sc = ServoController()
sc.servo_cur_limit(200)  # stroomlimiet voor servo 4 (grijper), net als in test_all.py
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

# Bewaar de originele rustposities van ALLE servo's vooraf, zodat we bij een
# onderbreking (ook halverwege een servo) altijd naar de juiste rustpositie
# kunnen terugkeren, niet naar de tijdelijke 90 graden testpositie.
oorspronkelijke_rest_alles = dict(sc.rest_pct)

try:
    for nr in sc.servo_pins:
        print(f"--- Servo {nr}: naar 50% van de slag (90 graden) ---")
        sc.set_rest_pct(nr, MIDDEN_PCT)
        sc.servo_rest(nr, SNELHEID_GRAD_S)
        wacht_tot_klaar(sc, nr)
        s = sc.servos[nr]
        print(f"Servo {nr} op midden: pos={s['pos_deg']:.1f} graden")

        print(f"--- Servo {nr}: kleine beweging (+{KLEINE_STAP_GRADEN} graden) ---")
        sc.servo_pos(nr, KLEINE_STAP_GRADEN, SNELHEID_GRAD_S)
        wacht_tot_klaar(sc, nr)
        s = sc.servos[nr]
        print(f"Servo {nr} kleine beweging klaar: pos={s['pos_deg']:.1f} graden")
        time.sleep(1)

        # Terug naar de oorspronkelijke rustpositie
        sc.set_rest_pct(nr, oorspronkelijke_rest_alles[nr])
        sc.servo_rest(nr, SNELHEID_GRAD_S)
        wacht_tot_klaar(sc, nr)

    toon(["Servo test", "Alle servos OK"])
    print("Alle servo's getest.")
except KeyboardInterrupt:
    toon(["Servo test", "Gestopt."])
    print("Test gestopt.")
finally:
    # Zet alle servo's terug naar hun oorspronkelijke rustpositie, ook als de
    # test halverwege werd onderbroken (rest_pct kan dan nog op de tijdelijke
    # 90 graden testpositie staan).
    for nr, pct in oorspronkelijke_rest_alles.items():
        sc.set_rest_pct(nr, pct)
    try:
        sc.servo_rest()
    except KeyboardInterrupt:
        print("Nogmaals onderbroken tijdens opruimen; controleer de servoposities handmatig.")
