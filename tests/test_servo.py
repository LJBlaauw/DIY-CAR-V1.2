import time
import machine
from servo_crl import ServoController
from ssd1306 import SSD1306_I2C

# --- Testscript: Servo (elk naar 90 graden, in botsingsveilige volgorde) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
#
# MECHANISCHE RANDVOORWAARDE (2026-09-19, gebruiker): staat servo 1 (schouder,
# J1) in rust, dan maakt servo 2 (elleboog, J2) al bij >ca. 30 graden vanuit
# ZIJN rust contact met het controlbord (PCB). Dit is een fysieke botsing,
# geen timing-kwestie -- de servo's bewegen precies zoals gecommandeerd.
#
# Veilige volgorde, in beide richtingen:
#   heen : J1 eerst naar 90 graden (uit de weg), DAARNA pas J2 naar 90 graden.
#   terug: J2 EERST terug naar rust (J1 staat dan nog op 90, dus veilig),
#          J1 DAARNA pas terug naar rust.
# Nooit J1 op rust laten staan terwijl J2 >30 graden van zijn rust af staat --
# dat geldt voor de TOESTAND, ongeacht welke servo als laatste bewoog.
#
# Servo 4 (J10, grijper) heeft ANDERE beperkingen (eigen open/dicht-bereik,
# stroomlimiet via servo_cur_limit) en zit niet in deze botsingsketen; die
# wordt onafhankelijk getest, als laatste.
#
# Vereist de fysieke koppeling GPIO2 -> GPIO5 (PWM-tick) zoals beschreven in
# lib/servo/servo_crl.py.

TEST_HOEK_GRADEN = 90.0
SNELHEID_GRAD_S = 20.0
TOLERANTIE_GRAD = 1.0
# Ruim boven de langst mogelijke beweging: 180 graden bij SNELHEID_GRAD_S=20
# duurt 9,0 s.
TIMEOUT_MS = 12000

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
            # LET OP: servo nr heeft zijn doel NIET bereikt. Gezien de
            # botsingsveilige volgorde hierboven kan dat betekenen dat de
            # volgende stap onveilig is (bv. J1 nog niet echt op 90 graden
            # terwijl J2 al beweegt) -- dus stoppen en controleren, niet
            # zomaar doorgaan.
            print(f"! Servo {nr} timeout: doel={s['target_deg']:.1f} "
                  f"pos={s['pos_deg']:.1f} (niet bereikt binnen {TIMEOUT_MS} ms)")
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


def naar_hoek(nr, label):
    print(f"--- Servo {nr} ({label}): naar {TEST_HOEK_GRADEN} graden ---")
    sc.servo_pos(nr, TEST_HOEK_GRADEN, SNELHEID_GRAD_S)
    wacht_tot_klaar(sc, nr)
    s = sc.servos[nr]
    print(f"Servo {nr} ({label}): pos={s['pos_deg']:.1f} graden")


def naar_rust(nr, label):
    print(f"--- Servo {nr} ({label}): terug naar rust ---")
    sc.servo_rest(nr, SNELHEID_GRAD_S)
    wacht_tot_klaar(sc, nr)
    s = sc.servos[nr]
    print(f"Servo {nr} ({label}): pos={s['pos_deg']:.1f} graden")


try:
    # HEEN: J1 eerst weg van rust, pas dan J2 -- zie de uitleg bovenaan.
    naar_hoek(1, "schouder")
    naar_hoek(2, "elleboog")
    naar_hoek(4, "grijper")
    time.sleep(1)

    toon(["Servo test", "Alle servos OK"])
    print("Alle servo's op testhoek.")
except KeyboardInterrupt:
    toon(["Servo test", "Gestopt."])
    print("Test gestopt.")
finally:
    # TERUG: exact omgekeerde, botsingsveilige volgorde -- J2 EERST terug
    # naar rust (J1 staat dan nog op de testhoek, dus veilig), J1 DAARNA.
    # Grijper (J4) is onafhankelijk.
    try:
        naar_rust(2, "elleboog")
        naar_rust(1, "schouder")
        naar_rust(4, "grijper")
    except KeyboardInterrupt:
        print("Nogmaals onderbroken tijdens opruimen; controleer de servoposities handmatig.")
