import time
import machine
from servo_crl import ServoController
from ssd1306 import SSD1306_I2C

# --- Testscript: Servo (elk naar 90 graden, in botsingsveilige volgorde) ---
# OLED (SSD1306): GPIO0 (SDA0) / GPIO1 (SCL0) -> I2C-bus 0
#
# MECHANISCHE RANDVOORWAARDEN -- TWEE, TEGENSTRIJDIGE BOTSINGEN:
#
# 1) (2026-09-19, gebruiker) Staat servo 1 (schouder, J1) in rust, dan maakt
#    servo 2 (elleboog, J2) al bij >ca. 30 graden vanuit ZIJN rust contact
#    met het controlbord (PCB).
# 2) (2026-10-31, gebruiker, hardwaretest tot 180 graden) Beweegt J1 alléén
#    naar een grote hoek terwijl J2 nog in rust staat, dan loopt de hand (de
#    grijperservo zit onder de vingers) tegen de ULTRASOONSENSOR aan. Zodra
#    J2 ook beweegt komt hij vanzelf los.
#
# Deze twee sluiten een EENZIJDIGE volgorde (eerst de ene servo helemaal,
# dan de andere) uit in beide richtingen: "J1 rust + J2 weg" botst op het
# controlbord, "J1 weg + J2 rust" botst op de ultrasoon. De enige veilige
# weg is dus J1 en J2 GELIJKTIJDIG bewegen (zelfde starttijd, zelfde
# snelheid/aankomsttijd), nooit de ene laten uitlopen voordat de andere
# start.
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


def naar_hoek_samen():
    """J1 (schouder) en J2 (elleboog) GELIJKTIJDIG naar TEST_HOEK_GRADEN --
    zie de botsingsrandvoorwaarden bovenaan: geen van beide mag op de ander
    vooruitlopen."""
    print(f"--- Servo 1+2 (schouder+elleboog): gelijktijdig naar {TEST_HOEK_GRADEN} graden ---")
    sc.servo_pos(1, TEST_HOEK_GRADEN, SNELHEID_GRAD_S)
    sc.servo_pos(2, TEST_HOEK_GRADEN, SNELHEID_GRAD_S)
    wacht_tot_klaar(sc, 1)
    wacht_tot_klaar(sc, 2)
    print(f"Servo 1: pos={sc.servos[1]['pos_deg']:.1f}  Servo 2: pos={sc.servos[2]['pos_deg']:.1f} graden")


def naar_rust_samen():
    """J1 en J2 GELIJKTIJDIG terug naar rust -- zelfde reden als hierboven."""
    print("--- Servo 1+2 (schouder+elleboog): gelijktijdig terug naar rust ---")
    sc.servo_rest(1, SNELHEID_GRAD_S)
    sc.servo_rest(2, SNELHEID_GRAD_S)
    wacht_tot_klaar(sc, 1)
    wacht_tot_klaar(sc, 2)
    print(f"Servo 1: pos={sc.servos[1]['pos_deg']:.1f}  Servo 2: pos={sc.servos[2]['pos_deg']:.1f} graden")


try:
    # HEEN: J1 en J2 gelijktijdig, dan pas de grijper (onafhankelijk, geen
    # deel van deze botsingsketen) -- zie de uitleg bovenaan.
    naar_hoek_samen()
    naar_hoek(4, "grijper")
    time.sleep(1)

    toon(["Servo test", "Alle servos OK"])
    print("Alle servo's op testhoek.")
except KeyboardInterrupt:
    toon(["Servo test", "Gestopt."])
    print("Test gestopt.")
finally:
    # TERUG: grijper eerst (onafhankelijk), dan J1+J2 weer gelijktijdig.
    try:
        naar_rust(4, "grijper")
        naar_rust_samen()
    except KeyboardInterrupt:
        print("Nogmaals onderbroken tijdens opruimen; controleer de servoposities handmatig.")
