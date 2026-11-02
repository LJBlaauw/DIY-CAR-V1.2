# ================================================================
# tests/test_drive.py
#
# Integratietest: beoordeelt of ALLE modules SAMEN correct functioneren
# (stepper, LDR-scan, ultrasoon, servo/grijper, gyro/kompas, display) in
# één doorlopende missie:
#
#   1) Scan 370 graden, richt de kar op de lichtbron (null-seek), laser aan.
#   2) Rij 1 m vooruit naar de lichtbron, met LDR+gyro-bijsturing (koers).
#   3) "Pak" het artikel (simulatie, geen echt voorwerp nodig) en trek de
#      arm daarna 160 graden terug.
#   4) Draai de kar 180 graden en rij terug naar het startpunt.
#   5) Strek de arm en open de grijper.
#
# Tijdens de hele rit toont de OLED (I2C0, GPIO0/1) vijf regels:
#   1: huidige stap ("1 scan" .. "5 open")
#   2: ultrasoon-afstand           "US =  xx.x cm"
#   3: LDR A/B als relatieve verlichtingssterkte (E ~ R^(-1/GAMMA), 0-100%
#      genormaliseerd; geen absolute lux) "LRa= xx% LRb= xx%"
#   4: stepper-modus+snelheid+afstand van het huidige segment
#      "MF xx.x cm/s xxxcm" (MF=vooruit, MB=achteruit, MR=rotatie, MS=sturen)
#   5: servo's, graden t.o.v. rust "S=xxx E=xxx G=xxx"
#      (S=schouder/J1, E=elleboog/J2, G=grijper/J10)
#
# BOTSINGSRANDVOORWAARDE (zie tests/test_servo.py en lib/servo/servo_crl.py):
# J1 (schouder) en J2 (elleboog) MOETEN altijd GELIJKTIJDIG bewegen, nooit de
# ene eerst laten uitlopen -- anders botst het op het controlbord of de
# ultrasoonsensor. Alle arm-bewegingen hieronder gaan daarom via sc.servo_pos
# op 1 en 2 in dezelfde instructie, gevolgd door wacht_tot_klaar() op beide.
#
# Vereist: GPIO2 -> GPIO5 fysiek verbonden (servo-PWM-tick, voor read_synced()
# en de LDR-fase-sync). Zie lib/servo/servo_crl.py.
# ================================================================

import time
import math
import machine
from machine import ADC
from ssd1306 import SSD1306_I2C

import stepper_ramp as sr
import ldr_scan_isr as ldr
import ultrasoon
from servo_crl import ServoController
from icm20948 import ICM20948, load_mag_calibration
import geometry as geo

# ---------------- Instellingen ----------------
SCAN_RICHTING = 'l'
SCAN_SNELHEID_CM_S = 5.0

CRUISE_AFSTAND_CM = 100.0          # "1 meter"
CRUISE_SNELHEID_CM_S = sr.V_MAX_CM_S

ARM_SNELHEID_GRAD_S = 20.0
GRIJPER_SNELHEID_GRAD_S = 20.0
ARM_REIK_GRAD = 20.0               # schouder+elleboog 20 graden vanaf rust om te "pakken"
ARM_TERUGTREK_GRAD = 160.0         # exact zoals gevraagd; trekt de arm bij deze
                                   # reikhoek in de praktijk helemaal naar rust
                                   # terug (servo_pos klemt onder 0 graden af)
ARM_STREK_GRAD = 180.0             # volledig gestrekt voor stap 5

# Ultrasoon-stopconditie tijdens de cruise (stap 2): stop zodra het voorwerp
# dichterbij is dan STOP_DIST_CM, anders de volle CRUISE_AFSTAND_CM (1 m).
STOP_CM = geo.STOP_DIST_CM

# LDR-weergave als relatieve VERLICHTINGSSTERKTE (niet spanning, niet de
# interne log-weerstandschaal): R ~ E^-GAMMA (gemeten 2026-11-01, zie
# tests/test_ldr_beam.py), dus E ~ R^(-1/GAMMA). Hergebruikt
# ldr._adc_to_res_ohm() voor de ADC->weerstand-stap; alleen de machtswet en de
# normalisatie naar 0..100% (tussen LDR_R_MIN_OHM en LDR_R_MAX_OHM) komen erbij.
# Geeft, anders dan een letterlijke spanningslineaire %, een schaal die recht
# evenredig is met het licht zelf -- voor buitenstaanders nog steeds een
# simpel percentage, maar natuurkundig het juiste begrip ("verlichtingssterkte",
# niet "spanning" of "weerstand"). GEEN absolute lux: zonder een lux-referentie-
# meting is alleen de 0..100%-schaal bekend, niet de absolute eenheid.
LDR_PIN_A = 26
LDR_PIN_B = 27
GAMMA = 0.55   # LDR-exponent, gemeten 2026-11-01 (tests/test_ldr_beam.py)
_adc_ldr_a = ADC(LDR_PIN_A)
_adc_ldr_b = ADC(LDR_PIN_B)
_E_REL_MIN = ldr.LDR_R_MAX_OHM ** (-1.0 / GAMMA)   # bij donker (R max) -> laagste E
_E_REL_MAX = ldr.LDR_R_MIN_OHM ** (-1.0 / GAMMA)   # bij fel licht (R min) -> hoogste E


def _ldr_licht_pct(raw_u16):
    r_ohm = ldr._adc_to_res_ohm(raw_u16)
    e_rel = r_ohm ** (-1.0 / GAMMA)
    pct = 100.0 * (e_rel - _E_REL_MIN) / (_E_REL_MAX - _E_REL_MIN)
    if pct < 0.0:
        pct = 0.0
    if pct > 100.0:
        pct = 100.0
    return pct

ROTATIE_GRADEN = 180.0
ROTATIE_SNELHEID_CM_S = sr.V_MAX_CM_S / 2.0

TOLERANTIE_GRAD = 1.0
SERVO_TIMEOUT_MS = 12000

# ---------------- OLED ----------------
i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                       freq=400000, timeout=200_000)
oled = SSD1306_I2C(128, 64, i2c_oled)

# ---------------- Gedeelde status voor het display ----------------
_stap_tekst = "0 start"
_mode_label = "MF"
_mode_snelheid = 0.0


def _zet_stap(tekst):
    global _stap_tekst
    _stap_tekst = tekst


def _zet_mode(label, snelheid_cm_s):
    global _mode_label, _mode_snelheid
    _mode_label = label
    _mode_snelheid = snelheid_cm_s


def _servo_graden(sc, nr):
    s = sc.servos[nr]
    rest_deg = sc._pct_to_deg(nr, sc.rest_pct[nr])
    return s["pos_deg"] - rest_deg


def update_display(sc):
    cm, kind = ultrasoon.read_cm()
    us_txt = "US = %5.1f cm" % cm if kind == 'ok' else "US = ----"

    pa = _ldr_licht_pct(_adc_ldr_a.read_u16())
    pb = _ldr_licht_pct(_adc_ldr_b.read_u16())

    afstand = sr.distance()
    regel4 = "%s %4.1f cm/s %3dcm" % (_mode_label, _mode_snelheid, round(afstand))

    s_s = _servo_graden(sc, 1)
    s_e = _servo_graden(sc, 2)
    s_g = _servo_graden(sc, 4)
    regel5 = "S=%3d E=%3d G=%3d" % (round(s_s), round(s_e), round(s_g))

    oled.fill(0)
    oled.text(_stap_tekst, 0, 0)
    oled.text(us_txt, 0, 10)
    oled.text("LRa=%3d%% LRb=%3d%%" % (round(pa), round(pb)), 0, 20)
    oled.text(regel4, 0, 30)
    oled.text(regel5, 0, 40)
    oled.show()


# ---------------- Servo-hulpfuncties (botsingsveilig, zie kop) ----------------
def wacht_arm_klaar(sc, duur_timeout_ms=SERVO_TIMEOUT_MS):
    """Wacht tot J1 EN J2 allebei hun doel bereikt hebben, met doorlopende
    display-updates. Nooit de ene servo apart laten uitlopen (botsingsrisico)."""
    t0 = time.ticks_ms()
    s1 = sc.servos[1]
    s2 = sc.servos[2]
    while (abs(s1["pos_deg"] - s1["target_deg"]) > TOLERANTIE_GRAD or
           abs(s2["pos_deg"] - s2["target_deg"]) > TOLERANTIE_GRAD):
        if time.ticks_diff(time.ticks_ms(), t0) > duur_timeout_ms:
            print("! Arm (J1/J2) timeout: doel niet bereikt binnen %d ms" % duur_timeout_ms)
            break
        update_display(sc)
        time.sleep_ms(50)
    update_display(sc)


def wacht_servo_klaar(sc, nr, duur_timeout_ms=SERVO_TIMEOUT_MS):
    t0 = time.ticks_ms()
    s = sc.servos[nr]
    while abs(s["pos_deg"] - s["target_deg"]) > TOLERANTIE_GRAD:
        if time.ticks_diff(time.ticks_ms(), t0) > duur_timeout_ms:
            print("! Servo %d timeout: doel niet bereikt binnen %d ms" % (nr, duur_timeout_ms))
            break
        update_display(sc)
        time.sleep_ms(50)
    update_display(sc)


def arm_naar(sc, graden, snelheid=ARM_SNELHEID_GRAD_S):
    """J1 en J2 GELIJKTIJDIG naar `graden` (relatief t.o.v. rust)."""
    sc.servo_pos(1, graden, snelheid)
    sc.servo_pos(2, graden, snelheid)
    wacht_arm_klaar(sc)


def kaak_hoek_voor_breedte(breedte_cm):
    """Graden (0=open/rust..180=dicht) om de kaken te sluiten tot de gegeven
    breedte, lineair tussen GRIP_OPEN_CM (0 graden) en GRIP_MIN_CM (180
    graden) -- dezelfde aanname als tip_pos_cm() in lib/gripper/geometry.py."""
    f = (geo.GRIP_OPEN_CM - breedte_cm) / (geo.GRIP_OPEN_CM - geo.GRIP_MIN_CM)
    f = max(0.0, min(1.0, f))
    return f * 180.0


# ---------------- Hoofdprogramma ----------------
def main():
    sc = ServoController()
    sc.servo_cur_limit(200)   # stroomlimiet grijper, net als in test_all.py/test_servo.py

    i2c_sensor = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
    sensor = ICM20948(i2c_sensor, **load_mag_calibration())
    gyro_rate = sr.gyro_z_deg_s(sensor)

    ldr.start_synced_sampling(sc)
    time.sleep_ms(200)   # eerste tick(s) laten landen voor read_synced()

    try:
        # --- 1) Scan + richten + laser aan ---
        _zet_stap("1 scan")
        _zet_mode("MR", SCAN_SNELHEID_CM_S)
        update_display(sc)
        print("--- Stap 1: 370-graden scan ---")
        res = ldr.scan(SCAN_RICHTING, SCAN_SNELHEID_CM_S, go_max=True, excel=False)
        print("scan-resultaat:", res)
        sc.laser_power(100.0)
        update_display(sc)
        time.sleep(1)

        # --- 2) Cruise 1 m met LDR+gyro-bijsturing ---
        _zet_stap("2 cruise")
        _zet_mode("MS", CRUISE_SNELHEID_CM_S)
        hc = sr.HeadingController(ldr_diff=ldr.read_synced_diff, gyro_rate=gyro_rate)
        print("--- Stap 2: cruise %.0f cm naar de lichtbron (stop <= %.1f cm ultrasoon) ---"
              % (CRUISE_AFSTAND_CM, STOP_CM))
        sr.reset_PIO_distance()
        mv = sr.drive(CRUISE_AFSTAND_CM, CRUISE_SNELHEID_CM_S, correction=hc)
        afgebroken = False
        while mv.service():
            if not afgebroken:
                cm, kind = ultrasoon.read_cm()
                if kind == 'ok' and cm <= STOP_CM:
                    print("Ultrasoon-stop: %.1f cm <= %.1f cm" % (cm, STOP_CM))
                    mv.finish()
                    afgebroken = True
            update_display(sc)
            time.sleep_ms(sr.SLICE_MS // 2)
        update_display(sc)
        print("cruise klaar: afstand=%.2f heading=%.2f" % (sr.distance(), sr.heading()))

        # --- 3) "Pak" het artikel, dan arm terugtrekken ---
        _zet_stap("3 pak")
        _zet_mode("MS", 0.0)
        update_display(sc)
        print("--- Stap 3: grijpen (simulatie) ---")
        arm_naar(sc, ARM_REIK_GRAD)
        kaak_graden = kaak_hoek_voor_breedte(geo.OBJECT_W_CM)
        sc.servo_pos(4, kaak_graden, GRIJPER_SNELHEID_GRAD_S)
        wacht_servo_klaar(sc, 4)
        time.sleep(1)
        print("--- arm %.0f graden terugtrekken ---" % ARM_TERUGTREK_GRAD)
        arm_naar(sc, ARM_REIK_GRAD - ARM_TERUGTREK_GRAD)

        # --- 4) 180 graden draaien en terugrijden naar start ---
        _zet_stap("4 terug")
        _zet_mode("MR", ROTATIE_SNELHEID_CM_S)
        update_display(sc)
        print("--- Stap 4: %.0f graden draaien ---" % ROTATIE_GRADEN)
        sr.reset_PIO_distance()
        sr.rotate_deg(ROTATIE_GRADEN, ROTATIE_SNELHEID_CM_S)
        while sr.busy():
            update_display(sc)
            time.sleep_ms(20)
        update_display(sc)
        print("draai klaar: heading=%.2f" % sr.heading())

        _zet_mode("MF", CRUISE_SNELHEID_CM_S)
        print("--- terugrijden %.0f cm naar het startpunt ---" % CRUISE_AFSTAND_CM)
        sr.reset_PIO_distance()
        mv = sr.drive(CRUISE_AFSTAND_CM, CRUISE_SNELHEID_CM_S)   # GEEN LDR-bijsturing:
        # de kar kijkt nu van de lichtbron AF, dus A-B zou hier het verkeerde
        # stuursignaal geven.
        while mv.service():
            update_display(sc)
            time.sleep_ms(sr.SLICE_MS // 2)
        update_display(sc)
        print("terug klaar: afstand=%.2f" % sr.distance())

        # --- 5) Arm strekken en grijper openen ---
        _zet_stap("5 open")
        _zet_mode("MF", 0.0)
        update_display(sc)
        print("--- Stap 5: arm strekken, grijper openen ---")
        arm_naar(sc, ARM_STREK_GRAD)
        sc.servo_pos(4, 0.0, GRIJPER_SNELHEID_GRAD_S)
        wacht_servo_klaar(sc, 4)
        time.sleep(1)

        print("Missie klaar.")
    except KeyboardInterrupt:
        print("Test onderbroken.")
    finally:
        # Veilig terug: grijper eerst (onafhankelijk), dan J1+J2 gelijktijdig
        # naar rust, laser uit, LDR-sync los -- zie servo_rest()/test_servo.py.
        try:
            sc.servo_rest(4, GRIJPER_SNELHEID_GRAD_S)
            wacht_servo_klaar(sc, 4)
            arm_naar(sc, 0.0)
        finally:
            sc.laser_off()
            ldr.stop_synced_sampling(sc)
            _zet_stap("klaar")
            update_display(sc)


main()
