# ================================================================
# tests/test_autodrive.py
#
# Echte autonome missie (opvolger van tests/test_drive.py, dat een
# gevalideerde integratietest/simulatie blijft -- geen echt voorwerp nodig,
# vaste grijphoek). Dit script pakt een ECHT voorwerp:
#
#   1) Scan 370 graden, richt de kar op de lichtbron (null-seek), laser aan.
#   2) Rij naar de lichtbron, met LDR+gyro-bijsturing (koers) en een
#      obstakelstop op de VEILIGSTE schatting uit ultrasoon+ToF samen (zie
#      _obstakel_afstand() hieronder) -- de afstand mag nu groter dan 1 meter
#      zijn, zie CRUISE_AFSTAND_CM. Daarna een stilstaande fijnpositionering
#      (lib/gripper/approach.finetune()) naar de exacte grijpafstand.
#   3) Pak het voorwerp: arm uitklappen (J1+J2 gelijktijdig, schuift vanzelf
#      over het voorwerp heen) en de grijper sluiten op STROOMREGELING
#      (contactdetectie i.p.v. een vaste hoek voor een aangenomen breedte --
#      vergevender bij een onbekende/afwijkende objectbreedte), dan arm
#      terugtrekken.
#   4) Draai de kar 180 graden en rij terug naar het startpunt, met
#      kompas-koershoud (de LDR kijkt nu van de lichtbron af, dus A-B is hier
#      geen bruikbaar stuursignaal meer -- zie stepper_ramp.HeadingController).
#   5) Strek de arm en open de grijper.
#
# Tijdens de hele rit toont de OLED (I2C0, GPIO0/1) vijf regels:
#   1: huidige stap ("1 scan" .. "5 open")
#   2: ultrasoon- EN ToF-afstand   "U= xx.x T= xx.x" (beide in cm; ToF op
#      dezelfde I2C0-bus als de OLED, net als in tests/test_TOF.py)
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
# op 1 en 2 in dezelfde instructie, gevolgd door wacht_arm_klaar() op beide.
#
# Vereist: GPIO2 -> GPIO5 fysiek verbonden (servo-PWM-tick, voor read_synced()
# en de LDR-fase-sync). Zie lib/servo/servo_crl.py.
#
# NOG NIET OP HARDWARE GEVERIFIEERD (nieuw t.o.v. test_drive.py, 2026-11-12):
# de fijnpositionering (finetune), de stroomgeregelde grip, en de
# ultrasoon+ToF-fusieregel voor de obstakelstop. Alle bijbehorende constanten
# hieronder zijn eerste schattingen.
# ================================================================

import time
import math
import machine
from machine import ADC
from ssd1306 import SSD1306_I2C

import stepper_ramp as sr
import ldr_scan_isr as ldr
import ultrasoon
import approach
from servo_crl import ServoController
from icm20948 import ICM20948, load_mag_calibration
from vl53l1x import VL53L1X, LONG, RANGE_VALID
import geometry as geo

# ---------------- Instellingen ----------------
SCAN_RICHTING = 'l'
SCAN_SNELHEID_CM_S = 5.0

# Nu > 1 m toegestaan: de daadwerkelijke stop gebeurt via _obstakel_afstand()
# hieronder (ultrasoon+ToF), dit is alleen de bovengrens/het "geef niet op"-
# plafond als er nergens een object gevonden wordt.
CRUISE_AFSTAND_CM = 300.0
CRUISE_SNELHEID_CM_S = sr.V_MAX_CM_S

ARM_SNELHEID_GRAD_S = 20.0
GRIJPER_SNELHEID_GRAD_S = 20.0
ARM_REIK_GRAD = 20.0               # schouder+elleboog 20 graden vanaf rust om te pakken
                                   # -- gelijktijdig bewegen laat de grijper vanzelf over
                                   # het voorwerp schuiven (gebruiker, 2026-11-12)
ARM_TERUGTREK_GRAD = 160.0         # exact zoals gevraagd; trekt de arm bij deze
                                   # reikhoek in de praktijk helemaal naar rust
                                   # terug (servo_pos klemt onder 0 graden af)
ARM_STREK_GRAD = 180.0             # volledig gestrekt voor stap 5

# Stroomgeregelde grip (contactdetectie i.p.v. een vaste hoek voor een
# aangenomen breedte, gebruiker 2026-11-12). GRIJP_STROOM_MA ruim onder de
# bestaande veiligheidslimiet (zie sc.servo_cur_limit(200) hieronder) --
# eerste schatting, nog niet op hardware geverifieerd.
GRIJP_STROOM_MA = 150.0
GRIJP_DUUR_MS = 3000               # tijd om de stroom-PI te laten convergeren
GRIJP_MISGREEP_MARGE_GRAD = 8.0    # bijna volledig dicht zonder weerstand = waarschijnlijk niets gegrepen

# Obstakelstop tijdens de cruise (stap 2): kortste/veiligste van ultrasoon+ToF
# tegen STOP_CM, zie _obstakel_afstand() hieronder voor de fusieregel.
STOP_CM = geo.STOP_DIST_CM
US_TOF_AGREE_MARGE_CM = 5.0        # 'ongeveer hetzelfde'-marge; eerste schatting

# LDR-weergave als relatieve VERLICHTINGSSTERKTE (niet spanning, niet de
# interne log-weerstandschaal): R ~ E^-GAMMA (ldr.GAMMA, config.json sectie
# "ldr", gemeten 2026-11-01 met tests/test_ldr_beam.py), dus E ~ R^(-1/GAMMA).
# Hergebruikt ldr._adc_to_res_ohm() voor de ADC->weerstand-stap; alleen de
# machtswet en de normalisatie naar 0..100% (tussen LDR_R_MIN_OHM en
# LDR_R_MAX_OHM) komen erbij.
LDR_PIN_A = 26
LDR_PIN_B = 27
_adc_ldr_a = ADC(LDR_PIN_A)
_adc_ldr_b = ADC(LDR_PIN_B)
_E_REL_MIN = ldr.LDR_R_MAX_OHM ** (-1.0 / ldr.GAMMA)   # bij donker (R max) -> laagste E
_E_REL_MAX = ldr.LDR_R_MIN_OHM ** (-1.0 / ldr.GAMMA)   # bij fel licht (R min) -> hoogste E


def _ldr_licht_pct(raw_u16):
    r_ohm = ldr._adc_to_res_ohm(raw_u16)
    e_rel = r_ohm ** (-1.0 / ldr.GAMMA)
    pct = 100.0 * (e_rel - _E_REL_MIN) / (_E_REL_MAX - _E_REL_MIN)
    if pct < 0.0:
        pct = 0.0
    if pct > 100.0:
        pct = 100.0
    return pct

ROTATIE_GRADEN = 180.0
ROTATIE_SNELHEID_CM_S = sr.V_MAX_CM_S / 2.0

# De kar draait om de wielas, niet om de ultrasoon aan de voorkant. Het
# zwenkwiel zit 150mm ACHTER de wielas (zie globale_specificatie_NL.md ->
# Afmetingen), de ultrasoon zit maar ~80mm VOOR de as (230mm totale lengte -
# 150mm). Bij een 180-graden-draai op de plek beschrijft de achterkant van de
# kar dus een veel grotere boog dan de voorkant bij het stoppen gebruikt --
# met de breedte van de kar erbij (de hoeken steken nog verder uit) is de
# marge bij STOP_DIST_CM te klein en stoot de achterkant de lichtbron om.
# Ruim geschatte, nog niet op hardware geverifieerde marge -- controleer en
# stel bij als de achterkant nog te dicht langs het voorwerp zwaait.
TERUG_VOOR_DRAAIEN_CM = 15.0

TOLERANTIE_GRAD = 1.0
SERVO_TIMEOUT_MS = 12000

# Kompas-koershoud voor de terugrit (stap 4): HeadingController accepteert
# elke callable als "ldr_diff" -- hier geven we 'm een kompas-koersfout in
# GRADEN i.p.v. een LDR-logverschil, dus een eigen, veel kleinere kp dan de
# LDR-kp_ldr=25,0 (die rekent in een heel andere, niet-lineaire eenheid).
# Beide waarden zijn een eerste schatting, NOG NIET op hardware geverifieerd.
# KOMPAS_DEADBAND_GRAD ruim boven de gemeten kompasruis houden (±1 uT
# run-tot-run ~= 3 graden, zie config.json -> "icm20948"/globale_specificatie
# -> IMU-sectie), anders regelt de controller op ruis i.p.v. een echte fout.
KOMPAS_KP = 1.5             # graden/s gecommandeerd per graad koersfout
KOMPAS_DEADBAND_GRAD = 3.0

# ---------------- OLED ----------------
i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                       freq=400000, timeout=200_000)
oled = SSD1306_I2C(128, 64, i2c_oled)

# ---------------- ToF (VL53L1X, zelfde I2C0-bus als de OLED) ----------------
# Zelfde instellingen als tests/test_TOF.py (33ms timing budget, 50ms periode).
# Wordt hier WEL gebruikt in de obstakelstop (zie _obstakel_afstand()), anders
# dan in tests/test_drive.py waar het alleen display is.
TOF_TIMING_BUDGET_US = 33_000
TOF_PERIOD_MS = 50
tof = VL53L1X(i2c_oled)
_tof_ok = tof.init()
if _tof_ok:
    tof.set_distance_mode(LONG)
    tof.set_timing_budget_us(TOF_TIMING_BUDGET_US)
    tof.start_continuous(TOF_PERIOD_MS)
else:
    print("! VL53L1X.init() mislukt -- ToF-afstand blijft leeg, obstakelstop valt terug op ultrasoon alleen")
_tof_last_cm = None

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
    global _tof_last_cm
    cm, kind = ultrasoon.read_cm()
    us_txt = "%5.1f" % cm if kind == 'ok' else " ----"

    if _tof_ok and tof.data_ready():
        mm = tof.read(blocking=False)
        if tof.range_status == RANGE_VALID:
            _tof_last_cm = mm / 10.0
    tof_txt = "%5.1f" % _tof_last_cm if _tof_last_cm is not None else " ----"

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
    oled.text("U=%s T=%s" % (us_txt, tof_txt), 0, 10)
    oled.text("LRa=%3d%% LRb=%3d%%" % (round(pa), round(pb)), 0, 20)
    oled.text(regel4, 0, 30)
    oled.text(regel5, 0, 40)
    oled.show()


def _obstakel_afstand():
    """Veiligste schatting van de afstand recht vooruit, uit ultrasoon+ToF.

    De ultrasoon geeft bij grotere afstand soms een vals-korte meting (de
    brede bundel raakt de vloer en kaatst terug) -- gevaarlijk, want dat
    veroorzaakt een voortijdige stop. Regel (gebruiker, 2026-11-12): geeft de
    ToF een GROTERE afstand aan dan de ultrasoon, dan is de ultrasoon
    waarschijnlijk zo'n vloer-bounce-artefact -- vertrouw dan de ToF. Liggen
    beide ongeveer gelijk (binnen US_TOF_AGREE_MARGE_CM), dan de KLEINSTE van
    de twee als veiligheidsmarge. Is er maar één geldige meting, gebruik die.

    Leest de ToF NIET zelf opnieuw uit (geen extra I2C-verkeer): hergebruikt
    de door update_display() bijgehouden _tof_last_cm, die op deze cruise-
    cadans (elke SLICE_MS//2) altijd recenter is dan de eigen 50ms ToF-periode.

    Eerste schatting, nog niet op hardware geverifieerd.
    """
    us_cm, us_kind = ultrasoon.read_cm()
    us_ok = (us_kind == 'ok')
    tof_ok = _tof_last_cm is not None

    if us_ok and tof_ok:
        if _tof_last_cm > us_cm + US_TOF_AGREE_MARGE_CM:
            return _tof_last_cm
        return min(us_cm, _tof_last_cm)
    if tof_ok:
        return _tof_last_cm
    if us_ok:
        return us_cm
    return None


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


def grijp_op_stroom(sc, mA=GRIJP_STROOM_MA, snelheid=GRIJPER_SNELHEID_GRAD_S,
                     duur_ms=GRIJP_DUUR_MS):
    """Sluit de grijper met stroomregeling (contactdetectie) i.p.v. een vaste
    hoek voor een aangenomen breedte -- vergevender bij een onbekende/
    afwijkende objectbreedte (gebruiker, 2026-11-12).

    servo_cur() negeert de positie en regelt op stroom; na duur_ms (tijd om
    de PI te laten convergeren) schakelen we terug naar gewone positie-
    holding op de dan bereikte hoek (GEEN doorlopende stroomregeling tijdens
    het vervoeren -- eenvoudiger en de PWM-positiehold houdt het voorwerp
    vast net als elke andere servo-stand).

    Geeft (eindhoek_graden, waarschijnlijk_gelukt) terug. "Waarschijnlijk
    gelukt" is een ruwe heuristiek (bijna volledig dicht zonder vroegtijdige
    convergentie = waarschijnlijk niets gegrepen), geen harde garantie --
    er is geen echte krachtsensor, alleen de servostroom.
    """
    sc.servo_cur(mA, snelheid)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < duur_ms:
        update_display(sc)
        time.sleep_ms(50)
    sc.stop_cur()
    eind_deg = _servo_graden(sc, 4)
    misgreep = eind_deg > (180.0 - GRIJP_MISGREEP_MARGE_GRAD)
    print("grijpen op stroom: eindhoek=%.1f graden%s"
          % (eind_deg, "  [WAARSCHUWING: mogelijk MISGREEP, weinig weerstand gevoeld]"
             if misgreep else ""))
    return eind_deg, not misgreep


def _kompas_heading_deg(sensor):
    """Ruwe, al-gekalibreerde kompaskoers in graden (0-360), zelfde methode
    als tests/test_kompas_richtingen.py en test_webserver_rc.py: de mount-/
    hard-/soft-iron-correctie zit al in `sensor.magnetic` (load_mag_calibration())."""
    mx, my, _mz = sensor.magnetic
    return math.degrees(math.atan2(my, mx)) % 360.0


def _hoekverschil(a, b):
    """a - b, gewrapt naar -180..180 (zelfde helper als test_kompas_richtingen.py)."""
    return (a - b + 540) % 360 - 180


def _kompas_fout_fn(sensor, doel_heading_deg):
    """Callable voor HeadingController(ldr_diff=...): positief = doel ligt
    rechts van de huidige koers (moet naar rechts bijsturen)."""
    def _f():
        return _hoekverschil(doel_heading_deg, _kompas_heading_deg(sensor))
    return _f


# ---------------- Hoofdprogramma ----------------
def main():
    sc = ServoController()
    sc.servo_cur_limit(200)   # stroomlimiet grijper, net als in test_all.py/test_servo.py
                              # (geldt in POSITIE-modus; grijp_op_stroom() hieronder
                              # schakelt tijdelijk naar SETPOINT-modus, zie servo_crl.py)

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

        # --- 2) Cruise met LDR+gyro-bijsturing, obstakelstop op ultrasoon+ToF ---
        _zet_stap("2 cruise")
        _zet_mode("MS", CRUISE_SNELHEID_CM_S)
        hc = sr.HeadingController(ldr_diff=ldr.read_synced_diff, gyro_rate=gyro_rate)
        print("--- Stap 2: cruise max %.0f cm naar de lichtbron (stop <= %.1f cm) ---"
              % (CRUISE_AFSTAND_CM, STOP_CM))
        sr.reset_PIO_distance()
        mv = sr.drive(CRUISE_AFSTAND_CM, CRUISE_SNELHEID_CM_S, correction=hc)
        afgebroken = False
        while mv.service():
            update_display(sc)   # ZET VOOR de obstakelcheck: ververst _tof_last_cm
            if not afgebroken:
                d = _obstakel_afstand()
                if d is not None and d <= STOP_CM:
                    print("Obstakelstop: %.1f cm <= %.1f cm" % (d, STOP_CM))
                    mv.finish()
                    afgebroken = True
            time.sleep_ms(sr.SLICE_MS // 2)
        update_display(sc)
        afstand_gereden = sr.distance()   # werkelijke afstand, ook als de obstakelstop eerder afbrak
        print("cruise klaar: afstand=%.2f heading=%.2f" % (afstand_gereden, sr.heading()))

        # --- 2b) Fijnpositioneren: stilstaand naar de exacte grijpafstand kruipen ---
        _zet_stap("2b fijn")
        update_display(sc)
        print("--- Stap 2b: fijnpositioneren (object_w_cm=%.1f) ---" % geo.OBJECT_W_CM)
        d_fijn, doel_fijn, gelukt_fijn = approach.finetune(
            ultrasoon.read_cm, object_w_cm=geo.OBJECT_W_CM)
        if not gelukt_fijn:
            print("! Fijnpositionering BUITEN VENSTER (gemeten=%s, doel=%.2f) -- grijppoging gaat toch door"
                  % (d_fijn, doel_fijn))
        update_display(sc)

        # --- 3) Pakken: arm uitklappen (schuift over het voorwerp), dan
        #        grijper sluiten op stroomregeling (contactdetectie) ---
        _zet_stap("3 pak")
        _zet_mode("MS", 0.0)
        update_display(sc)
        print("--- Stap 3: grijpen ---")
        arm_naar(sc, ARM_REIK_GRAD)
        eind_deg, gelukt_grijp = grijp_op_stroom(sc)
        if not gelukt_grijp:
            print("! Grijpen waarschijnlijk MISLUKT -- ga toch door met de missie")
        time.sleep(1)
        print("--- arm %.0f graden terugtrekken ---" % ARM_TERUGTREK_GRAD)
        arm_naar(sc, ARM_REIK_GRAD - ARM_TERUGTREK_GRAD)

        # --- 4) Eerst achteruit (zie TERUG_VOOR_DRAAIEN_CM hierboven), dan
        #        180 graden draaien en terugrijden naar start ---
        _zet_stap("4 terug")
        _zet_mode("MB", CRUISE_SNELHEID_CM_S)
        update_display(sc)
        print("--- Stap 4a: %.0f cm achteruit (ruimte voor de draai) ---" % TERUG_VOOR_DRAAIEN_CM)
        sr.reset_PIO_distance()
        mv = sr.drive(TERUG_VOOR_DRAAIEN_CM, CRUISE_SNELHEID_CM_S, forward=False)
        while mv.service():
            update_display(sc)
            time.sleep_ms(sr.SLICE_MS // 2)
        update_display(sc)

        _zet_mode("MR", ROTATIE_SNELHEID_CM_S)
        update_display(sc)
        print("--- Stap 4b: %.0f graden draaien ---" % ROTATIE_GRADEN)
        sr.reset_PIO_distance()
        sr.rotate_deg(ROTATIE_GRADEN, ROTATIE_SNELHEID_CM_S)
        while sr.busy():
            update_display(sc)
            time.sleep_ms(20)
        update_display(sc)
        print("draai klaar: heading=%.2f" % sr.heading())

        # Kompasdoel voor de terugrit: niet de (voor-draai-heading + 180)
        # BEREKENEN -- dat zou een kompasmeetfout EN een draaifout bij elkaar
        # optellen. In plaats daarvan: na een korte settle-pauze gewoon meten
        # waar de kar nu daadwerkelijk heen wijst, en DIE koers de hele
        # terugrit vasthouden.
        time.sleep_ms(200)
        doel_heading_terugweg = _kompas_heading_deg(sensor)
        print("kompasdoel terugweg: %.1f graden" % doel_heading_terugweg)

        # Terug naar het ECHTE startpunt: de heenweg (afstand_gereden, kan
        # korter zijn dan CRUISE_AFSTAND_CM als de obstakelstop eerder
        # afbrak -- wat bij een normale missie ALTIJD het geval is) minus de
        # TERUG_VOOR_DRAAIEN_CM die net al achteruit is gereden -- anders
        # schiet de kar er 15cm voorbij.
        terugrit_cm = max(0.0, afstand_gereden - TERUG_VOOR_DRAAIEN_CM)
        _zet_mode("MF", CRUISE_SNELHEID_CM_S)
        print("--- terugrijden %.0f cm naar het startpunt (kompas-koershoud, doel=%.1f graden) ---"
              % (terugrit_cm, doel_heading_terugweg))
        hc_terug = sr.HeadingController(
            ldr_diff=_kompas_fout_fn(sensor, doel_heading_terugweg),
            gyro_rate=gyro_rate,
            kp_ldr=KOMPAS_KP, deadband=KOMPAS_DEADBAND_GRAD)
        # GEEN LDR-bijsturing meer: de kar kijkt nu van de lichtbron AF, dus
        # A-B zou hier het verkeerde stuursignaal geven (zie de kop).
        sr.reset_PIO_distance()
        mv = sr.drive(terugrit_cm, CRUISE_SNELHEID_CM_S, correction=hc_terug)
        while mv.service():
            update_display(sc)
            time.sleep_ms(sr.SLICE_MS // 2)
        update_display(sc)
        print("terug klaar: afstand=%.2f eindheading=%.1f (doel %.1f, fout %.1f)"
              % (sr.distance(), _kompas_heading_deg(sensor), doel_heading_terugweg,
                 _hoekverschil(_kompas_heading_deg(sensor), doel_heading_terugweg)))

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
        # Veilig terug: eerst stroomregeling loslaten (voor het geval een
        # onderbreking tijdens grijp_op_stroom() gebeurde), dan grijper naar
        # rust (onafhankelijk), dan J1+J2 gelijktijdig naar rust, laser uit,
        # LDR-sync los -- zie servo_rest()/test_servo.py.
        try:
            sc.stop_cur()
            sc.servo_rest(4, GRIJPER_SNELHEID_GRAD_S)
            wacht_servo_klaar(sc, 4)
            arm_naar(sc, 0.0)
        finally:
            sc.laser_off()
            ldr.stop_synced_sampling(sc)
            _zet_stap("klaar")
            update_display(sc)


main()
