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
#      (fijnpositioneren() hieronder, zelfde patroon als lib/gripper/
#      approach.finetune() maar met een eigen, empirisch gecorrigeerd doel --
#      zie FIJNPOSITIE_CORRECTIE_CM) naar de exacte grijpafstand.
#   3) Pak het voorwerp: arm uitklappen (J1+J2 gelijktijdig naar ARM_REIK_GRAD,
#      schuift vanzelf over het voorwerp heen) en de grijper sluiten op
#      STROOMREGELING (SETPOINT-modus, regelt door naar dicht totdat de
#      ingestelde stroom bereikt is) i.p.v. een berekende hoek voor een
#      aangenomen breedte -- vergevender bij een onbekende/afwijkende
#      objectbreedte (zie grijp_dicht() voor de geschiedenis van drie
#      eerdere, verworpen aanpakken), dan arm terugtrekken.
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
#   6: gemeten grijperstroom (sc.i_filt_mA, servo 4) "I_grijp=xxxxmA"
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
# HARDWAREGESCHIEDENIS (nieuw t.o.v. test_drive.py, 2026-11-12): meerdere
# volledige missies gedraaid, met een echt flesje (kunststof, diameter 4,5cm,
# OBJECT_W_CM bewust op de standaard 6,0cm gelaten). Obstakelstop
# (ultrasoon+ToF-fusie), kompas-terugrit en de algehele missiestructuur
# werkten steeds foutloos. Drie echte fouten gevonden en gefixt:
#   - ARM_REIK_GRAD=20 (uit test_drive.py overgenomen, daar nooit tegen een
#     echt voorwerp geverifieerd) bracht de vingertop geometrisch maar 1,8cm
#     voor de ultrasoon i.p.v. de benodigde ~12,4cm -- nu 170.
#   - FIJNPOSITIE_CORRECTIE_CM=1.0: de fijnpositionering kwam zonder deze
#     correctie herhaaldelijk ~0,5-1cm buiten venster uit (te ver van het
#     voorwerp). MET de correctie: "OK" bij herhaling (10,72cm en 11,37cm
#     gemeten, beide binnen venster). Waarom finetune()/fijnpositioneren()
#     niet vanzelf convergeerde is niet verder uitgezocht -- hier alleen met
#     een offset omzeild.
#   - grijp_dicht(): drie verworpen pogingen (zie de toelichting bij de
#     GRIJP_*-constanten hieronder) voordat STROOMREGELING met een hogere
#     doelstroom (400mA) en wachten-op-stabilisatie werd gekozen. Nog NIET
#     op hardware getest.
# De stroomsensor (ADC2/GPIO28, servo 4) is apart geverifieerd met
# tests/test_grijpstroom.py: de versterkingsformule klopt exact met een
# gemeten datapunt (1488,4 mA bij volledig dichtknijpen tegen het flesje,
# ADC2=38425), en het analoge RC-filter (Sallen-Key, ~812 Hz afkap) is veel
# sneller dan het software-EMA-filter -- dat laatste domineert de reactietijd.
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
ARM_REIK_GRAD = 170.0              # schouder+elleboog GELIJKTIJDIG naar deze hoek vanaf
                                   # rust om te pakken. Was 20.0 (klakkeloos overgenomen uit
                                   # test_drive.py, waar het nooit tegen een echt voorwerp
                                   # geverifieerd kon worden -- gesimuleerd). HARDWARE-FOUT
                                   # GEVONDEN 2026-11-12: geo.vingertop_cm(theta, theta) laat
                                   # zien dat bij gelijktijdig bewegen de vingertop x=1,8cm
                                   # haalt bij 20 graden, tegen het doel van ~12,4cm (STOP_CM)
                                   # -- veel te kort, de arm moest bijna volledig strekken om
                                   # het voorwerp te raken (gebruiker, visueel bevestigd).
                                   # LET OP: over het hele 0-180-bereik komt de vingertop met
                                   # deze gelijktijdige beweging NOOIT lager dan ~13,7cm boven
                                   # de grond (geo.vingertop_cm()); 170 graden (x=16,1cm,
                                   # z=15,0cm) werkt dus alleen voor een voorwerp van
                                   # ongeveer die hoogte, niet voor iets vlak op de grond.
                                   # Eerste schatting op basis van gebruikersobservatie, nog
                                   # verder te verfijnen.
ARM_TERUGTREK_GRAD = 160.0         # exact zoals gevraagd; trekt de arm bij deze
                                   # reikhoek in de praktijk helemaal naar rust
                                   # terug (servo_pos klemt onder 0 graden af)
ARM_STREK_GRAD = 180.0             # volledig gestrekt voor stap 5

# Grip: SETPOINT-modus (servo_cur()) -- de PI stuurt continu bij richting
# dicht totdat de gemeten stroom de ingestelde GRIJP_STROOM_MA nadert, en
# stopt daar vanzelf (gebruiker, 2026-11-12: "naar 180 graden en regelen op
# de ingestelde stroom; als deze behaald is kan het voorwerp opgepakt
# worden"). Geschiedenis van eerdere, VERWORPEN pogingen (alle 2026-11-12):
#   1. SETPOINT op 150mA, vaste duur 3s: stopte al bij 58-59 graden -- maar
#      dat was nog met de FOUTE ARM_REIK_GRAD=20 (zie boven), dus de grijper
#      sloot toen waarschijnlijk in VRIJE LUCHT, niet tegen het voorwerp. De
#      150mA kwam kennelijk al uit pure mechanische wrijving van het
#      vierstangenmechanisme, niet uit objectcontact.
#   2. Positie-modus, direct doel 180, ingebouwde LIMIT-modus als
#      bescherming: GEVAARLIJK TRAAG. Doorgerekend: die regelaar se
#      correctie haalt pas na ~0,2-0,3s de normale stapgrootte in; tegen een
#      MEEGEVEND voorwerp (het testflesje) bleef positie/doel bovendien
#      eindeloos langzaam naar elkaar toe kruipen zonder ooit (binnen de
#      servo-timeout) strak te convergeren.
#   3. Kleine stapjes (2 graden) met een eigen, snel stroomdrempel-per-stap
#      (200mA): stopte veel te vroeg (18 graden) -- een opstartstroom-piek
#      (stiction vanuit stilstand) bleek al boven de drempel te komen, ruim
#      voordat de kaken ook maar in de buurt van het voorwerp komen.
# Deze (vierde) versie gebruikt dus WEL de SETPOINT-regelaar, maar met een
# hogere, beter onderbouwde doelstroom. EERST geprobeerd: wachten tot de
# positie stabiliseert (aanname: dat betekent "de interne PI is klaar").
# GEVAARLIJK GEBLEKEN (hardwaretest 2026-11-12): servo_crl.py's SETPOINT-
# modus heeft GEEN anti-windup op zijn integrator. Tijdens de eerste, vrije
# sluitfase (voordat de kaken het voorwerp raken) blijft de stroomfout lang
# groot, dus de integrator loopt fors op; zodra er dan echt weerstand komt,
# schiet de aangestuurde positie (en dus de stroom) ver door het doel heen
# voordat de regelaar terugveert. Waargenomen: 1644mA bij een doel van
# 400mA (geen zichtbare schade, maar duidelijk meer dan bedoeld) -- en de
# positie "stabiliseerde" braaf rond dat te hoge niveau, dus de stabilisatie-
# check op POSITIE ving dit helemaal niet op. Nu dus: de STROOM zelf
# rechtstreeks bewaken, elke 100ms, met een harde veiligheidsgrens.
GRIJP_STROOM_MA = 400.0            # doelstroom voor een stevige greep. Eerste
                                   # schatting tussen de losse mechanische
                                   # wrijvingsstroom (~150mA) en de volledig-
                                   # dichtgeknepen waarde (1488mA gemeten met
                                   # test_grijpstroom.py) -- nog niet op
                                   # hardware geverifieerd.
GRIJP_STROOM_VEILIGHEID_MA = 900.0 # harde noodgrens, ruim onder de waargenomen
                                   # 1644mA-overshoot maar met marge boven het
                                   # doel voor een normale, iets hogere afronding.
                                   # Eerste schatting.
GRIJP_MAX_DUUR_MS = 10000          # veiligheidslimiet; stopt hoe dan ook
GRIJP_MISGREEP_MARGE_GRAD = 8.0    # eindhoek dicht bij 180 (volledig dicht zonder
                                   # dat de stroom de doelwaarde al raakte) =
                                   # waarschijnlijk niets gegrepen

# Obstakelstop tijdens de cruise (stap 2): kortste/veiligste van ultrasoon+ToF
# tegen STOP_CM, zie _obstakel_afstand() hieronder voor de fusieregel.
STOP_CM = geo.STOP_DIST_CM
US_TOF_AGREE_MARGE_CM = 5.0        # 'ongeveer hetzelfde'-marge; eerste schatting

# Fijnpositionering (stap 2b): lib/gripper/approach.finetune() rekent zelf een
# doelafstand uit (geometry.stop_dist_cm()) en staat geen eigen doel toe. De
# eerste twee hardwaretests (2026-11-12) kwamen BEIDE ~0,5-1cm BUITEN VENSTER
# uit (te ver van het voorwerp, niet te dichtbij) -- gebruiker: "de kar kan
# ongeveer een cm meer naar voren". Daarom hier een eigen fijnpositionerings-
# functie (zelfde patroon als finetune(), zie fijnpositioneren() hieronder)
# met een expliciete correctie op het berekende doel. Eerste schatting.
FIJNPOSITIE_CORRECTIE_CM = 1.0

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

# ---------------- OLED / ToF (gedeeld) ----------------
# `oled`/`tof`/`_tof_ok` zijn hier GEEN module-aanmaak meer (dat zou bij
# "import test_autodrive" vanuit tests/test_webserver_rc.py een TWEEDE
# I2C(0,...)/VL53L1X-object op dezelfde bus/sensor aanmaken, naast de bus die
# de webserver zelf al beheert). Standalone gebruik (mpremote run) maakt ze
# zelf aan via _init_own_hardware() (aangeroepen vanuit main()); een
# aanroeper die de bus al zelf beheert geeft zijn EIGEN objecten door via
# set_hardware() (zie tests/test_webserver_rc.py, cmd "autodrive").
TOF_TIMING_BUDGET_US = 33_000
TOF_PERIOD_MS = 50
oled = None
tof = None
_tof_ok = False
_tof_last_cm = None


def set_hardware(oled_obj, tof_obj, tof_ok):
    """Hergebruik AL BESTAANDE OLED/ToF-objecten (bv. van tests/
    test_webserver_rc.py) i.p.v. ze hier opnieuw aan te maken. oled_obj mag
    None zijn (geen display beschikbaar); update_display() slaat de OLED-
    tekening dan over."""
    global oled, tof, _tof_ok
    oled = oled_obj
    tof = tof_obj
    _tof_ok = tof_ok


def _init_own_hardware():
    """Standalone-gebruik (mpremote run): eigen I2C0 + OLED + ToF aanmaken."""
    global oled, tof, _tof_ok
    i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0),
                           freq=400000, timeout=200_000)
    oled = SSD1306_I2C(128, 64, i2c_oled)
    tof = VL53L1X(i2c_oled)
    _tof_ok = tof.init()
    if _tof_ok:
        tof.set_distance_mode(LONG)
        tof.set_timing_budget_us(TOF_TIMING_BUDGET_US)
        tof.start_continuous(TOF_PERIOD_MS)
    else:
        print("! VL53L1X.init() mislukt -- ToF-afstand blijft leeg, obstakelstop valt terug op ultrasoon alleen")

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
    if _tof_ok and tof.data_ready():
        mm = tof.read(blocking=False)
        if tof.range_status == RANGE_VALID:
            _tof_last_cm = mm / 10.0

    if oled is None:
        return   # geen display beschikbaar (gedeeld gebruik, zie set_hardware())

    cm, kind = ultrasoon.read_cm()
    us_txt = "%5.1f" % cm if kind == 'ok' else " ----"
    tof_txt = "%5.1f" % _tof_last_cm if _tof_last_cm is not None else " ----"

    pa = _ldr_licht_pct(_adc_ldr_a.read_u16())
    pb = _ldr_licht_pct(_adc_ldr_b.read_u16())

    afstand = sr.distance()
    # Zonder "cm/s"-tekst: met CRUISE_AFSTAND_CM=300 (i.p.v. 100 in test_drive.py)
    # kan de afstand nu 3 cijfers worden, en "MF 19.1 cm/s 300cm" (19 tekens)
    # paste al niet meer op de 128px/16-tekens-brede regel.
    regel4 = "%s %4.1f %3dcm" % (_mode_label, _mode_snelheid, round(afstand))

    s_s = _servo_graden(sc, 1)
    s_e = _servo_graden(sc, 2)
    s_g = _servo_graden(sc, 4)
    # Zonder "="-tekens: met hoeken tot 170 graden paste "S=170 E=170 G=170"
    # (18 tekens) niet meer op de 128px/16-tekens-brede regel -- de G-waarde
    # viel van het scherm af. "S170 E170 G170" (14 tekens) past ruim.
    regel5 = "S%3d E%3d G%3d" % (round(s_s), round(s_e), round(s_g))
    regel6 = "I_grijp=%5.0fmA" % sc.i_filt_mA

    oled.fill(0)
    oled.text(_stap_tekst, 0, 0)
    oled.text("U=%s T=%s" % (us_txt, tof_txt), 0, 10)
    # Zonder "="-tekens: "LRa=100% LRb=100%" (17 tekens) kon bij felle
    # verlichting (100%) ook al over de 16-tekens-breedte heen gaan.
    oled.text("LRa%3d%% LRb%3d%%" % (round(pa), round(pb)), 0, 20)
    oled.text(regel4, 0, 30)
    oled.text(regel5, 0, 40)
    oled.text(regel6, 0, 50)
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


def fijnpositioneren(read_cm, doel_cm, tol_cm=0.1, pogingen=3, speed_cm_s=2.0, n_meet=8):
    """Zelfde patroon als lib/gripper/approach.finetune() (stilstaand
    meten + stepper_ramp.creep() tot binnen tol_cm), maar met een ZELF
    opgegeven doelafstand i.p.v. de door geometry.stop_dist_cm() berekende --
    zie FIJNPOSITIE_CORRECTIE_CM hierboven voor waarom. Geeft (gemeten, doel,
    gelukt) terug; "gelukt" toetst tegen hetzelfde grip-venster als
    finetune(), ook met de correctie erop toegepast."""
    d = None
    for _ in range(max(1, int(pogingen))):
        d = approach.mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel_cm, False
        fout = d - doel_cm
        if abs(fout) <= tol_cm:
            break
        sr.creep(fout, speed_cm_s)
        while sr.busy():
            pass
    else:
        d = approach.mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel_cm, False

    lo, hi = geo.grip_window_cm(geo.OBJECT_W_CM)
    lo -= FIJNPOSITIE_CORRECTIE_CM
    hi -= FIJNPOSITIE_CORRECTIE_CM
    gelukt = lo <= d <= hi + tol_cm
    print("fijnpositioneren: gemeten %.2f cm, doel %.2f cm, venster %.2f-%.2f -> %s"
          % (d, doel_cm, lo, hi, "OK" if gelukt else "BUITEN VENSTER"))
    return d, doel_cm, gelukt


def grijp_dicht(sc, mA=GRIJP_STROOM_MA, snelheid=GRIJPER_SNELHEID_GRAD_S):
    """Sluit de grijper op STROOMREGELING (SETPOINT-modus, servo_cur()
    houdt 'm continu richting dicht sturend), maar bewaakt de stroom ZELF
    elke 100ms i.p.v. te wachten tot de interne PI "klaar" lijkt (zie de
    geschiedenis bij de constanten hierboven -- positie-stabilisatie bleek
    een overshoot tot 1644mA niet te detecteren, omdat servo_crl.py's
    SETPOINT-modus geen anti-windup op zijn integrator heeft). Stopt zodra
    de gemeten stroom het doel (mA) OF de harde veiligheidsgrens
    (GRIJP_STROOM_VEILIGHEID_MA) bereikt.

    Geeft (eindhoek_graden, waarschijnlijk_gelukt) terug. "Waarschijnlijk
    gelukt" is een ruwe heuristiek (eindhoek dicht bij 180 = volledig dicht
    zonder dat de stroom het doel raakte = waarschijnlijk niets gegrepen),
    geen harde garantie -- er is geen echte krachtsensor, alleen de
    servostroom.
    """
    sc.servo_cur(mA, snelheid)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < GRIJP_MAX_DUUR_MS:
        update_display(sc)
        time.sleep_ms(100)
        if sc.i_filt_mA >= GRIJP_STROOM_VEILIGHEID_MA:
            print("! Stroom-veiligheidsgrens %.0f mA bereikt (doel was %.0f mA) -- stoppen"
                  % (sc.i_filt_mA, mA))
            break
        if sc.i_filt_mA >= mA:
            break
    sc.stop_cur()
    # BUG GEVONDEN EN GEFIXT (2026-11-12): stop_cur() bevriest target_deg niet.
    # Die bleef op de laatst door de PI berekende waarde staan (kan AFWIJKEN
    # van de huidige positie, bv. net terugregelend na een stroom-overshoot),
    # en de gewone snelheidsramp (speed blijft actief) liep daarna gewoon door
    # naar die verouderde target -- waargenomen: de grijper dreef over een
    # paar seconden geleidelijk helemaal open (naar 0 graden), los van elke
    # andere actie. Hier expliciet bevriezen op de HUIDIGE positie.
    sc.servos[4]["target_deg"] = sc.servos[4]["pos_deg"]
    eind_deg = _servo_graden(sc, 4)
    misgreep = eind_deg > (180.0 - GRIJP_MISGREEP_MARGE_GRAD)
    print("grijpen (stroomregeling, doel %.0f mA): eindhoek=%.1f graden, I=%.0f mA%s"
          % (mA, eind_deg, sc.i_filt_mA,
             "  [WAARSCHUWING: mogelijk MISGREEP, weinig weerstand gevoeld]"
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


# ---------------- Missie (herbruikbaar: ook aangeroepen vanuit tests/
# test_webserver_rc.py, cmd "autodrive", met de daar AL bestaande sc/sensor/
# tof -- zie set_hardware() hierboven. BLOKKEREND, zelfde bewust aanvaarde
# tradeoff als ldr.scan() in dat bestand: tijdens de hele missie bevriezen
# telemetrie EN de STOP-knop daar, zie de kop van test_webserver_rc.py) ----
def missie(sc, sensor):
    gyro_rate = sr.gyro_z_deg_s(sensor)
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
        doel_fijn = geo.stop_dist_cm(geo.OBJECT_W_CM) - FIJNPOSITIE_CORRECTIE_CM
        print("--- Stap 2b: fijnpositioneren (object_w_cm=%.1f, doel=%.2f cm) ---"
              % (geo.OBJECT_W_CM, doel_fijn))
        d_fijn, _, gelukt_fijn = fijnpositioneren(ultrasoon.read_cm, doel_fijn)
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
        eind_deg, gelukt_grijp = grijp_dicht(sc)
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


# ---------------- Standalone (mpremote run) ----------------
def main():
    sc = ServoController()
    sc.servo_cur_limit(200)   # stroomlimiet grijper, net als in test_all.py/test_servo.py
                              # (geldt in POSITIE-modus; grijp_op_stroom() hieronder
                              # schakelt tijdelijk naar SETPOINT-modus, zie servo_crl.py)

    i2c_sensor = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
    sensor = ICM20948(i2c_sensor, **load_mag_calibration())

    ldr.start_synced_sampling(sc)
    time.sleep_ms(200)   # eerste tick(s) laten landen voor read_synced()

    _init_own_hardware()
    missie(sc, sensor)


if __name__ == "__main__":
    main()
