# ================================================================
# tests/test_grijpstroom.py
#
# Diagnosescript voor de grijper-stroomsensor (servo 4, ADC2/GPIO28):
# handmatige bediening + live uitlezing van de RUWE ADC-waarde en de
# berekende stroom (zowel instantaan als het gefilterde signaal dat de
# regelaar echt gebruikt), om te controleren of de stroomregeling/-limiet
# (lib/servo/servo_crl.py: servo_cur()/servo_cur_limit()) daadwerkelijk
# reageert op mechanische weerstand.
#
# Aanleiding (2026-11-12): tests/test_autodrive.py's grijp_dicht() gaf een
# vals-positieve "MISGREEP"-waarschuwing (eindhoek 172,6 graden, servo-
# timeout) terwijl het voorwerp volgens de gebruiker WEL gegrepen was --
# vermoeden dat de stroomregeling niet (goed) reageert.
#
# Gebruik: dit is een INTERACTIEF REPL-script (zelfde aanpak als
# tests/test_ldr_beam.py) -- draai het vanuit een interactieve mpremote-
# of Thonny-REPL-sessie, NIET via "mpremote run" op de achtergrond (dat
# heeft geen toetsenbordinvoer, input() blijft dan voor altijd hangen).
#
#   Typ een hoek (0-180) en Enter -> grijper beweegt daarheen (positie-modus,
#     MET de stroomlimiet actief, zelfde mechanisme als grijp_dicht() in
#     test_autodrive.py), met live ADC/stroom-uitlezing tijdens de beweging
#     en een paar seconden erna (settling).
#   "l"  -> stroomlimiet aan/uit schakelen
#   "s"  -> alleen de huidige status tonen, zonder te bewegen
#   "q"  -> grijper terug naar rust, script stoppen
#
# Zet tijdens het testen zelf weerstand tegen de kaken (met de hand, of het
# voorwerp) en kijk of ADC2/I_nu/I_gefilterd daadwerkelijk oplopen.
# ================================================================

import time
from servo_crl import ServoController

GRIJP_SNELHEID_GRAD_S = 20.0
STROOMLIMIET_MA = 200.0
LIVE_INTERVAL_MS = 100
NASLEEP_S = 2.0
TIMEOUT_MS = 12000

sc = ServoController()
_limiet_aan = True
sc.servo_cur_limit(STROOMLIMIET_MA)


def _status_regel():
    s = sc.servos[4]
    raw = sc.adc_servo4.read_u16()
    i_nu = sc.adc_to_current_mA(raw)
    rest_deg = sc._pct_to_deg(4, sc.rest_pct[4])
    hoek = s["pos_deg"] - rest_deg
    print("hoek=%6.1f graden  ADC2=%5d  I_nu=%6.1f mA  I_gefilterd=%6.1f mA"
          % (hoek, raw, i_nu, sc.i_filt_mA))


def _live(duur_s):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < duur_s * 1000:
        _status_regel()
        time.sleep_ms(LIVE_INTERVAL_MS)


def status():
    """Toon de huidige hoek/ADC/stroom, zonder te bewegen."""
    _status_regel()


def naar(graden):
    """Beweeg de grijper naar een absolute hoek (0=open rust..180=dicht),
    met live ADC/stroom-uitlezing tijdens de beweging en NASLEEP_S erna."""
    graden = max(0.0, min(180.0, float(graden)))
    sc.servo_pos(4, graden, GRIJP_SNELHEID_GRAD_S)
    s = sc.servos[4]
    t0 = time.ticks_ms()
    while abs(s["pos_deg"] - s["target_deg"]) > 1.0:
        _status_regel()
        if time.ticks_diff(time.ticks_ms(), t0) > TIMEOUT_MS:
            print("! timeout: doel niet bereikt binnen %d ms" % TIMEOUT_MS)
            break
        time.sleep_ms(LIVE_INTERVAL_MS)
    print("-- doel bereikt (of timeout), nog %.0fs nameten --" % NASLEEP_S)
    _live(NASLEEP_S)


def limiet_toggle():
    global _limiet_aan
    _limiet_aan = not _limiet_aan
    if _limiet_aan:
        sc.servo_cur_limit(STROOMLIMIET_MA)
        print("stroomlimiet AAN (%d mA)" % STROOMLIMIET_MA)
    else:
        sc.clear_cur_limit()
        print("stroomlimiet UIT -- voorzichtig, geen bescherming meer tegen vastlopen")


def help():
    print("""
Commando's (typ en Enter):
  <getal 0-180>  grijper naar die hoek, live ADC/stroom tonen
  s              alleen status tonen, niet bewegen
  l              stroomlimiet aan/uit
  q              stoppen (grijper terug naar rust)

Zet tijdens een beweging handmatig weerstand tegen de kaken en kijk of
ADC2/I_nu/I_gefilterd oplopen.
""")


def main():
    help()
    try:
        while True:
            cmd = input("grijpstroom> ").strip().lower()
            if cmd == "q":
                break
            if cmd == "l":
                limiet_toggle()
                continue
            if cmd == "s":
                status()
                continue
            if cmd == "":
                continue
            try:
                naar(float(cmd))
            except ValueError:
                print("? onbekend commando, typ 'help()' voor de lijst")
    except KeyboardInterrupt:
        pass
    finally:
        sc.servo_rest(4, GRIJP_SNELHEID_GRAD_S)
        print("grijper terug naar rust, klaar.")


main()
