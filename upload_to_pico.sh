#!/usr/bin/env bash
# Synct lib/, tests/ en config.json naar de Raspberry Pi Pico via mpremote.
#
# LET OP: config.json wordt alleen ingelezen bij het opstarten van
# ServoController(). Na dit script moet het lopende MicroPython-programma op
# de Pico dus herstart worden, anders gelden de oude waarden nog.
#
# Gebruik:
#   ./upload_to_pico.sh
#
# Vereist: mpremote (installeren met "pipx install mpremote"), de Pico
# aangesloten via USB, en geen andere seriële verbinding open (bv. de
# MicroPico-REPL in VSCode of Thonny -- die houdt de poort bezet).

set -euo pipefail

# Werk altijd relatief aan de locatie van dit script, ongeacht vanwaar het
# aangeroepen wordt.
cd "$(dirname "${BASH_SOURCE[0]}")"

if ! command -v mpremote >/dev/null 2>&1; then
    echo "mpremote niet gevonden. Installeer met: pipx install mpremote" >&2
    exit 1
fi

devices="$(mpremote devs 2>/dev/null || true)"
if [ -z "$devices" ]; then
    echo "Geen Pico gevonden. Controleer de USB-verbinding, en dat geen" >&2
    echo "ander programma (VSCode/MicroPico, Thonny) de seriele poort al" >&2
    echo "gebruikt (mpremote devs geeft dan ook niks terug)." >&2
    exit 1
fi
echo "Gevonden: $devices"

echo "--- lib/ uploaden (afgeplat) ---"
# GEEN "mpremote cp -r lib/ :": dat zet de mapstructuur 1-op-1 over (dus
# /lib/servo/servo_crl.py, /lib/LDR/ldr_scan_isr.py, ...). De testscripts
# verwachten echter een PLATTE /lib op de Pico -- bv. test_servo.py doet
# "from servo_crl import ServoController" zonder submap, wat alleen werkt als
# servo_crl.py rechtstreeks in /lib staat. Daarom hier elke module-map se
# .py-bestanden rechtstreeks (zonder submap) naar /lib kopiëren.
mpremote mkdir :lib >/dev/null 2>&1 || true
mpremote cp lib/*/*.py :lib/

echo "--- tests/ uploaden ---"
mpremote cp -r tests/ :

echo "--- config.json uploaden ---"
mpremote cp config.json :

echo "Klaar. Herstart het lopende programma op de Pico om config.json-wijzigingen actief te maken."
