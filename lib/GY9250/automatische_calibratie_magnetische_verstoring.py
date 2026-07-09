`python

import machine
import time
import math
from mpu9250 import MPU9250

# --- STEP/DIR CONFIGURATIE (Pas de pinnen aan uw hardware aan) ---
# We gaan ervan uit dat de motoren in tegengestelde richting draaien om te spinnen
step_links = machine.Pin(10, machine.Pin.OUT)
dir_links = machine.Pin(11, machine.Pin.OUT)
step_rechts = machine.Pin(12, machine.Pin.OUT)
dir_rechts = machine.Pin(13, machine.Pin.OUT)

def laat_robot_draaien(actief):
    """Zet de motoren in een modus om om zijn as te spinnen"""
    if actief:
        dir_links.value(1)   # Links vooruit
        dir_rechts.value(0)  # Rechts achteruit
    else:
        # Stop de stappenpulsen (motoren staan stil maar houden wel spanning/stroom)
        pass

def geef_stappen_puls():
    """Genereert één stap voor beide motoren (handmatige klokpuls)"""
    step_links.value(1)
    step_rechts.value(1)
    time.sleep_us(500) # Snelheid van het draaien (pas aan indien nodig)
    step_links.value(0)
    step_rechts.value(0)
    time.sleep_us(500)

# --- HOOFDPROGRAMMA VOOR KALIBRATIE ---

# Initialiseer I2C
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=400000)
sensor = MPU9250(i2c)

print("START ACCOUNTDOWN: Zet de robot vrij op de grond...")
time.sleep(3)

print("Motoren ingeschakeld. Kalibratie start nu!")
laat_robot_draaien(True)

# Arrays om de uiterste waarden op te slaan
min_x = max_x = min_y = max_y = min_z = max_z = None

# We nemen 1000 metingen tijdens het draaien (ca. 15-20 seconden)
for i in range(1000):
    # Geef continu stappenpulsen zodat de robot blijft spinnen
    # We doen dit een aantal keer per sensor-meting
    for _ in range(10):
        geef_stappen_puls()

    try:
        # Lees de ruwe, ongekalibreerde magnetometer waarden
        mx, my, mz = sensor.magnetic

        # Initialiseer de uitersten bij de eerste meting
        if min_x is None:
            min_x = max_x = mx
            min_y = max_y = my
            min_z = max_z = mz
        else:
            # Update de minimale en maximale waarden
            if mx < min_x: min_x = mx
            if mx > max_x: max_x = mx
            if my < min_y: min_y = my
            if my > max_y: max_y = my
            if mz < min_z: min_z = mz
            if mz > max_z: max_z = mz

    except Exception:
        pass

# Stop de robot na de metingen
laat_robot_draaien(False)
print("\n--- KALIBRATIE VOLTOOID ---")

# 1. BEREKEN HARD-IRON OFFSETS (Het verschoven middelpunt)
hard_iron_x = (max_x + min_x) / 2
hard_iron_y = (max_y + min_y) / 2
hard_iron_z = (max_z + min_z) / 2

# 2. BEREKEN SOFT-IRON SCHALING (De vervorming van de cirkel)
# Bereken de gemiddelde straal per as
chord_x = (max_x - min_x) / 2
chord_y = (max_y - min_y) / 2
chord_z = (max_z - min_z) / 2

gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

soft_iron_x = gemiddelde_straal / chord_x
soft_iron_y = gemiddelde_straal / chord_y
soft_iron_z = gemiddelde_straal / chord_z

# Print de kant-en-klare regels voor uw hoofdprogramma
print("\nKopieer de onderstaande regels naar uw hoofdprogramma (Stap B):")
print(f"sensor.ak8963.offset = ({hard_iron_x:.2f}, {hard_iron_y:.2f}, {hard_iron_z:.2f})")
print(f"sensor.ak8963.scale = ({soft_iron_x:.2f}, {soft_iron_y:.2f}, {soft_iron_z:.2f})")
print("\n---------------------------")
`
