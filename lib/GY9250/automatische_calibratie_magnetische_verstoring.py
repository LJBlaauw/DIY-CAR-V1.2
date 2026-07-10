import machine
import time
from mpu9250 import MPU9250
from stepper import rotate, stop, disable

# --- AUTOMATISCHE KALIBRATIE (robot draait zelf rond) ---
# Gebruikt de bestaande PIO-steppermotor-driver (lib/stepper/stepper.py) om
# de robot op zijn as te laten draaien tijdens het kalibreren. Zo hoeft u de
# robot niet met de hand rond te draaien.

DRAAI_SNELHEID_CM_S = 8      # rotatiesnelheid van elk wiel
DRAAI_AFSTAND_CM = 500       # ruim voldoende voor meerdere volle rondjes

# GY9250 zit op GPIO10 (SDA1) en GPIO11 (SCL1) -> hardware I2C-bus 1
i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
sensor = MPU9250(i2c)

print("START AFTELLEN: Zet de robot vrij op de grond...")
time.sleep(3)

print("Motoren ingeschakeld. Kalibratie start nu!")
rotate('r', DRAAI_SNELHEID_CM_S, DRAAI_AFSTAND_CM)

# Arrays om de uiterste waarden op te slaan
min_x = max_x = min_y = max_y = min_z = max_z = None

# We nemen 1000 metingen tijdens het draaien (ca. 15-20 seconden)
for i in range(1000):
    time.sleep_ms(15)

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
stop()
disable()
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

# Voorkom deling door nul als een as toevallig geen variatie vertoonde
chord_x = chord_x or 1e-6
chord_y = chord_y or 1e-6
chord_z = chord_z or 1e-6

gemiddelde_straal = (chord_x + chord_y + chord_z) / 3

soft_iron_x = gemiddelde_straal / chord_x
soft_iron_y = gemiddelde_straal / chord_y
soft_iron_z = gemiddelde_straal / chord_z

# Print de kant-en-klare regels voor uw hoofdprogramma
print("\nKopieer de onderstaande regels naar uw hoofdprogramma (AK8963-constructor):")
print(f"offset=({hard_iron_x:.2f}, {hard_iron_y:.2f}, {hard_iron_z:.2f})")
print(f"scale=({soft_iron_x:.4f}, {soft_iron_y:.4f}, {soft_iron_z:.4f})")
print("\n---------------------------")
