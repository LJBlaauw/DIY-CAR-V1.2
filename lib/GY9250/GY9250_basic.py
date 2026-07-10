import machine
import time
import _thread
import math
from mpu9250 import MPU9250
from ak8963 import AK8963

# Globale variabele voor de actuele, gecorrigeerde koers (heading)
# Core 1 schrijft hier continu in, Core 0 leest dit live uit
actuele_koers = 0.0


def verwerk_kompas_core1():
    global actuele_koers

    # 1. Initialiseer I2C altijd BINNEN de core-functie
    # GY9250 zit op GPIO10 (SDA1) en GPIO11 (SCL1) -> hardware I2C-bus 1
    i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)

    # 2. VUL HIER UW EIGEN KALIBRATIERESULTATEN IN (uit Calibrate_GY9250.py):
    dummy = MPU9250(i2c)  # opent de I2C-bypass naar de AK8963
    ak8963 = AK8963(
        i2c,
        offset=(-24.5, 12.3, -5.1),      # Hard-iron offsets (X, Y, Z)
        scale=(0.95, 1.02, 1.03),        # Soft-iron schaling (X, Y, Z)
    )
    sensor = MPU9250(i2c, ak8963=ak8963)

    # Variabele voor het complementair filter
    # We starten op 0.0, het filter stabiliseert zichzelf snel
    gefilterde_hoek = 0.0
    dt = 0.02  # Looptijd van de loop (20 milliseconden = 50 Hz)

    print("Core 1: Gekalibreerde kompas-taak actief op 50Hz")

    while True:
        try:
            # Lees de gecorrigeerde magnetometer- en gyro-waarden uit
            # De driver past hierboven automatisch uw offsets toe
            mx, my, mz = sensor.magnetic
            gx, gy, gz = sensor.gyro  # gz is de rotatiesnelheid om de Z-as (rad/s)

            # Converteer de gyro-rotatie van radialen naar graden per seconde
            gyro_z_graden = gz * (180.0 / math.pi)

            # Bereken de ruwe magnetische hoek in graden (0 tot 360)
            ruwe_hoek = math.atan2(my, mx) * (180.0 / math.pi)
            if ruwe_hoek < 0:
                ruwe_hoek += 360.0

            # --- COMPLEMENTAIR FILTER (Sensor Fusion) ---
            # Dit filtert de PWM-storing van uw TMC2209 drivers effectief weg.
            # 96% gewicht naar de snelle gyroscoop, 4% naar de corrigerende magneetsensor.
            gefilterde_hoek = 0.96 * (gefilterde_hoek + gyro_z_graden * dt) + 0.04 * ruwe_hoek
            gefilterde_hoek = gefilterde_hoek % 360.0

            # Schrijf het resultaat atomair naar de globale variabele
            actuele_koers = gefilterde_hoek

        except Exception:
            # Voorkom dat Core 1 crasht bij een incidentele I2C-glitch door de motoren
            pass

        time.sleep_ms(20)  # 50 Hz updatefrequentie


# --- HOOFDPROGRAMMA (Draait op Core 0) ---

# Start de kompasberekening op Core 1
_thread.start_new_thread(verwerk_kompas_core1, ())

print("Core 0: Motorbesturing gereed")
time.sleep(1)  # Geef Core 1 kort de tijd om op te starten

while True:
    # Haal de meest actuele, stabiele richting op
    robot_richting = actuele_koers

    # --- MOTOR LOGICA HIER ---
    # Gebruik 'robot_richting' om uw NEMA 17 motoren bij te sturen tijdens het rijden
    # Als de richting afwijkt van uw doelkoers, pas de snelheid van de linkse of rechtse motor aan.

    print("Live richting robot:", robot_richting)
    time.sleep_ms(100)  # Core 0 mag op zijn eigen tempo draaien
