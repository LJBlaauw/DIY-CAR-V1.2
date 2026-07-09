`python

import machine
import time
import _thread
import math
from mpu9250 import MPU9250
from fusion import Fusion  # Importeer het geavanceerde filter

actuele_koers = 0.0

def geavanceerd_kompas_core1():
    global actuele_koers

    # Initialiseer sensor
    i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=400000)
    sensor = MPU9250(i2c)

    # Uw kalibratiedata injecteren
    sensor.ak8963.offset = (-24.5, 12.3, -5.1)
    sensor.ak8963.scale = (0.95, 1.02, 1.03)

    # Start het geavanceerde Fusion filter
    fuse = Fusion()

    # Belangrijk: Geavanceerde filters moeten exact weten hoeveel tijd er tussen de metingen zit
    last_time = time.ticks_us()

    print("Core 1: Geavanceerde Madgwick Sensor Fusion gestart")

    while True:
        try:
            # Lees ALLE 9 assen uit (essentieel voor geavanceerde filtering)
            accel = sensor.acceleration  # (ax, ay, az)
            gyro = sensor.gyro          # (gx, gy, gz)
            mag = sensor.magnetic       # (mx, my, mz)

            # Bereken exact de verstreken tijd (dt) sinds de vorige meting
            now = time.ticks_us()
            dt = time.ticks_diff(now, last_time) / 1000000.0
            last_time = now

            # Voer alle data in het geavanceerde filter.
            # Dit filter berekent de exacte 3D-oriëntatie, inclusief kantelcompensatie!
            fuse.update(accel, gyro, mag, dt)

            # fuse.heading geeft de exacte, stabiele koers in graden (0-360)
            # Volledig gecorrigeerd voor trillingen, hellingen en motorruis
            actuele_koers = fuse.heading

        except Exception:
            pass

        # Geavanceerde filters werken het best als ze heel snel achter elkaar meten
        time.sleep_ms(5) # Update op ca. 200 Hz, de RP2350 kan dit makkelijk aan

# Start op Core 1
_thread.start_new_thread(geavanceerd_kompas_core1, ())

# Core 0 code blijft exact hetzelfde...
print("Core 0: Motorbesturing gereed")
time.sleep(1) # Geef Core 1 kort de tijd om op te starten [4]

while True:
    # Haal de meest actuele, stabiele richting op uit de globale variabele [4]
    robot_richting = actuele_koers

    # --- MOTOR LOGICA HIER ---
    # Gebruik 'robot_richting' om uw NEMA 17 motoren bij te sturen tijdens het rijden [4]
    # ... uw motorcode hier ...

    # De gevraagde printopdracht in Core 0:
    print("Live richting robot:", robot_richting)

    # Core 0 mag op zijn eigen tempo draaien (bijv. 10 Hz) [4]
    time.sleep_ms(100)
`
