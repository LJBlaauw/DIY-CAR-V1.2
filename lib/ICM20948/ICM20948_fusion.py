import machine
import time
import _thread
from icm20948 import ICM20948, load_mag_calibration
from fusion import Fusion  # Importeer het complementaire fusion-filter

actuele_koers = 0.0


def geavanceerd_kompas_core1():
    global actuele_koers

    # Initialiseer sensor
    # ICM-20948 zit op GPIO10 (SDA1) en GPIO11 (SCL1) -> hardware I2C-bus 1
    i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)

    # Kalibratie komt uit config.json (sectie "icm20948", zie Calibrate_ICM20948.py)
    sensor = ICM20948(
        i2c,
        **load_mag_calibration(),  # mag_offset/mag_scale uit config.json
    )

    # Start het fusion-filter. Fusion houdt zelf de verstreken tijd (dt)
    # tussen updates bij via deltat.DeltaT (time.ticks_us onder MicroPython).
    fuse = Fusion()

    print("Core 1: Sensor fusion (complementair filter) gestart")

    while True:
        try:
            # Lees ALLE 9 assen uit (essentieel voor tilt-gecompenseerde koers)
            accel = sensor.acceleration  # (ax, ay, az)
            gyro = sensor.gyro           # (gx, gy, gz)
            mag = sensor.magnetic        # (mx, my, mz)

            # Voer alle data in het filter. Dit berekent roll, pitch en een
            # kantelgecompenseerde, gyro-gladgestreken koers.
            fuse.update(accel, gyro, mag)

            # fuse.heading geeft de gefilterde koers in graden (0-360)
            actuele_koers = fuse.heading

        except Exception:
            pass

        # Snel achter elkaar meten voor een stabiel filterresultaat
        time.sleep_ms(5)  # ca. 200 Hz, de RP2350 kan dit makkelijk aan


# Start op Core 1
_thread.start_new_thread(geavanceerd_kompas_core1, ())

# Core 0 code blijft exact hetzelfde...
print("Core 0: Motorbesturing gereed")
time.sleep(1)  # Geef Core 1 kort de tijd om op te starten

while True:
    # Haal de meest actuele, stabiele richting op uit de globale variabele
    robot_richting = actuele_koers

    # --- MOTOR LOGICA HIER ---
    # Gebruik 'robot_richting' om uw NEMA 17 motoren bij te sturen tijdens het rijden
    # ... uw motorcode hier ...

    print("Live richting robot:", robot_richting)

    # Core 0 mag op zijn eigen tempo draaien (bijv. 10 Hz)
    time.sleep_ms(100)
