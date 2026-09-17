import machine
import time
from mpu9250 import MPU9250
from ak8963 import AK8963

# --- HANDMATIGE MAGNETOMETER-KALIBRATIE ---
# Til de robot op en draai hem tijdens het meten langzaam en volledig rond
# alle assen (net als een 8-vormige beweging). Voor een gemotoriseerde
# variant die de robot zelf laat ronddraaien, zie
# automatische_calibratie_magnetische_verstoring.py

# GY9250 zit op GPIO10 (SDA1) en GPIO11 (SCL1) -> hardware I2C-bus 1
i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)

dummy = MPU9250(i2c)  # opent de I2C-bypass naar de AK8963
ak8963 = AK8963(i2c)

print("START KALIBRATIE: draai de robot nu langzaam rond alle assen...")
time.sleep(3)

# count=256 metingen met 200 ms tussenpozen (~1 minuut)
offset, scale = ak8963.calibrate(count=256, delay=200)

print("\n--- KALIBRATIE VOLTOOID ---")
print("Kopieer de onderstaande regel(s) naar uw hoofdprogramma:")
print(f"offset=({offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f})")
print(f"scale=({scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f})")
print("---------------------------")
