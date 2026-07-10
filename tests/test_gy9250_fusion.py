import machine
import time
from mpu9250 import MPU9250
from ak8963 import AK8963
from fusion import Fusion
from ssd1306 import SSD1306_I2C

# --- Testscript: GY9250 fusion (complementair filter met kantelcompensatie) op OLED ---
# GY9250 (MPU9250): GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
# OLED (SSD1306):   GPIO0  (SDA0) / GPIO1  (SCL0) -> I2C-bus 0

i2c_sensor = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
i2c_oled = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)

oled = SSD1306_I2C(128, 64, i2c_oled)

dummy = MPU9250(i2c_sensor)  # opent de I2C-bypass naar de AK8963
ak8963 = AK8963(
    i2c_sensor,
    offset=(-24.5, 12.3, -5.1),      # VUL HIER UW KALIBRATIEWAARDEN IN (zie test_gy9250_calibrate.py)
    scale=(0.95, 1.02, 1.03),
)
sensor = MPU9250(i2c_sensor, ak8963=ak8963)
fuse = Fusion()


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


toon(["GY9250 fusion test", "Sensor gevonden", "start over 2s..."])
print("GY9250 fusion test: sensor gevonden, whoami=" + hex(sensor.whoami))
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        accel = sensor.acceleration
        gyro = sensor.gyro
        mag = sensor.magnetic

        heading = fuse.update(accel, gyro, mag)

        toon([
            "GY9250 FUSION",
            f"Head :{heading:6.1f}",
            f"Roll :{fuse.roll:6.1f}",
            f"Pitch:{fuse.pitch:6.1f}",
            f"Mx{mag[0]:5.1f} My{mag[1]:5.1f}",
            f"Mz{mag[2]:5.1f}",
        ])

        print(
            f"heading={heading:.1f} roll={fuse.roll:.1f} pitch={fuse.pitch:.1f} "
            f"mag=({mag[0]:.1f},{mag[1]:.1f},{mag[2]:.1f})"
        )

        time.sleep_ms(5)  # ca. 200 Hz
except KeyboardInterrupt:
    toon(["GY9250 fusion test", "Gestopt."])
    print("Test gestopt.")
