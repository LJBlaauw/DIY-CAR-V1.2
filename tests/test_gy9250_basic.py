import machine
import time
import math
from mpu9250 import MPU9250
from ak8963 import AK8963
from ssd1306 import SSD1306_I2C

# --- Testscript: GY9250 basic (complementair filter) met live weergave op OLED ---
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


def toon(regels):
    oled.fill(0)
    for i, regel in enumerate(regels):
        oled.text(regel, 0, i * 10)
    oled.show()


gefilterde_hoek = 0.0
dt = 0.02  # 50 Hz

toon(["GY9250 basic test", "Sensor gevonden", "start over 2s..."])
print("GY9250 basic test: sensor gevonden, whoami=" + hex(sensor.whoami))
time.sleep(2)

print("Test gestart. Ctrl-C om te stoppen.")

try:
    while True:
        ax, ay, az = sensor.acceleration
        gx, gy, gz = sensor.gyro
        mx, my, mz = sensor.magnetic

        gyro_z_graden = gz * (180.0 / math.pi)
        ruwe_hoek = math.atan2(my, mx) * (180.0 / math.pi)
        if ruwe_hoek < 0:
            ruwe_hoek += 360.0

        gefilterde_hoek = 0.96 * (gefilterde_hoek + gyro_z_graden * dt) + 0.04 * ruwe_hoek
        gefilterde_hoek %= 360.0

        toon([
            "GY9250 BASIC",
            f"Heading:{gefilterde_hoek:6.1f}",
            f"Ax{ax:5.2f} Ay{ay:5.2f}",
            f"Az{az:5.2f}",
            f"Gz{gz:6.2f} rad/s",
            f"Mx{mx:5.1f} My{my:5.1f}",
            f"Mz{mz:5.1f}",
        ])

        print(
            f"heading={gefilterde_hoek:.1f} "
            f"accel=({ax:.2f},{ay:.2f},{az:.2f}) "
            f"gyro=({gx:.2f},{gy:.2f},{gz:.2f}) "
            f"mag=({mx:.1f},{my:.1f},{mz:.1f})"
        )

        time.sleep_ms(int(dt * 1000))
except KeyboardInterrupt:
    toon(["GY9250 basic test", "Gestopt."])
    print("Test gestopt.")
