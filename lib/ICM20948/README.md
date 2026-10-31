# ICM-20948 (gyro + accelerometer + kompas)

De **ICM-20948** (TDK InvenSense) is de opvolger van de MPU-9250. Het is een System-in-Package met een 3-assige gyroscoop, een 3-assige accelerometer en een **AK09916** 3-assige magnetometer in één behuizing. Een echte ICM-20948 heeft dus altijd een kompas. De 6-assige zusjes ICM-20648 en ICM-20649 hebben dat niet, maar die hebben een ander WHO_AM_I (`0xE0`/`0xE1`).

## Bestanden

| Bestand | Inhoud |
|---|---|
| `icm20948.py` | Driver: klassen `ICM20948` en `AK09916` |
| `fusion.py`, `deltat.py` | Complementair filter met kantelcompensatie (roll, pitch, heading) |
| `ICM20948_basic.py` | Voorbeeld: kompas + gyro-Z op core 1 (alleen koers) |
| `ICM20948_fusion.py` | Voorbeeld: alle 9 assen via `fusion.py` op core 1 |
| `Calibrate_ICM20948.py` | Handmatige magnetometerkalibratie (robot met de hand ronddraaien) |
| `automatische_calibratie_magnetische_verstoring.py` | Kalibratie terwijl de robot zelf ronddraait |

Tests met OLED-weergave: `tests/test_icm20948_*.py`. Eerste test na het aansluiten is altijd `tests/test_icm20948_i2c_scan.py`.

## Aansluiting

I2C1: SDA = GPIO10, SCL = GPIO11. Het adres van de ICM-20948 hangt af van pin AD0: **0x69** (AD0 hoog, standaard bij o.a. Adafruit en SparkFun) of **0x68** (AD0 laag, veel Chinese bordjes). De driver probeert beide. De magnetometer verschijnt pas na het zetten van de I2C-bypass op **0x0C**; dat doet de driver zelf.

## Gebruik

De API is gelijk aan die van de vroegere Tuupola MPU-9250-driver (voor het inmiddels vervangen MPU-6500-bordje):

```python
import machine
from icm20948 import ICM20948

i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
sensor = ICM20948(
    i2c,
    mag_offset=(-24.5, 12.3, -5.1),   # uit Calibrate_ICM20948.py
    mag_scale=(0.95, 1.02, 1.03),
)

print(hex(sensor.whoami))   # 0xea
print(sensor.acceleration)  # m/s^2
print(sensor.gyro)          # rad/s
print(sensor.magnetic)      # uT
print(sensor.temperature)   # graden C
```

Eenheden zijn te wijzigen met `accel_sf=SF_G` en `gyro_sf=SF_DEG_S` (zie `icm20948.py`). Voor de koersregelaar in `stepper_ramp.py` altijd `gyro_z_deg_s(sensor)` gebruiken: die zet rad/s om naar graden/s.

## Kalibratie

- **Magnetometer** (hard-iron offset + soft-iron scale): `Calibrate_ICM20948.py` of `sensor.ak09916.calibrate(count=256, delay=200)`. De uitkomst gaat als `mag_offset=` / `mag_scale=` naar de constructor.
- **Gyro-bias**: `sensor.calibrate()` met de sensor stil.

## Verschillen met de MPU-9250 / AK8963

- WHO_AM_I `0xEA` op register 0x00, met registerbanken (REG_BANK_SEL 0x7F); de driver regelt dat zelf.
- De AK09916 heeft geen fuse-ROM-gevoeligheidscorrectie (ASA) en altijd 16 bit, 0,15 µT/LSB.
- **Asuitlijning:** volgens de datasheet wijst de X-as van de AK09916 dezelfde kant op als die van de accel/gyro, maar staan Y en Z tegengesteld. `AK09916(align=True)` (standaard) zet dat om, zodat `magnetic` in hetzelfde assenstelsel staat als `acceleration` en `gyro`. Dat is nodig voor de kantelcompensatie in `fusion.py`. **Nog op hardware te controleren:** draai de robot 90° rechtsom; de magnetometerkoers en de geïntegreerde gyro-Z moeten dezelfde kant op veranderen.
- Standaardinstellingen: ±2 g, ±250 °/s, digitaal laagdoorlaatfilter ~51 Hz, samplefrequentie 225 Hz, magnetometer continu op 100 Hz.

## Datasheets

- ICM-20948: TDK InvenSense DS-000189
- AK09916: Asahi Kasei Microdevices AK09916C
