"""
icm20948.py - MicroPython I2C-driver voor de TDK InvenSense ICM-20948.

De ICM-20948 is de opvolger van de MPU-9250: één behuizing (SiP) met een
3-assige gyroscoop + 3-assige accelerometer en een AK09916 3-assige
magnetometer (kompas). De magnetometer zit op een interne I2C-bus; deze
driver zet de BYPASS-modus aan zodat de AK09916 direct op de externe bus op
adres 0x0C verschijnt.

De publieke API volgt bewust die van de vroegere Tuupola-driver (voor de
inmiddels vervangen MPU-6500, zie globale_specificatie_NL.md), zodat
bestaande code (fusion.py, stepper_ramp.gyro_z_deg_s) ongewijzigd werkt:

    sensor.acceleration  -> (x, y, z) in m/s^2   (SF_G voor g)
    sensor.gyro          -> (x, y, z) in rad/s   (SF_DEG_S voor graden/s)
    sensor.magnetic      -> (x, y, z) in uT, gekalibreerd met offset/scale
    sensor.temperature   -> graden Celsius
    sensor.whoami        -> 0xEA

Verschil met de MPU-9250: de assen van de magnetometer worden hier al
uitgelijnd op die van de accelerometer/gyro (zie AK09916.align). Bij de
MPU-9250-driver was dat niet zo.

MONTAGE: `mount=(sx, sy, sz)` zet het assenstelsel van de SENSOR om naar dat
van de KAR (X naar voren, Y naar links, Z omhoog) door per as het teken om te
draaien. Geldt voor acceleration, gyro en magnetic van het ICM20948-object.
Huidige montage (2026-10-26, 180 graden om Z): mount=(-1, -1, 1), staat in
config.json. Alleen tekenwisselingen; een draai van 90 graden (assen
verwisselen) wordt niet ondersteund.

    sensor.ak09916.magnetic  -> SENSOR-assenstelsel (voor de kalibratiescripts)
    sensor.magnetic          -> KAR-assenstelsel

De mag-kalibratie (offset/scale) staat in het SENSOR-assenstelsel en wordt
toegepast VOOR de montage-omzetting. Daardoor blijft een kalibratie geldig als
de montage-instelling verandert, en hoeven de kalibratiescripts niets van de
montage te weten.

Datasheets:
  ICM-20948: DS-000189 (TDK InvenSense)
  AK09916:   AKM AK09916C
"""

import ustruct
import utime
from micropython import const

# --- ICM-20948 registers -----------------------------------------------------
# De registers zijn verdeeld over 4 banken; REG_BANK_SEL (0x7F) bestaat in elke
# bank. Bank 0: status/data, bank 2: configuratie van gyro en accelerometer.
_REG_BANK_SEL = const(0x7F)

# Bank 0
_WHO_AM_I = const(0x00)
_USER_CTRL = const(0x03)
_PWR_MGMT_1 = const(0x06)
_PWR_MGMT_2 = const(0x07)
_INT_PIN_CFG = const(0x0F)
_ACCEL_XOUT_H = const(0x2D)
_GYRO_XOUT_H = const(0x33)
_TEMP_OUT_H = const(0x39)

# Bank 2
_GYRO_SMPLRT_DIV = const(0x00)
_GYRO_CONFIG_1 = const(0x01)
_ACCEL_SMPLRT_DIV_1 = const(0x10)
_ACCEL_SMPLRT_DIV_2 = const(0x11)
_ACCEL_CONFIG = const(0x14)

_WHOAMI_ICM20948 = const(0xEA)

_USER_CTRL_I2C_MST_EN = const(0x20)
_PWR_MGMT_1_RESET = const(0x80)
_PWR_MGMT_1_CLK_AUTO = const(0x01)   # SLEEP=0, beste beschikbare klok
_INT_PIN_CFG_BYPASS_EN = const(0x02)

# Volle schaal (waarde van het FS_SEL-veld; wordt intern op bit 2:1 gezet)
ACCEL_FS_SEL_2G = const(0)
ACCEL_FS_SEL_4G = const(1)
ACCEL_FS_SEL_8G = const(2)
ACCEL_FS_SEL_16G = const(3)

GYRO_FS_SEL_250DPS = const(0)
GYRO_FS_SEL_500DPS = const(1)
GYRO_FS_SEL_1000DPS = const(2)
GYRO_FS_SEL_2000DPS = const(3)

_ACCEL_SO = (16384, 8192, 4096, 2048)   # LSB per g
_GYRO_SO = (131.0, 65.5, 32.8, 16.4)    # LSB per graad/s

# Digitaal laagdoorlaatfilter (DLPFCFG 0..7). 3 = ca. 51 Hz -3dB voor zowel
# gyro als accelerometer: dempt motortrillingen, ruim snel genoeg voor 50 Hz
# koersregeling en 200 Hz fusion.
DLPF_51HZ = const(3)

# Samplefrequentie = 1125 Hz / (1 + div). 4 -> 225 Hz.
_SMPLRT_DIV = const(4)

_TEMP_SO = 333.87
_TEMP_OFFSET = 21

SF_G = 1
SF_M_S2 = 9.80665  # 1 g = 9.80665 m/s2, standaard zwaartekracht
SF_DEG_S = 1
SF_RAD_S = 0.017453292519943  # 1 graad/s = 0.017453292519943 rad/s

# --- AK09916 registers -------------------------------------------------------
_AK_ADDRESS = const(0x0C)
_AK_WIA2 = const(0x01)
_AK_ST1 = const(0x10)
_AK_CNTL2 = const(0x31)
_AK_CNTL3 = const(0x32)

_AK_WHOAMI = const(0x09)            # WIA2, device ID van de AK09916
_AK_ST2_HOFL = const(0x08)          # magnetische overflow, meting ongeldig

MODE_POWER_DOWN = const(0x00)
MODE_SINGLE_MEASURE = const(0x01)
MODE_CONTINUOUS_10HZ = const(0x02)
MODE_CONTINUOUS_20HZ = const(0x04)
MODE_CONTINUOUS_50HZ = const(0x06)
MODE_CONTINUOUS_100HZ = const(0x08)

_AK_SO = 0.15  # uT per LSB (vast 16-bit, geen fuse-ROM-correctie zoals bij AK8963)


CONFIG_PATH = "config.json"


def load_mag_calibration(path=CONFIG_PATH):
    """Lees mag_offset/mag_scale en mount uit de "icm20948"-sectie van config.json.

    Ontbreekt de sectie of is het bestand corrupt, dan volgt (0,0,0)/(1,1,1):
    ongekalibreerd, maar bruikbaar. Ontbreekt alleen "mount", dan (1,1,1). Bewust geen automatische inlezing in de
    constructor: de kalibratiescripts hebben juist ongekalibreerde waarden nodig.

        sensor = ICM20948(i2c, **load_mag_calibration())
    """
    try:
        import json
        with open(path) as f:
            cfg = json.load(f).get("icm20948", {})
        return {"mag_offset": tuple(cfg["mag_offset"]),
                "mag_scale": tuple(cfg["mag_scale"]),
                "mount": tuple(cfg.get("mount", (1, 1, 1)))}
    except Exception:
        return {"mag_offset": (0, 0, 0), "mag_scale": (1, 1, 1), "mount": (1, 1, 1)}


class ICM20948:
    """Gyroscoop + accelerometer + (via bypass) AK09916-magnetometer."""

    def __init__(
        self, i2c, address=None,
        accel_fs=ACCEL_FS_SEL_2G, gyro_fs=GYRO_FS_SEL_250DPS,
        accel_sf=SF_M_S2, gyro_sf=SF_RAD_S,
        gyro_offset=(0, 0, 0),
        dlpf=DLPF_51HZ,
        ak09916=None, mag_offset=(0, 0, 0), mag_scale=(1, 1, 1),
        mount=(1, 1, 1),
    ):
        """
        address=None zoekt zelf: 0x69 (AD0 hoog, o.a. Adafruit/SparkFun) en
        daarna 0x68 (AD0 laag, veel Chinese bordjes).

        mag_offset/mag_scale zijn de hard-/soft-iron-kalibratiewaarden uit
        Calibrate_ICM20948.py. Wordt een eigen `ak09916` meegegeven, dan
        worden die genegeerd.

        mount: teken per as, sensor -> kar (zie de kop van dit bestand).
        gyro_offset staat in het KAR-assenstelsel (zoals calibrate() hem bepaalt).
        """
        for v in mount:
            if v not in (1, -1):
                raise ValueError("mount moet uit +1/-1 bestaan, kreeg %r" % (mount,))
        self._mount = tuple(mount)
        self.i2c = i2c
        self._bank = -1
        self.address = self._find(address)

        # Reset; na reset staat de chip in SLEEP
        self._write(0, _PWR_MGMT_1, _PWR_MGMT_1_RESET)
        utime.sleep_ms(100)
        self._bank = -1  # reset zet REG_BANK_SEL terug op 0
        self._write(0, _PWR_MGMT_1, _PWR_MGMT_1_CLK_AUTO)
        utime.sleep_ms(50)
        self._write(0, _PWR_MGMT_2, 0x00)  # alle assen van accel en gyro aan

        # Configuratie (bank 2): samplefrequentie, volle schaal, DLPF
        self._write(2, _GYRO_SMPLRT_DIV, _SMPLRT_DIV)
        self._write(2, _GYRO_CONFIG_1, (dlpf << 3) | (gyro_fs << 1) | 1)
        self._write(2, _ACCEL_SMPLRT_DIV_1, 0)
        self._write(2, _ACCEL_SMPLRT_DIV_2, _SMPLRT_DIV)
        self._write(2, _ACCEL_CONFIG, (dlpf << 3) | (accel_fs << 1) | 1)

        self._accel_so = _ACCEL_SO[accel_fs]
        self._gyro_so = _GYRO_SO[gyro_fs]
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf
        self._gyro_offset = gyro_offset

        # Interne I2C-master uit en BYPASS aan: AK09916 verschijnt op 0x0C
        user_ctrl = self._read(0, _USER_CTRL)
        self._write(0, _USER_CTRL, user_ctrl & ~_USER_CTRL_I2C_MST_EN)
        pin_cfg = self._read(0, _INT_PIN_CFG)
        self._write(0, _INT_PIN_CFG, pin_cfg | _INT_PIN_CFG_BYPASS_EN)
        utime.sleep_ms(10)

        if ak09916 is None:
            ak09916 = AK09916(i2c, offset=mag_offset, scale=mag_scale)
        self.ak09916 = ak09916

    @property
    def acceleration(self):
        """(x, y, z) in m/s^2, of in g bij accel_sf=SF_G."""
        so = self._accel_so
        sf = self._accel_sf
        m = self._mount
        x, y, z = self._read_three_shorts(_ACCEL_XOUT_H)
        return (m[0] * x / so * sf, m[1] * y / so * sf, m[2] * z / so * sf)

    @property
    def gyro(self):
        """(x, y, z) in rad/s, of in graden/s bij gyro_sf=SF_DEG_S."""
        so = self._gyro_so
        sf = self._gyro_sf
        m = self._mount
        ox, oy, oz = self._gyro_offset
        x, y, z = self._read_three_shorts(_GYRO_XOUT_H)
        return (m[0] * x / so * sf - ox, m[1] * y / so * sf - oy, m[2] * z / so * sf - oz)

    @property
    def magnetic(self):
        """(x, y, z) in uT, gekalibreerd, in het KAR-assenstelsel (na mount)."""
        m = self._mount
        x, y, z = self.ak09916.magnetic
        return (m[0] * x, m[1] * y, m[2] * z)

    @property
    def temperature(self):
        """Temperatuur van de chip in graden Celsius."""
        buf = bytearray(2)
        self._select_bank(0)
        self.i2c.readfrom_mem_into(self.address, _TEMP_OUT_H, buf)
        temp = ustruct.unpack(">h", buf)[0]
        return temp / _TEMP_SO + _TEMP_OFFSET

    @property
    def whoami(self):
        """Waarde van het WHO_AM_I-register (0xEA voor een echte ICM-20948)."""
        return self._read(0, _WHO_AM_I)

    def calibrate(self, count=256, delay=0):
        """Gyro-bias bepalen. De sensor moet hierbij stil liggen."""
        ox = oy = oz = 0.0
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        while count:
            utime.sleep_ms(delay)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz
            count -= 1

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    def _find(self, address):
        candidates = (0x69, 0x68) if address is None else (address,)
        found = self.i2c.scan()
        for addr in candidates:
            if addr in found:
                self.address = addr
                self._bank = -1
                if self._read(0, _WHO_AM_I) == _WHOAMI_ICM20948:
                    return addr
        raise RuntimeError(
            "ICM-20948 niet gevonden op I2C-bus (gezocht: {}, bus: {})".format(
                [hex(a) for a in candidates], [hex(a) for a in found]))

    def _select_bank(self, bank):
        if bank != self._bank:
            self.i2c.writeto_mem(self.address, _REG_BANK_SEL, bytes((bank << 4,)))
            self._bank = bank

    def _read(self, bank, register):
        self._select_bank(bank)
        return self.i2c.readfrom_mem(self.address, register, 1)[0]

    def _write(self, bank, register, value):
        self._select_bank(bank)
        self.i2c.writeto_mem(self.address, register, bytes((value & 0xFF,)))

    def _read_three_shorts(self, register, buf=bytearray(6)):
        self._select_bank(0)
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass


class AK09916:
    """AK09916-magnetometer in de ICM-20948 (bereikbaar na BYPASS_EN).

    Maak dit object pas aan NA ICM20948(i2c): pas dan staat de bypass open.
    """

    def __init__(
        self, i2c, address=_AK_ADDRESS, mode=MODE_CONTINUOUS_100HZ,
        offset=(0, 0, 0), scale=(1, 1, 1), align=True,
    ):
        """
        align=True zet de magnetometerassen om naar het assenstelsel van de
        accelerometer/gyro. Volgens de ICM-20948-datasheet wijst X van de
        AK09916 dezelfde kant op, maar staan Y en Z tegengesteld. Zonder deze
        omzetting klopt de kantelcompensatie in fusion.py niet.
        """
        self.i2c = i2c
        self.address = address
        self._offset = offset
        self._scale = scale
        self._sign = (1, -1, -1) if align else (1, 1, 1)
        self._last = (0.0, 0.0, 0.0)
        self._buf = bytearray(9)

        if self.whoami != _AK_WHOAMI:
            raise RuntimeError("AK09916 niet gevonden op I2C-bus (bypass niet open?)")

        # Soft reset, daarna via power-down naar de gevraagde meetmodus
        self._write(_AK_CNTL3, 0x01)
        utime.sleep_ms(10)
        self._write(_AK_CNTL2, MODE_POWER_DOWN)
        utime.sleep_ms(1)
        self._write(_AK_CNTL2, mode)
        utime.sleep_ms(10)

    @property
    def magnetic(self):
        """(x, y, z) in uT, na uitlijning, hard-iron offset en soft-iron scale.

        Bij een magnetische overflow (HOFL) wordt de vorige geldige meting
        teruggegeven.
        """
        # Burst ST1..ST2 (9 bytes). Het lezen van ST2 geeft de volgende meting
        # vrij; zonder dat blijft de AK09916 op de oude waarde hangen.
        buf = self._buf
        self.i2c.readfrom_mem_into(self.address, _AK_ST1, buf)
        if buf[8] & _AK_ST2_HOFL:
            return self._last

        raw = ustruct.unpack_from("<hhh", buf, 1)
        sign = self._sign
        offset = self._offset
        scale = self._scale
        self._last = tuple(
            (raw[i] * _AK_SO * sign[i] - offset[i]) * scale[i] for i in range(3)
        )
        return self._last

    @property
    def whoami(self):
        """Waarde van WIA2 (0x09 voor de AK09916)."""
        return self.i2c.readfrom_mem(self.address, _AK_WIA2, 1)[0]

    def calibrate(self, count=256, delay=200):
        """Hard-iron offset en soft-iron scale bepalen via min/max per as.

        Draai de sensor tijdens het meten langzaam rond alle assen.
        """
        self._offset = (0, 0, 0)
        self._scale = (1, 1, 1)

        reading = self.magnetic
        lo = list(reading)
        hi = list(reading)

        while count:
            utime.sleep_ms(delay)
            reading = self.magnetic
            for i in range(3):
                lo[i] = min(lo[i], reading[i])
                hi[i] = max(hi[i], reading[i])
            count -= 1

        self._offset = tuple((hi[i] + lo[i]) / 2 for i in range(3))
        chord = [(hi[i] - lo[i]) / 2 or 1e-6 for i in range(3)]  # voorkom deling door nul
        avg = sum(chord) / 3
        self._scale = tuple(avg / c for c in chord)

        return self._offset, self._scale

    def _write(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes((value,)))

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
