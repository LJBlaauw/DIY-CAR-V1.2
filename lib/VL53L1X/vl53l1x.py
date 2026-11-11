# vl53l1x.py
#
# MicroPython-driver voor de VL53L1X time-of-flight-afstandssensor.
#
# Poort van de Pololu VL53L1X Arduino-library (zelf gebaseerd op ST's officiele
# "Ultra Lite Driver", STSW-IMG007/009): https://github.com/pololu/vl53l1x-arduino
# Registeradressen, de init-sequentie, en de timeout-encodering/-decodering
# zijn 1-op-1 overgenomen (rechtstreeks tegen de broncode gecontroleerd, niet
# uit het geheugen gereconstrueerd -- een foute registerwaarde hier geeft geen
# foutmelding, alleen een stille, verkeerde afstand).
#
# I2C: standaardadres 0x29, deelbaar met andere apparaten op dezelfde bus
# (bv. de SSD1306-OLED op 0x3C) zolang de adressen niet overlappen.
#
# GPIO1 (interrupt-uitgang): open-drain, active-low, gaat laag zodra een
# nieuwe meting gereed is. Vereist een externe pull-up (datasheet: 10 kOhm)
# naar de 3V3-rail van de microcontroller -- NIET naar VIN van de sensor als
# die op 5V staat (zie de toelichting elders: niet alle breakoutbordjes
# leveren GPIO1 als level-shifted signaal).
#
# Gebruik (zie tests/test_TOF.py voor een volledig voorbeeld):
#
#     tof = VL53L1X(i2c)
#     tof.init()
#     tof.set_distance_mode(VL53L1X.LONG)
#     tof.set_timing_budget_us(33000)
#     tof.start_continuous(50)          # 50 ms inter-measurement period
#     ...
#     if tof.data_ready():              # of: via de GPIO1-interrupt
#         mm = tof.read()                # leest, kalibreert zo nodig, clear't de interrupt
#
# GEEN I2C-aanroepen vanuit een hard-IRQ-handler (zie lib/LDR/ldr_scan_isr.py
# en lib/ultrasoon/ultrasoon.py voor de reden): de GPIO1-IRQ in het testscript
# zet daarom alleen een vlag; het echte werk (lezen + clear_interrupt) gebeurt
# in de hoofdlus.

import time

# ---- Registeradressen (16-bit, big-endian op de bus) ----
SOFT_RESET = 0x0000
I2C_SLAVE__DEVICE_ADDRESS = 0x0001
ANA_CONFIG__VHV_REF_SEL_VDDPIX = 0x0002
OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006
VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008
VHV_CONFIG__INIT = 0x000B
ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E
MM_CONFIG__OUTER_OFFSET_MM = 0x0022
DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024
PAD_I2C_HV__CONFIG = 0x002D
PAD_I2C_HV__EXTSUP_CONFIG = 0x002E
GPIO_HV_PAD__CTRL = 0x002F
GPIO__TIO_HV_STATUS = 0x0031
GPIO__FIO_HV_STATUS = 0x0032
SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036
SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037
ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039
ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E
ALGO__RANGE_MIN_CLIP = 0x003F
ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040
CAL_CONFIG__VCSEL_START = 0x0047
SYSTEM__INTERRUPT_CONFIG_GPIO = 0x0046
PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B
PHASECAL_CONFIG__OVERRIDE = 0x004D
DSS_CONFIG__ROI_MODE_CONTROL = 0x004F
SYSTEM__THRESH_RATE_HIGH = 0x0050
SYSTEM__THRESH_RATE_LOW = 0x0052
DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054
DSS_CONFIG__APERTURE_ATTENUATION = 0x0057
MM_CONFIG__TIMEOUT_MACROP_A = 0x005A
MM_CONFIG__TIMEOUT_MACROP_B = 0x005C
RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E
RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060
RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061
RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063
RANGE_CONFIG__SIGMA_THRESH = 0x0064
RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066
RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069
SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C
SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071
SYSTEM__SEED_CONFIG = 0x0077
SD_CONFIG__WOI_SD0 = 0x0078
SD_CONFIG__WOI_SD1 = 0x0079
SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A
SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B
SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C
SD_CONFIG__QUANTIFIER = 0x007E
ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F
ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080
SYSTEM__SEQUENCE_CONFIG = 0x0081
SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082
SYSTEM__INTERRUPT_CLEAR = 0x0086
SYSTEM__MODE_START = 0x0087
RESULT__RANGE_STATUS = 0x0089
PHASECAL_RESULT__REFERENCE_PHASE = 0x00D6
PHASECAL_RESULT__VCSEL_START = 0x00D8
VHV_RESULT__COLDBOOT_STATUS = 0x00DB
RESULT__OSC_CALIBRATE_VAL = 0x00DE
ANA_CONFIG__POWERDOWN_GO1 = 0x00E0
FIRMWARE__SYSTEM_STATUS = 0x00E5
FIRMWARE__MODE_STATUS = 0x00E6
IDENTIFICATION__MODEL_ID = 0x010F
IDENTIFICATION__MODULE_TYPE = 0x0110
IDENTIFICATION__REVISION_ID = 0x0111
IDENTIFICATION__MODULE_ID = 0x0112

# ---- Config (config.json, sectie "vl53l1x"); onderstaande waarde is de fallback ----
CONFIG_PATH = "config.json"


def _load_vl53l1x_config():
    """Lees de "vl53l1x"-sectie uit config.json; {} als die ontbreekt of corrupt is.
    Mag nooit een crash bij het opstarten veroorzaken (zelfde patroon als
    lib/servo/servo_crl.py en lib/LDR/ldr_scan_isr.py)."""
    try:
        import json
        with open(CONFIG_PATH) as f:
            return json.load(f).get("vl53l1x", {})
    except Exception:
        return {}


_cfg = _load_vl53l1x_config()

# ---- Constanten (Pololu VL53L1X.h) ----
ADDRESS_DEFAULT = 0x29         # 0b0101001
MODEL_ID_EXPECTED = 0xEACC
TIMING_GUARD = 4528            # us, LOWPOWER_AUTO_OVERHEAD
TARGET_RATE = 0x0A00           # DSS-doelwaarde

# [GEMETEN 2026-11-08] Dit exemplaar meet structureel te kort, een vrijwel
# pure schaalfout (geen vaste offset): 9 punten tussen 10 en 250cm gaven
# gemeten = 0,9544 * werkelijk - 0,2mm (R^2 = 0,9998) -- de kleine offset valt
# binnen de meetruis van een los meetlint, dus hier als pure schaalcorrectie
# toegepast. Te schoon lineair (over een factor 25 in afstand) om een
# softwarefout te zijn; waarschijnlijk een kalibratie-eigenschap van dit
# specifieke exemplaar/bordje. RANGE_SCALE_CORRECTION = 1/0,9544. Komt uit
# config.json als die sectie bestaat, anders deze waarde als fallback.
RANGE_SCALE_CORRECTION = float(_cfg.get("range_scale_correction", 1.04779))

# distance modes
UNKNOWN = 0
SHORT = 1
MEDIUM = 2
LONG = 3

# range_status (ranging_data), zie getRangingData()
RANGE_VALID = 0
SIGMA_FAIL = 1
SIGNAL_FAIL = 2
RANGE_VALID_MIN_RANGE_CLIPPED = 3
OUT_OF_BOUNDS_FAIL = 4
HARDWARE_FAIL = 5
RANGE_VALID_NO_WRAP_CHECK_FAIL = 6
WRAP_TARGET_FAIL = 7
XTALK_SIGNAL_FAIL = 8
SYNCHRONIZATION_INT = 9
MIN_RANGE_FAIL = 10
NONE_STATUS = 11

_STATUS_NAMES = {
    RANGE_VALID: "range valid",
    SIGMA_FAIL: "sigma fail",
    SIGNAL_FAIL: "signal fail",
    RANGE_VALID_MIN_RANGE_CLIPPED: "range valid, min range clipped",
    OUT_OF_BOUNDS_FAIL: "out of bounds fail",
    HARDWARE_FAIL: "hardware fail",
    RANGE_VALID_NO_WRAP_CHECK_FAIL: "range valid, no wrap check fail",
    WRAP_TARGET_FAIL: "wrap target fail",
    XTALK_SIGNAL_FAIL: "xtalk signal fail",
    SYNCHRONIZATION_INT: "synchronization int",
    MIN_RANGE_FAIL: "min range fail",
    NONE_STATUS: "no update",
}


def status_name(status):
    return _STATUS_NAMES.get(status, "unknown status")


# ---- Timeout-rekenhulp (VL53L1_calc_timeout_us/-mclks, decode/encodeTimeout) ----

def _decode_timeout(reg_val):
    return ((reg_val & 0xFF) << (reg_val >> 8)) + 1


def _encode_timeout(timeout_mclks):
    if timeout_mclks <= 0:
        return 0
    ls_byte = timeout_mclks - 1
    ms_byte = 0
    while (ls_byte & 0xFFFFFF00) > 0:
        ls_byte >>= 1
        ms_byte += 1
    return (ms_byte << 8) | (ls_byte & 0xFF)


def _timeout_mclks_to_us(timeout_mclks, macro_period_us):
    return (timeout_mclks * macro_period_us + 0x800) >> 12


def _timeout_us_to_mclks(timeout_us, macro_period_us):
    return ((timeout_us << 12) + (macro_period_us >> 1)) // macro_period_us


class VL53L1X:
    def __init__(self, i2c, address=ADDRESS_DEFAULT):
        self.i2c = i2c
        self.addr = address
        self.io_timeout_ms = 0          # 0 = geen timeout (zoals Pololu-default)
        self.did_timeout = False
        self.calibrated = False
        self.saved_vhv_init = 0
        self.saved_vhv_timeout = 0
        self.distance_mode = UNKNOWN
        self.fast_osc_frequency = 0
        self.osc_calibrate_val = 0
        # laatst gelezen resultaat (readResults())
        self._range_status_raw = 0
        self._stream_count = 0
        self._dss_actual_effective_spads = 0
        self._ambient_count_rate = 0
        self._peak_signal_count_rate_xtalk = 0
        # publiek resultaat (getRangingData())
        self.range_mm = 0
        self.range_status = NONE_STATUS

    # ---------- lage-niveau I2C (16-bit registeradres, big-endian) ----------
    def _write(self, reg, data):
        self.i2c.writeto(self.addr, bytes((reg >> 8, reg & 0xFF)) + data)

    def _write8(self, reg, val):
        self._write(reg, bytes((val & 0xFF,)))

    def _write16(self, reg, val):
        self._write(reg, bytes(((val >> 8) & 0xFF, val & 0xFF)))

    def _write32(self, reg, val):
        self._write(reg, bytes(((val >> 24) & 0xFF, (val >> 16) & 0xFF,
                                 (val >> 8) & 0xFF, val & 0xFF)))

    def _read(self, reg, n):
        self.i2c.writeto(self.addr, bytes((reg >> 8, reg & 0xFF)), False)
        return self.i2c.readfrom(self.addr, n)

    def _read8(self, reg):
        return self._read(reg, 1)[0]

    def _read16(self, reg):
        b = self._read(reg, 2)
        return (b[0] << 8) | b[1]

    # ---------- init (VL53L1_DataInit + VL53L1_StaticInit) ----------
    def init(self, io_2v8=True, timeout_ms=1000):
        if self._read16(IDENTIFICATION__MODEL_ID) != MODEL_ID_EXPECTED:
            return False

        self._write8(SOFT_RESET, 0x00)
        time.sleep_us(100)
        self._write8(SOFT_RESET, 0x01)
        time.sleep_ms(1)

        t0 = time.ticks_ms()
        while (self._read8(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                self.did_timeout = True
                return False

        if io_2v8:
            self._write8(PAD_I2C_HV__EXTSUP_CONFIG,
                         self._read8(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01)

        self.fast_osc_frequency = self._read16(OSC_MEASURED__FAST_OSC__FREQUENCY)
        self.osc_calibrate_val = self._read16(RESULT__OSC_CALIBRATE_VAL)

        # static config
        self._write16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE)
        self._write8(GPIO__TIO_HV_STATUS, 0x02)
        self._write8(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8)
        self._write8(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16)
        self._write8(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01)
        self._write8(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF)
        self._write8(ALGO__RANGE_MIN_CLIP, 0)
        self._write8(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2)

        # general config
        self._write16(SYSTEM__THRESH_RATE_HIGH, 0x0000)
        self._write16(SYSTEM__THRESH_RATE_LOW, 0x0000)
        self._write8(DSS_CONFIG__APERTURE_ATTENUATION, 0x38)

        # timing config
        self._write16(RANGE_CONFIG__SIGMA_THRESH, 360)
        self._write16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192)

        # dynamic config
        self._write8(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01)
        self._write8(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01)
        self._write8(SD_CONFIG__QUANTIFIER, 2)

        self._write8(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00)
        self._write8(SYSTEM__SEED_CONFIG, 1)

        self._write8(SYSTEM__SEQUENCE_CONFIG, 0x8B)
        self._write16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8)
        self._write8(DSS_CONFIG__ROI_MODE_CONTROL, 2)

        self.set_distance_mode(LONG)
        self.set_timing_budget_us(50000)

        self._write16(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
                      self._read16(MM_CONFIG__OUTER_OFFSET_MM) * 4)
        return True

    # ---------- distance mode / timing budget ----------
    def set_distance_mode(self, mode):
        budget_us = self.get_timing_budget_us()

        if mode == SHORT:
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07)
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05)
            self._write8(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38)
            self._write8(SD_CONFIG__WOI_SD0, 0x07)
            self._write8(SD_CONFIG__WOI_SD1, 0x05)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD0, 6)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD1, 6)
        elif mode == MEDIUM:
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B)
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09)
            self._write8(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78)
            self._write8(SD_CONFIG__WOI_SD0, 0x0B)
            self._write8(SD_CONFIG__WOI_SD1, 0x09)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD0, 10)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD1, 10)
        elif mode == LONG:
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F)
            self._write8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D)
            self._write8(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8)
            self._write8(SD_CONFIG__WOI_SD0, 0x0F)
            self._write8(SD_CONFIG__WOI_SD1, 0x0D)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD0, 14)
            self._write8(SD_CONFIG__INITIAL_PHASE_SD1, 14)
        else:
            return False

        self.set_timing_budget_us(budget_us)
        self.distance_mode = mode
        return True

    def _calc_macro_period_us(self, vcsel_period):
        pll_period_us = (1 << 30) // self.fast_osc_frequency
        vcsel_period_pclks = (vcsel_period + 1) << 1
        macro_period_us = 2304 * pll_period_us
        macro_period_us >>= 6
        macro_period_us *= vcsel_period_pclks
        macro_period_us >>= 6
        return macro_period_us

    def set_timing_budget_us(self, budget_us):
        if budget_us <= TIMING_GUARD:
            return False
        range_config_timeout_us = budget_us - TIMING_GUARD
        if range_config_timeout_us > 1100000:
            return False
        range_config_timeout_us //= 2

        macro_period_us = self._calc_macro_period_us(
            self._read8(RANGE_CONFIG__VCSEL_PERIOD_A))

        phasecal_timeout_mclks = _timeout_us_to_mclks(1000, macro_period_us)
        if phasecal_timeout_mclks > 0xFF:
            phasecal_timeout_mclks = 0xFF
        self._write8(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks)

        self._write16(MM_CONFIG__TIMEOUT_MACROP_A,
                      _encode_timeout(_timeout_us_to_mclks(1, macro_period_us)))
        self._write16(RANGE_CONFIG__TIMEOUT_MACROP_A,
                      _encode_timeout(_timeout_us_to_mclks(range_config_timeout_us, macro_period_us)))

        macro_period_us = self._calc_macro_period_us(
            self._read8(RANGE_CONFIG__VCSEL_PERIOD_B))

        self._write16(MM_CONFIG__TIMEOUT_MACROP_B,
                      _encode_timeout(_timeout_us_to_mclks(1, macro_period_us)))
        self._write16(RANGE_CONFIG__TIMEOUT_MACROP_B,
                      _encode_timeout(_timeout_us_to_mclks(range_config_timeout_us, macro_period_us)))
        return True

    def get_timing_budget_us(self):
        macro_period_us = self._calc_macro_period_us(
            self._read8(RANGE_CONFIG__VCSEL_PERIOD_A))
        range_config_timeout_us = _timeout_mclks_to_us(
            _decode_timeout(self._read16(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us)
        return 2 * range_config_timeout_us + TIMING_GUARD

    # ---------- continu meten ----------
    def start_continuous(self, period_ms):
        """Start autonome, doorlopende metingen om de `period_ms` ms (moet >=
        de timing budget zijn, met marge). Vereist geen verdere host-actie
        tot je read()/clear_interrupt() aanroept of stop_continuous()."""
        self._write32(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * self.osc_calibrate_val)
        self._write8(SYSTEM__INTERRUPT_CLEAR, 0x01)
        self._write8(SYSTEM__MODE_START, 0x40)   # mode_range__timed

    def stop_continuous(self):
        self._write8(SYSTEM__MODE_START, 0x80)   # mode_range__abort
        self.calibrated = False
        if self.saved_vhv_init != 0:
            self._write8(VHV_CONFIG__INIT, self.saved_vhv_init)
        if self.saved_vhv_timeout != 0:
            self._write8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, self.saved_vhv_timeout)
        self._write8(PHASECAL_CONFIG__OVERRIDE, 0x00)

    def data_ready(self):
        """Niet-blokkerend: True als een nieuwe meting gereed is (zelfde bit
        als GPIO1 bewaakt). Gebruik dit voor polling; met de interrupt (GPIO1)
        hoef je dit niet aan te roepen, lees dan rechtstreeks na de IRQ."""
        return (self._read8(GPIO__TIO_HV_STATUS) & 0x01) == 0

    def clear_interrupt(self):
        self._write8(SYSTEM__INTERRUPT_CLEAR, 0x01)

    # ---------- resultaat lezen ----------
    def _setup_manual_calibration(self):
        self.saved_vhv_init = self._read8(VHV_CONFIG__INIT)
        self.saved_vhv_timeout = self._read8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND)
        self._write8(VHV_CONFIG__INIT, self.saved_vhv_init & 0x7F)
        self._write8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                     (self.saved_vhv_timeout & 0x03) + (3 << 2))
        self._write8(PHASECAL_CONFIG__OVERRIDE, 0x01)
        self._write8(CAL_CONFIG__VCSEL_START, self._read8(PHASECAL_RESULT__VCSEL_START))

    def _read_results(self):
        buf = self._read(RESULT__RANGE_STATUS, 17)
        self._range_status_raw = buf[0]
        # buf[1] = report_status, niet gebruikt
        self._stream_count = buf[2]
        self._dss_actual_effective_spads = (buf[3] << 8) | buf[4]
        # buf[5:7] = peak_signal_count_rate (ongecorrigeerd), niet gebruikt
        self._ambient_count_rate = (buf[7] << 8) | buf[8]
        # buf[9:13] = sigma/phase, niet gebruikt
        self._final_range_mm = (buf[13] << 8) | buf[14]
        self._peak_signal_count_rate_xtalk = (buf[15] << 8) | buf[16]

    def _update_dss(self):
        spad_count = self._dss_actual_effective_spads
        if spad_count != 0:
            total_rate = self._peak_signal_count_rate_xtalk + self._ambient_count_rate
            if total_rate > 0xFFFF:
                total_rate = 0xFFFF
            total_rate <<= 16
            total_rate //= spad_count
            if total_rate != 0:
                required_spads = (TARGET_RATE << 16) // total_rate
                if required_spads > 0xFFFF:
                    required_spads = 0xFFFF
                self._write16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, required_spads)
                return
        self._write16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000)

    def _get_ranging_data(self):
        # correctiefactor 2011/2048 (~98%), ST-tuningparameter default
        range_mm = (self._final_range_mm * 2011 + 0x0400) // 0x0800
        # empirische schaalcorrectie voor dit exemplaar, zie RANGE_SCALE_CORRECTION
        self.range_mm = round(range_mm * RANGE_SCALE_CORRECTION)

        s = self._range_status_raw
        if s in (17, 2, 1, 3):
            self.range_status = HARDWARE_FAIL
        elif s == 13:
            self.range_status = MIN_RANGE_FAIL
        elif s == 18:
            self.range_status = SYNCHRONIZATION_INT
        elif s == 5:
            self.range_status = OUT_OF_BOUNDS_FAIL
        elif s == 4:
            self.range_status = SIGNAL_FAIL
        elif s == 6:
            self.range_status = SIGMA_FAIL
        elif s == 7:
            self.range_status = WRAP_TARGET_FAIL
        elif s == 12:
            self.range_status = XTALK_SIGNAL_FAIL
        elif s == 8:
            self.range_status = RANGE_VALID_MIN_RANGE_CLIPPED
        elif s == 9:
            self.range_status = (RANGE_VALID_NO_WRAP_CHECK_FAIL if self._stream_count == 0
                                  else RANGE_VALID)
        else:
            self.range_status = NONE_STATUS

    def read(self, blocking=True, timeout_ms=None):
        """Geef de afstand in mm. Met blocking=True (default) wordt gewacht
        tot data_ready(); anders wordt meteen het (mogelijk oude) resultaat
        verwerkt. Clear't zelf de interrupt aan het eind."""
        if blocking:
            t0 = time.ticks_ms()
            budget = self.io_timeout_ms if timeout_ms is None else timeout_ms
            while not self.data_ready():
                if budget and time.ticks_diff(time.ticks_ms(), t0) > budget:
                    self.did_timeout = True
                    return 0

        self._read_results()
        if not self.calibrated:
            self._setup_manual_calibration()
            self.calibrated = True
        self._update_dss()
        self._get_ranging_data()
        self.clear_interrupt()
        return self.range_mm
