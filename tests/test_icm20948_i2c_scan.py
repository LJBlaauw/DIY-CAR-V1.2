import machine
import time

# --- Diagnose: is dit een echte ICM-20948 MET magnetometer? ---
# ICM-20948: GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
#
# Draai dit als los script (niet plakken in de REPL: multi-line plakken zonder
# paste-mode (Ctrl-E) raakt makkelijk in de war).
#
# Werkt bewust ZONDER de icm20948.py-driver, met kale registertoegang, zodat
# ook een verkeerd/namaak bordje herkend wordt in plaats van alleen een
# RuntimeError op te leveren. Verwacht resultaat bij een goed bordje:
#
#   voor bypass: ['0x68'] of ['0x69']      (adres hangt af van AD0)
#   WHO_AM_I    : 0xea                     -> ICM-20948
#   na bypass  : 0x0c erbij                -> AK09916-magnetometer aanwezig
#   AK09916 WIA : 0x48 / 0x09

_REG_BANK_SEL = 0x7F
_WHO_AM_I = 0x00         # bank 0 (ICM-20x48-familie)
_MPU_WHO_AM_I = 0x75     # MPU-6500/9250-familie, ter herkenning van namaak
_USER_CTRL = 0x03
_PWR_MGMT_1 = 0x06
_INT_PIN_CFG = 0x0F
_BYPASS_EN = 0x02
_I2C_MST_EN = 0x20

BEKEND = {
    0xEA: "ICM-20948 (9 assen, met AK09916-magnetometer)",
    0xE0: "ICM-20648 (6 assen, GEEN magnetometer)",
    0xE1: "ICM-20649 (6 assen, GEEN magnetometer)",
}
MPU_BEKEND = {
    0x70: "MPU-6500 (6 assen, GEEN magnetometer)",
    0x71: "MPU-9250",
    0x73: "MPU-9255",
}

i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)


def lees(adres, reg):
    return i2c.readfrom_mem(adres, reg, 1)[0]


def schrijf(adres, reg, waarde):
    i2c.writeto_mem(adres, reg, bytes((waarde,)))


voor = i2c.scan()
print("voor bypass:", [hex(a) for a in voor])

adres = None
for kandidaat in (0x69, 0x68):
    if kandidaat in voor:
        adres = kandidaat
        break

if adres is None:
    print("FOUT: niets op 0x68/0x69. Controleer bedrading, voeding (3V3) en pull-ups.")
    raise SystemExit

schrijf(adres, _REG_BANK_SEL, 0x00)
whoami = lees(adres, _WHO_AM_I)
print("adres:", hex(adres), " WHO_AM_I (0x00):", hex(whoami), "->", BEKEND.get(whoami, "onbekend"))

if whoami != 0xEA:
    mpu = lees(adres, _MPU_WHO_AM_I)
    print("WHO_AM_I MPU-stijl (0x75):", hex(mpu), "->", MPU_BEKEND.get(mpu, "onbekend"))
    print("FOUT: geen ICM-20948. Bordje is verkeerd geleverd of namaak.")
    raise SystemExit

# Wakker maken (na power-on staat de chip in SLEEP), interne I2C-master uit
# en BYPASS aan: de AK09916 hoort dan op 0x0C te verschijnen.
schrijf(adres, _PWR_MGMT_1, 0x01)
time.sleep_ms(50)
schrijf(adres, _USER_CTRL, lees(adres, _USER_CTRL) & ~_I2C_MST_EN)
schrijf(adres, _INT_PIN_CFG, lees(adres, _INT_PIN_CFG) | _BYPASS_EN)
time.sleep_ms(10)

na = i2c.scan()
print("na bypass:  ", [hex(a) for a in na])

if 0x0C not in na:
    print("FOUT: AK09916 (0x0c) verschijnt niet na bypass. Probeer ook 100 kHz:")
    i2c_100k = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=100000)
    print("op 100kHz:  ", [hex(a) for a in i2c_100k.scan()])
    raise SystemExit

wia1 = lees(0x0C, 0x00)
wia2 = lees(0x0C, 0x01)
print("AK09916 WIA1/WIA2:", hex(wia1), "/", hex(wia2), "(verwacht 0x48 / 0x09)")
if (wia1, wia2) == (0x48, 0x09):
    print("OK: echte ICM-20948 met werkende magnetometer.")
else:
    print("FOUT: onverwachte magnetometer-ID.")
