import machine
import time
from mpu6500 import MPU6500

# --- Diagnose: AK8963 (magnetometer) niet gevonden (EIO) ---
# GY9250 (MPU9250): GPIO10 (SDA1) / GPIO11 (SCL1) -> I2C-bus 1
#
# Draai dit als los script (niet plakken in de REPL: multi-line plakken zonder
# paste-mode (Ctrl-E) raakt makkelijk in de war en geeft precies dit soort
# door-elkaar-gelopen output).
#
# Doel: onderscheiden of de AK8963 er wel/niet is, VOORDAT de mpu9250.py/
# ak8963.py-driver er iets van vindt. Scant de I2C-bus vóór en ná het zetten
# van de BYPASS_EN-bit (dezelfde registerhandeling als in mpu9250.py).
#
#   - 0x0c verschijnt NA bypass, niet ervoor  -> AK8963 zit er, driver-timing
#     of iets anders is het probleem.
#   - 0x0c verschijnt NOOIT                   -> AK8963 reageert niet/ontbreekt
#     fysiek; geen softwareprobleem.

_INT_PIN_CFG = 0x37
_I2C_BYPASS_MASK = 0b00000010
_I2C_BYPASS_EN = 0b00000010

i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=400000)
print("voor bypass:", [hex(a) for a in i2c.scan()])

m = MPU6500(i2c)  # accel/gyro-deel; moet lukken (whoami 0x68 op de bus)
print("MPU6500 whoami:", hex(m.whoami))

char = m._register_char(_INT_PIN_CFG)
char &= ~_I2C_BYPASS_MASK
char |= _I2C_BYPASS_EN
m._register_char(_INT_PIN_CFG, char)
time.sleep_ms(10)

print("na bypass:  ", [hex(a) for a in i2c.scan()])

# Extra: dezelfde scan nog eens op 100 kHz, om signaalkwaliteit als oorzaak
# uit te sluiten (lange draden/breadboard kunnen bij 400 kHz net niet werken).
i2c_100k = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=100000)
print("op 100kHz:  ", [hex(a) for a in i2c_100k.scan()])
