# ultrasoon.py
from machine import Pin
import rp2, micropython

# ====== PIO-programma ======
@rp2.asm_pio(
    set_init=rp2.PIO.OUT_LOW,
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopush=False,
)
def usonic_pio():
    pull(block)                 # OSR = interval_cycles
    mov(y, osr)                 # Y = interval (persistent)
    pull(block)                 # OSR = timeout_us (persistent)

    wrap_target()
    label("LOOP")
    # Interval delay
    mov(x, y)
    label("DLY")
    jmp(x_dec, "DLY")

    # Trigger pulse 10 µs @ 2 MHz
    set(pins, 1) [19]
    set(pins, 0)

    # Wacht tot echo laag is
    label("WAIT_LOW")
    jmp(pin, "WAIT_LOW")

    # Timeout teller laden
    mov(x, osr)
    label("WAIT_RISE")
    jmp(pin, "GOT_RISE")        # spring als hoog
    jmp(x_dec, "WAIT_RISE")     # anders aftellen

    # Timeout pad → push 0 en exit
    set(isr, 0)
    jmp("PUSH_EXIT")

    # Echo hoog → aftellen
    label("GOT_RISE")
    mov(x, osr)
    label("COUNT_HIGH")
    jmp(pin, "DEC_HIGH")
    jmp("DONE")
    label("DEC_HIGH")
    jmp(x_dec, "COUNT_HIGH")

    label("DONE")
    mov(isr, x)

    # Gemeenschappelijke exit
    label("PUSH_EXIT")
    push(block)
    irq(rel(0))
    jmp("LOOP")
    wrap()


# ====== Config ======
PIO_FREQ_HZ   = 2_000_000
INTERVAL_MS   = 50
TIMEOUT_US    = 30_000

# Ping-pong buffer
_buf = [0, 0]
_sid = 0
_widx = 0

# ====== IRQ handler ======
def _irq_handler(sm):
    global _widx, _sid
    try:
        remainder = sm.get()
    except:
        return
    _widx ^= 1
    _buf[_widx] = remainder
    _sid = (_sid + 1) & 0x7fffffff

# ====== Init state machine 6 ======
sm = rp2.StateMachine(
    4, usonic_pio, freq=PIO_FREQ_HZ,
    set_base=Pin(20, Pin.OUT),
    in_base=Pin(19, Pin.IN),
    jmp_pin=Pin(19, Pin.IN),
)
sm.put(int(PIO_FREQ_HZ * (INTERVAL_MS/1000)))
sm.put(TIMEOUT_US)
sm.irq(_irq_handler)
sm.active(1)

# ====== API ======
def stop():
    sm.active(0)

def read_us():
    """Geef (us, kind) terug: kind ∈ {'ok','timeout','overflow'}"""
    remainder = _buf[_widx]
    measured = TIMEOUT_US - remainder
    if measured <= 0:
        return None, 'timeout'
    elif measured >= TIMEOUT_US:
        return TIMEOUT_US, 'overflow'
    else:
        return measured, 'ok'

def read_cm():
    """Geef (cm, kind) terug, cm afgerond op 0,1 cm"""
    us, kind = read_us()
    if kind == 'timeout':
        return None, kind
    # integer afronding naar tienden cm
    cm_tenths = (us * 10 + 29) // 58   # +29 voor afronding
    cm = cm_tenths / 10.0
    return cm, kind
