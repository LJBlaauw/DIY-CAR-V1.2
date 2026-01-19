# ================================================
# Dual Stepper Controller zonder ramp, met PIO counters
# ================================================
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

# -----------------------------
# PIO stepper generator (delay via OSR -> X/Y)
# -----------------------------
@asm_pio(sideset_init=PIO.OUT_LOW)
def stepper():
    pull(noblock) .side(1)
    nop()        [7]
    mov(x, osr)              # recycle laatste delay in X
    mov(y, x)
    label("delay")
    nop()         .side(0)
    jmp(y_dec, "delay")

# === STEP COUNTER PIO ===
# STEP = pin 0, DIR = pin 1 (in_base instellen bij StateMachine)
@asm_pio()
def stepper_counter():
    label("done")
    wait(0, pin, 1)       # Wacht op STEP laag
    wait(1, pin, 1)       # Wacht op STEP hoog
    jmp(pin, "fwd")       # DIR=1 → vooruit
    jmp(x_dec, "done")    # DIR=0 → achteruit (X--)
    label("fwd")
    jmp(y_dec, "done")    # Vooruit (Y--)

# -----------------------------------------
# Constantes en afleiding
# -----------------------------------------
F_PIO = 3_000_000
WHEEL_CIRC   = 20.94       # cm
STEPS_REV    = 1600        # 1/8 microstepping
CM_PER_STEP  = WHEEL_CIRC / STEPS_REV

# -----------------------------------------
# Pin definitie
# -----------------------------------------
ENA1  = Pin(22, Pin.OUT); ENA1.value(0)
STEP1 = Pin(17, Pin.OUT)
DIR1  = Pin(16, Pin.OUT); DIR1.value(0)


ENA2  = Pin(15, Pin.OUT); ENA2.value(0)
STEP2 = Pin(12, Pin.OUT)
DIR2  = Pin(11, Pin.OUT); DIR2.value(1)


# -----------------------------------------
# State machines voor stappen
# -----------------------------------------
sm0 = StateMachine(0, stepper, freq=F_PIO, sideset_base=STEP1)
sm1 = StateMachine(1, stepper, freq=F_PIO, sideset_base=STEP2)

# -----------------------------------------
# PIO counters voor afstandsmeting
# -----------------------------------------
# Let op: de counter SM verwacht STEP op pin offset 0 en DIR op pin offset 1 binnen dezelfde bank.
sm2 = StateMachine(2, stepper_counter, in_base=DIR1, jmp_pin=DIR1, freq=F_PIO, set_base=DIR1)
sm3 = StateMachine(3, stepper_counter, in_base=DIR2, jmp_pin=DIR2, freq=F_PIO, set_base=DIR2)

sm2.active(1); sm2.exec("mov(y, 0xFFFFFFFF)"); sm2.exec("mov(x, 0xFFFFFFFF)")
sm3.active(1); sm3.exec("mov(y, 0xFFFFFFFF)"); sm3.exec("mov(x, 0xFFFFFFFF)")

# -----------------------------------------
# Posities (software teller via IRQ) en targets
# -----------------------------------------
pos1 = 0
pos2 = 0
target_pos1 = 0
target_pos2 = 0

# -----------------------------------------
# Hulpfuncties
# -----------------------------------------
def speed_to_delay(speed_cm_s):
    # Overhead afgestemd op deze PIO codepad (zonder irq in de PIO)
    OVERHEAD = 9
    f_step = speed_cm_s / CM_PER_STEP
    y = int(F_PIO / (2 * f_step) - OVERHEAD)
    return max(5, y)

def enable():
    ENA1.value(0); ENA2.value(0)

def disable():
    ENA1.value(1); ENA2.value(1)

def stop():
    sm0.active(0)
    sm1.active(0)

def status():
    print("=== STATUS ===")
    print("Motor A:", "running" if sm0.active() else "stopped",
          f"Pos: {pos1*CM_PER_STEP:.2f} cm",
          f"Target: {target_pos1*CM_PER_STEP:.2f} cm")
    print("Motor B:", "running" if sm1.active() else "stopped",
          f"Pos: {pos2*CM_PER_STEP:.2f} cm",
          f"Target: {target_pos2*CM_PER_STEP:.2f} cm")
    print("================")

# -----------------------------------------
# PIO counter helpers voor afstand (hardware teller)
# -----------------------------------------
def reset_PIO_distance():
    """Reset de tellerstand in de PIO counters naar -1 (0xFFFFFFFF)."""
    for sm in (sm2, sm3):
        sm.put(0xFFFFFFFF); sm.exec("pull()"); sm.exec("mov(y, osr)")
        sm.put(0xFFFFFFFF); sm.exec("pull()"); sm.exec("mov(x, osr)")

def _pio_read_pos(sm):
    sm.exec("mov(isr, y)"); sm.exec("push()"); y_val = sm.get()
    sm.exec("mov(isr, x)"); sm.exec("push()"); x_val = sm.get()
    return y_val - x_val

def pio_pos1():
    """Signed positie Motor A in stappen (DIR1=0 vooruit)."""
    return _pio_read_pos(sm2)

def pio_pos2():
    """Signed positie Motor B in stappen (DIR2=1 vooruit)."""
    return -_pio_read_pos(sm3)

def distance():
    p1 = pio_pos1(); p2 = pio_pos2()
    d1 = p1 * CM_PER_STEP
    d2 = p2 * CM_PER_STEP
    print("Afstand (PIO) Motor A:", d1, "cm")
    print("Afstand (PIO) Motor B:", d2, "cm")
    return d1, d2

# -----------------------------------------
# IRQ handlers – positie bijhouden en stoppen op target
# -----------------------------------------
def step_irq0(pin):
    # Motor A: DIR1=0 is forward → pos1++
    global pos1
    pos1 += 1 if DIR1.value() == 0 else -1
    if pos1 == target_pos1:
        sm0.active(0)

def step_irq1(pin):
    # Motor B is omgekeerd: DIR2=1 is forward → pos2++
    global pos2
    pos2 += 1 if DIR2.value() == 1 else -1
    if pos2 == target_pos2:
        sm1.active(0)

# GPIO falling-edge interrupts op STEP om iedere stap te tellen
STEP1.irq(trigger=Pin.IRQ_FALLING, handler=step_irq0)
STEP2.irq(trigger=Pin.IRQ_FALLING, handler=step_irq1)

# -----------------------------------------
# Start helpers
# -----------------------------------------
def _start_motor(sm, delay):
    sm.put(delay)
    sm.active(1)
    enable()

# -----------------------------------------
# Hoog-niveau commando's (dir-format: f,b,l,r)
# -----------------------------------------
def s1(dir, speed, distance):
    """Motor A: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos1
    DIR1.value(0 if dir.lower() == 'f' else 1)
    steps = int(distance / CM_PER_STEP)
    target_pos1 = pos1 + (steps if dir.lower() == 'f' else -steps)
    _start_motor(sm0, speed_to_delay(abs(speed)))

def s2(dir, speed, distance):
    """Motor B: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos2
    DIR2.value(1 if dir.lower() == 'f' else 0)
    steps = int(distance / CM_PER_STEP)
    target_pos2 = pos2 + (steps if dir.lower() == 'f' else -steps)
    _start_motor(sm1, speed_to_delay(abs(speed)))

def mov(dir, speed, distance):
    """Beide motoren dezelfde kant en afstand."""
    s1(dir, speed, distance)
    s2(dir, speed, distance)

def rotate(dir, speed, distance):
    """Rotatie: 'l' (left) of 'r' (right)."""
    if dir.lower() == 'r':
        s1('b', speed, distance)
        s2('f', speed, distance)
    else:
        s1('f', speed, distance)
        s2('b', speed, distance)
