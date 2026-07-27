# ================================================
# Dual Stepper Controller zonder ramp, met PIO counters (geoptimaliseerd)
# ================================================
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

# -----------------------------
# PIO stepper generator (delay via OSR -> X/Y)
# -----------------------------

@asm_pio(sideset_init=PIO.OUT_LOW) 
def stepper():
    pull(noblock) .side(1)  # type: ignore 
    nop()        [7]        # type: ignore
    mov(x, osr)             # type: ignore  recycle laatste delay in X
    mov(y, x)               # type: ignore
    label("delay")          # type: ignore
    nop()         .side(0)  # type: ignore
    jmp(y_dec, "delay")     # type: ignore

# === NIEUWE TARGET COUNTER PIO ===
# STEP = pin 1, DIR = pin 0 (in_base instellen bij StateMachine, maar DIR wordt hier niet meer gebruikt)
@asm_pio()
def stepper_counter():
    # X bevat het aantal resterende stappen (wordt vanuit Python geladen)
    label("loop")           # type: ignore
    wait(0, pin, 1)         # type: ignore  Wacht op STEP laag (pin index 1)
    wait(1, pin, 1)         # type: ignore  Wacht op STEP hoog
    jmp(y_dec, "dec_x")     # type: ignore  Afstands teller telt vanaf 0xFFFFFFFF naar 0
    label("dec_x")          # type: ignore
    jmp(x_dec, "loop")      # type: ignore  X--, zolang X != 0 blijf in loop
    irq(rel(0))                  # type: ignore  Target bereikt → één IRQ naar Python
    jmp("loop")             # type: ignore  Wacht tot de motor weer gaat lopen.


# -----------------------------------------
# Constantes en afleiding
# -----------------------------------------
F_PIO = 3_000_000          # mag verhoogd worden (fijnere delay-resolutie); STEP-puls ≥ ±100 ns (TMC2209) houden
WHEEL_CIRC   = 20.94       # cm
STEPS_REV    = 12800       # 1/64 microstepping (TMC2209: 200 volle stappen × 64; MS1→GND, MS2→+5V)
CM_PER_STEP  = WHEEL_CIRC / STEPS_REV   # ≈ 16,4 µm/stap

# -----------------------------------------
# Pin definitie
# -----------------------------------------
ENA1  = Pin(18, Pin.OUT); ENA1.value(0)
STEP1 = Pin(17, Pin.OUT)
DIR1  = Pin(16, Pin.OUT); DIR1.value(0)

ENA2  = Pin(14, Pin.OUT); ENA2.value(0)
STEP2 = Pin(13, Pin.OUT)
DIR2  = Pin(12, Pin.OUT); DIR2.value(1)

# -----------------------------------------
# State machines voor stappen
# -----------------------------------------
sm0 = StateMachine(0, stepper, freq=F_PIO, sideset_base=STEP1)
sm1 = StateMachine(1, stepper, freq=F_PIO, sideset_base=STEP2)

# -----------------------------------------
# PIO counters voor target-detectie
# -----------------------------------------
# Let op: in_base zo gekozen dat STEP op pin index 1 zit (DIR op index 0).
sm2 = StateMachine(2, stepper_counter, in_base=DIR1, freq=F_PIO)
sm3 = StateMachine(3, stepper_counter, in_base=DIR2, freq=F_PIO)

# -----------------------------------------
# PIO counters voor afstandsmeting wordt bij opstart op 0 gezet
# -----------------------------------------
sm2.active(1); sm2.exec("mov(y, 0xFFFFFFFF)")
sm3.active(1); sm3.exec("mov(y, 0xFFFFFFFF)")
# -----------------------------------------
# Posities (software teller) en targets
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
          f"Start_Pos: {pos1*CM_PER_STEP:.2f} cm",
          f"Target_Pos: {target_pos1*CM_PER_STEP:.2f} cm",
          f"Distance: {pio_pos1()*CM_PER_STEP:.2f} cm",)
    print("Motor B:", "running" if sm1.active() else "stopped",
          f"Start_Pos: {pos2*CM_PER_STEP:.2f} cm",
          f"Target_Pos: {target_pos2*CM_PER_STEP:.2f} cm",
          f"Distance: {pio_pos2()*CM_PER_STEP:.2f} cm",)
    print("================")
# -----------------------------------------
# PIO counter helpers voor afstand (hardware teller)
# -----------------------------------------
def reset_PIO_distance():
    """Reset de tellerstand in de PIO counters naar -1 (0xFFFFFFFF)."""
    for sm in (sm2, sm3):
        sm.put(0xFFFFFFFF); sm.exec("pull()"); sm.exec("mov(y, osr)")

def pio_pos1():
    """Number of step pulses motor A"""
    sm2.exec("mov(isr, y)"); sm2.exec("push()"); y_val = sm2.get()
    return 0xFFFFFFFF - y_val

def pio_pos2():
    """Number of step pulses motor B"""
    sm3.exec("mov(isr, y)"); sm3.exec("push()"); y_val = sm3.get()
    return 0xFFFFFFFF - y_val

def distance():
    p1 = pio_pos1(); p2 = pio_pos2()
    d1 = p1 * CM_PER_STEP
    d2 = p2 * CM_PER_STEP
    print("Afstand (PIO) Motor A:", d1, "cm")
    print("Afstand (PIO) Motor B:", d2, "cm")
    return d1, d2
# -----------------------------------------
# PIO counter IRQ handlers – target bereikt
# -----------------------------------------
def counter_irq0(sm):
    # Motor A: target bereikt → pos1 = target_pos1, sm0 stoppen
    global pos1
    sm0.active(0)
#    sm2.active(0)
    pos1 += target_pos1

def counter_irq1(sm):
    # Motor B: target bereikt → pos2 = target_pos2, sm1 stoppen
    global pos2
    sm1.active(0)
#    sm3.active(0)
    pos2 += target_pos2

# -----------------------------------------
# koppel interrupt handlers
# -----------------------------------------
sm2.irq(handler=counter_irq0); sm3.irq(handler=counter_irq1)

# -----------------------------------------
# Start helper – nu ook target-stappen naar sm2/sm3
# -----------------------------------------
def _start_motor(sm, delay, sm_counter,steps):
    # Laad het aantal stappen in X van de counter-SM  (target count)  
    sm_counter.put(steps)
    sm_counter.exec("pull()")
    sm_counter.exec("mov(x, osr)")
    sm_counter.active(1)
    # Start de stepper-SM met de gewenste delay
    sm.put(delay)
    sm.active(1)
    enable()

# -----------------------------------------
# Hoog-niveau commando's (dir-format: f,b,l,r)
# -----------------------------------------
def s1(dir, speed, distance):
    """Motor A: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos1
    forward = (dir.lower() == 'f')
    DIR1.value(0 if forward else 1)
    steps = int(distance / CM_PER_STEP)
    # Targetpositie in software bijhouden
    target_pos1 = (steps if forward else -steps)
    _start_motor(sm0, speed_to_delay(abs(speed)), sm2, steps)

def s2(dir, speed, distance):
    """Motor B: dir 'f'/'b', speed cm/s, distance cm."""
    global target_pos2
    forward = (dir.lower() == 'f')
    DIR2.value(1 if forward else 0)
    steps = int(distance / CM_PER_STEP)
    target_pos2 = (steps if forward else -steps)
    _start_motor(sm1, speed_to_delay(abs(speed)), sm3, steps)

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
