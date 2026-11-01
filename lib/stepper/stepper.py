# ================================================
# Dual Stepper Controller zonder ramp, met PIO counters (geoptimaliseerd)
# ================================================
from machine import Pin
from array import array
import rp2
from rp2 import PIO, StateMachine, asm_pio

# PIO-instructies voor sm.exec(), één keer vooraf gecodeerd. exec() met een
# tekst laat de assembler bij ELKE aanroep opnieuw draaien: gemeten ~12 ms en
# ~230 bytes per positie-uitlezing (2026-10-21), tegen ~25 us en 0 bytes zo.
_EXEC_PULL      = rp2.asm_pio_encode("pull()", 0)
_EXEC_MOV_Y_OSR = rp2.asm_pio_encode("mov(y, osr)", 0)
_EXEC_MOV_X_OSR = rp2.asm_pio_encode("mov(x, osr)", 0)
_EXEC_MOV_ISR_Y = rp2.asm_pio_encode("mov(isr, y)", 0)
_EXEC_PUSH      = rp2.asm_pio_encode("push()", 0)

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
F_PIO = 15_000_000         # 150 MHz sysclk / 10 → integer klokdeler (geen fractionele jitter).
                           # STEP-puls blijft 11 cycles = 733 ns ≥ 100 ns (TMC2209-minimum).
WHEEL_CIRC   = 21.33       # cm — HERIJKT 2026-11-01: na het goed afstellen van het zwenkwiel
                             # (los van de bandenwissel) gaf 30cm gecommandeerd op het meetlint
                             # 33,5cm werkelijk, bij twee verschillende snelheden (19,1 en
                             # 24cm/s) — dus geen snelheidsafhankelijke slip, maar een
                             # verschoven effectieve rolomtrek door de zwenkwiel-afstelling.
                             # 19,1 x 33,5/30 = 21,33. Was 19,1 (was 20,94; die gaf 8,8% te
                             # korte afstanden).
TRACK_WIDTH  = 13.6        # cm — geometrische spoorbreedte hart-op-hart (alleen documentatie)
TRACK_WIDTH_EFF = 13.66      # cm — eerst analytisch meegeschaald met WHEEL_CIRC (12,20 x
                             # 21,33/19,1 = 13,62), daarna kompasmeting rotate_deg(360) (met de
                             # gecorrigeerde WHEEL_CIRC=21,33): 352,7° (-7,3°) -> 13,90; een
                             # derde meting over 3 omwentelingen (1080°, betere signaal/ruis)
                             # gaf 1099,0° (+19°) -> 13,66 — binnen de kompasruis van de
                             # 13,62/13,90-tussenstappen, dus hier geconvergeerd.
                             # Geschiedenis (met de OUDE
                             # WHEEL_CIRC=19,1): HERIJKT 2026-11-01 na de nieuwe TPU-banden via
                             # het kompas (ICM-20948). Iteratief verfijnd: eerste rotate_deg(360)
                             # met de oude 11,26 kwam maar tot 321,9° (-38,1°, visueel bevestigd
                             # "ca 30 graden te weinig") -> 12,59; herhaling met 12,59 gaf nog
                             # +9,95° op 360° -> 12,25; een derde meting over 3 omwentelingen
                             # (1080°, betere signaal/ruis t.o.v. de ±5° kompasruis) gaf +34,85°
                             # -> 12,20. Was 11,26 (gemeten 2026-10-21 met de oude banden: rotate
                             # 360° links gaf 434,9° op de gyro, kompas ~420°; 13,6 x 360 /
                             # 434,9 = 11,26).
STEPS_REV    = 12800       # 1/64 microstepping (TMC2209: 200 volle stappen × 64; MS1→GND, MS2→+5V)
CM_PER_STEP  = WHEEL_CIRC / STEPS_REV   # ≈ 14,9 µm/stap
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH_EFF / CM_PER_STEP   # ≈ 132 stappen verschil per graad koers

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
# ("mov(y, 0xFFFFFFFF)" is geen geldige PIO-instructie en liet y op 0 staan,
#  waardoor pio_pos1() tot de eerste reset_PIO_distance() onzin gaf.)
for _sm in (sm2, sm3):
    _sm.active(1); _sm.put(0xFFFFFFFF); _sm.exec(_EXEC_PULL); _sm.exec(_EXEC_MOV_Y_OSR)
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
        sm.put(0xFFFFFFFF); sm.exec(_EXEC_PULL); sm.exec(_EXEC_MOV_Y_OSR)

_ybuf1 = array('i', [0])
_ybuf2 = array('i', [0])

def pio_pos1():
    """Number of step pulses motor A"""
    # get(buf): y telt af vanaf 0xFFFFFFFF = -1 als signed int32, dus
    # pulsen = -1 - y zonder groot (heap-)getal; bruikbaar vanuit een ISR.
    sm2.exec(_EXEC_MOV_ISR_Y); sm2.exec(_EXEC_PUSH); sm2.get(_ybuf1)
    return -1 - _ybuf1[0]

def pio_pos2():
    """Number of step pulses motor B"""
    # get(buf): y telt af vanaf 0xFFFFFFFF = -1 als signed int32, dus
    # pulsen = -1 - y zonder groot (heap-)getal; bruikbaar vanuit een ISR.
    sm3.exec(_EXEC_MOV_ISR_Y); sm3.exec(_EXEC_PUSH); sm3.get(_ybuf2)
    return -1 - _ybuf2[0]

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
    sm_counter.exec(_EXEC_PULL)
    sm_counter.exec(_EXEC_MOV_X_OSR)
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
