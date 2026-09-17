# ================================================================
# Ultrasonic Driver (HC-SR04 / RCWL-1601) via PIO
# ================================================================
#
# Runs on separate PIO block to avoid conflicts.
# Measures the pulse width of the ECHO pin and converts it to centimeters.
#
# Rationale: Sound speed ~343 m/s -> 34300 cm/s -> 1 cm round trip takes
# approx. 58.3 microseconds.
# ================================================================

import machine
import time
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

# --- CONFIG / PINOUT ---
TRIGGER_PIN = 15
ECHO_PIN    = 14
US_SM_ID    = 4       # PIO1 SM0 — separate from stepper and LDR clock

# Clock divider for PIO to get precise microsecond or cycle counting
F_PIO_US = 1_000_000

@asm_pio(set_init=PIO.OUT_LOW)
def ultrasonic_trigger_echo():
    """PIO program to generate a 10us trigger pulse and measure ECHO duration."""
    label("start")          # type: ignore
    # Generate 10us trigger pulse
    set(pins, 1)        [9] # type: ignore  10 cycles at 1MHz = 10us
    set(pins, 0)            # type: ignore

    # Wait for ECHO pin to go high
    wait(1, pin, 0)         # type: ignore  ECHO is at in_base index 0

    # Count cycles until ECHO goes low
    mov(x, 0)               # type: ignore  Clear X counter
    label("count_loop")     # type: ignore
    jmp(pin, "decrement")   # type: ignore  If ECHO is still high, jump to decrement
    jmp("capture")          # type: ignore  Else, ECHO dropped -> capture result

    label("decrement")      # type: ignore
    jmp(x_dec, "count_loop")# type: ignore  X-- and loop (counts down from 0)

    label("capture")        # type: ignore
    mov(isr, x)             # type: ignore  Move counter value to ISR
    push()                  # type: ignore  Push to Python FIFO

    # Timeout/delay before next measurement to prevent false reflections
    nop()# type: ignore
    nop()# type: ignore
    jmp("start")            # type: ignore

# Initialize Pins
_trigger = Pin(TRIGGER_PIN, Pin.OUT)
_echo    = Pin(ECHO_PIN, Pin.IN)

# Initialize State Machine
_sm = StateMachine(US_SM_ID, ultrasonic_trigger_echo, freq=F_PIO_US,
                   set_base=_trigger, in_base=_echo, jmp_pin=_echo)
_sm.active(1)

def read_cm():
    """Read the distance in centimeters.

    Returns a tuple: (distance_cm, status_string)
    Statuses: 'ok', 'overflow', 'timeout'
    """
    # Flush old values from the FIFO queue if present
    while _sm.rx_fifo():
        _sm.get()

    # Wait for a fresh measurement from the PIO block
    t0 = time.ticks_ms()
    while not _sm.rx_fifo():
        if time.ticks_diff(time.ticks_ms(), t0) > 60:
            return 0.0, 'timeout'

    raw = _sm.get()
    # PIO counts down from 0xFFFFFFFF (4294967295)
    cycles = 0xFFFFFFFF - raw

    # At 1MHz, 1 cycle = 1 microsecond.
    # Division by 2 for round-trip path, then by 29.15 for cm conversion (1 / 0.0343)
    # Total divisor: 2 * 29.15 = 58.3
    if cycles >= 30000 or cycles <= 100:
        return 500.0, 'overflow'

    distance_cm = cycles / 58.3
    return distance_cm, 'ok'

def stop():
    """Disable the ultrasonic state machine."""
    _sm.active(0)
