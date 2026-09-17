# ================================================================
# stepper_rmp.py — stepper.py-API, maar met PIO/DMA-ramp eronder
# ================================================================
#
# Doel: een drop-in vervanger voor stepper.py (dezelfde functienamen en
# signaturen: enable, disable, stop, status, reset_PIO_distance, pio_pos1,
# pio_pos2, distance, s1, s2, mov, rotate, plus sm0/sm1), maar dan met een
# vloeiende versnellingsramp (via PIO + DMA) i.p.v. een directe snelheids-
# sprong vanuit stilstand — dezelfde ramp-truc als stepper_ramp_v2.py.
#
# ZELFSTANDIG, GEEN IMPORT VAN stepper_ramp_v2
# ----------------------------------------------
# Dit bestand deelt met opzet GEEN code met stepper_ramp_v2.py, ook al is de
# PIO/DMA-kern in essentie hetzelfde. Reden: v2 (en v1) zijn gebouwd voor een
# kar MET kompas/gyro (GY9250) en LDR's, en bevatten daardoor concepten die
# op de oudere hardware — waar dit bestand voor bedoeld is — niet bestaan:
#
#   - HeadingController, gyro_z_deg_s(), damp_yaw_rate()  -- verwerken
#     sensordata die er niet is;
#   - Move/drive()/adrive()                               -- bestaan puur om
#     tijdens het rijden bij te sturen op basis van die sensoren;
#   - heading(), rotate_deg(), turn_authority_deg_s(),
#     MOTOR_TURN_SIGN/GYRO_Z_SIGN/LDR_DIFF_SIGN            -- de teken- en
#     regelconventies die daarbij horen.
#
# Importeer je in plaats daarvan v2 en gebruik je alleen s1/s2/mov/rotate/
# status(), dan ZIT die machinerie er nog steeds bij (ongebruikt, maar wel
# aanwezig en verwarrend als je leest wat er allemaal gebeurt). Dit bestand
# bevat daarom een eigen, kleinere kopie van alleen het mechanische deel:
# PIO-programma's, DMA, rampwiskunde en de vier bewegingsfuncties.
#
# WAT WEL ANDERS IS DAN stepper.py, EN WAAROM DAT GEEN SENSOR-CODE IS
# ---------------------------------------------------------------------
#   sm0 / sm1 — in stepper.py stopte de generator-SM zelf bij het bereiken
#   van de target, dus sm.active() was letterlijk "nog aan het rijden". Hier
#   blijft de generator-SM ALTIJD actief (hij stalt op een lege FIFO i.p.v.
#   uit te gaan — nodig voor de ramp: zie de PIO hieronder). sm0/sm1 zijn
#   daarom proxy-objecten die .active() vertalen naar "zijn er nog
#   niet-uitgevoerde stappen weggeschreven" — puur mechanisch, geen sensor.
#
#   Validatie in s1/s2/mov/rotate (ongeldige richting/afstand geeft een
#   ValueError i.p.v. stilzwijgend verkeerd gedrag) — een controle op de
#   eigen argumenten, niets met sensoren te maken.
#
# GEEN drop-in samen met stepper.py, stepper_ramp.py of stepper_ramp_v2.py:
# ze claimen allemaal dezelfde PIO0 SM0..SM3 en dezelfde GPIO's. Importeer
# er altijd maar één.
# ================================================================

from array import array
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio
import rp2


# ----------------------------------------------------------------
# PIO: stapgenerator met (repeat, delay)-woorden
# ----------------------------------------------------------------
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT)
def ramp_stepper():
    # 7 instructies. Valt na de laatste jmp door en wrapt automatisch naar 0.
    pull(block)             .side(0)        # type: ignore  stalt hier met STEP laag
    out(y, 16)              .side(0)        # type: ignore  y = repeat-1
    out(x, 16)              .side(0)        # type: ignore  x = delay
    mov(isr, x)             .side(0)        # type: ignore  delay bewaren (ISR = kladregister)
    label("pulse")                          # type: ignore
    mov(x, isr)             .side(1)  [2]   # type: ignore  STEP hoog 3 cycles = 200 ns @15 MHz
    label("wait")                           # type: ignore
    jmp(x_dec, "wait")      .side(0)        # type: ignore  STEP laag, delay uitlopen
    jmp(y_dec, "pulse")                     # type: ignore  volgende stap in dit segment


# ----------------------------------------------------------------
# PIO: stappenteller (hardware-odometer, geen CPU)
# ----------------------------------------------------------------
@asm_pio()
def step_counter():
    # Y loopt af vanaf 0xFFFFFFFF. Pulsen = 0xFFFFFFFF - Y.
    label("loop")                           # type: ignore
    wait(0, pin, 0)                         # type: ignore  wacht tot STEP laag
    wait(1, pin, 0)                         # type: ignore  wacht op de stijgende flank
    jmp(y_dec, "loop")                      # type: ignore


# ----------------------------------------------------------------
# Constanten
# ----------------------------------------------------------------
F_PIO         = 15_000_000   # 150 MHz sysclk / 10 -> integer klokdeler, geen fractionele jitter
CYCLES_FIXED  = 5            # vaste overhead per stap binnen een segment (zie stepper_ramp_NL.md §3)

WHEEL_CIRC    = 19.1         # cm — gemeten wielomtrek
TRACK_WIDTH   = 13.6         # cm — spoorbreedte hart-op-hart (geometrie, niet gebruikt door
                             # rotate()/mov() hieronder, maar stond ook al zo in stepper.py)
STEPS_REV     = 12800        # 1/64 microstepping (TMC2209, 200 volle stappen x 64)
CM_PER_STEP   = WHEEL_CIRC / STEPS_REV                 # ~14,9 um/stap
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH / CM_PER_STEP   # idem: alleen geometrie

# Snelheidsgrenzen (motor 17HS8401, TMC2209 stealthChop, VREF 1 V = 0,71 A RMS)
V_MAX_CM_S    = 19.1         # 1,0 omw/s
V_START_CM_S  = 1.91         # 0,1 omw/s — veilige startsnelheid vanuit stilstand
ACCEL_CM_S2   = 55.0         # bepaalt de ramp-afstand via (v1^2-v0^2)/(2a); zie stepper_ramp_NL.md

RAMP_SEGMENTS = 256          # snelheidssprong <1,8% per segment over een 10:1 bereik


# ----------------------------------------------------------------
# Eenheidsconversies en validatie
# ----------------------------------------------------------------
def cm_to_steps(cm):
    return int(abs(cm) / CM_PER_STEP + 0.5)


def steps_to_cm(steps):
    return steps * CM_PER_STEP


def rate_of(cm_s):
    """cm/s -> stappen/s."""
    return abs(cm_s) / CM_PER_STEP


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_dir(value, valid, name):
    if not isinstance(value, str) or value.lower() not in valid:
        raise ValueError("%s moet een van %r zijn, kreeg %r" % (name, valid, value))
    return value.lower()


def _check_pos(value, name):
    if not _is_number(value) or not (value > 0) or value - value != 0:
        raise ValueError("%s moet een positief eindig getal zijn, kreeg %r" % (name, value))
    return value


def _check_dist(value, name="dist"):
    if not _is_number(value) or value < 0 or value - value != 0:
        raise ValueError("%s moet 0 of een positief eindig getal zijn, kreeg %r"
                         % (name, value))
    return value


def _clamp_delay(d):
    if d < 1:
        return 1
    if d > 65535:
        return 65535
    return d


def _delay_for(rate):
    """Stapfrequentie (stappen/s) -> delay-waarde voor het FIFO-woord."""
    return _clamp_delay(int(F_PIO / rate + 0.5) - CYCLES_FIXED)


def _word(repeat, delay):
    """Pak (aantal stappen, delay) in één 32-bit FIFO-woord."""
    if repeat < 1:
        repeat = 1
    elif repeat > 65536:
        repeat = 65536
    return (_clamp_delay(delay) << 16) | ((repeat - 1) & 0xFFFF)


# ----------------------------------------------------------------
# Rampaprofiel — zuiver mechanisch, geen sensor betrokken
# ----------------------------------------------------------------
def ramp_steps(rate0, rate1, accel_cm_s2=ACCEL_CM_S2):
    a = _check_pos(accel_cm_s2, "acceleratie") / CM_PER_STEP
    return int(abs(rate1 * rate1 - rate0 * rate0) / (2.0 * a) + 0.5)


def ramp_words(n_steps, rate0, rate1, n_seg=RAMP_SEGMENTS):
    """S-curve rampatabel: lijst van FIFO-woorden, samen exact n_steps stappen.
    Werkt zowel op als af: geef rate0 > rate1 voor een afremramp."""
    if n_steps <= 0:
        return []
    if n_seg > n_steps:
        n_seg = n_steps
    base = n_steps // n_seg
    extra = n_steps - base * n_seg
    out = []
    for i in range(n_seg):
        r = base + (1 if i < extra else 0)
        p = (i + 0.5) / n_seg
        s = p * p * (3.0 - 2.0 * p)
        out.append(_word(r, _delay_for(rate0 + (rate1 - rate0) * s)))
    return out


def cruise_words(n_steps, rate):
    """Kruisfase als zo weinig mogelijk woorden (max 65536 stappen per woord)."""
    out = []
    d = _delay_for(rate)
    rest = n_steps
    while rest > 0:
        chunk = 65536 if rest > 65536 else rest
        out.append(_word(chunk, d))
        rest -= chunk
    return out


def plan(n_total, v_cruise_cm_s, v_start_cm_s=V_START_CM_S, accel=ACCEL_CM_S2):
    """(n_ramp, rate_start, rate_cruise) voor een beweging van n_total stappen."""
    _check_pos(v_cruise_cm_s, "snelheid")
    _check_pos(v_start_cm_s, "startsnelheid")
    _check_pos(accel, "acceleratie")
    if n_total <= 0:
        return 0, rate_of(v_start_cm_s), rate_of(v_start_cm_s)
    if v_cruise_cm_s > V_MAX_CM_S:
        v_cruise_cm_s = V_MAX_CM_S
    r0 = rate_of(v_start_cm_s)
    r1 = rate_of(v_cruise_cm_s)
    if r1 <= r0:
        return 0, r1, r1
    n_ramp = ramp_steps(r0, r1, accel)
    if 2 * n_ramp > n_total:
        a = accel / CM_PER_STEP
        from math import sqrt
        r1 = sqrt(r0 * r0 + a * n_total)
        n_ramp = n_total // 2
    return n_ramp, r0, r1


def profile_words(n_total, v_cruise_cm_s, v_start_cm_s=V_START_CM_S, accel=ACCEL_CM_S2):
    """Compleet profiel (op + kruis + af) als één woordenlijst, in ÉÉN DMA-transfer
    (dus nul CPU-overhead tijdens de beweging zelf)."""
    if n_total <= 0:
        return []
    n_ramp, r0, r1 = plan(n_total, v_cruise_cm_s, v_start_cm_s, accel)
    words = ramp_words(n_ramp, r0, r1)
    words.extend(cruise_words(n_total - 2 * n_ramp, r1))
    words.extend(ramp_words(n_ramp, r1, r0))
    return words


# ----------------------------------------------------------------
# Motor
# ----------------------------------------------------------------
class _Motor:
    """Eén stappenmotor: generator-SM, teller-SM en een eigen DMA-kanaal.

    Geen signed positie/richtingsgeheugen (travel/_sign uit v1-v2): zonder
    Move-koerscorrectie is dat niet nodig. distance()/pio_pos1()/pio_pos2()
    werken hier, net als in stepper.py, met de RUWE (richtingsblinde)
    pulsteller.
    """

    def __init__(self, sm_id, cnt_id, step_gpio, dir_gpio, ena_gpio, fwd_level):
        self.step = Pin(step_gpio, Pin.OUT)
        self.dir = Pin(dir_gpio, Pin.OUT)
        self.ena = Pin(ena_gpio, Pin.OUT)
        self.ena.value(1)                   # start uitgeschakeld
        self.fwd_level = fwd_level

        self.sm = StateMachine(sm_id, ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.cnt = StateMachine(cnt_id, step_counter, freq=F_PIO, in_base=self.step)

        self.dma = rp2.DMA()
        self._ctrl = self.dma.pack_ctrl(size=2, inc_read=True, inc_write=False,
                                        treq_sel=(0 << 3) + sm_id)
        self._buf = None
        self.committed = 0                  # weggeschreven pulsen sinds reset

        self.cnt.active(1)
        self.sm.active(1)
        self.reset_pos()
        self.dir.value(fwd_level)

    def set_dir(self, forward):
        self.dir.value(self.fwd_level if forward else 1 - self.fwd_level)

    def reset_pos(self):
        self.cnt.put(0xFFFFFFFF)
        self.cnt.exec("pull()")
        self.cnt.exec("mov(y, osr)")
        self.committed = 0

    def pulses(self):
        """Monotone hardware-pulsteller sinds de laatste reset_pos(). Kent
        geen richting — net als stepper.py's pio_pos1()/pio_pos2()."""
        self.cnt.exec("mov(isr, y)")
        self.cnt.exec("push()")
        return 0xFFFFFFFF - self.cnt.get()

    def start_table(self, words, n_steps):
        """Hang een woordenlijst achter wat er nog in de FIFO staat, via DMA.
        Weigert zolang er al een transfer loopt."""
        if not words or n_steps <= 0:
            return False
        if self.dma.active():
            return False
        self._buf = array('I', words)
        self.dma.config(read=self._buf, write=self.sm,
                        count=len(self._buf), ctrl=self._ctrl, trigger=True)
        self.committed += n_steps
        return True

    def busy(self):
        """True zolang niet alle weggeschreven pulsen ook uitgestuurd zijn."""
        return self.pulses() < self.committed

    def _clear(self):
        self.dma.active(0)
        self.sm.active(0)
        self.sm.init(ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.sm.active(1)
        self.committed = self.pulses()

    def halt(self):
        """Onmiddellijke stop, driver blijft AAN — zoals stepper.py's stop()."""
        self._clear()

    def enable(self):
        self.ena.value(0)

    def disable(self):
        self.ena.value(1)


# ----------------------------------------------------------------
# Hardware-instantiatie — zelfde GPIO's als stepper.py/stepper_ramp_v2.py
# ----------------------------------------------------------------
MA = _Motor(0, 2, step_gpio=17, dir_gpio=16, ena_gpio=18, fwd_level=0)
MB = _Motor(1, 3, step_gpio=13, dir_gpio=12, ena_gpio=14, fwd_level=1)
_MOTORS = (MA, MB)


# ----------------------------------------------------------------
# sm0/sm1: compat-proxy i.p.v. de rauwe generator-SM (zie kop)
# ----------------------------------------------------------------
class _SMProxy:
    """.active() geeft "nog stappen te doen" terug (busy()), zodat
    `while stepper.sm0.active(): pass` blijft werken. .active(0) stopt de
    motor via halt(), zoals stepper.py's oude stop()-body deed."""

    def __init__(self, motor):
        self._motor = motor

    def active(self, value=None):
        if value is not None:
            if not value:
                self._motor.halt()
            return None
        return self._motor.busy()


sm0 = _SMProxy(MA)
sm1 = _SMProxy(MB)


# ----------------------------------------------------------------
# Publieke API — zelfde namen en signaturen als stepper.py
# ----------------------------------------------------------------
def enable():
    MA.enable()
    MB.enable()


def disable():
    MA.disable()
    MB.disable()


def stop():
    """Onmiddellijke stop, drivers blijven aan (motoren houden hun positie)."""
    MA.halt()
    MB.halt()


def pio_pos1():
    return MA.pulses()


def pio_pos2():
    return MB.pulses()


def reset_PIO_distance():
    """Zet beide odometers op nul. Alleen stilstaand aanroepen."""
    if MA.busy() or MB.busy():
        raise RuntimeError("reset_PIO_distance() tijdens een beweging; roep eerst stop() aan")
    MA.reset_pos()
    MB.reset_pos()


def distance():
    """Print de afstand van beide motoren apart en geeft (d1, d2) terug."""
    d1 = pio_pos1() * CM_PER_STEP
    d2 = pio_pos2() * CM_PER_STEP
    print("Afstand (PIO) Motor A:", d1, "cm")
    print("Afstand (PIO) Motor B:", d2, "cm")
    return d1, d2


def status():
    print("=== STATUS ===")
    for naam, m in (("A", MA), ("B", MB)):
        print("Motor %s:" % naam, "running" if m.busy() else "stopped",
              "Distance: %.2f cm" % (m.pulses() * CM_PER_STEP),
              "committed: %d stappen" % m.committed)
    print("================")


def _launch(dirs, speed, n):
    """Start hetzelfde profiel op de gegeven motoren. dirs = ((motor, forward), ...)"""
    if n <= 0:
        return False
    words = profile_words(n, speed)
    if not words:
        return False
    for m, fwd in dirs:
        m.halt()            # een nieuw commando overschrijft een lopende beweging
        m.set_dir(fwd)
        m.enable()
    for m, _ in dirs:
        m.start_table(words, n)
    return True


def _turn_dirs(right):
    """(motor, forward)-paren voor rotate(). right=True -> naar rechts.
    Zelfde vaste bedradingsconventie als stepper.py: bij een draai naar
    rechts gaat motor A achteruit en motor B vooruit."""
    return ((MA, not right), (MB, right))


def s1(direction, speed, dist):
    """Alleen motor A. direction 'f'/'b', speed cm/s, dist cm (>= 0)."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    _check_dist(dist)
    return _launch(((MA, fwd),), _check_pos(speed, "speed"), cm_to_steps(dist))


def s2(direction, speed, dist):
    """Alleen motor B. direction 'f'/'b', speed cm/s, dist cm (>= 0)."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    _check_dist(dist)
    return _launch(((MB, fwd),), _check_pos(speed, "speed"), cm_to_steps(dist))


def mov(direction, speed, dist):
    """Beide motoren dezelfde kant, dezelfde afstand (dist >= 0)."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    _check_dist(dist)
    return _launch(((MA, fwd), (MB, fwd)), _check_pos(speed, "speed"), cm_to_steps(dist))


def rotate(direction, speed, dist):
    """Draai op de as. direction 'l'/'r', dist = booglengte per wiel in cm (>= 0)."""
    right = _check_dir(direction, ('l', 'r'), "direction") == 'r'
    _check_dist(dist)
    return _launch(_turn_dirs(right), _check_pos(speed, "speed"), cm_to_steps(dist))
