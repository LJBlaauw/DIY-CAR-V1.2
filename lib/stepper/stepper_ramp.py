# ================================================================
# Dual Stepper Controller MET ramp (PIO + DMA) en koerscorrectie
# ================================================================
#
# LET OP — dit is een VERVANGER van stepper.py, geen aanvulling.
# Beide claimen PIO0 SM0..SM3 en dezelfde GPIO's. Importeer er altijd maar één.
#
# WERKINGSPRINCIPE
# ----------------
# De stapgenerator in de PIO leest 32-bit woorden uit zijn TX-FIFO. Elk woord
# codeert een heel SEGMENT in plaats van één stap:
#
#     bits 15..0   = aantal stappen in dit segment - 1   (max 65536 stappen)
#     bits 31..16  = delay per stap in PIO-cycles        (max 65535)
#
# Daardoor kost een complete ramp van duizenden stappen maar een paar honderd
# woorden. Een ramp van 256 segmenten is 1 kB; één woord per stap zou 36 kB zijn
# en dat past niet betrouwbaar in de MicroPython-heap.
#
# Fasen van een beweging:
#
#   ramp op    -> DMA-tabel (256 woorden + 1 brugsegment). Segmenten van ~2 ms
#                 zijn te snel voor MicroPython, dus dit MOET via DMA. Immuun
#                 voor GC-pauzes.
#   kruisfase  -> zonder bijsturing: één woord, dus onderdeel van dezelfde
#                 DMA-transfer -> nul CPU-overhead.
#                 met bijsturing: de CPU pusht één woord per SLICE_MS per motor
#                 (~50 put()'s per seconde per motor, ca. 0,1% CPU).
#   ramp af    -> DMA-tabel, getriggerd zodra het aantal weggeschreven stappen
#                 het afremmpunt bereikt.
#
# DMA EN CPU MOGEN NOOIT GELIJKTIJDIG IN DEZELFDE FIFO SCHRIJVEN. De volgorde
# zou dan door elkaar lopen en de twee motoren konden verschillende segment-
# volgordes krijgen. Daarom:
#   - tijdens _RAMP_UP pusht de CPU niets; er wordt gewacht tot dma.active()
#     van BEIDE motoren False is (de DMA is dan gestopt met SCHRIJVEN, terwijl
#     de FIFO nog data bevat -- precies de voorsprong die de CPU nodig heeft);
#   - de ramp-af-DMA wordt pas gestart nadat de CPU is gestopt met pushen.
#
# Wachten tot de rampstappen ook UITGEVOERD zijn zou fout zijn: dan loopt de
# FIFO leeg en pauzeert de motor tussen ramp en kruisfase.
#
# BRUGSEGMENT
# -----------
# Als de opramp-DMA klaar is met schrijven, staat er nog maximaal 4 woorden in
# de FIFO. Aan het eind van de ramp zitten we op topsnelheid, dus die 4 woorden
# zijn samen maar ~3 ms. Met service() elke 10 ms zou de FIFO alsnog leeglopen.
# Daarom eindigt de opramptabel met één BRUGSEGMENT op kruissnelheid dat
# BRIDGE_SLICES * SLICE_MS lang duurt. Die stappen horen bij de kruisfase maar
# worden niet bijgestuurd - een correctie 40 ms eerder of later maakt niets uit.
#
# EXACTE AFSTAND
# --------------
# Bijsturen verandert WANNEER stappen komen, niet HOEVEEL. We houden per motor
# `committed` bij: de som van alle repeat-waarden die we hebben weggeschreven.
# Het afremmpunt wordt op `committed` bepaald, niet op een gemeten positie, dus
# de eindafstand is exact - onafhankelijk van wanneer de regellus aanroept.
#
# ODOMETRIE
# ---------
# De teller-SM's tellen STEP-FLANKEN en weten niets van de DIR-pin. Een ruwe
# pulsteller is dus geen positie: bij een rotatie lopen beide tellers positief
# op terwijl de wielen tegengesteld draaien. Daarom houdt elke motor naast de
# monotone pulsteller een SIGNED positie bij (`travel()`), die bij elke
# richtingswisseling het teken meeneemt. `pulses()` blijft de monotone teller en
# wordt gebruikt voor `busy()`, waar juist ongesigneerd geteld moet worden.
#
# KOERSCORRECTIE
# --------------
# Koersverandering komt van een VERSCHIL IN STAPPENAANTAL tussen de wielen, niet
# van een verschil in snelheid als beide wielen aan hetzelfde totaal vastzitten.
# In de kruisfase ligt dat totaal niet vast, dus daar integreert een
# snelheidsverschil wel tot een echte koersverandering. Zie HeadingController.
#
# Het klokdeler-alternatief (SMn_CLKDIV tijdens runtime wijzigen) is bewust NIET
# gebruikt: dat zit buiten het datapad, is niet synchroon met de segmentgrenzen,
# en zou de rampatabellen onderweg herschalen (a schaalt met f^2).
# ================================================================

from array import array
from math import sqrt
import rp2
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio


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
    # in_base = de STEP-pin, dus pin-index 0. Kent de DIR-pin NIET; het teken
    # wordt in software bijgehouden (zie _Motor.travel).
    label("loop")                           # type: ignore
    wait(0, pin, 0)                         # type: ignore  wacht tot STEP laag
    wait(1, pin, 0)                         # type: ignore  wacht op de stijgende flank
    jmp(y_dec, "loop")                      # type: ignore


# ----------------------------------------------------------------
# Constanten
# ----------------------------------------------------------------
F_PIO         = 15_000_000   # 150 MHz sysclk / 10 -> integer klokdeler, geen fractionele jitter
CYCLES_FIXED  = 5            # vaste cycles per stap in ramp_stepper (mov[2]=3, jmp_y=1, +1)
                             # TE VERIFIEREN met een logic analyzer: meet de werkelijke
                             # STEP-frequentie tegen _delay_for() (zie meet_frequentie()).

WHEEL_CIRC    = 19.1         # cm — gemeten wielomtrek
TRACK_WIDTH   = 13.6         # cm — spoorbreedte hart-op-hart
STEPS_REV     = 12800        # 1/64 microstepping (TMC2209, 200 volle stappen x 64)
CM_PER_STEP   = WHEEL_CIRC / STEPS_REV                 # ~14,9 um/stap
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH / CM_PER_STEP   # ~159 stappen verschil per graad

RAD_TO_DEG    = 57.29577951308232

# Snelheidsgrenzen (motor 17HS8401, TMC2209 stealthChop, VREF 1 V = 0,71 A RMS)
V_MAX_CM_S    = 19.1         # 1,0 omw/s — stealthChop haalt ~300 rpm, we zitten op 60
V_START_CM_S  = 1.91         # 0,1 omw/s — veilige startsnelheid, empirisch te verifieren
ACCEL_CM_S2   = 55.0         # bepaalt de ramp-AFSTAND via (v1^2-v0^2)/(2a) = 3,28 cm.
                             # De S-curve piekt op 1,5x = 82 cm/s2 = 0,084 g.
                             # LET OP: omdat de S-curve aan begin en eind traag is,
                             # DUURT de ramp 0,55 s, niet de 0,31 s van een lineaire
                             # ramp over dezelfde afstand. Verhoog deze waarde om
                             # zowel de rampafstand als de -duur te verkorten.

# Ramp- en sliceparameters
RAMP_SEGMENTS = 256          # snelheidssprong <1,8% per segment over een 10:1 bereik
SLICE_MS      = 20           # duur van één kruis-slice; bepaalt de regellatentie
FIFO_TARGET   = 3            # aantal slices dat vooruit in de FIFO staat (runway = 60 ms)
FIFO_DEPTH    = 4            # TX-FIFO diepte zonder fifo_join
BRIDGE_SLICES = 2            # brugsegment aan het eind van de opramp (zie kop)
MAX_DIFF_FRAC = 0.20         # maximale snelheidsdifferentie per wiel (+/- 20%)

# Afgeleid: maximaal stuurgezag in graden/s
MAX_TURN_DEG_S = (2 * MAX_DIFF_FRAC * V_MAX_CM_S / CM_PER_STEP) / STEPS_PER_DEG

# Tekenconventie voor bijsturen, heading() en de gyro. Positief = NAAR RECHTS.
# Welke fysieke motor links of rechts zit volgt niet uit de code — dat is montage.
# Stuurt de kar de verkeerde kant op, zet deze op -1. Dat is de enige plek.
# Verifieren met opgeheven wielen: een handmatige draai naar rechts moet zowel
# een positieve gemeten als een positieve gewenste draaisnelheid geven.
TURN_SIGN     = +1


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


def _check_dir(value, valid, name):
    """Weiger een ongeldige richting in plaats van hem stil als 'anders' te lezen."""
    if not isinstance(value, str) or value.lower() not in valid:
        raise ValueError("%s moet een van %r zijn, kreeg %r" % (name, valid, value))
    return value.lower()


def _check_pos(value, name):
    """Weiger nul, negatief en niet-eindige getallen."""
    if not (value > 0) or value != value or value - value != 0:
        raise ValueError("%s moet een positief eindig getal zijn, kreeg %r" % (name, value))
    return value


def _clamp_delay(d):
    """Houd de delay binnen het 16-bit veld; anders zou hij stil afkappen."""
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
# Rampaprofiel
# ----------------------------------------------------------------
def ramp_steps(rate0, rate1, accel_cm_s2=ACCEL_CM_S2):
    """Aantal stappen dat een ramp van rate0 naar rate1 beslaat."""
    a = _check_pos(accel_cm_s2, "acceleratie") / CM_PER_STEP     # stappen/s2
    return int(abs(rate1 * rate1 - rate0 * rate0) / (2.0 * a) + 0.5)


def ramp_words(n_steps, rate0, rate1, n_seg=RAMP_SEGMENTS):
    """S-curve rampatabel: lijst van FIFO-woorden, samen exact n_steps stappen.

    De snelheid volgt een smoothstep (3p^2 - 2p^3) in de AFGELEGDE WEG. Daardoor
    is de versnelling aan begin en eind nul, dus geen koppelschok op de
    overgangen (jerk-begrensd). Werkt zowel op als af: geef rate0 > rate1.

    Kost niets extra t.o.v. een lineaire ramp, want de tabel wordt hier in
    Python gegenereerd en daarna alleen nog door DMA afgespeeld.

    Geeft een LEGE lijst bij n_steps <= 0; aanroepers moeten daarop controleren.
    """
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
    """Kruisfase als zo weinig woorden mogelijk (max 65536 stappen per woord)."""
    out = []
    d = _delay_for(rate)
    rest = n_steps
    while rest > 0:
        chunk = 65536 if rest > 65536 else rest
        out.append(_word(chunk, d))
        rest -= chunk
    return out


def plan(n_total, v_cruise_cm_s, v_start_cm_s=V_START_CM_S, accel=ACCEL_CM_S2):
    """Bepaal (n_ramp, rate_start, rate_cruise) voor een beweging van n_total stappen.

    - Is de gevraagde snelheid al lager dan de startsnelheid, dan is er geen ramp
      nodig en rijdt de hele beweging op de GEVRAAGDE snelheid (niet op
      V_START_CM_S; de startsnelheid is een bovengrens voor veilig starten).
    - Is de beweging te kort om de topsnelheid te halen, dan wordt het profiel
      driehoekig: de topsnelheid wordt verlaagd tot op + af precies passen.
    """
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
        r1 = sqrt(r0 * r0 + a * n_total)
        n_ramp = n_total // 2
    return n_ramp, r0, r1


def profile_words(n_total, v_cruise_cm_s, v_start_cm_s=V_START_CM_S, accel=ACCEL_CM_S2):
    """Compleet profiel (op + kruis + af) als één woordenlijst, exact n_total stappen.

    Dit is de nul-CPU variant: de hele beweging in één DMA-transfer, één IRQ aan
    het eind. Gedrag identiek aan het oude fire-and-forget concept, maar mét ramp.
    Geeft een lege lijst bij n_total <= 0.
    """
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
    """Eén stappenmotor: generator-SM, teller-SM en een eigen DMA-kanaal."""

    def __init__(self, name, sm_id, cnt_id, step_gpio, dir_gpio, ena_gpio, fwd_level):
        self.name = name
        self.step = Pin(step_gpio, Pin.OUT)
        self.dir = Pin(dir_gpio, Pin.OUT)
        self.ena = Pin(ena_gpio, Pin.OUT)
        self.ena.value(1)                   # start uitgeschakeld: geen 1,8 W bij import
        self.fwd_level = fwd_level

        self.sm = StateMachine(sm_id, ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.cnt = StateMachine(cnt_id, step_counter, freq=F_PIO, in_base=self.step)

        # treq_sel = (pio_num << 3) + sm_num. Klopt ook op RP2350: DREQ_PIO2_TX0 = 16.
        self.dma = rp2.DMA()
        self._ctrl = self.dma.pack_ctrl(size=2, inc_read=True, inc_write=False,
                                        treq_sel=(0 << 3) + sm_id)
        self._buf = None                    # referentie vasthouden zolang de DMA loopt
        self.committed = 0                  # weggeschreven pulsen sinds reset
        self._sign = 1                      # huidige richting: +1 vooruit, -1 achteruit
        self._pos = 0                       # signed positie in stappen
        self._seen = 0                      # pulsteller-stand bij de laatste _sync()
        self._plan = None                   # (pulsen_bij_start, n_ramp, r0, r1, n_total)

        # De SM's blijven altijd actief. Zonder data in de FIFO stalt de
        # generator met STEP laag; dat is de rusttoestand. Zo is er geen
        # aparte start/stop-toestand die uit de pas kan lopen.
        self.cnt.active(1)
        self.sm.active(1)
        self.reset_pos()
        self.dir.value(fwd_level)

    # -- richting -------------------------------------------------
    def set_dir(self, forward):
        """Zet de richting. Verrekent eerst de tot nu toe gelopen stappen met het
        OUDE teken, zodat travel() over een richtingswisseling heen blijft kloppen."""
        sign = 1 if forward else -1
        if sign != self._sign:
            self.travel()                   # fold met het oude teken
            self._sign = sign
        self.dir.value(self.fwd_level if forward else 1 - self.fwd_level)

    # -- odometer -------------------------------------------------
    def reset_pos(self):
        self.cnt.put(0xFFFFFFFF)
        self.cnt.exec("pull()")
        self.cnt.exec("mov(y, osr)")
        self.committed = 0
        self._pos = 0
        self._seen = 0
        self._plan = None

    def pulses(self):
        """Monotone hardware-pulsteller. Kent geen richting.

        LET OP: dit telt COMMANDO'S, geen beweging. Bij wielslip (hobbel, gleuf)
        loopt deze teller door. Voor werkelijke beweging heb je de GY9250 nodig.
        """
        self.cnt.exec("mov(isr, y)")
        self.cnt.exec("push()")
        return 0xFFFFFFFF - self.cnt.get()

    def travel(self):
        """Signed positie in stappen: vooruit positief, achteruit negatief."""
        p = self.pulses()
        d = p - self._seen
        if d:
            self._pos += self._sign * d
            self._seen = p
        return self._pos

    # -- data naar de FIFO ---------------------------------------
    def push(self, repeat, delay):
        """Zet één segment in de FIFO (kruisfase; CPU-pad)."""
        self.sm.put(_word(repeat, delay))
        self.committed += repeat

    def start_table(self, words, n_steps):
        """Speel een woordenlijst af via DMA (rampafasen; nul-CPU pad).

        Doet niets bij een lege lijst: een DMA-transfer met count=0 is
        firmware-afhankelijk gedrag en moeten we niet opzoeken.
        """
        if not words or n_steps <= 0:
            return False
        # Loopt er nog een transfer, dan wordt die afgekapt in plaats van erop te
        # wachten. Wachten zou hier seconden kunnen blokkeren, en de aanroeper
        # heeft expliciet besloten een nieuwe tabel te starten.
        if self.dma.active():
            self.dma.active(0)
        self._buf = array('I', words)
        self.dma.config(read=self._buf, write=self.sm,
                        count=len(self._buf), ctrl=self._ctrl, trigger=True)
        self.committed += n_steps
        return True

    def note_plan(self, n_ramp, r0, r1, n_total):
        """Bewaar het profiel zodat current_rate() de snelheid kan schatten."""
        self._plan = (self.pulses(), n_ramp, r0, r1, n_total)

    def current_rate(self):
        """Schat de huidige stapfrequentie uit de voortgang door het profiel.

        Nodig om te kunnen afremmen zonder synchronismeverlies: afremmen vanaf
        een te hoog veronderstelde snelheid geeft eerst een snelheidssprong
        omhoog. Geeft None als er geen profiel bekend is.
        """
        if self._plan is None:
            return None
        p0, n_ramp, r0, r1, n_total = self._plan
        done = self.pulses() - p0
        if done < 0:
            done = 0
        if n_ramp > 0:
            if done < n_ramp:
                q = done / n_ramp
            elif done > n_total - n_ramp:
                q = (n_total - done) / n_ramp
                if q < 0.0:
                    q = 0.0
            else:
                return r1
            s = q * q * (3.0 - 2.0 * q)
            return r0 + (r1 - r0) * s
        return r1

    # -- status ---------------------------------------------------
    def busy(self, snapshot=None):
        """True zolang niet alle weggeschreven pulsen ook uitgestuurd zijn."""
        p = self.pulses() if snapshot is None else snapshot
        return p < self.committed

    # -- stoppen --------------------------------------------------
    def _clear(self):
        """Stop de DMA en wis de TX-FIFO.

        sm.init() is hier bewust gebruikt: MicroPython heeft geen API om de FIFO
        te wissen, en init() doet intern clear_fifos + restart. Dat is veiliger
        dan zelf in SHIFTCTRL schrijven. Daarna klopt `committed` niet meer met
        wat er nog gaat gebeuren, dus die wordt op de werkelijke pulsstand gezet.
        """
        self.dma.active(0)
        self.sm.active(0)
        self.sm.init(ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.sm.active(1)
        self.travel()                       # signed positie bijwerken vóór de reset
        self.committed = self.pulses()
        self._plan = None

    def halt(self):
        """Onmiddellijke stop, driver blijft AAN: de motor houdt zijn positie."""
        self._clear()

    def abort(self):
        """Noodstop: driver UIT, motor loopt vrij. De kar kan doorrollen."""
        self.ena.value(1)
        self._clear()

    def brake(self, accel=ACCEL_CM_S2):
        """Rem af vanaf de geschatte huidige snelheid naar de startsnelheid."""
        rate = self.current_rate()
        self._clear()
        if rate is None:
            return
        r0 = rate_of(V_START_CM_S)
        if rate <= r0:
            return
        n = ramp_steps(rate, r0, accel)
        self.start_table(ramp_words(n, rate, r0), n)

    def enable(self):
        self.ena.value(0)

    def disable(self):
        self.ena.value(1)


# ----------------------------------------------------------------
# Hardware-instantiatie
# ----------------------------------------------------------------
# GPIO-toewijzing overgenomen uit stepper.py, zodat dit een drop-in vervanger is
# en rotate('l') dezelfde kant op draait.
#
# LET OP: hardware/gpio_pinout.md noemt GPIO16/17/18 "stepper B" (U2) en
# GPIO12/13/14 "stepper A" (U1) — precies omgekeerd aan de namen hieronder.
# Functioneel maakt het niets uit, maar code en schema spreken elkaar tegen.
MA = _Motor("A", 0, 2, step_gpio=17, dir_gpio=16, ena_gpio=18, fwd_level=0)
MB = _Motor("B", 1, 3, step_gpio=13, dir_gpio=12, ena_gpio=14, fwd_level=1)
_MOTORS = (MA, MB)


def enable():
    for m in _MOTORS:
        m.enable()


def disable():
    for m in _MOTORS:
        m.disable()


# ----------------------------------------------------------------
# Publieke API — fire-and-forget (geen bijsturing, nul CPU-overhead)
# ----------------------------------------------------------------
def _launch(dirs, speed, n):
    """Start hetzelfde profiel op de gegeven motoren. dirs = ((motor, forward), ...)"""
    if n <= 0:
        return False
    n_ramp, r0, r1 = plan(n, speed)
    words = profile_words(n, speed)
    if not words:
        return False
    for m, fwd in dirs:
        m.halt()            # een nieuw commando overschrijft een lopende beweging
        m.set_dir(fwd)
    enable()
    for m, _ in dirs:
        m.note_plan(n_ramp, r0, r1, n)
        m.start_table(words, n)
    return True


def s1(direction, speed, dist):
    """Alleen motor A. direction 'f'/'b', speed cm/s, dist cm."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    return _launch(((MA, fwd),), _check_pos(speed, "speed"), cm_to_steps(dist))


def s2(direction, speed, dist):
    """Alleen motor B. direction 'f'/'b', speed cm/s, dist cm."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    return _launch(((MB, fwd),), _check_pos(speed, "speed"), cm_to_steps(dist))


def mov(direction, speed, dist):
    """Beide motoren dezelfde kant, dezelfde afstand."""
    fwd = _check_dir(direction, ('f', 'b'), "direction") == 'f'
    return _launch(((MA, fwd), (MB, fwd)), _check_pos(speed, "speed"), cm_to_steps(dist))


def rotate(direction, speed, dist):
    """Draai op de as. direction 'l'/'r', dist = booglengte per wiel in cm."""
    right = _check_dir(direction, ('l', 'r'), "direction") == 'r'
    return _launch(((MA, not right), (MB, right)),
                   _check_pos(speed, "speed"), cm_to_steps(dist))


def rotate_deg(degrees, speed=V_MAX_CM_S / 2):
    """Draai op de as over een hoek. Positief = naar rechts (zie TURN_SIGN).

    Per graad legt elk wiel STEPS_PER_DEG/2 stappen af, tegengesteld.
    360 graden = 28632 stappen per wiel = 2,24 wielomwentelingen.
    """
    n = int(abs(degrees) * STEPS_PER_DEG / 2 + 0.5)
    right = (TURN_SIGN * degrees) >= 0
    return _launch(((MA, not right), (MB, right)), _check_pos(speed, "speed"), n)


# ----------------------------------------------------------------
# Eindpositionering — het laatste correctiemoment
# ----------------------------------------------------------------
def creep(dist_cm, speed_cm_s=2.0):
    """Kruipcorrectie: kleine, exacte verplaatsing. Positief = vooruit.

    Bij 2 cm/s is er praktisch geen ramp nodig (dat ligt al boven de veilige
    startsnelheid van 1,91 cm/s), dus de beweging is meteen exact en zacht.
    Resolutie is 14,9 um; de meting is dus altijd de beperkende factor.
    """
    if abs(dist_cm) < CM_PER_STEP:
        return False
    return mov('f' if dist_cm > 0 else 'b', speed_cm_s, abs(dist_cm))


def _mean_dist(read_cm, n=8, wacht_ms=60):
    """Gemiddelde afstand uit n ONAFHANKELIJKE ultrasoonmetingen.

    wacht_ms moet groter zijn dan ultrasoon.INTERVAL_MS (50 ms), anders lees je
    dezelfde gebufferde meting meerdere keren en doet het gemiddelde niets.
    """
    import time
    som, tel = 0.0, 0
    for i in range(n):
        if i:
            time.sleep_ms(wacht_ms)
        d = read_cm()
        if d and d > 0:
            som += d
            tel += 1
    return (som / tel) if tel else None


def finetune(read_cm, object_w_cm=None, tol_cm=0.1, pogingen=3,
             speed_cm_s=2.0, n_meet=8):
    """Meet stilstaand na en kruip naar de exacte stopafstand.

    Dit is het LAATSTE correctiemoment. Zodra de arm uitklapt kijkt de ultrasoon
    naar de eigen vingers en is er geen terugkoppeling meer -- alles daarna is
    open-loop. Stilstaand meten haalt de rijsnelheid en de meetlatentie uit de
    fout, waardoor de onzekerheid van ~0,74 cm naar ~0,3 cm zakt.

    `read_cm` is een callable die de ultrasoonafstand in cm geeft (bv.
    ultrasoon.read_cm), zodat deze module sensor-agnostisch blijft.

    Geeft (gemeten_afstand, doel, gelukt) terug.
    """
    doel = stop_dist_cm(object_w_cm)
    lo, hi = grip_window_cm(object_w_cm)
    d = None
    for _ in range(max(1, int(pogingen))):
        d = _mean_dist(read_cm, n_meet)
        if d is None:
            return None, doel, False
        fout = d - doel
        if abs(fout) <= tol_cm:
            break
        creep(fout, speed_cm_s)
        while busy():
            pass
    gelukt = d is not None and lo <= d <= hi + tol_cm
    print("finetune: gemeten %.2f cm, doel %.2f cm, venster %.2f-%.2f -> %s"
          % (d if d else -1, doel, lo, hi, "OK" if gelukt else "BUITEN VENSTER"))
    return d, doel, gelukt


def halt():
    """Onmiddellijke stop, drivers blijven aan: de motoren houden hun positie.

    Dit is het gedrag van stepper.stop() uit de oude module. Er wordt NIET
    afgeremd; gebruik brake() als je een remramp wilt.
    """
    for m in _MOTORS:
        m.halt()


def brake(accel=ACCEL_CM_S2):
    """Nette stop: rem beide motoren af vanaf de geschatte huidige snelheid.

    De snelheid wordt geschat uit de voortgang door het lopende profiel, dus dit
    werkt alleen voor bewegingen die via deze module gestart zijn. Bij een
    Move() is Move.finish() nauwkeuriger, want die weet exact waar hij zit.
    """
    for m in _MOTORS:
        m.brake(accel)


def emergency_stop():
    """Noodstop: drivers uit, DMA stil, FIFO's leeg. Motoren lopen vrij.

    De kar kan doorrollen, dus de positie is daarna geen betrouwbare meting meer.
    """
    for m in _MOTORS:
        m.abort()


# stop() heette in stepper.py een directe stop en doet dat hier ook, zodat
# bestaande code hetzelfde gedrag houdt.
stop = halt


def busy():
    return MA.busy() or MB.busy()


# ----------------------------------------------------------------
# Grijpergeometrie
# ----------------------------------------------------------------
# De kaken sluiten HORIZONTAAL, maar de vingertoppen bewegen daarbij naar VOREN.
# Gemeten: maximaal open (9 cm) -> toppen 12 cm vóór de ultrasoon;
#          bijna dicht  (2 cm) -> toppen 15 cm vóór de ultrasoon.
#
# LET OP: tip_pos_cm() is een RECHTE door die twee meetpunten. Een
# vierstangenmechanisme geeft in werkelijkheid een kromme; één extra meting bij
# ~5 cm opening laat zien hoeveel dat afwijkt.
#
# Tijdens het rijden staan de servo's in rustpositie: de kaken liggen dan ACHTER
# de ultrasoon en vallen buiten de bundel, dus de afstandsmeting is dan zuiver.
# Zodra de arm uitklapt staan de kaken (9 cm open) in een bundel die op 12-15 cm
# ongeveer 8 cm breed is -> vanaf dat moment kijkt de sensor naar de eigen
# vingers en is er GEEN terugkoppeling meer.
GRIP_OPEN_CM  = 9.0      # kaakopening maximaal open
GRIP_MIN_CM   = 2.0      # kaakopening bijna dicht
TIP_NEAR_CM   = 12.0     # toppen t.o.v. ultrasoon bij maximaal open
TIP_FAR_CM    = 15.0     # toppen t.o.v. ultrasoon bij bijna dicht
OBJECT_W_CM   = 6.0      # aangenomen objectbreedte; per missie te overschrijven


def tip_pos_cm(opening_cm):
    """Afstand van de vingertoppen tot de ultrasoon bij een gegeven kaakopening."""
    f = (GRIP_OPEN_CM - opening_cm) / (GRIP_OPEN_CM - GRIP_MIN_CM)
    return TIP_NEAR_CM + f * (TIP_FAR_CM - TIP_NEAR_CM)


def stop_dist_cm(object_w_cm=None):
    """Doelafstand (ultrasoon) voor een voorwerp van deze breedte.

    De kaken raken het voorwerp op het moment dat de opening gelijk is aan de
    objectbreedte; dan staan de toppen op tip_pos_cm(breedte).

    Contra-intuïtief maar juist: een SMALLER voorwerp vraagt een GROTERE
    stopafstand. Smaller betekent verder sluiten, dus meer vooruitgang van de
    toppen, dus moet de kar verder terug blijven staan.

        3 cm -> 14,6 cm      6 cm -> 13,3 cm
        5 cm -> 13,7 cm      8 cm -> 12,4 cm
    """
    return tip_pos_cm(OBJECT_W_CM if object_w_cm is None else object_w_cm)


def grip_window_cm(object_w_cm=None):
    """(min, max) ultrasoonafstand waarbij het voorwerp nog gegrepen wordt.

    Het voorwerp wordt gegrepen zolang de toppen er langs vegen terwijl de
    opening nog ruimer is dan het voorwerp. De vooruitgang van 3 cm is dus een
    gratis venster bovenop de stopnauwkeurigheid:

        3 cm breed -> 2,6 cm venster      7 cm breed -> 0,9 cm venster
        5 cm breed -> 1,7 cm venster      8 cm breed -> 0,4 cm venster

    De ondergrens is conservatief TIP_NEAR_CM. De werkelijke ondergrens ligt
    lager en wordt bepaald door de KAAKDIEPTE (palm t.o.v. toppen), die nog niet
    is opgemeten.
    """
    return TIP_NEAR_CM, stop_dist_cm(object_w_cm)


def lateral_tolerance_cm(object_w_cm=None):
    """Maximale laterale afwijking waarbij het voorwerp nog tussen de kaken past.

    De kaken vegen tijdens het uitklappen door de ruimte waar het voorwerp
    staat. Bij een grotere afwijking raakt één kaak het voorwerp en STOOT die
    het om -- een vervelender faalmodus dan alleen misgrijpen. Klap de arm
    daarom uit BOVEN het voorwerp en laat hem zakken; dan komen de kaken er
    van boven om heen in plaats van er horizontaal in.

        3 cm -> +/- 3,0 cm      6 cm -> +/- 1,5 cm
        5 cm -> +/- 2,0 cm      7 cm -> +/- 1,0 cm
    """
    w = OBJECT_W_CM if object_w_cm is None else object_w_cm
    return 0.5 * (GRIP_OPEN_CM - w)


STOP_DIST_CM = tip_pos_cm(OBJECT_W_CM)      # 13,3 cm bij een voorwerp van 6 cm


def stopping_distance_cm(speed_cm_s=V_MAX_CM_S, accel=ACCEL_CM_S2):
    """Afstand die de kar nog aflegt nadat je Move.finish() aanroept.

    Som van twee posten die je niet meer kunt intrekken:
      - de afremramp vanaf `speed` naar de startsnelheid;
      - de kruis-slices die al in de FIFO's staan (FIFO_TARGET * SLICE_MS).

    Gebruik dit om het afremmen vooruit te plannen; roep je finish() pas aan
    OP de doelafstand, dan schiet je er met deze afstand voorbij:

        doel = STOP_DIST_CM + stopping_distance_cm(snelheid)
        if ultrasoon.read_cm() <= doel:
            mv.finish()

    Bij 19,1 cm/s is dit ~4,4 cm; bij 5 cm/s nog maar ~0,5 cm. Rem daarom voor
    de laatste centimeters af naar een lage snelheid — dat is veel nauwkeuriger
    dan proberen de stopafstand exact te voorspellen.

    NIET meegerekend: de meetlatentie van de ultrasoon zelf (INTERVAL_MS = 50 ms
    in ultrasoon.py, dus tot 0,96 cm bij volle snelheid en 0,25 cm bij 5 cm/s).
    """
    r0 = rate_of(V_START_CM_S)
    r1 = rate_of(min(abs(speed_cm_s), V_MAX_CM_S))
    ramp = ramp_steps(r0, r1, accel) if r1 > r0 else 0
    in_fifo = r1 * (SLICE_MS * FIFO_TARGET) / 1000.0
    return steps_to_cm(ramp + in_fifo)


# ----------------------------------------------------------------
# Odometrie — beide grootheden komen uit de hardware-tellers
# ----------------------------------------------------------------
def pio_pos1():
    return MA.pulses()


def pio_pos2():
    return MB.pulses()


def reset_PIO_distance():
    for m in _MOTORS:
        m.reset_pos()


def distance():
    """Afgelegde weg van het midden van de kar, in cm.

    Gebruikt de SIGNED posities, dus een rotatie op de plaats geeft ~0 cm en
    achteruit rijden telt negatief.
    """
    return steps_to_cm((MA.travel() + MB.travel()) / 2.0)


def heading():
    """Koersverandering sinds de laatste reset, in graden. Positief = rechts.

    Volgt uit het SIGNED stappenverschil tussen de wielen, dus dit werkt ook bij
    een rotatie op de plaats. Ziet GEEN wielslip; voor de werkelijke koers moet
    je dit fuseren met de GY9250-gyro.
    """
    return TURN_SIGN * (MA.travel() - MB.travel()) / STEPS_PER_DEG


def status():
    """Print de toestand. Elke teller wordt één keer gelezen, zodat de regels
    onderling consistent zijn."""
    print("=== STEPPER RAMP ===")
    snap = []
    for m in _MOTORS:
        p = m.pulses()
        snap.append((m, p, m.travel()))
    for m, p, t in snap:
        print(" Motor %s: %-5s pulsen=%d  positie=%+d (%+.2f cm)  committed=%d" %
              (m.name, "bezig" if p < m.committed else "stil",
               p, t, steps_to_cm(t), m.committed))
    mid = steps_to_cm((snap[0][2] + snap[1][2]) / 2.0)
    hdg = TURN_SIGN * (snap[0][2] - snap[1][2]) / STEPS_PER_DEG
    print(" Midden: %+.2f cm   koers: %+.2f graden" % (mid, hdg))
    print(" Max stuurgezag: %.1f graden/s" % MAX_TURN_DEG_S)
    print("====================")


# ----------------------------------------------------------------
# Beweging MET doorlopende koerscorrectie
# ----------------------------------------------------------------
class Move:
    """Een rechte beweging waarbij de koers tijdens het rijden bijgestuurd wordt.

    De ramps lopen via DMA, de kruisfase wordt per slice door de CPU gevuld.
    Roep service() minstens één keer per SLICE_MS/2 aan, vanuit een asyncio-taak
    of een while-lus. Dat kost ~0,1% CPU.

    De ramps zelf worden NIET bijgestuurd: die duren maar ~3,3 cm en symmetrisch
    houden is eenvoudiger dan de winst waard.
    """

    _RAMP_UP, _CRUISE, _RAMP_DOWN, _DONE = 0, 1, 2, 3

    def __init__(self, dist_cm, speed_cm_s, correction=None,
                 v_start=V_START_CM_S, accel=ACCEL_CM_S2, forward=True):
        self.correction = correction         # callable -> gewenste draaisnelheid in graden/s
        self.n_total = cm_to_steps(dist_cm)
        self.n_ramp, self.r0, self.r1 = plan(self.n_total, speed_cm_s, v_start, accel)
        self._centre = 0                     # midden-stappen die al weggeschreven zijn
        self._slice_base = max(1, int(self.r1 * SLICE_MS / 1000.0 + 0.5))

        if self.n_total <= 0:
            self._state = self._DONE
            return

        # Brugsegment: houdt de FIFO gevuld over de DMA -> CPU overgang heen.
        n_cruise = self.n_total - 2 * self.n_ramp
        bridge = BRIDGE_SLICES * self._slice_base
        if bridge > n_cruise:
            bridge = n_cruise

        for m in _MOTORS:
            m.set_dir(forward)
        reset_PIO_distance()
        enable()

        up = ramp_words(self.n_ramp, self.r0, self.r1)
        up.extend(cruise_words(bridge, self.r1))
        n_up = self.n_ramp + bridge
        for m in _MOTORS:
            m.note_plan(self.n_ramp, self.r0, self.r1, self.n_total)
            m.start_table(up, n_up)
        self._centre = n_up
        self._state = self._RAMP_UP

    # -- interne helpers -----------------------------------------
    def _push_slice(self):
        """Schrijf één kruis-slice weg, met de actuele koerscorrectie erin."""
        remaining = self.n_total - self.n_ramp - self._centre
        base = self._slice_base if self._slice_base < remaining else remaining
        if base < 1:
            return False

        # Gewenste draaisnelheid -> stappenverschil over deze slice.
        delta = 0
        if self.correction is not None:
            t = base / self.r1                       # duur van deze slice in s
            omega = TURN_SIGN * self.correction()    # graden/s, positief = rechts
            delta = int(omega * STEPS_PER_DEG * t / 2.0 + (0.5 if omega >= 0 else -0.5))
            lim = int(base * MAX_DIFF_FRAC)
            if delta > lim:
                delta = lim
            elif delta < -lim:
                delta = -lim

        ra = base + delta
        rb = base - delta
        if ra < 1 or rb < 1:
            ra = rb = base

        # Zelfde slice-DUUR voor beide motoren, verschillend stappenaantal.
        # Zo blijven ze in de tijd synchroon en is ra + rb exact 2 * base,
        # waardoor het midden precies `base` stappen opschuift.
        cyc = F_PIO * base / self.r1                 # PIO-cycles voor deze slice
        MA.push(ra, _clamp_delay(int(cyc / ra + 0.5) - CYCLES_FIXED))
        MB.push(rb, _clamp_delay(int(cyc / rb + 0.5) - CYCLES_FIXED))
        self._centre += base
        return True

    # -- aanroepen uit de regellus -------------------------------
    def service(self):
        """Vul de FIFO's bij en pas de correctie toe. False = beweging klaar."""
        if self._state == self._RAMP_UP:
            # Wacht tot de DMA is gestopt met SCHRIJVEN. De FIFO bevat dan nog
            # data (o.a. het brugsegment), dus de motor loopt door terwijl wij
            # de kruisfase overnemen. Zou de CPU nu al pushen, dan konden
            # correctie- en rampsegmenten door elkaar lopen.
            if MA.dma.active() or MB.dma.active():
                return True
            self._state = self._CRUISE

        if self._state == self._CRUISE:
            while MA.sm.tx_fifo() < FIFO_TARGET and MB.sm.tx_fifo() < FIFO_TARGET:
                if not self._push_slice():
                    self._state = self._RAMP_DOWN
                    break

        if self._state == self._RAMP_DOWN:
            # De FIFO bevat nog kruis-slices; de DMA schrijft daar netjes
            # achteraan. De CPU pusht vanaf hier niets meer, dus de volgorde
            # kan niet door elkaar lopen.
            down = ramp_words(self.n_ramp, self.r1, self.r0)
            for m in _MOTORS:
                m.start_table(down, self.n_ramp)
            self._state = self._DONE

        if self._state == self._DONE and not busy():
            return False
        return True

    def finish(self):
        """Breek de kruisfase af en rem meteen netjes af."""
        if self._state in (self._RAMP_UP, self._CRUISE):
            self._state = self._RAMP_DOWN
            self.n_total = self._centre + self.n_ramp
        return self.service()


def drive(dist_cm, speed_cm_s=V_MAX_CM_S, correction=None, **kw):
    """Start een beweging met bijsturing. Geeft een Move terug.

    Zonder `correction` is `mov()` beter: dan loopt alles in één DMA-transfer.

        mv = drive(50, 19.1, correction=hc.output)
        while mv.service():
            time.sleep_ms(SLICE_MS // 2)
    """
    return Move(dist_cm, speed_cm_s, correction=correction, **kw)


async def adrive(dist_cm, speed_cm_s=V_MAX_CM_S, correction=None, **kw):
    """asyncio-variant van drive()."""
    try:
        import asyncio
    except ImportError:
        import uasyncio as asyncio
    mv = Move(dist_cm, speed_cm_s, correction=correction, **kw)
    while mv.service():
        await asyncio.sleep_ms(SLICE_MS // 2)
    return mv


# ----------------------------------------------------------------
# GY9250-koppeling
# ----------------------------------------------------------------
def gyro_z_deg_s(sensor):
    """Maak van een MPU6500/MPU9250-object een callable die GRADEN/s teruggeeft.

    De driver in lib/GY9250 heeft `gyro_sf=SF_RAD_S` als default en levert dus
    RADIALEN/s, terwijl HeadingController in graden/s rekent. Rechtstreeks
    `sensor.gyro[2]` doorgeven maakt de gyrocorrectie 57,3x te klein.

    Draait de sensor op core 1, geef dan geen sensor-object mee maar een eigen
    callable die de door core 1 gepubliceerde waarde leest. Lees dezelfde
    I2C-sensor niet vanaf twee cores.

        hc = HeadingController(ldr_diff=..., gyro_rate=gyro_z_deg_s(sensor))
    """
    def _read():
        return sensor.gyro[2] * RAD_TO_DEG
    return _read


# ----------------------------------------------------------------
# Koersregelaar: LDR (richting) + gyro (storingsonderdrukking)
# ----------------------------------------------------------------
class HeadingController:
    """Cascade-koersregelaar voor het naderen van de lichtbron.

    BUITENLUS (langzaam, enkele Hz) — LDR
        Het verschil tussen de twee LDR's bepaalt WAAR we heen moeten.
        A == B betekent recht op de bron. Levert de gewenste draaisnelheid.
        Vereist de LDR-gevoeligheidskalibratie; zonder die correctie stuurt de
        kar structureel scheef.

    BINNENLUS (snel, elke slice) — GY9250 gyro-Z
        Onderdrukt storingen: hobbels, gleuven, wielslip, ongelijke vloer.
        Meet de WERKELIJKE draaisnelheid; het verschil met de gewenste
        draaisnelheid wordt weggeregeld. Werkt ook als de lichtbron even
        wordt afgedekt.

    Dit is géén dubbele besturing: de LDR bepaalt de richting, de gyro alleen
    de storingsonderdrukking. Ze sturen niet hetzelfde ding.

    Het magnetometer/kompas is tijdens het rijden expres NIET gebruikt: de
    stappenmotoren verstoren het veld (zie de GY9250-stappenmotorkalibratie).
    Het kompas is voor de terugweg, waar een absolute koers nodig is.

    `gyro_rate` moet GRADEN/s leveren — gebruik gyro_z_deg_s(sensor).
    """

    def __init__(self, ldr_diff=None, gyro_rate=None,
                 kp_ldr=25.0, kp_gyro=0.6, deadband=0.03, outer_div=5,
                 slip_deg_s=8.0, slip_ticks=10):
        self.ldr_diff = ldr_diff        # callable -> (A-B)/(A+B) na gain-correctie, in [-1, 1]
        self.gyro_rate = gyro_rate      # callable -> gemeten draaisnelheid in GRADEN/s, rechts +
        self.kp_ldr = kp_ldr            # graden/s per eenheid LDR-verschil
        self.kp_gyro = kp_gyro          # versterking van de binnenlus
        self.deadband = deadband        # LDR-verschil waaronder we recht doorrijden
        self.outer_div = max(1, int(outer_div))
        self.slip_deg_s = slip_deg_s    # drempel voor de slipmelding
        self.slip_ticks = max(1, int(slip_ticks))
        self._tick = 0
        self._setpoint = 0.0            # gewenste draaisnelheid in graden/s
        self._last_cmd = 0.0            # wat we vorige tick gecommandeerd hebben
        self._slip_run = 0
        self.slipping = False

    def output(self):
        """Geef de te commanderen draaisnelheid in graden/s. Positief = rechts.

        Dit is de callable die je aan drive(correction=...) meegeeft.
        """
        # --- buitenlus: LDR bepaalt het setpoint --------------------
        if self.ldr_diff is not None and self._tick % self.outer_div == 0:
            d = self.ldr_diff()
            if -self.deadband < d < self.deadband:
                self._setpoint = 0.0
            else:
                self._setpoint = self.kp_ldr * d
        self._tick += 1

        cmd = self._setpoint

        # --- binnenlus: gyro onderdrukt storingen -------------------
        if self.gyro_rate is not None:
            measured = self.gyro_rate()
            cmd += self.kp_gyro * (self._setpoint - measured)

            # Slipdetectie: blijft de GEMETEN draaisnelheid aanhoudend afwijken
            # van wat we hebben GECOMMANDEERD, dan slipt er een wiel. Merk op
            # dat lineaire slip (beide wielen slippen recht vooruit) hiermee
            # NIET te zien is - dat vraagt een externe positiereferentie.
            # Alleen "motor actief + weinig versnelling" werkt niet: bij
            # constante snelheid is de voorwaartse versnelling immers nul.
            if abs(self._last_cmd - measured) > self.slip_deg_s:
                self._slip_run += 1
            else:
                self._slip_run = 0
            self.slipping = self._slip_run >= self.slip_ticks

        if cmd > MAX_TURN_DEG_S:
            cmd = MAX_TURN_DEG_S
        elif cmd < -MAX_TURN_DEG_S:
            cmd = -MAX_TURN_DEG_S
        self._last_cmd = cmd
        return cmd


def hold_heading(gyro_rate):
    """Kortste variant: rij recht en houd de koers vast met alleen de gyro.

        mv = drive(50, 19.1, correction=hold_heading(gyro_z_deg_s(sensor)))
    """
    return HeadingController(ldr_diff=None, gyro_rate=gyro_rate).output


# ----------------------------------------------------------------
# Zelftest / referentiegetallen
# ----------------------------------------------------------------
def info():
    """Print de afgeleide ontwerpgetallen. Geen hardware nodig."""
    r0 = rate_of(V_START_CM_S)
    r1 = rate_of(V_MAX_CM_S)
    n_ramp = ramp_steps(r0, r1)
    print("CM_PER_STEP      %.5f cm (%.2f um)" % (CM_PER_STEP, CM_PER_STEP * 1e4))
    print("STEPS_PER_DEG    %.1f stappen verschil per graad" % STEPS_PER_DEG)
    print("360 graden       %d stappen per wiel" % int(360 * STEPS_PER_DEG / 2))
    print("startsnelheid    %.2f cm/s = %d stappen/s, delay %d"
          % (V_START_CM_S, r0, _delay_for(r0)))
    print("topsnelheid      %.2f cm/s = %d stappen/s, delay %d"
          % (V_MAX_CM_S, r1, _delay_for(r1)))
    print("ramp             %d stappen = %.2f cm, %d segmenten = %d bytes"
          % (n_ramp, steps_to_cm(n_ramp), RAMP_SEGMENTS, 4 * RAMP_SEGMENTS))
    print("piekversnelling  %.0f cm/s2 (1,5x de ingestelde %.0f)"
          % (1.5 * ACCEL_CM_S2, ACCEL_CM_S2))
    print("slice            %d ms = %d stappen, runway %d ms, brug %d ms"
          % (SLICE_MS, int(r1 * SLICE_MS / 1000), SLICE_MS * FIFO_TARGET,
             SLICE_MS * BRIDGE_SLICES))
    print("stuurgezag       +/- %.1f graden/s, resolutie %.4f graden"
          % (MAX_TURN_DEG_S, 1.0 / STEPS_PER_DEG))
    print("max kruis-woord  %d stappen = %.1f cm" % (65536, steps_to_cm(65536)))


def meet_frequentie(rate=None, n_stappen=12800):
    """Meet de werkelijke STEP-frequentie tegen _delay_for(), om CYCLES_FIXED
    te verifieren. Draait één motor op een vaste snelheid en klokt de tijd.

    Verifieer dit ook met een logic analyzer: die ziet ook de pulsbreedte
    (verwacht 200 ns) en of er geen stappen wegvallen.
    """
    import time
    if rate is None:
        rate = rate_of(V_MAX_CM_S)
    d = _delay_for(rate)
    MA.set_dir(True)
    MA.enable()
    p0 = MA.pulses()
    t0 = time.ticks_us()
    rest = n_stappen
    while rest > 0:
        chunk = 65536 if rest > 65536 else rest
        MA.push(chunk, d)
        rest -= chunk
    while MA.busy():
        time.sleep_ms(2)
    dt = time.ticks_diff(time.ticks_us(), t0) / 1e6
    n = MA.pulses() - p0
    gemeten = n / dt
    print("delay=%d  verwacht %.1f st/s  gemeten %.1f st/s  afwijking %+.2f%%"
          % (d, rate, gemeten, 100.0 * (gemeten / rate - 1.0)))
    print("implied CYCLES_FIXED = %.2f (nu %d)" % (F_PIO / gemeten - d, CYCLES_FIXED))
    return gemeten
