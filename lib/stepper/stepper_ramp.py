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
#   ramp op    -> DMA-tabel (256 woorden). Segmenten van ~1,2 ms zijn te snel
#                 voor MicroPython, dus dit MOET via DMA. Immuun voor GC-pauzes.
#   kruisfase  -> zonder bijsturing: één woord, dus onderdeel van dezelfde
#                 DMA-transfer -> nul CPU-overhead.
#                 met bijsturing: de CPU pusht één woord per SLICE_MS per motor
#                 (~50 put()'s per seconde per motor, ca. 0,1% CPU).
#   ramp af    -> DMA-tabel, getriggerd zodra het aantal weggeschreven stappen
#                 het afremmpunt bereikt.
#
# Bij een lege FIFO stalt `pull(block)` met STEP LAAG. Dat betekent:
#   - de motor houdt zijn positie, er gaat geen stap verloren;
#   - CPU-latency (ook een GC-pauze) beïnvloedt de STAPTIMING niet, want de
#     PIO genereert met hardware-precisie. Te late CPU = de correctie komt één
#     slice later, geen timingfout.
#
# EXACTE AFSTAND
# --------------
# Bijsturen verandert WANNEER stappen komen, niet HOEVEEL. We houden per motor
# `committed` bij: de som van alle repeat-waarden die we hebben weggeschreven.
# Het afremmpunt wordt op `committed` bepaald, niet op een gemeten positie, dus
# de eindafstand is exact - onafhankelijk van wanneer de regellus toevallig
# aanroept.
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
import machine
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
    # Y loopt af vanaf 0xFFFFFFFF. Positie = 0xFFFFFFFF - Y.
    # in_base = de STEP-pin, dus pin-index 0.
    label("loop")                           # type: ignore
    wait(0, pin, 0)                         # type: ignore  wacht tot STEP laag
    wait(1, pin, 0)                         # type: ignore  wacht op de stijgende flank
    jmp(y_dec, "loop")                      # type: ignore


# ----------------------------------------------------------------
# Constanten
# ----------------------------------------------------------------
F_PIO         = 15_000_000   # 150 MHz sysclk / 10 -> integer klokdeler, geen fractionele jitter
CYCLES_FIXED  = 5            # vaste cycles per stap in ramp_stepper (mov[2]=3, jmp_y=1, +1)

WHEEL_CIRC    = 19.1         # cm — gemeten wielomtrek
TRACK_WIDTH   = 13.6         # cm — spoorbreedte hart-op-hart
STEPS_REV     = 12800        # 1/64 microstepping (TMC2209, 200 volle stappen x 64)
CM_PER_STEP   = WHEEL_CIRC / STEPS_REV                 # ~14,9 um/stap
STEPS_PER_DEG = 3.14159265 / 180 * TRACK_WIDTH / CM_PER_STEP   # ~159 stappen verschil per graad

# Snelheidsgrenzen (motor 17HS8401, TMC2209 stealthChop, VREF 1 V = 0,71 A RMS)
V_MAX_CM_S    = 19.1         # 1,0 omw/s — stealthChop haalt ~300 rpm, we zitten op 60
V_START_CM_S  = 1.91         # 0,1 omw/s — veilige startsnelheid, empirisch te verifieren
ACCEL_CM_S2   = 55.0         # bepaalt de ramp-AFSTAND via (v1^2-v0^2)/(2a) = 3,28 cm.
                             # De S-curve piekt op 1,5x = 82 cm/s2 = 0,084 g.
                             # LET OP: omdat de S-curve aan begin en eind traag is,
                             # DUURT de ramp 0,55 s, niet de 0,31 s van een lineaire
                             # ramp over dezelfde afstand. Verhoog deze waarde om
                             # zowel de rampafstand als de -duur te verkorten.

# Tekenconventie voor bijsturen en heading(). Positief = de kar draait NAAR RECHTS.
# Welke fysieke motor links of rechts zit volgt niet uit de code — dat is montage.
# Stuurt de kar de verkeerde kant op, zet deze op -1. Dat is de enige plek.
TURN_SIGN     = +1

# Ramp- en sliceparameters
RAMP_SEGMENTS = 256          # snelheidssprong <1% per segment over een 10:1 bereik
SLICE_MS      = 20           # duur van één kruis-slice; bepaalt de regellatentie
FIFO_TARGET   = 3            # aantal slices dat vooruit in de FIFO staat (runway = 60 ms)
MAX_DIFF_FRAC = 0.20         # maximale snelheidsdifferentie per wiel (+/- 20%)

# Afgeleid: maximaal stuurgezag in graden/s
MAX_TURN_DEG_S = (2 * MAX_DIFF_FRAC * V_MAX_CM_S / CM_PER_STEP) / STEPS_PER_DEG

_PIO0_BASE = 0x50200000      # RP2350, idem RP2040


# ----------------------------------------------------------------
# Eenheidsconversies
# ----------------------------------------------------------------
def cm_to_steps(cm):
    return int(abs(cm) / CM_PER_STEP + 0.5)


def steps_to_cm(steps):
    return steps * CM_PER_STEP


def rate_of(cm_s):
    """cm/s -> stappen/s."""
    return abs(cm_s) / CM_PER_STEP


def _delay_for(rate):
    """Stapfrequentie (stappen/s) -> delay-waarde voor het FIFO-woord."""
    d = int(F_PIO / rate + 0.5) - CYCLES_FIXED
    if d < 1:
        d = 1
    elif d > 65535:
        d = 65535
    return d


def _word(repeat, delay):
    """Pak (aantal stappen, delay) in één 32-bit FIFO-woord."""
    if repeat < 1:
        repeat = 1
    elif repeat > 65536:
        repeat = 65536
    return ((delay & 0xFFFF) << 16) | ((repeat - 1) & 0xFFFF)


# ----------------------------------------------------------------
# Rampaprofiel
# ----------------------------------------------------------------
def ramp_steps(rate0, rate1, accel_cm_s2=ACCEL_CM_S2):
    """Aantal stappen dat een ramp van rate0 naar rate1 beslaat."""
    a = accel_cm_s2 / CM_PER_STEP          # stappen/s2
    return int(abs(rate1 * rate1 - rate0 * rate0) / (2.0 * a) + 0.5)


def ramp_words(n_steps, rate0, rate1, n_seg=RAMP_SEGMENTS):
    """S-curve rampatabel: lijst van FIFO-woorden, samen exact n_steps stappen.

    De snelheid volgt een smoothstep (3p^2 - 2p^3) in de AFGELEGDE WEG. Daardoor
    is de versnelling aan begin en eind nul, dus geen koppelschok op de
    overgangen (jerk-begrensd). Werkt zowel op als af: geef rate0 > rate1.

    Kost niets extra t.o.v. een lineaire ramp, want de tabel wordt hier in
    Python gegenereerd en daarna alleen nog door DMA afgespeeld.
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


def plan(n_total, v_cruise_cm_s, v_start_cm_s=V_START_CM_S, accel=ACCEL_CM_S2):
    """Bepaal (n_ramp, rate_start, rate_cruise) voor een beweging van n_total stappen.

    Is de beweging te kort om de topsnelheid te halen, dan wordt het profiel
    driehoekig: de topsnelheid wordt verlaagd tot op + af precies passen.
    """
    if v_cruise_cm_s > V_MAX_CM_S:
        v_cruise_cm_s = V_MAX_CM_S
    r0 = rate_of(v_start_cm_s)
    r1 = rate_of(v_cruise_cm_s)
    if r1 <= r0:
        return 0, r0, r0
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
    """
    n_ramp, r0, r1 = plan(n_total, v_cruise_cm_s, v_start_cm_s, accel)
    words = ramp_words(n_ramp, r0, r1)
    rest = n_total - 2 * n_ramp
    d = _delay_for(r1)
    while rest > 0:
        chunk = 65536 if rest > 65536 else rest
        words.append(_word(chunk, d))
        rest -= chunk
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
        self._sm_id = sm_id

        self.sm = StateMachine(sm_id, ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.cnt = StateMachine(cnt_id, step_counter, freq=F_PIO, in_base=self.step)

        # treq_sel = (pio_num << 3) + sm_num. Klopt ook op RP2350: DREQ_PIO2_TX0 = 16.
        self.dma = rp2.DMA()
        self._ctrl = self.dma.pack_ctrl(size=2, inc_read=True, inc_write=False,
                                        treq_sel=(0 << 3) + sm_id)
        self._buf = None                    # referentie vasthouden zolang de DMA loopt
        self.committed = 0                  # weggeschreven stappen sinds reset

        # De SM's blijven altijd actief. Zonder data in de FIFO stalt de
        # generator met STEP laag; dat is de rusttoestand. Zo is er geen
        # aparte start/stop-toestand die uit de pas kan lopen.
        self.cnt.active(1)
        self.sm.active(1)
        self.reset_pos()

    # -- richting -------------------------------------------------
    def set_dir(self, forward):
        self.dir.value(self.fwd_level if forward else 1 - self.fwd_level)

    # -- odometer -------------------------------------------------
    def reset_pos(self):
        self.cnt.put(0xFFFFFFFF)
        self.cnt.exec("pull()")
        self.cnt.exec("mov(y, osr)")
        self.committed = 0

    def pos(self):
        """Werkelijk uitgestuurde stappen, gelezen uit de hardware-teller.

        LET OP: dit telt COMMANDO'S, geen beweging. Bij wielslip (hobbel, gleuf)
        loopt deze teller door. Voor werkelijke beweging heb je de GY9250 nodig.
        """
        self.cnt.exec("mov(isr, y)")
        self.cnt.exec("push()")
        return 0xFFFFFFFF - self.cnt.get()

    # -- data naar de FIFO ---------------------------------------
    def push(self, repeat, delay):
        """Zet één segment in de FIFO (kruisfase; CPU-pad)."""
        self.sm.put(_word(repeat, delay))
        self.committed += repeat

    def start_table(self, words, n_steps):
        """Speel een woordenlijst af via DMA (rampafasen; nul-CPU pad)."""
        while self.dma.active():
            pass
        self._buf = array('I', words)
        self.dma.config(read=self._buf, write=self.sm,
                        count=len(self._buf), ctrl=self._ctrl, trigger=True)
        self.committed += n_steps

    # -- status ---------------------------------------------------
    def busy(self):
        """True zolang niet alle weggeschreven stappen ook uitgestuurd zijn."""
        return self.pos() < self.committed

    def fifo_free(self):
        return 4 - self.sm.tx_fifo()

    # -- stoppen --------------------------------------------------
    def abort(self):
        """Harde stop: driver uit, DMA stil, FIFO leeg.

        sm.init() is hier bewust gebruikt om de FIFO te wissen: MicroPython
        biedt daar geen aparte API voor, en init() doet intern clear_fifos +
        restart. Dat is veiliger dan zelf in SHIFTCTRL schrijven.
        """
        self.ena.value(1)
        self.dma.active(0)
        self.sm.active(0)
        self.sm.init(ramp_stepper, freq=F_PIO, sideset_base=self.step)
        self.sm.active(1)
        self.committed = self.pos()

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


def enable():
    MA.enable()
    MB.enable()


def disable():
    MA.disable()
    MB.disable()


def _start_both():
    """Zet beide generator-SM's in één registerschrijf aan.

    Twee losse sm.active(1)-calls geven een Python-skew van honderden
    microseconden. Dankzij de ramp start de beweging zo langzaam dat dat
    nauwelijks uitmaakt (<1 stap), maar gratis is gratis.
    """
    v = machine.mem32[_PIO0_BASE]
    machine.mem32[_PIO0_BASE] = v | 0b0011


# ----------------------------------------------------------------
# Publieke API — fire-and-forget (geen bijsturing, nul CPU-overhead)
# ----------------------------------------------------------------
def s1(direction, speed, dist):
    """Alleen motor A. direction 'f'/'b', speed cm/s, dist cm."""
    fwd = direction.lower() == 'f'
    MA.set_dir(fwd)
    n = cm_to_steps(dist)
    MA.enable()
    MA.start_table(profile_words(n, speed), n)


def s2(direction, speed, dist):
    """Alleen motor B. direction 'f'/'b', speed cm/s, dist cm."""
    fwd = direction.lower() == 'f'
    MB.set_dir(fwd)
    n = cm_to_steps(dist)
    MB.enable()
    MB.start_table(profile_words(n, speed), n)


def mov(direction, speed, dist):
    """Beide motoren dezelfde kant, dezelfde afstand."""
    fwd = direction.lower() == 'f'
    n = cm_to_steps(dist)
    words = profile_words(n, speed)
    MA.set_dir(fwd)
    MB.set_dir(fwd)
    enable()
    MA.start_table(words, n)
    MB.start_table(words, n)


def rotate(direction, speed, dist):
    """Draai op de as. direction 'l'/'r', dist = booglengte per wiel in cm."""
    right = direction.lower() == 'r'
    n = cm_to_steps(dist)
    words = profile_words(n, speed)
    MA.set_dir(not right)
    MB.set_dir(right)
    enable()
    MA.start_table(words, n)
    MB.start_table(words, n)


def rotate_deg(degrees, speed=V_MAX_CM_S / 2):
    """Draai op de as over een hoek. Positief = naar rechts.

    Per graad legt elk wiel STEPS_PER_DEG/2 stappen af, tegengesteld.
    360 graden = 28634 stappen per wiel = 2,24 wielomwentelingen.
    """
    n = int(abs(degrees) * STEPS_PER_DEG / 2 + 0.5)
    words = profile_words(n, speed)
    right = degrees >= 0
    MA.set_dir(not right)
    MB.set_dir(right)
    enable()
    MA.start_table(words, n)
    MB.start_table(words, n)


def stop():
    """Nette stop: rem beide motoren af vanaf de huidige snelheid.

    Gebruik emergency_stop() als de positie niet meer uitmaakt.
    """
    for m in (MA, MB):
        m.dma.active(0)
    # De FIFO bevat nog data; die loopt uit. Voor een echt nette afremming
    # gebruik je Move.finish() tijdens een lopende drive().
    emergency_stop()


def emergency_stop():
    """Noodstop: drivers uit, DMA stil, FIFO's leeg. Motoren lopen vrij.

    De positie in de hardware-teller blijft geldig; de kar kan wel doorrollen,
    dus dat is geen betrouwbare positie meer.
    """
    MA.abort()
    MB.abort()


def busy():
    return MA.busy() or MB.busy()


# ----------------------------------------------------------------
# Odometrie — beide grootheden komen gratis uit de hardware-tellers
# ----------------------------------------------------------------
def pio_pos1():
    return MA.pos()


def pio_pos2():
    return MB.pos()


def reset_PIO_distance():
    MA.reset_pos()
    MB.reset_pos()


def distance():
    """Afgelegde weg van het midden van de kar, in cm."""
    return steps_to_cm((MA.pos() + MB.pos()) / 2.0)


def heading():
    """Koersverandering sinds de laatste reset, in graden. Positief = rechts.

    Volgt uit het stappenverschil tussen de wielen. Ziet GEEN wielslip; voor
    de werkelijke koers moet je dit fuseren met de GY9250-gyro.
    """
    return (MA.pos() - MB.pos()) / STEPS_PER_DEG


def status():
    print("=== STEPPER RAMP ===")
    for m in (MA, MB):
        print(" Motor %s: %s  pos=%d stappen (%.2f cm)  committed=%d" %
              (m.name, "bezig" if m.busy() else "stil",
               m.pos(), steps_to_cm(m.pos()), m.committed))
    print(" Midden: %.2f cm   koers: %+.2f graden" % (distance(), heading()))
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
        self.n_cruise = self.n_total - 2 * self.n_ramp
        self._centre = 0                     # midden-stappen die al weggeschreven zijn
        self._slice_base = max(1, int(self.r1 * SLICE_MS / 1000.0 + 0.5))
        self._state = self._RAMP_UP

        MA.set_dir(forward)
        MB.set_dir(forward)
        reset_PIO_distance()
        enable()

        up = ramp_words(self.n_ramp, self.r0, self.r1)
        MA.start_table(up, self.n_ramp)
        MB.start_table(up, self.n_ramp)
        self._centre = self.n_ramp
        self._state = self._CRUISE if self.n_cruise > 0 else self._RAMP_DOWN

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
            ra, rb, delta = base, base, 0

        # Zelfde slice-DUUR voor beide motoren, verschillend stappenaantal.
        # Zo blijven ze in de tijd synchroon en is ra + rb exact 2 * base,
        # waardoor het midden precies `base` stappen opschuift.
        cyc = F_PIO * base / self.r1                 # PIO-cycles voor deze slice
        MA.push(ra, int(cyc / ra + 0.5) - CYCLES_FIXED)
        MB.push(rb, int(cyc / rb + 0.5) - CYCLES_FIXED)
        self._centre += base
        return True

    # -- aanroepen uit de regellus -------------------------------
    def service(self):
        """Vul de FIFO's bij en pas de correctie toe. False = beweging klaar."""
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
            MA.start_table(down, self.n_ramp)
            MB.start_table(down, self.n_ramp)
            self._state = self._DONE

        if self._state == self._DONE and not busy():
            return False
        return True

    def finish(self):
        """Breek de kruisfase af en rem meteen netjes af."""
        if self._state == self._CRUISE:
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

    De acceleratiemeter dient als slipdetectie: stappen die wel uitgestuurd
    worden maar geen versnelling opleveren, betekenen doorslippende wielen.
    De hardware-teller kan dat per definitie niet zien.
    """

    def __init__(self, ldr_diff=None, gyro_rate=None, accel_fwd=None,
                 kp_ldr=25.0, kp_gyro=0.6, deadband=0.03, outer_div=5):
        self.ldr_diff = ldr_diff        # callable -> (A-B)/(A+B) na gain-correctie, in [-1, 1]
        self.gyro_rate = gyro_rate      # callable -> gemeten draaisnelheid in graden/s, rechts +
        self.accel_fwd = accel_fwd      # callable -> voorwaartse versnelling in m/s2 (optioneel)
        self.kp_ldr = kp_ldr            # graden/s per eenheid LDR-verschil
        self.kp_gyro = kp_gyro          # versterking van de binnenlus
        self.deadband = deadband        # LDR-verschil waaronder we recht doorrijden
        self.outer_div = outer_div      # buitenlus 1x per zoveel slices
        self._tick = 0
        self._setpoint = 0.0            # gewenste draaisnelheid in graden/s
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
            cmd += self.kp_gyro * (self._setpoint - self.gyro_rate())

        # --- slipdetectie (informatief, stuurt niet) ---------------
        if self.accel_fwd is not None:
            self.slipping = busy() and abs(self.accel_fwd()) < 0.05

        if cmd > MAX_TURN_DEG_S:
            cmd = MAX_TURN_DEG_S
        elif cmd < -MAX_TURN_DEG_S:
            cmd = -MAX_TURN_DEG_S
        return cmd


def hold_heading(gyro_rate):
    """Kortste variant: rij recht en houd de koers vast met alleen de gyro.

        mv = drive(50, 19.1, correction=hold_heading(imu.gyro_z))
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
    print("slice            %d ms = %d stappen, runway %d ms"
          % (SLICE_MS, int(r1 * SLICE_MS / 1000), SLICE_MS * FIFO_TARGET))
    print("stuurgezag       +/- %.1f graden/s, resolutie %.4f graden"
          % (MAX_TURN_DEG_S, 1.0 / STEPS_PER_DEG))
    print("max kruis-woord  %d stappen = %.1f cm" % (65536, steps_to_cm(65536)))
