# ================================================================
# Dual Stepper Controller MET ramp (PIO + DMA) en koerscorrectie — V2
# ================================================================
#
# V2 T.O.V. V1 (stepper_ramp.py) — WAT IS ER VERANDERD EN WAAROM
# ----------------------------------------------------------------
# Aanleiding: het idee om terug te gaan naar de oude stepper.py-stijl (één
# FIFO-woord per STAP i.p.v. per segment, met de target-tellende
# stepper_counter-PIO en zijn IRQ) om de kruisfase "gratis" te laten
# doorlopen op de laatst geladen delay (recycle-truc in stepper.py's
# stepper-PIO) en de opramp->kruisfase overgang hardware-matig te
# detecteren i.p.v. te pollen.
#
# Dat idee is BEWUST NIET één-op-één overgenomen, om twee redenen:
#
#   1) RAM. Eén woord per stap i.p.v. per segment is precies de "naïeve
#      aanpak" die V1 al afwees (zie de kop van stepper_ramp.py, §2): een
#      ramp van 2200 stappen wordt dan 8,8 kB i.p.v. 1 kB, x2 richtingen x2
#      motoren tientallen kB — op een RP2350 waar microdot+asyncio+lwIP het
#      geheugen al krap maken (zie globale_specificatie.md, "Geheugen").
#      Een hogere PIO-frequentie verandert daar niets aan: dat vergroot de
#      tijdresolutie per stap, niet het aantal fysieke stappen in de ramp.
#
#   2) Bijsturing verliest zijn garantie. `_push_slice()` in V1 rekent per
#      slice een EXACT stappenaantal per motor uit (repeat-veld), juist om
#      onafhankelijk te zijn van CPU-timing. Met "gewoon een nieuwe delay in
#      de FIFO zetten" (recycle-stijl) hangt het aantal stappen tijdens een
#      correctie-interval af van de WALL-CLOCK tijd tot de volgende push —
#      een GC-pauze geeft dan een echte koersfout in plaats van alleen een
#      latere correctie. Dat is exact het lek dat de repeat-tabel in V1
#      dichthoudt.
#
# Wat WEL is overgenomen — het bruikbare deel van het idee:
#
#   De opramp->kruisfase overgang (Move._RAMP_UP -> _CRUISE) hoeft niet op
#   de cadans van service() te wachten. rp2.DMA ondersteunt een
#   completion-IRQ (DMA.irq()); die gebruiken we om de state-overgang en het
#   eerste keer bijvullen van de FIFO's METEEN te doen zodra de opramp-DMA
#   stopt met schrijven, i.p.v. pas bij de eerstvolgende service()-aanroep
#   (tot SLICE_MS/2 = 10 ms later in V1). Dat is dezelfde soort winst als de
#   target-IRQ in stepper.py: een hardware-event i.p.v. een pollende lus.
#
#   VEILIGHEIDSNET: als rp2.DMA().irq() niet bestaat op de gebruikte
#   firmware, of de IRQ (nog) niet gevuurd heeft tegen de tijd dat
#   service() alsnog langskomt, valt service() terug op precies dezelfde
#   dma.active()-polling als V1. Functioneel is V2 dus nooit slechter dan
#   V1; in het beste geval reageert hij sneller.
#
# TE VERIFIEREN OP HARDWARE (dit is waar de stall-testjes voor bedoeld zijn):
#   - bestaat rp2.DMA.irq() op deze firmwareversie? (self._irq_supported)
#   - is de schedule-latentie van de callback inderdaad << SLICE_MS?
#   - kan BRIDGE_SLICES nu omlaag (of zelfs naar 0), nu de overgang niet
#     meer aan de service()-cadans hangt? Dat was de oorspronkelijke reden
#     voor het brugsegment (zie stepper_ramp.py, §7) — met een snellere
#     overgang is de marge die het brugsegment moet overbruggen kleiner.
#
# Voor de rest is dit bestand functioneel identiek aan stepper_ramp.py; zie
# daar (en stepper_ramp_NL.md) voor de volledige uitleg van PIO, DMA,
# odometrie, koerscorrectie en de rest van de API. Alleen de secties die
# hierboven genoemd zijn, zijn aangepast — gemarkeerd met "NIEUW IN V2".
#
# LET OP — dit is een VERVANGER van stepper.py EN van stepper_ramp.py, geen
# aanvulling. Alle drie claimen PIO0 SM0..SM3 en dezelfde GPIO's. Importeer
# er altijd maar één.
# ================================================================

from array import array
from math import sqrt
import rp2
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio


# ----------------------------------------------------------------
# PIO: stapgenerator met (repeat, delay)-woorden — ongewijzigd t.o.v. V1
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
# PIO: stappenteller (hardware-odometer, geen CPU) — ongewijzigd t.o.v. V1
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
# Constanten — ongewijzigd t.o.v. V1
# ----------------------------------------------------------------
F_PIO         = 15_000_000   # 150 MHz sysclk / 10 -> integer klokdeler, geen fractionele jitter
CYCLES_FIXED  = 5            # vaste cycles PER STAP binnen een segment:
                             #   mov(x, isr)[2]      = 3
                             #   jmp(x_dec, "wait")  = delay + 1  (bij x == 0 wordt de
                             #                         instructie nog één keer uitgevoerd
                             #                         zonder te springen)
                             #   jmp(y_dec, "pulse") = 1
                             #   -> stapperiode = delay + 5 cycles
                             # NIET meegerekend: 4 cycles PER SEGMENT voor pull/out/out/mov.
                             # De effectieve overhead is dus 5 + 4/repeat. Bij de 256
                             # rampsegmenten (~22 stappen) is dat 0,18 cycle op ~1000
                             # (0,02%); bij segmenten van 1 stap 0,4%.
                             # BEREKEND, nog niet gemeten. TE VERIFIEREN met een logic
                             # analyzer op zowel lange segmenten als segmenten van
                             # 1, 2, 8 en 256 stappen (zie meet_frequentie()).

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
                             #
                             # DIT is de knop voor de stall-testjes: hoger ACCEL_CM_S2
                             # (of een hogere V_MAX_CM_S) tot de motor synchronisme
                             # verliest, dan weer een stap terug voor marge.

# Ramp- en sliceparameters
RAMP_SEGMENTS = 256          # snelheidssprong <1,8% per segment over een 10:1 bereik
SLICE_MS      = 20           # duur van één kruis-slice; bepaalt de regellatentie
FIFO_TARGET   = 3            # aantal slices dat vooruit in de FIFO staat (runway = 60 ms)
FIFO_DEPTH    = 4            # TX-FIFO diepte zonder fifo_join
BRIDGE_SLICES = 2            # brugsegment aan het eind van de opramp (zie stepper_ramp.py §7).
                             # NIEUW IN V2: met de DMA-IRQ hieronder hoeft deze marge
                             # niet meer de volledige service()-cadans (10 ms) te dekken,
                             # alleen nog de schedule-latentie van de IRQ-callback. Op
                             # hardware verifiëren of dit omlaag kan (bijv. naar 1).
MAX_DIFF_FRAC = 0.20         # maximale snelheidsdifferentie per wiel (+/- 20%)


# ----------------------------------------------------------------
# Tekenconventies — PUBLIEK GELDT ALTIJD: POSITIEF = NAAR RECHTS
# ----------------------------------------------------------------
# Drie ONAFHANKELIJKE hardwaregrenzen, elk met een eigen teken. Ze corrigeren
# verschillende fouten en horen daarom niet in één constante: een omgekeerd
# gemonteerde GY9250 vraagt een ander gyroteken zonder dat er iets mankeert aan
# de motorbekabeling of aan rotate_deg(). Zet de tekens ALLEEN hier, aan de
# hardwaregrens; de rest van de code rekent in de publieke conventie.
#
# MOTOR_TURN_SIGN  +1 = motor A is het RECHTER wiel, motor B het linker.
#                  Raakt rotate(), rotate_deg(), heading() en status() en de
#                  bijsturing in Move.
#                  Verifieren: rotate_deg(+90) moet de kar naar RECHTS draaien
#                  en heading() moet daarna ongeveer +90 geven.
# GYRO_Z_SIGN      +1 = gyro-Z is positief bij een draai naar rechts.
#                  Verifieren met opgeheven wielen: draai de kar met de hand
#                  naar rechts, gyro_z_deg_s(sensor)() moet positief zijn.
# LDR_DIFF_SIGN    +1 = een positief LDR-verschil betekent dat de bron RECHTS
#                  ligt. Verifieren: houd een lamp rechts, ldr_diff() > 0.
MOTOR_TURN_SIGN = +1
GYRO_Z_SIGN     = +1
LDR_DIFF_SIGN   = +1


# ----------------------------------------------------------------
# Eenheidsconversies en validatie — ongewijzigd t.o.v. V1
# ----------------------------------------------------------------
def cm_to_steps(cm):
    return int(abs(cm) / CM_PER_STEP + 0.5)


def steps_to_cm(steps):
    return steps * CM_PER_STEP


def rate_of(cm_s):
    """cm/s -> stappen/s."""
    return abs(cm_s) / CM_PER_STEP


def turn_authority_deg_s(rate):
    """Maximale draaisnelheid (graden/s) bij een kruissnelheid van `rate` stappen/s.

    Het stuurgezag is een FRACTIE van de rijsnelheid (MAX_DIFF_FRAC), dus het
    schaalt mee: bij halve snelheid is er half zoveel gezag. Altijd aftoppen op
    de waarde bij V_MAX zou de regelaar bij lage snelheid een setpoint laten
    commanderen dat de wielen niet kunnen leveren — en dat verschil zou daarna
    ten onrechte als koersvolgfout worden gemeld.
    """
    return 2.0 * MAX_DIFF_FRAC * rate / STEPS_PER_DEG


# Maximaal stuurgezag, bij topsnelheid. Bij lagere snelheid is het evenredig
# minder; gebruik dan turn_authority_deg_s(rate).
MAX_TURN_DEG_S = turn_authority_deg_s(rate_of(V_MAX_CM_S))


def _is_number(value):
    """True voor een echt getal. bool telt niet mee: True als afstand is onzin."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_dir(value, valid, name):
    """Weiger een ongeldige richting in plaats van hem stil als 'anders' te lezen."""
    if not isinstance(value, str) or value.lower() not in valid:
        raise ValueError("%s moet een van %r zijn, kreeg %r" % (name, valid, value))
    return value.lower()


def _check_pos(value, name):
    """Weiger nul, negatief, niet-eindig en niet-numeriek.

    De typecontrole staat vooraan: zonder die controle zou een string een
    TypeError geven in plaats van de beloofde ValueError.
    """
    if not _is_number(value) or not (value > 0) or value - value != 0:
        raise ValueError("%s moet een positief eindig getal zijn, kreeg %r" % (name, value))
    return value


def _check_dist(value, name="dist"):
    """Afstanden zijn ONGETEKEND: de richting staat in een aparte parameter.

    cm_to_steps() neemt de absolute waarde, dus zonder deze controle zou
    mov('f', 10, -5) stil 5 cm VOORUIT rijden terwijl de aanroeper duidelijk
    iets anders bedoelde. Nul is toegestaan en levert een no-op.

    creep() is de uitzondering: die neemt bewust een SIGNED afstand, omdat het
    teken daar de correctierichting is.
    """
    if not _is_number(value) or value < 0 or value - value != 0:
        raise ValueError("%s moet 0 of een positief eindig getal zijn, kreeg %r"
                         % (name, value))
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
# Rampaprofiel — ongewijzigd t.o.v. V1
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

    Dit is de nul-CPU variant: de hele beweging in één DMA-transfer. Er is GEEN
    interrupt aan het eind — deze module configureert daar nergens een DMA- of
    PIO-IRQ voor. Tijdens de uitvoering doet de CPU niets; voltooiing wordt
    gepolld met busy(). Verder identiek aan het oude fire-and-forget concept,
    maar mét ramp. Geeft een lege lijst bij n_total <= 0.
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

        # NIEUW IN V2: DMA-completion IRQ. `_ramp_done_cb` is een door Move
        # geregistreerde eenmalige callback (zie Move._arm_ramp_irq); zonder
        # registratie (o.a. tijdens _launch()'s fire-and-forget pad) doet
        # _dma_irq niets. Ontbreekt rp2.DMA.irq() op deze firmware, dan blijft
        # _irq_supported False en valt Move.service() terug op pure polling —
        # functioneel identiek aan V1.
        self._ramp_done_cb = None
        self._irq_supported = True
        try:
            self.dma.irq(handler=self._dma_irq, hard=False)
        except AttributeError:
            self._irq_supported = False

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

    # -- DMA-completion IRQ (NIEUW IN V2) --------------------------
    def _dma_irq(self, *_args):
        """Loopt via micropython.schedule (hard=False), dus gewoon Python-context.

        Consumeert `_ramp_done_cb` eenmalig; is hij None (het gebruikelijke
        geval, of na afhandeling), dan gebeurt er niets. Twee motoren kunnen
        allebei dezelfde Move-callback geregistreerd hebben (zie
        _arm_ramp_irq); dat is bewust, want de callback zelf checkt of
        BEIDE motor-DMA's al gestopt zijn met schrijven voordat hij iets doet.
        """
        cb = self._ramp_done_cb
        if cb is not None:
            self._ramp_done_cb = None
            cb()

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
        loopt deze teller door. De GY9250 ziet daar alleen de ROTATIE-component
        van; werkelijke lineaire verplaatsing vraagt een externe referentie
        (zie de kop, "WAT DE ODOMETER WEL EN NIET IS").
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
        """Hang een woordenlijst ACHTER wat er al in de FIFO staat, via DMA.

        Dit is het TOEVOEG-pad (rampafasen; nul-CPU). Het WEIGERT zolang er nog
        een transfer loopt: dan zou een tweede DMA in dezelfde FIFO schrijven,
        en zou `committed` pulsen tellen die door het afkappen nooit komen —
        waarna busy() eeuwig True blijft. Gebruik replace_table() om een lopende
        beweging te vervangen.

        Doet ook niets bij een lege lijst: een DMA-transfer met count=0 is
        firmware-afhankelijk gedrag en moeten we niet opzoeken.
        """
        if not words or n_steps <= 0:
            return False
        if self.dma.active():
            return False
        self._buf = array('I', words)
        self.dma.config(read=self._buf, write=self.sm,
                        count=len(self._buf), ctrl=self._ctrl, trigger=True)
        self.committed += n_steps
        return True

    def replace_table(self, words, n_steps):
        """Vervang een lopende beweging door een nieuwe tabel.

        Transactioneel: DMA stil, FIFO leeg, signed odometrie bijgewerkt en
        `committed` terug op de WERKELIJKE pulsstand — pas daarna de nieuwe
        tabel. Zonder dat laatste blijven de afgekapte, wel al meegetelde
        pulsen voor altijd in `committed` staan.
        """
        self._clear()
        return self.start_table(words, n_steps)

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

        NIEUW IN V2: ook `_ramp_done_cb` wordt hier gewist. Zonder dat zou een
        callback van een AFGEBROKEN Move nog kunnen vuren op de eerstvolgende,
        totaal andere DMA-transfer (bijv. van een nieuw fire-and-forget
        commando) — de state-guard in _on_ramp_up_done vangt dat weliswaar af
        (die checkt self._state van de OUDE Move), maar opruimen is goedkoper
        en explicieter dan op die guard vertrouwen.
        """
        self._ramp_done_cb = None
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
        m.enable()          # alleen de driver(s) die ook echt gaan draaien; de
                            # globale enable() zou s1() ook motor B laten trekken
    for m, _ in dirs:
        m.note_plan(n_ramp, r0, r1, n)
        m.start_table(words, n)
    return True


def _turn_dirs(right):
    """(motor, forward)-paren voor een draai op de as. right=True is naar rechts.

    Bij MOTOR_TURN_SIGN = +1 is motor A het rechter wiel; dat loopt bij een
    draai naar rechts dus ACHTERUIT. Alle draaifuncties gaan hier doorheen,
    zodat rotate(), rotate_deg() en heading() niet uit elkaar kunnen lopen.
    """
    a_forward = (not right) if MOTOR_TURN_SIGN > 0 else right
    return ((MA, a_forward), (MB, not a_forward))


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


def rotate_deg(degrees, speed=V_MAX_CM_S / 2):
    """Draai op de as over een hoek. Positief = naar rechts (zie MOTOR_TURN_SIGN).

    Per graad legt elk wiel STEPS_PER_DEG/2 stappen af, tegengesteld.
    360 graden = 28632 stappen per wiel = 2,24 wielomwentelingen.

    Na afloop moet heading() ongeveer `degrees` teruggeven — hetzelfde teken.
    Klopt dat niet, dan staat MOTOR_TURN_SIGN verkeerd.
    """
    if not _is_number(degrees) or degrees - degrees != 0:
        raise ValueError("degrees moet een eindig getal zijn, kreeg %r" % (degrees,))
    n = int(abs(degrees) * STEPS_PER_DEG / 2 + 0.5)
    return _launch(_turn_dirs(degrees >= 0), _check_pos(speed, "speed"), n)


# ----------------------------------------------------------------
# Eindpositionering — het laatste correctiemoment
# ----------------------------------------------------------------
def creep(dist_cm, speed_cm_s=2.0):
    """Kruipcorrectie: kleine, exacte verplaatsing. Positief = vooruit.

    Dit is de enige bewegingsfunctie met een SIGNED afstand: het teken is hier
    de correctierichting, niet een vergissing.

    Bij 2 cm/s is er praktisch geen ramp nodig (dat ligt al boven de veilige
    startsnelheid van 1,91 cm/s), dus de beweging is meteen exact en zacht.
    Resolutie is 14,9 um; de meting is dus altijd de beperkende factor.
    """
    if not _is_number(dist_cm) or dist_cm - dist_cm != 0:
        raise ValueError("dist_cm moet een eindig getal zijn, kreeg %r" % (dist_cm,))
    if abs(dist_cm) < CM_PER_STEP:
        return False
    return mov('f' if dist_cm > 0 else 'b', speed_cm_s, abs(dist_cm))


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


def stopping_distance_cm(speed_cm_s=V_MAX_CM_S, accel=ACCEL_CM_S2):
    """Afstand die de kar nog aflegt nadat je Move.finish() aanroept.

    Som van twee posten die je niet meer kunt intrekken:
      - de afremramp vanaf `speed` naar de startsnelheid;
      - de kruis-slices die al in de FIFO's staan (FIFO_TARGET * SLICE_MS).

    Gebruik dit om het afremmen vooruit te plannen; roep je finish() pas aan
    OP de doelafstand, dan schiet je er met deze afstand voorbij.
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
    """Zet beide odometers op nul.

    ALLEEN STILSTAAND AANROEPEN. De hardwareteller wordt hier teruggezet zonder
    de DMA, de FIFO of `committed` mee te nemen; doe je dit tijdens een
    beweging, dan telt busy() op een teller die net op nul is gezet en loopt de
    signed positie mis. Elk bewegingscommando doet zelf al een halt() vooraf,
    dus in normaal gebruik is dit niet nodig.
    """
    if busy():
        raise RuntimeError("reset_PIO_distance() tijdens een beweging; "
                           "roep eerst halt() of brake() aan")
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

    Bij MOTOR_TURN_SIGN = +1 is A het RECHTER wiel: naar rechts draaien betekent
    dat B (links) verder loopt, dus B - A > 0. Hetzelfde teken als rotate_deg()
    en als de correctie in Move.
    """
    return MOTOR_TURN_SIGN * (MB.travel() - MA.travel()) / STEPS_PER_DEG


def status():
    """Print de toestand. Elke teller wordt één keer gelezen, zodat de regels
    onderling consistent zijn."""
    print("=== STEPPER RAMP V2 ===")
    snap = []
    for m in _MOTORS:
        p = m.pulses()
        snap.append((m, p, m.travel()))
    for m, p, t in snap:
        print(" Motor %s: %-5s pulsen=%d  positie=%+d (%+.2f cm)  committed=%d  irq=%s" %
              (m.name, "bezig" if p < m.committed else "stil",
               p, t, steps_to_cm(t), m.committed, m._irq_supported))
    mid = steps_to_cm((snap[0][2] + snap[1][2]) / 2.0)
    hdg = MOTOR_TURN_SIGN * (snap[1][2] - snap[0][2]) / STEPS_PER_DEG
    print(" Midden: %+.2f cm   koers: %+.2f graden" % (mid, hdg))
    print(" Max stuurgezag: %.1f graden/s" % MAX_TURN_DEG_S)
    print("========================")


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

    `correction` is een callable die de gewenste draaisnelheid in graden/s geeft
    (positief = rechts), of een HeadingController. Geef je de controller zelf,
    dan wordt zijn stuurgezag op de WERKELIJKE rijsnelheid gezet en krijgt hij
    terugkoppeling over wat er daadwerkelijk is weggeschreven.

    NIEUW IN V2: de overgang van opramp naar kruisfase wordt niet meer alleen
    gepolld in service(), maar ook (opportunistisch) aangestoten door een
    DMA-completion IRQ — zie _arm_ramp_irq()/_on_ramp_up_done() en de kop van
    dit bestand. Werkt de IRQ niet (oude firmware, of nog niet gevuurd), dan
    doet service() precies wat V1 deed.
    """

    _RAMP_UP, _CRUISE, _RAMP_DOWN, _DONE = 0, 1, 2, 3

    def __init__(self, dist_cm, speed_cm_s, correction=None,
                 v_start=V_START_CM_S, accel=ACCEL_CM_S2, forward=True):
        _check_dist(dist_cm, "dist_cm")
        self._accel = accel
        self._controller = correction if isinstance(correction, HeadingController) else None
        # callable -> gewenste draaisnelheid in graden/s
        self.correction = self._controller.output if self._controller else correction
        self.n_total = cm_to_steps(dist_cm)
        self.n_ramp, self.r0, self.r1 = plan(self.n_total, speed_cm_s, v_start, accel)
        self._centre = 0                     # midden-stappen die al weggeschreven zijn
        self._slice_base = max(1, int(self.r1 * SLICE_MS / 1000.0 + 0.5))

        if self._controller is not None:
            # Het stuurgezag schaalt met de RIJSNELHEID, niet met V_MAX.
            self._controller.authority = turn_authority_deg_s(self.r1)

        if self.n_total <= 0:
            self._state = self._DONE
            return

        # Brugsegment: houdt de FIFO gevuld over de DMA -> CPU overgang heen.
        n_cruise = self.n_total - 2 * self.n_ramp
        bridge = BRIDGE_SLICES * self._slice_base
        if bridge > n_cruise:
            bridge = n_cruise

        # Eerst een eventueel lopende beweging transactioneel stilzetten. Zonder
        # deze halt() zou DIR wisselen terwijl er nog oude stappen in de FIFO
        # staan, zouden de tellers midden in een beweging op nul gaan, en zouden
        # oude en nieuwe profielwoorden achter elkaar worden uitgevoerd.
        for m in _MOTORS:
            m.halt()
            m.set_dir(forward)
            m.enable()
        reset_PIO_distance()

        up = ramp_words(self.n_ramp, self.r0, self.r1)
        up.extend(cruise_words(bridge, self.r1))
        n_up = self.n_ramp + bridge
        for m in _MOTORS:
            m.note_plan(self.n_ramp, self.r0, self.r1, self.n_total)
            m.start_table(up, n_up)
        self._centre = n_up
        self._state = self._RAMP_UP
        self._arm_ramp_irq()

    # -- opramp -> kruisfase overgang (NIEUW IN V2) ----------------
    def _arm_ramp_irq(self):
        """Registreer de DMA-completion callback op beide motoren.

        Beide motoren krijgen dezelfde bound method; welke van de twee als
        eerste zijn DMA ziet stoppen maakt niet uit, want _on_ramp_up_done
        checkt zelf of ALLEBEI al klaar zijn (net als V1's polling-conditie
        `MA.dma.active() or MB.dma.active()`). Bestaat rp2.DMA.irq() niet op
        deze firmware, dan is _irq_supported False en is dit een no-op; de
        polling-fallback in service() dekt dat geval.
        """
        for m in _MOTORS:
            if m._irq_supported:
                m._ramp_done_cb = self._on_ramp_up_done

    def _on_ramp_up_done(self, *_args):
        """Zet de state om zodra BEIDE motor-DMA's gestopt zijn met schrijven,
        en vult meteen de FIFO's bij via _advance(). Wordt aangeroepen vanuit
        de DMA-IRQ (async, potentieel twee keer — zie _arm_ramp_irq) en als
        fallback vanuit service(). Idempotent: doet niets als de state al
        voorbij _RAMP_UP is.
        """
        if self._state != self._RAMP_UP:
            return
        if MA.dma.active() or MB.dma.active():
            return
        self._state = self._CRUISE
        self._advance()

    def _advance(self):
        """Kruisfase bijvullen en zo nodig doorschakelen naar rampaf/klaar.

        Bevat geen _RAMP_UP-logica; dat zit uitsluitend in _on_ramp_up_done,
        zodat deze methode veilig meermaals aangeroepen kan worden (vanuit de
        IRQ en vanuit service()) zonder iets dubbel te doen — de state is na
        de eerste keer al voorbij het punt waar de volgende aanroep nog iets
        zou veranderen.
        """
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

    # -- interne helpers -----------------------------------------
    def _push_slice(self):
        """Schrijf één kruis-slice weg, met de actuele koerscorrectie erin."""
        remaining = self.n_total - self.n_ramp - self._centre
        base = self._slice_base if self._slice_base < remaining else remaining
        if base < 1:
            return False

        # Gewenste draaisnelheid -> stappenverschil over deze slice.
        # `delta` is het aantal stappen dat het LINKER wiel extra krijgt:
        # naar rechts draaien betekent dat links verder loopt.
        delta = 0
        if self.correction is not None:
            t = base / self.r1                       # duur van deze slice in s
            omega = self.correction()                # graden/s, positief = rechts
            d = omega * STEPS_PER_DEG * t / 2.0
            delta = int(d + (0.5 if d >= 0 else -0.5))
            lim = int(base * MAX_DIFF_FRAC)
            if delta > lim:
                delta = lim
            elif delta < -lim:
                delta = -lim
            if self._controller is not None and t > 0:
                # Meld terug wat er WERKELIJK is weggeschreven, na afronden en
                # aftoppen. Zonder deze terugkoppeling zou de koersvolgfout
                # worden gemeten tegen een setpoint dat de wielen nooit hebben
                # uitgevoerd, en dat leest als een storing die er niet is.
                self._controller.note_applied(2.0 * delta / (STEPS_PER_DEG * t))

        # Bij MOTOR_TURN_SIGN = +1 is A het rechter wiel en B het linker.
        ra = base - MOTOR_TURN_SIGN * delta
        rb = base + MOTOR_TURN_SIGN * delta
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
        """Vul de FIFO's bij en pas de correctie toe. False = beweging klaar.

        NIEUW IN V2: in de _RAMP_UP-state is dit nu de FALLBACK, niet het
        primaire pad — de DMA-IRQ heeft de overgang normaliter al gedaan
        (_on_ramp_up_done, aangeroepen als de FIFO's al netjes bijgevuld
        zijn). Roept _on_ramp_up_done() hier alsnog aan; is de state daarna
        nog steeds _RAMP_UP, dan schrijft de DMA écht nog.
        """
        if self._state == self._RAMP_UP:
            self._on_ramp_up_done()
            if self._state == self._RAMP_UP:
                return True
        else:
            self._advance()

        if self._state == self._DONE and not busy():
            return False
        return True

    def _brake_from_ramp_up(self):
        """Afbreken terwijl de opramp-DMA nog schrijft.

        Dit kan NIET met de gewone afremramp erachteraan:
          - `committed` is bij de start al met de HELE opramp verhoogd, maar de
            DMA heeft nog lang niet alles naar de FIFO geschreven. Kap je de
            transfer zomaar af, dan wacht busy() eeuwig op pulsen die niet komen;
          - de afremtabel begint op r1 terwijl de motor nog ergens tussen r0 en
            r1 zit, dus dat zou eerst een sprong OMHOOG geven — precies de
            situatie waar de ramp voor is.

        Daarom: eerst de werkelijke snelheid schatten, dan transactioneel wissen
        (DMA stil, FIFO leeg, committed terug op de echte pulsstand) en pas
        daarna een verse afremramp vanaf díe snelheid.
        """
        rate = MA.current_rate()
        if rate is None:
            rate = self.r1
        # De schatting loopt iets achter op de werkelijkheid; te laag is de
        # veilige kant (kleine snelheidsdaling i.p.v. een sprong omhoog).
        if rate < self.r0:
            rate = self.r0
        n = ramp_steps(rate, self.r0, self._accel)
        down = ramp_words(n, rate, self.r0) if n > 0 else []
        for m in _MOTORS:
            m.halt()                        # _clear(): DMA, FIFO, committed en _ramp_done_cb
            if down:
                m.start_table(down, n)
        self.n_total = self._centre = MA.committed

    def finish(self):
        """Breek de beweging af en rem meteen netjes af."""
        if self._state == self._RAMP_UP:
            self._brake_from_ramp_up()
            self._state = self._DONE
        elif self._state == self._CRUISE:
            self._state = self._RAMP_DOWN
            self.n_total = self._centre + self.n_ramp
        return self.service()


def drive(dist_cm, speed_cm_s=V_MAX_CM_S, correction=None, **kw):
    """Start een beweging met bijsturing. Geeft een Move terug.

    Zonder `correction` is `mov()` beter: dan loopt alles in één DMA-transfer.

    Geef bij voorkeur de HeadingController zelf mee in plaats van `hc.output`:
    dan zet Move() zijn stuurgezag op de werkelijke rijsnelheid en krijgt hij
    terugkoppeling over wat er is weggeschreven.

        mv = drive(50, 19.1, correction=hc)
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
# GY9250-koppeling — ongewijzigd t.o.v. V1
# ----------------------------------------------------------------
def gyro_z_deg_s(sensor, sign=None):
    """Maak van een MPU6500/MPU9250-object een callable die GRADEN/s teruggeeft.

    Past GYRO_Z_SIGN toe, zodat de publieke conventie "positief = rechts" ook
    geldt als de sensor omgekeerd gemonteerd is. Geef `sign` mee om die
    constante voor deze ene sensor te overrulen.

    De driver in lib/GY9250 heeft `gyro_sf=SF_RAD_S` als default en levert dus
    RADIALEN/s, terwijl HeadingController in graden/s rekent. Rechtstreeks
    `sensor.gyro[2]` doorgeven maakt de gyrocorrectie 57,3x te klein.

    Draait de sensor op core 1, geef dan geen sensor-object mee maar een eigen
    callable die de door core 1 gepubliceerde waarde leest. Lees dezelfde
    I2C-sensor niet vanaf twee cores.

        hc = HeadingController(ldr_diff=..., gyro_rate=gyro_z_deg_s(sensor))
    """
    s = GYRO_Z_SIGN if sign is None else sign

    def _read():
        return s * sensor.gyro[2] * RAD_TO_DEG
    return _read


# ----------------------------------------------------------------
# Koersregelaar: LDR (richting) + gyro (storingsonderdrukking) — ongewijzigd
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
                 track_err_deg_s=8.0, track_err_ticks=10):
        self.ldr_diff = ldr_diff        # callable -> (A-B)/(A+B) na gain-correctie, in [-1, 1]
        self.gyro_rate = gyro_rate      # callable -> gemeten draaisnelheid in GRADEN/s, rechts +
        self.kp_ldr = kp_ldr            # graden/s per eenheid LDR-verschil
        self.kp_gyro = kp_gyro          # versterking van de binnenlus
        self.deadband = deadband        # LDR-verschil waaronder we recht doorrijden
        self.outer_div = max(1, int(outer_div))
        # Stuurgezag in graden/s. Move() zet dit op turn_authority_deg_s(r1), dus
        # op de WERKELIJKE rijsnelheid; de default hoort bij topsnelheid.
        self.authority = MAX_TURN_DEG_S
        self.track_err_deg_s = track_err_deg_s   # drempel voor de koersvolgfout
        self.track_err_ticks = max(1, int(track_err_ticks))
        self._tick = 0
        self._setpoint = 0.0            # gewenste draaisnelheid in graden/s
        self._last_cmd = 0.0            # wat er vorige tick WERKELIJK is weggeschreven
        self._err_run = 0
        self.yaw_tracking_error = False

    def output(self):
        """Geef de te commanderen draaisnelheid in graden/s. Positief = rechts.

        Dit is de callable die je aan drive(correction=...) meegeeft.
        """
        # --- buitenlus: LDR bepaalt het setpoint --------------------
        if self.ldr_diff is not None and self._tick % self.outer_div == 0:
            d = LDR_DIFF_SIGN * self.ldr_diff()
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

            # KOERSVOLGFOUT, geen slipbewijs. Wijkt de GEMETEN draaisnelheid
            # aanhoudend af van wat er WERKELIJK is weggeschreven (zie
            # note_applied), dan klopt er iets niet — maar wat, staat hiermee
            # niet vast: regeldynamiek, gyrovertraging, verzadiging en een
            # verkeerde gain geven precies hetzelfde beeld als een slippend
            # wiel. Lineaire slip (beide wielen recht vooruit) is hiermee
            # sowieso niet te zien; dat vraagt een externe positiereferentie.
            # Alleen "motor actief + weinig versnelling" werkt ook niet: bij
            # constante snelheid is de voorwaartse versnelling immers nul.
            if abs(self._last_cmd - measured) > self.track_err_deg_s:
                self._err_run += 1
            else:
                self._err_run = 0
            self.yaw_tracking_error = self._err_run >= self.track_err_ticks

        # Aftoppen op het gezag dat er bij DEZE rijsnelheid werkelijk is.
        lim = self.authority
        if cmd > lim:
            cmd = lim
        elif cmd < -lim:
            cmd = -lim
        self._last_cmd = cmd
        return cmd

    def note_applied(self, omega_deg_s):
        """Vertel de regelaar welke draaisnelheid er WERKELIJK is weggeschreven.

        Move() roept dit elke slice aan, na het afronden en aftoppen van het
        stappenverschil. Zonder deze terugkoppeling vergelijkt de koersvolgfout
        de gyro met een setpoint dat de wielen nooit hebben uitgevoerd, en meldt
        hij bij lage snelheid structureel vals alarm.
        """
        self._last_cmd = omega_deg_s


def damp_yaw_rate(gyro_rate):
    """Demp de draaisnelheid met alleen de gyro. HOUDT GEEN KOERS VAST.

    Heette hold_heading(), maar dat beloofde te veel. Het setpoint is hier
    altijd nul, dus de uitgang is simpelweg -kp_gyro * gemeten draaisnelheid:
    een externe verdraaiing wordt tegengewerkt ZOLANG die plaatsvindt, maar de
    hoekfout die overblijft wordt daarna niet teruggedraaid. Gyrobias geeft
    bovendien blijvende koersdrift.

    Echt koershouden vraagt integratie van gyro-Z tot een relatieve hoek en het
    wegregelen van díe hoekfout — dat zit hier NIET in. Voor langere trajecten
    heb je daarnaast een absolute referentie nodig (de LDR-buitenlus, of het
    kompas op stilstand).

        mv = drive(50, 19.1, correction=damp_yaw_rate(gyro_z_deg_s(sensor)))
    """
    return HeadingController(ldr_diff=None, gyro_rate=gyro_rate)


# ----------------------------------------------------------------
# Zelftest / referentiegetallen — ongewijzigd t.o.v. V1
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
    print("stuurgezag       +/- %.1f graden/s bij %.1f cm/s, +/- %.1f bij 5 cm/s"
          % (MAX_TURN_DEG_S, V_MAX_CM_S, turn_authority_deg_s(rate_of(5.0))))
    print("                 resolutie %.4f graden" % (1.0 / STEPS_PER_DEG))
    print("max kruis-woord  %d stappen = %.1f cm" % (65536, steps_to_cm(65536)))
    print("DMA-IRQ          motor A: %s, motor B: %s (V2)"
          % (MA._irq_supported, MB._irq_supported))


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


def stall_test(accel_cm_s2, speed_cm_s=V_MAX_CM_S, dist_cm=60.0, v_start=V_START_CM_S):
    """Rijdt één rechte lijn met een gegeven versnelling en rapporteert of het
    gecommandeerde stappentotaal ook echt is uitgekomen.

    Bedoeld voor het handmatig zoeken naar de stall-grens: begin met de huidige
    ACCEL_CM_S2 en verhoog stapsgewijs. Zolang de motor synchroon blijft, komt
    "afwijking" overeen met de PIO-tellerresolutie (~0); loopt de rotor uit de
    pas, dan telt de PIO-teller nog steeds elke pulsFLANK die de PIO zelf
    genereert -- een stall is dus GEEN afwijking in dit getal, want de PIO
    weet niet dat de rotor niet meedraait.

    GEBRUIK DAAROM ALTIJD EEN ONAFHANKELIJKE REFERENTIE erbij (zie
    stepper_ramp_NL.md, "Nog te verifiëren op hardware"):
      - kijk/luister of de motor stil blijft staan of zoemt i.p.v. draait;
      - of leg de GY9250 gyro-Z ernaast als de kar recht vooruit rijdt.

    Deze functie meet dus vooral TIMING (haalt de beweging de verwachte duur,
    en werkt de opramp-IRQ), niet de mechanische stall zelf.
    """
    import time
    n = cm_to_steps(dist_cm)
    p0_a, p0_b = MA.pulses(), MB.pulses()
    t0 = time.ticks_ms()
    mv = Move(dist_cm, speed_cm_s, v_start=v_start, accel=accel_cm_s2)
    ticks = 0
    while mv.service():
        time.sleep_ms(SLICE_MS // 2)
        ticks += 1
    dt = time.ticks_diff(time.ticks_ms(), t0)
    da = MA.pulses() - p0_a
    db = MB.pulses() - p0_b
    print("accel=%.1f cm/s2  speed=%.1f cm/s  dist=%.1f cm" % (accel_cm_s2, speed_cm_s, dist_cm))
    print("  verwacht %d stappen, motor A kreeg %d, motor B kreeg %d" % (n, da, db))
    print("  duur %d ms over %d service()-ticks" % (dt, ticks))
    if da != n or db != n:
        print("  LET OP: afwijking in stappentotaal -- controleer de tabelopbouw, "
              "dit is geen stall-detectie (zie docstring)")
    return da == n and db == n
