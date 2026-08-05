# ================================================================
# tests/test_stepper_ramp_math.py
#
# Pure-Python tests voor lib/stepper/stepper_ramp.py. GEEN hardware nodig:
# machine en rp2 worden gestubd, dus dit draait ook op de PC met CPython.
#
#     python3 tests/test_stepper_ramp_math.py
#
# Gedekt: plan(), ramp_words(), profile_words(), nulafstanden, grenswaarden,
# invoervalidatie, de slice-rekenkunde, en de DMA -> CPU overgang in Move
# (daar zat een fout: de CPU mocht niet pushen terwijl de DMA nog schreef).
# ================================================================

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "lib", "stepper"))

_fails = []


def check(naam, conditie, detail=""):
    if conditie:
        print("  ok   %s" % naam)
    else:
        print("  FOUT %s %s" % (naam, detail))
        _fails.append(naam)


# ----------------------------------------------------------------
# Stubs voor machine en rp2
# ----------------------------------------------------------------
class _Types:
    pass


class _FakePin:
    OUT = 1
    IN = 0

    def __init__(self, n, mode=None):
        self.n = n
        self._v = 0

    def value(self, v=None):
        if v is None:
            return self._v
        self._v = v


class _FakeSM:
    """Genereert niets, maar houdt de FIFO en een gesimuleerde pulsteller bij."""

    def __init__(self, sm_id, prog, freq=None, **kw):
        self.sm_id = sm_id
        self.fifo = []
        self.sim_pulses = 0

    def active(self, v=None):
        return 1

    def put(self, w):
        self.fifo.append(w)

    def exec(self, instr):
        # reset_pos() zet de hardwareteller terug; dat moet de stub ook doen,
        # anders lekken pulsen van de ene test naar de volgende.
        if "mov(y, osr)" in instr:
            self.sim_pulses = 0

    def get(self):
        # pulses() rekent 0xFFFFFFFF - y, dus geef y terug
        return 0xFFFFFFFF - self.sim_pulses

    def tx_fifo(self):
        return len(self.fifo)

    def init(self, prog, freq=None, **kw):
        self.fifo = []


class _FakeDMA:
    """Houdt de nog te schrijven woorden vast; active() is True zolang die er zijn."""

    def __init__(self):
        self.pending = []
        self.target = None

    def pack_ctrl(self, **kw):
        return 0

    def config(self, read=None, write=None, count=None, ctrl=None, trigger=False):
        self.pending = list(read)
        self.target = write

    def active(self, v=None):
        if v is not None:
            self.pending = []
            return None
        return len(self.pending) > 0


class _FakePIO:
    OUT_LOW = 0
    OUT_HIGH = 1
    SHIFT_RIGHT = 1
    SHIFT_LEFT = 0
    JOIN_NONE = 0
    JOIN_TX = 1
    JOIN_RX = 2


def _fake_asm_pio(*a, **kw):
    def deco(fn):
        return fn          # de body wordt nooit uitgevoerd
    return deco


_machine = _Types()
_machine.Pin = _FakePin
_machine.mem32 = {}
sys.modules["machine"] = _machine

_rp2 = _Types()
_rp2.PIO = _FakePIO
_rp2.StateMachine = _FakeSM
_rp2.DMA = _FakeDMA
_rp2.asm_pio = _fake_asm_pio
sys.modules["rp2"] = _rp2

import stepper_ramp as sr          # noqa: E402


# ----------------------------------------------------------------
# Hulpfuncties
# ----------------------------------------------------------------
def decode(w):
    """FIFO-woord -> (aantal stappen, delay)."""
    return (w & 0xFFFF) + 1, w >> 16


def totaal(words):
    return sum(decode(w)[0] for w in words)


def drain(motor):
    """Simuleer dat de PIO alles uitvoert wat aangeboden is."""
    while motor.dma.pending:
        motor.sm.fifo.append(motor.dma.pending.pop(0))
    n = 0
    while motor.sm.fifo:
        n += decode(motor.sm.fifo.pop(0))[0]
    motor.cnt.sim_pulses += n


def fresh():
    """Volledig schone begintoestand, alsof de kar net is opgestart."""
    for m in sr._MOTORS:
        m.sm.fifo = []
        m.dma.pending = []
        m.cnt.sim_pulses = 0
    sr.reset_PIO_distance()


def run_to_end(mv, limiet=200000):
    """Draai een Move helemaal uit. False = niet binnen de limiet geeindigd."""
    for _ in range(limiet):
        for m in sr._MOTORS:
            drain(m)
        if not mv.service():
            return True
    return False


# ----------------------------------------------------------------
print("\n--- ramp_words: exact stappenaantal en geldige velden ---")
r0 = sr.rate_of(sr.V_START_CM_S)
r1 = sr.rate_of(sr.V_MAX_CM_S)

for n in (0, 1, 2, 3, 7, 100, 255, 256, 257, 2200, 65536, 70000):
    w = sr.ramp_words(n, r0, r1)
    if n <= 0:
        check("n=%d -> lege lijst" % n, w == [])
        continue
    check("n=%d som klopt" % n, totaal(w) == n, "(%d)" % totaal(w))
    reps = [decode(x)[0] for x in w]
    dels = [decode(x)[1] for x in w]
    check("n=%d velden binnen bereik" % n,
          all(1 <= r <= 65536 for r in reps) and all(1 <= d <= 65535 for d in dels))
    freqs = [sr.F_PIO / (d + sr.CYCLES_FIXED) for d in dels]
    check("n=%d snelheid monotoon stijgend" % n,
          all(freqs[i] <= freqs[i + 1] + 1e-9 for i in range(len(freqs) - 1)))

w = sr.ramp_words(2200, r1, r0)
dels = [decode(x)[1] for x in w]
freqs = [sr.F_PIO / (d + sr.CYCLES_FIXED) for d in dels]
check("aflopende ramp is monotoon dalend",
      all(freqs[i] >= freqs[i + 1] - 1e-9 for i in range(len(freqs) - 1)))
check("aflopende ramp begint op topsnelheid",
      abs(freqs[0] * sr.CM_PER_STEP - sr.V_MAX_CM_S) < 0.5,
      "(%.2f cm/s)" % (freqs[0] * sr.CM_PER_STEP))

# ----------------------------------------------------------------
print("\n--- plan(): driehoek, ondergrens en nulafstand ---")
n_ramp, a, b = sr.plan(0, sr.V_MAX_CM_S)
check("nulafstand -> geen ramp", n_ramp == 0)

for n in (1, 2, 10, 500, 2000, 4400, 4401, 33508, 500000):
    n_ramp, a, b = sr.plan(n, sr.V_MAX_CM_S)
    check("n=%d ramps passen (2*%d <= %d)" % (n, n_ramp, n), 2 * n_ramp <= n)
    check("n=%d rate_cruise >= rate_start" % n, b >= a - 1e-9)

n_ramp, a, b = sr.plan(100000, 1.0)          # onder V_START_CM_S
check("snelheid onder startsnelheid -> geen ramp", n_ramp == 0)
check("snelheid onder startsnelheid wordt gerespecteerd",
      abs(b * sr.CM_PER_STEP - 1.0) < 1e-6, "(%.3f cm/s)" % (b * sr.CM_PER_STEP))

n_ramp, a, b = sr.plan(100000, 999.0)        # boven V_MAX_CM_S
check("snelheid boven maximum wordt afgetopt",
      abs(b * sr.CM_PER_STEP - sr.V_MAX_CM_S) < 1e-6)

# ----------------------------------------------------------------
print("\n--- profile_words(): exact totaal voor alle afstanden ---")
for n in (0, 1, 2, 3, 10, 100, 2200, 4400, 4401, 33508, 65536, 200000):
    w = sr.profile_words(n, sr.V_MAX_CM_S)
    if n <= 0:
        check("n=%d -> lege lijst" % n, w == [])
        continue
    check("n=%d som klopt" % n, totaal(w) == n, "(%d)" % totaal(w))
    dels = [decode(x)[1] for x in w]
    check("n=%d delays binnen bereik" % n, all(1 <= d <= 65535 for d in dels))

w = sr.profile_words(33508, sr.V_MAX_CM_S)
check("50 cm past in 513 woorden", len(w) == 513, "(%d)" % len(w))
check("kruisfase is één woord",
      len([x for x in w if decode(x)[0] > 1000]) == 1)

# ----------------------------------------------------------------
print("\n--- cruise_words(): splitsen boven 65536 stappen ---")
for n in (1, 65536, 65537, 200000):
    w = sr.cruise_words(n, r1)
    check("cruise n=%d som klopt" % n, totaal(w) == n, "(%d)" % totaal(w))
    check("cruise n=%d segmenten <= 65536" % n,
          all(decode(x)[0] <= 65536 for x in w))

# ----------------------------------------------------------------
print("\n--- invoervalidatie ---")
def raises(fn, *a, **kw):
    try:
        fn(*a, **kw)
    except ValueError:
        return True
    except Exception:
        return False
    return False

check("mov('x', ...) weigert onbekende richting", raises(sr.mov, 'x', 10, 10))
check("rotate('f', ...) weigert onbekende richting", raises(sr.rotate, 'f', 10, 10))
check("snelheid 0 geweigerd", raises(sr.mov, 'f', 0, 10))
check("snelheid negatief geweigerd", raises(sr.mov, 'f', -5, 10))
check("acceleratie 0 geweigerd", raises(sr.plan, 1000, 10.0, 1.0, 0.0))
check("nulafstand geeft False, geen fout", sr.mov('f', 10, 0) is False)
check("richting is case-insensitive", sr.mov('F', 10, 0) is False)

# ----------------------------------------------------------------
print("\n--- _clamp_delay en _word: geen stille afkapping ---")
check("_clamp_delay ondergrens", sr._clamp_delay(-100) == 1)
check("_clamp_delay bovengrens", sr._clamp_delay(999999) == 65535)
check("_word klemt delay i.p.v. maskeren", decode(sr._word(5, 999999))[1] == 65535)
check("_word klemt repeat", decode(sr._word(0, 100))[0] == 1)
check("_word repeat maximum", decode(sr._word(70000, 100))[0] == 65536)

# ----------------------------------------------------------------
print("\n--- Move: CPU mag niet pushen terwijl de DMA nog schrijft ---")
fresh()
mv = sr.Move(50, sr.V_MAX_CM_S, correction=lambda: 10.0)
check("start in RAMP_UP", mv._state == mv._RAMP_UP)
check("opramp staat bij de DMA, niet in de FIFO",
      len(sr.MA.dma.pending) > 0 and len(sr.MA.sm.fifo) == 0)

for _ in range(5):
    mv.service()
check("service() pusht niets zolang dma.active()",
      len(sr.MA.sm.fifo) == 0 and mv._state == mv._RAMP_UP,
      "(fifo=%d state=%d)" % (len(sr.MA.sm.fifo), mv._state))

# DMA klaar met schrijven -> nu mag de CPU over
for m in sr._MOTORS:
    m.sm.fifo.extend(m.dma.pending)
    m.dma.pending = []
mv.service()
check("na dma klaar gaat hij naar CRUISE", mv._state == mv._CRUISE)

# ----------------------------------------------------------------
print("\n--- Move: exact totaal, met en zonder correctie ---")
for corr, naam in ((None, "zonder correctie"),
                   (lambda: 0.0, "correctie 0"),
                   (lambda: 8.0, "correctie +8 gr/s"),
                   (lambda: -30.0, "correctie -30 gr/s (afgetopt)")):
    fresh()
    mv = sr.Move(50, sr.V_MAX_CM_S, correction=corr)
    n_tot = mv.n_total
    if not run_to_end(mv):
        check("%s: beweging eindigt" % naam, False)
        continue
    ca, cb = sr.MA.committed, sr.MB.committed
    check("%s: midden exact %d stappen" % (naam, n_tot), (ca + cb) == 2 * n_tot,
          "(A=%d B=%d)" % (ca, cb))
    check("%s: alles uitgestuurd" % naam,
          sr.MA.pulses() == ca and sr.MB.pulses() == cb)
    if corr is not None and corr() != 0.0:
        check("%s: wielen verschillen (koers veranderd)" % naam, ca != cb,
              "(A=%d B=%d)" % (ca, cb))

# ----------------------------------------------------------------
print("\n--- Move: korte en nul-afstanden ---")
for d in (0, 0.001, 0.01, 0.1, 1.0, 3.0, 6.6, 7.0):
    fresh()
    mv = sr.Move(d, sr.V_MAX_CM_S, correction=lambda: 0.0)
    n_tot = mv.n_total
    ok = run_to_end(mv, 100000)
    check("dist=%.3f cm (%d stappen) eindigt netjes" % (d, n_tot), ok)
    if ok and n_tot > 0:
        check("dist=%.3f cm exact totaal" % d,
              (sr.MA.committed + sr.MB.committed) == 2 * n_tot,
              "(A=%d B=%d)" % (sr.MA.committed, sr.MB.committed))

# ----------------------------------------------------------------
print("\n--- slice-rekenkunde: gelijke duur, exact midden ---")
base = int(r1 * sr.SLICE_MS / 1000.0 + 0.5)
for k in (0.0, 0.02, 0.05, 0.10, 0.20):
    delta = int(k * base)
    ra, rb = base + delta, base - delta
    cyc = sr.F_PIO * base / r1
    da = sr._clamp_delay(int(cyc / ra + 0.5) - sr.CYCLES_FIXED)
    db = sr._clamp_delay(int(cyc / rb + 0.5) - sr.CYCLES_FIXED)
    ta = ra * (da + sr.CYCLES_FIXED) / sr.F_PIO
    tb = rb * (db + sr.CYCLES_FIXED) / sr.F_PIO
    check("k=%.2f midden blijft exact" % k, (ra + rb) == 2 * base)
    check("k=%.2f slice-duur wijkt <0,5%% af" % k, abs(ta - tb) < 0.005 * ta,
          "(%.1f us)" % ((ta - tb) * 1e6))

# ----------------------------------------------------------------
print("\n--- odometrie: teken bij rotatie ---")
fresh()
sr.MA.set_dir(True)
sr.MB.set_dir(False)                 # tegengesteld = rotatie
sr.MA.cnt.sim_pulses += 28632
sr.MB.cnt.sim_pulses += 28632
check("rotatie geeft ~0 cm voorwaarts", abs(sr.distance()) < 0.01,
      "(%.3f cm)" % sr.distance())
check("rotatie geeft ~360 graden koers", abs(abs(sr.heading()) - 360.0) < 1.0,
      "(%.2f graden)" % sr.heading())

fresh()
sr.MA.set_dir(True)
sr.MB.set_dir(True)
sr.MA.cnt.sim_pulses += 33508
sr.MB.cnt.sim_pulses += 33508
check("recht vooruit geeft 50 cm", abs(sr.distance() - 50.0) < 0.05,
      "(%.2f cm)" % sr.distance())
check("recht vooruit geeft 0 graden", abs(sr.heading()) < 0.01)

fresh()
sr.MA.set_dir(False)
sr.MB.set_dir(False)
sr.MA.cnt.sim_pulses += 33508
sr.MB.cnt.sim_pulses += 33508
check("achteruit telt negatief", abs(sr.distance() + 50.0) < 0.05,
      "(%.2f cm)" % sr.distance())

# ----------------------------------------------------------------
print("\n--- HeadingController: eenheden, aftopping, slipdetectie ---")
hc = sr.HeadingController(ldr_diff=lambda: 0.0, gyro_rate=lambda: 0.0)
check("LDR in deadband -> 0", abs(hc.output()) < 1e-9)

# De standaard kp_ldr=25 geeft bij een vol LDR-verschil 25 gr/s, net onder het
# plafond van 32,2 gr/s -- de regelaar verzadigt dus normaal gesproken niet.
hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=None, kp_ldr=25.0)
check("standaard kp_ldr verzadigt niet bij vol LDR-verschil",
      abs(hc.output() - 25.0) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=None, kp_ldr=100.0)
check("te grote uitslag wordt afgetopt op MAX_TURN_DEG_S",
      abs(hc.output() - sr.MAX_TURN_DEG_S) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: -1.0, gyro_rate=None, kp_ldr=100.0)
check("aftopping werkt ook negatief",
      abs(hc.output() + sr.MAX_TURN_DEG_S) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: 0.1, gyro_rate=lambda: 0.0,
                          kp_ldr=25.0, kp_gyro=0.6, deadband=0.03)
o = hc.output()
check("gyro op 0 versterkt het setpoint", abs(o - (2.5 + 0.6 * 2.5)) < 1e-6,
      "(%.3f)" % o)

hc = sr.HeadingController(ldr_diff=None, gyro_rate=lambda: 0.0, slip_ticks=3,
                          slip_deg_s=8.0)
for _ in range(6):
    hc.output()
check("recht rijden met kloppende gyro is GEEN slip", hc.slipping is False)

hc = sr.HeadingController(ldr_diff=lambda: 0.5, gyro_rate=lambda: 0.0,
                          slip_ticks=3, slip_deg_s=8.0)
for _ in range(6):
    hc.output()
check("gecommandeerd draaien zonder gemeten rotatie IS slip", hc.slipping is True)

# gyro-eenheden
class _FakeIMU:
    gyro = (0.0, 0.0, 1.0)           # 1 rad/s

f = sr.gyro_z_deg_s(_FakeIMU())
check("gyro_z_deg_s zet rad/s om naar graden/s", abs(f() - 57.29577951) < 1e-4,
      "(%.4f)" % f())

# ----------------------------------------------------------------
print("\n--- Move.finish(): breekt af en remt af ---")
fresh()
mv = sr.Move(200, sr.V_MAX_CM_S, correction=lambda: 0.0)
volledig = mv.n_total
for _ in range(6):
    for m in sr._MOTORS:
        drain(m)
    mv.service()
half = sr.MA.committed
mv.finish()
run_to_end(mv, 100000)
check("finish() stopt eerder dan de volle afstand", sr.MA.committed < volledig,
      "(%d < %d)" % (sr.MA.committed, volledig))
check("finish() voegt een afremramp toe", sr.MA.committed > half,
      "(%d > %d)" % (sr.MA.committed, half))
check("finish() laat alles netjes uitlopen", sr.MA.pulses() == sr.MA.committed,
      "(%d vs %d)" % (sr.MA.pulses(), sr.MA.committed))

# ----------------------------------------------------------------
print("\n================================")
if _fails:
    print("%d TEST(S) GEFAALD:" % len(_fails))
    for f in _fails:
        print("  - %s" % f)
    sys.exit(1)
print("Alle tests geslaagd.")
