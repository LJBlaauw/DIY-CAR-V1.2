# ================================================================
# tests/test_stepper_ramp_math.py
#
# Pure-Python tests for lib/stepper/stepper_ramp.py. NO hardware needed:
# machine and rp2 are stubbed, so this also runs on PC with CPython.
#
#     python3 tests/test_stepper_ramp_math.py
#
# Covered: plan(), ramp_words(), profile_words(), zero distances, boundary values,
# input validation, the slice arithmetic, and the DMA -> CPU transition in Move
# (there was a bug: the CPU was not allowed to push while the DMA was still writing).
#
# Regressions guarded here because they were previously faulty:
#   - finish() during the ramp-up DMA left busy() forever True (committed counted
#     the entire ramp-up, while the truncated DMA never sent out those steps);
#   - a new Move() during an ongoing movement changed DIR and the counters
#     without stopping first;
#   - rotate_deg(+90) gave heading() = -90: the rotation direction was opposite
#     to the odometry and the correction.
# ================================================================

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "lib", "stepper"))

_fails = []


def check(name, condition, detail=""):
    if condition:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        _fails.append(name)


# ----------------------------------------------------------------
# Stubs for machine and rp2
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
    """Generates nothing, but keeps track of the FIFO and a simulated pulse counter."""

    def __init__(self, sm_id, prog, freq=None, **kw):
        self.sm_id = sm_id
        self.fifo = []
        self.sim_pulses = 0

    def active(self, v=None):
        return 1

    def put(self, w):
        self.fifo.append(w)

    def exec(self, instr):
        # reset_pos() resets the hardware counter; the stub must do that too,
        # otherwise pulses leak from one test to the next.
        if "mov(y, osr)" in instr:
            self.sim_pulses = 0

    def get(self):
        # pulses() calculates 0xFFFFFFFF - y, so return y
        return 0xFFFFFFFF - self.sim_pulses

    def tx_fifo(self):
        return len(self.fifo)

    def init(self, prog, freq=None, **kw):
        self.fifo = []


class _FakeDMA:
    """Holds the words still to be written; active() is True as long as they exist."""

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
        return fn          # the body is never executed
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
# Helper functions
# ----------------------------------------------------------------
def decode(w):
    """FIFO word -> (number of steps, delay)."""
    return (w & 0xFFFF) + 1, w >> 16


def total(words):
    return sum(decode(w)[0] for w in words)


def drain(motor):
    """Simulate the PIO executing everything that has been offered."""
    while motor.dma.pending:
        motor.sm.fifo.append(motor.dma.pending.pop(0))
    n = 0
    while motor.sm.fifo:
        n += decode(motor.sm.fifo.pop(0))[0]
    motor.cnt.sim_pulses += n


def drain_words(motor, k):
    """Simulate the PIO executing exactly k words (DMA shifts through first).

    Needed to be able to intervene MIDWAY through a DMA transfer; drain() always
    finishes everything and therefore never gets to this.
    """
    for _ in range(k):
        if motor.dma.pending:
            motor.sm.fifo.append(motor.dma.pending.pop(0))
        if not motor.sm.fifo:
            return
        motor.cnt.sim_pulses += decode(motor.sm.fifo.pop(0))[0]


def fresh():
    """Completely clean starting state, as if the cart has just booted up."""
    sr.halt()                       # committed back to the pulse position
    for m in sr._MOTORS:
        m.sm.fifo = []
        m.dma.pending = []
        m.cnt.sim_pulses = 0
        m.committed = 0
    sr.reset_PIO_distance()


def run_to_end(mv, limit=200000):
    """Run a Move entirely to the end. False = did not finish within the limit."""
    for _ in range(limit):
        for m in sr._MOTORS:
            drain(m)
        if not mv.service():
            return True
    return False


# ----------------------------------------------------------------
print("\n--- ramp_words: exact step count and valid fields ---")
r0 = sr.rate_of(sr.V_START_CM_S)
r1 = sr.rate_of(sr.V_MAX_CM_S)

for n in (0, 1, 2, 3, 7, 100, 255, 256, 257, 2200, 65536, 70000):
    w = sr.ramp_words(n, r0, r1)
    if n <= 0:
        check("n=%d -> empty list" % n, w == [])
        continue
    check("n=%d sum matches" % n, total(w) == n, "(%d)" % total(w))
    reps = [decode(x)[0] for x in w]
    dels = [decode(x)[1] for x in w]
    check("n=%d fields within range" % n,
          all(1 <= r <= 65536 for r in reps) and all(1 <= d <= 65535 for d in dels))
    freqs = [sr.F_PIO / (d + sr.CYCLES_FIXED) for d in dels]
    check("n=%d speed monotonically increasing" % n,
          all(freqs[i] <= freqs[i + 1] + 1e-9 for i in range(len(freqs) - 1)))

w = sr.ramp_words(2200, r1, r0)
dels = [decode(x)[1] for x in w]
freqs = [sr.F_PIO / (d + sr.CYCLES_FIXED) for d in dels]
check("descending ramp is monotonically decreasing",
      all(freqs[i] >= freqs[i + 1] - 1e-9 for i in range(len(freqs) - 1)))
check("descending ramp starts at top speed",
      abs(freqs[0] * sr.CM_PER_STEP - sr.V_MAX_CM_S) < 0.5,
      "(%.2f cm/s)" % (freqs[0] * sr.CM_PER_STEP))

# ----------------------------------------------------------------
print("\n--- plan(): triangle, lower bound and zero distance ---")
n_ramp, a, b = sr.plan(0, sr.V_MAX_CM_S)
check("zero distance -> no ramp", n_ramp == 0)

for n in (1, 2, 10, 500, 2000, 4400, 4401, 33508, 500000):
    n_ramp, a, b = sr.plan(n, sr.V_MAX_CM_S)
    check("n=%d ramps fit (2*%d <= %d)" % (n, n_ramp, n), 2 * n_ramp <= n)
    check("n=%d rate_cruise >= rate_start" % n, b >= a - 1e-9)

n_ramp, a, b = sr.plan(100000, 1.0)          # below V_START_CM_S
check("speed below starting speed -> no ramp", n_ramp == 0)
check("speed below starting speed is respected",
      abs(b * sr.CM_PER_STEP - 1.0) < 1e-6, "(%.3f cm/s)" % (b * sr.CM_PER_STEP))

n_ramp, a, b = sr.plan(100000, 999.0)        # above V_MAX_CM_S
check("speed above maximum is capped",
      abs(b * sr.CM_PER_STEP - sr.V_MAX_CM_S) < 1e-6)

# ----------------------------------------------------------------
print("\n--- profile_words(): exact total for all distances ---")
for n in (0, 1, 2, 3, 10, 100, 2200, 4400, 4401, 33508, 65536, 200000):
    w = sr.profile_words(n, sr.V_MAX_CM_S)
    if n <= 0:
        check("n=%d -> empty list" % n, w == [])
        continue
    check("n=%d sum matches" % n, total(w) == n, "(%d)" % total(w))
    dels = [decode(x)[1] for x in w]
    check("n=%d delays within range" % n, all(1 <= d <= 65535 for d in dels))

w = sr.profile_words(33508, sr.V_MAX_CM_S)
check("50 cm fits in 513 words", len(w) == 513, "(%d)" % len(w))
check("cruise phase is one word",
      len([x for x in w if decode(x)[0] > 1000]) == 1)

# ----------------------------------------------------------------
print("\n--- cruise_words(): splitting above 65536 steps ---")
for n in (1, 65536, 65537, 200000):
    w = sr.cruise_words(n, r1)
    check("cruise n=%d sum matches" % n, total(w) == n, "(%d)" % total(w))
    check("cruise n=%d segments <= 65536" % n,
          all(decode(x)[0] <= 65536 for x in w))

# ----------------------------------------------------------------
print("\n--- input validation ---")
def raises(fn, *a, **kw):
    try:
        fn(*a, **kw)
    except ValueError:
        return True
    except Exception:
        return False
    return False

check("mov('x', ...) refuses unknown direction", raises(sr.mov, 'x', 10, 10))
check("rotate('f', ...) refuses unknown direction", raises(sr.rotate, 'f', 10, 10))
check("speed 0 refused", raises(sr.mov, 'f', 0, 10))
check("speed negative refused", raises(sr.mov, 'f', -5, 10))
check("acceleration 0 refused", raises(sr.plan, 1000, 10.0, 1.0, 0.0))
check("zero distance returns False, no error", sr.mov('f', 10, 0) is False)
check("direction is case-insensitive", sr.mov('F', 10, 0) is False)

# A string previously raised a TypeError instead of the promised ValueError.
check("speed as string refused with ValueError", raises(sr.mov, 'f', "10", 10))
check("speed None refused with ValueError", raises(sr.mov, 'f', None, 10))
check("speed inf refused", raises(sr.mov, 'f', float('inf'), 10))
check("speed nan refused", raises(sr.mov, 'f', float('nan'), 10))

# Negative distance was silently made positive by abs() in cm_to_steps().
check("negative distance refused for mov", raises(sr.mov, 'f', 10, -5))
check("negative distance refused for s1", raises(sr.s1, 'f', 10, -5))
check("negative distance refused for s2", raises(sr.s2, 'f', 10, -5))
check("negative distance refused for rotate", raises(sr.rotate, 'r', 10, -5))
check("distance as string refused", raises(sr.mov, 'f', 10, "5"))
check("negative distance refused for Move", raises(sr.Move, -5, 10))
check("creep() keeps a SIGNED distance", sr.creep(-1.0) is True)
sr.halt()
check("creep(nan) refused", raises(sr.creep, float('nan')))

# ----------------------------------------------------------------
print("\n--- _clamp_delay and _word: no silent truncation ---")
check("_clamp_delay lower bound", sr._clamp_delay(-100) == 1)
check("_clamp_delay upper bound", sr._clamp_delay(999999) == 65535)
check("_word clamps delay instead of masking", decode(sr._word(5, 999999))[1] == 65535)
check("_word clamps repeat", decode(sr._word(0, 100))[0] == 1)
check("_word repeat maximum", decode(sr._word(70000, 100))[0] == 65536)

# ----------------------------------------------------------------
print("\n--- Move: CPU must not push while the DMA is still writing ---")
fresh()
mv = sr.Move(50, sr.V_MAX_CM_S, correction=lambda: 10.0)
check("starts in RAMP_UP", mv._state == mv._RAMP_UP)
check("ramp-up is with the DMA, not in the FIFO",
      len(sr.MA.dma.pending) > 0 and len(sr.MA.sm.fifo) == 0)

for _ in range(5):
    mv.service()
check("service() pushes nothing as long as dma.active()",
      len(sr.MA.sm.fifo) == 0 and mv._state == mv._RAMP_UP,
      "(fifo=%d state=%d)" % (len(sr.MA.sm.fifo), mv._state))

# DMA finished writing -> now the CPU can take over
for m in sr._MOTORS:
    m.sm.fifo.extend(m.dma.pending)
    m.dma.pending = []
mv.service()
check("after dma finishes it goes to CRUISE", mv._state == mv._CRUISE)

# ----------------------------------------------------------------
print("\n--- Move: exact total, with and without correction ---")
for corr, name in ((None, "without correction"),
                   (lambda: 0.0, "correction 0"),
                   (lambda: 8.0, "correction +8 deg/s"),
                   (lambda: -30.0, "correction -30 deg/s (capped)")):
    fresh()
    mv = sr.Move(50, sr.V_MAX_CM_S, correction=corr)
    n_tot = mv.n_total
    if not run_to_end(mv):
        check("%s: movement ends" % name, False)
        continue
    ca, cb = sr.MA.committed, sr.MB.committed
    check("%s: middle exactly %d steps" % (name, n_tot), (ca + cb) == 2 * n_tot,
          "(A=%d B=%d)" % (ca, cb))
    check("%s: everything sent out" % name,
          sr.MA.pulses() == ca and sr.MB.pulses() == cb)
    if corr is not None and corr() != 0.0:
        check("%s: wheels differ (heading changed)" % name, ca != cb,
              "(A=%d B=%d)" % (ca, cb))

# ----------------------------------------------------------------
print("\n--- Move: short and zero distances ---")
for d in (0, 0.001, 0.01, 0.1, 1.0, 3.0, 6.6, 7.0):
    fresh()
    mv = sr.Move(d, sr.V_MAX_CM_S, correction=lambda: 0.0)
    n_tot = mv.n_total
    ok = run_to_end(mv, 100000)
    check("dist=%.3f cm (%d steps) ends neatly" % (d, n_tot), ok)
    if ok and n_tot > 0:
        check("dist=%.3f cm exact total" % d,
              (sr.MA.committed + sr.MB.committed) == 2 * n_tot,
              "(A=%d B=%d)" % (sr.MA.committed, sr.MB.committed))

# ----------------------------------------------------------------
print("\n--- slice arithmetic: equal duration, exact middle ---")
base = int(r1 * sr.SLICE_MS / 1000.0 + 0.5)
for k in (0.0, 0.02, 0.05, 0.10, 0.20):
    delta = int(k * base)
    ra, rb = base + delta, base - delta
    cyc = sr.F_PIO * base / r1
    da = sr._clamp_delay(int(cyc / ra + 0.5) - sr.CYCLES_FIXED)
    db = sr._clamp_delay(int(cyc / rb + 0.5) - sr.CYCLES_FIXED)
    ta = ra * (da + sr.CYCLES_FIXED) / sr.F_PIO
    tb = rb * (db + sr.CYCLES_FIXED) / sr.F_PIO
    check("k=%.2f middle remains exact" % k, (ra + rb) == 2 * base)
    check("k=%.2f slice duration deviates <0.5%%" % k, abs(ta - tb) < 0.005 * ta,
          "(%.1f us)" % ((ta - tb) * 1e6))

# ----------------------------------------------------------------
print("\n--- odometry: sign during rotation ---")
fresh()
sr.MA.set_dir(True)
sr.MB.set_dir(False)                 # opposite = rotation
sr.MA.cnt.sim_pulses += 28632
sr.MB.cnt.sim_pulses += 28632
check("rotation gives ~0 cm forward", abs(sr.distance()) < 0.01,
      "(%.3f cm)" % sr.distance())
check("rotation gives ~360 degrees heading", abs(abs(sr.heading()) - 360.0) < 1.0,
      "(%.2f degrees)" % sr.heading())

fresh()
sr.MA.set_dir(True)
sr.MB.set_dir(True)
sr.MA.cnt.sim_pulses += 33508
sr.MB.cnt.sim_pulses += 33508
check("straight ahead gives 50 cm", abs(sr.distance() - 50.0) < 0.05,
      "(%.2f cm)" % sr.distance())
check("straight ahead gives 0 degrees", abs(sr.heading()) < 0.01)

fresh()
sr.MA.set_dir(False)
sr.MB.set_dir(False)
sr.MA.cnt.sim_pulses += 33508
sr.MB.cnt.sim_pulses += 33508
check("backward counts negative", abs(sr.distance() + 50.0) < 0.05,
      "(%.2f cm)" % sr.distance())

# ----------------------------------------------------------------
print("\n--- HeadingController: units, capping, slip detection ---")
hc = sr.HeadingController(ldr_diff=lambda: 0.0, gyro_rate=lambda: 0.0)
check("LDR in deadband -> 0", abs(hc.output()) < 1e-9)

# The default kp_ldr=25 gives at full LDR difference 25 deg/s, just below the
# ceiling of 32.2 deg/s -- so the controller normally does not saturate.
hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=None, kp_ldr=25.0)
check("default kp_ldr does not saturate at full LDR difference",
      abs(hc.output() - 25.0) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=None, kp_ldr=100.0)
check("excessive deviation is capped at MAX_TURN_DEG_S",
      abs(hc.output() - sr.MAX_TURN_DEG_S) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: -1.0, gyro_rate=None, kp_ldr=100.0)
check("capping also works negatively",
      abs(hc.output() + sr.MAX_TURN_DEG_S) < 1e-6, "(%.2f)" % hc.output())

hc = sr.HeadingController(ldr_diff=lambda: 0.1, gyro_rate=lambda: 0.0,
                          kp_ldr=25.0, kp_gyro=0.6, deadband=0.03)
o = hc.output()
check("gyro at 0 amplifies the setpoint", abs(o - (2.5 + 0.6 * 2.5)) < 1e-6,
      "(%.3f)" % o)

hc = sr.HeadingController(ldr_diff=None, gyro_rate=lambda: 0.0,
                          track_err_ticks=3, track_err_deg_s=8.0)
for _ in range(6):
    hc.output()
check("driving straight with matching gyro gives no heading tracking error",
      hc.yaw_tracking_error is False)

hc = sr.HeadingController(ldr_diff=lambda: 0.5, gyro_rate=lambda: 0.0,
                          track_err_ticks=3, track_err_deg_s=8.0)
for _ in range(6):
    hc.output()
check("commanded turning without measured rotation IS a heading tracking error",
      hc.yaw_tracking_error is True)

# note_applied(): the error is measured against what was ACTUALLY written out,
# not against a setpoint that the wheels could never deliver at this speed.
hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=lambda: 0.0,
                          kp_ldr=25.0, track_err_ticks=3, track_err_deg_s=8.0)
for _ in range(6):
    hc.output()
    hc.note_applied(0.0)             # in reality nothing turned
check("feedback of 0 degrees/s suppresses false alarm",
      hc.yaw_tracking_error is False)

# --- steering authority scales with driving speed ---
check("turn_authority at V_MAX == MAX_TURN_DEG_S",
      abs(sr.turn_authority_deg_s(sr.rate_of(sr.V_MAX_CM_S)) - sr.MAX_TURN_DEG_S) < 1e-9)
check("half speed gives half steering authority",
      abs(sr.turn_authority_deg_s(sr.rate_of(sr.V_MAX_CM_S / 2))
          - sr.MAX_TURN_DEG_S / 2) < 1e-9)

fresh()
hc = sr.HeadingController(ldr_diff=lambda: 1.0, gyro_rate=None, kp_ldr=100.0)
mv = sr.Move(50, 5.0, correction=hc)
expected = sr.turn_authority_deg_s(mv.r1)
check("Move sets the authority of the controller to the driving speed",
      abs(hc.authority - expected) < 1e-9, "(%.2f vs %.2f)" % (hc.authority, expected))
check("authority at 5 cm/s lies well below that at top speed",
      hc.authority < 0.4 * sr.MAX_TURN_DEG_S,
      "(%.2f vs %.2f)" % (hc.authority, sr.MAX_TURN_DEG_S))
check("the controller caps at the reduced authority",
      abs(hc.output() - hc.authority) < 1e-6, "(%.2f)" % hc.output())
sr.halt()

# gyro units
class _FakeIMU:
    gyro = (0.0, 0.0, 1.0)           # 1 rad/s


f = sr.gyro_z_deg_s(_FakeIMU())
check("gyro_z_deg_s converts rad/s to degrees/s",
      abs(f() - sr.GYRO_Z_SIGN * 57.29577951) < 1e-4, "(%.4f)" % f())
f = sr.gyro_z_deg_s(_FakeIMU(), sign=-1)
check("gyro_z_deg_s lets the sign be overruled", abs(f() + 57.29577951) < 1e-4,
      "(%.4f)" % f())

# ----------------------------------------------------------------
print("\n--- Move.finish(): aborts and brakes ---")
fresh()
mv = sr.Move(200, sr.V_MAX_CM_S, correction=lambda: 0.0)
full = mv.n_total
for _ in range(6):
    for m in sr._MOTORS:
        drain(m)
    mv.service()
half = sr.MA.committed
mv.finish()
run_to_end(mv, 100000)
check("finish() stops sooner than the full distance", sr.MA.committed < full,
      "(%d < %d)" % (sr.MA.committed, full))
check("finish() adds a braking ramp", sr.MA.committed > half,
      "(%d > %d)" % (sr.MA.committed, half))
check("finish() lets everything coast to a stop neatly", sr.MA.pulses() == sr.MA.committed,
      "(%d vs %d)" % (sr.MA.pulses(), sr.MA.committed))

# ----------------------------------------------------------------
# REGRESSION: busy() remained True forever here. committed was increased by the
# ENTIRE ramp-up at the start, but the truncated DMA never sent out those steps.
print("\n--- Move.finish() during the ramp-up DMA ---")
for k, name in ((0, "immediately after construction"),
                (40, "halfway through the ramp-up"),
                (250, "just before the DMA end")):
    fresh()
    mv = sr.Move(200, sr.V_MAX_CM_S, correction=lambda: 0.0)
    for m in sr._MOTORS:
        drain_words(m, k)
    done = sr.MA.pulses()
    still_busy = sr.MA.dma.active()
    mv.finish()
    # The ramp-up table has been discarded; what remains is at most the fresh
    # braking ramp, and that is shorter than the 257 words of the ramp-up.
    check("%s: the ramp-up table is gone" % name, len(sr.MA.dma.pending) < 257,
          "(%d words)" % len(sr.MA.dma.pending))
    check("%s: the FIFO is cleared" % name, len(sr.MA.sm.fifo) == 0,
          "(%d words)" % len(sr.MA.sm.fifo))
    ok = False
    for _ in range(100000):
        for m in sr._MOTORS:
            drain_words(m, 4)
        if not mv.service():
            ok = True
            break
    check("%s: movement ends (no hang)" % name, ok,
          "(pulses=%d committed=%d)" % (sr.MA.pulses(), sr.MA.committed))
    check("%s: pulses == committed" % name, sr.MA.pulses() == sr.MA.committed,
          "(%d vs %d)" % (sr.MA.pulses(), sr.MA.committed))
    check("%s: both wheels equally far" % name, sr.MA.committed == sr.MB.committed,
          "(A=%d B=%d)" % (sr.MA.committed, sr.MB.committed))
    check("%s: has driven further than what already occurred" % name,
          sr.MA.committed >= done, "(%d >= %d)" % (sr.MA.committed, done))
    check("%s: has not driven the full 200 cm" % name,
          sr.MA.committed < sr.cm_to_steps(200),
          "(%d)" % sr.MA.committed)
    if k > 0:
        check("%s: the ramp-up was still running when we intervened" % name, still_busy)

# ----------------------------------------------------------------
# REGRESSION: Move() changed DIR and the counters without stopping the ongoing
# movement, so old and new profile words could come out sequentially.
print("\n--- new command during an ongoing movement ---")
fresh()
mv1 = sr.Move(200, sr.V_MAX_CM_S, correction=lambda: 0.0)
for m in sr._MOTORS:
    drain_words(m, 30)
check("the first movement is still running", sr.MA.dma.active())
mv2 = sr.Move(10, sr.V_MAX_CM_S, correction=lambda: 0.0)
check("the old DMA content is gone", len(sr.MA.dma.pending) == len(sr.MA._buf),
      "(pending=%d new=%d)" % (len(sr.MA.dma.pending), len(sr.MA._buf)))
check("the old FIFO content is gone", len(sr.MA.sm.fifo) == 0,
      "(%d words)" % len(sr.MA.sm.fifo))
check("the counter is at zero", sr.MA.pulses() == 0)
check("committed only counts the new movement",
      sr.MA.committed == mv2._centre,
      "(%d vs %d)" % (sr.MA.committed, mv2._centre))
check("the second movement ends neatly", run_to_end(mv2, 100000))
check("exactly the second distance", sr.MA.committed == mv2.n_total,
      "(%d vs %d)" % (sr.MA.committed, mv2.n_total))

fresh()
sr.mov('f', sr.V_MAX_CM_S, 200)
for m in sr._MOTORS:
    drain_words(m, 30)
sr.mov('b', sr.V_MAX_CM_S, 10)
check("mov() successively: only the new table is ready",
      len(sr.MA.dma.pending) == len(sr.MA._buf) and len(sr.MA.sm.fifo) == 0)
for m in sr._MOTORS:
    drain(m)
check("mov() successively: no longer busy", sr.busy() is False)

# ----------------------------------------------------------------
print("\n--- start_table / replace_table: no two DMAs in one FIFO ---")
fresh()
words = sr.ramp_words(1000, r0, r1)
check("start_table starts when the DMA is idle", sr.MA.start_table(words, 1000) is True)
check("start_table REFUSES while the DMA is still writing",
      sr.MA.start_table(words, 1000) is False)
before = sr.MA.committed
check("a refused start_table does not increase committed",
      sr.MA.committed == before)
drain_words(sr.MA, 5)
check("replace_table does succeed", sr.MA.replace_table(words, 1000) is True)
check("replace_table resynchronizes committed to the pulse position",
      sr.MA.committed == sr.MA.pulses() + 1000,
      "(%d vs %d+1000)" % (sr.MA.committed, sr.MA.pulses()))
drain(sr.MA)
check("after replace_table everything finishes neatly", sr.MA.busy() is False)
sr.halt()

# ----------------------------------------------------------------
# REGRESSION: rotate_deg(+90) gave heading() = -90. The rotation direction was
# opposite to both the odometry and the correction.
print("\n--- sign convention: positive is to the right everywhere ---")
fresh()
sr.rotate_deg(+90)
for m in sr._MOTORS:
    drain(m)
h_rot = sr.heading()
check("rotate_deg(+90) gives heading ~ +90", abs(h_rot - 90.0) < 1.0,
      "(%.2f degrees)" % h_rot)

fresh()
sr.rotate_deg(-90)
for m in sr._MOTORS:
    drain(m)
check("rotate_deg(-90) gives heading ~ -90", abs(sr.heading() + 90.0) < 1.0,
      "(%.2f degrees)" % sr.heading())

fresh()
sr.rotate('r', 10.0, 10.0)
for m in sr._MOTORS:
    drain(m)
check("rotate('r') turns the same way as rotate_deg(+)", sr.heading() > 0,
      "(%.2f degrees)" % sr.heading())

fresh()
mv = sr.Move(30, sr.V_MAX_CM_S, correction=lambda: +10.0)
run_to_end(mv, 100000)
h_corr = sr.heading()
check("a positive correction also turns to the right", h_corr > 0,
      "(%.2f degrees)" % h_corr)

fresh()
mv = sr.Move(30, sr.V_MAX_CM_S, correction=lambda: -10.0)
run_to_end(mv, 100000)
check("a negative correction turns to the left", sr.heading() < 0,
      "(%.2f degrees)" % sr.heading())

# MOTOR_TURN_SIGN = -1 is supposed to invert the ENTIRE chain (which motor goes
# backward, the sign of heading(), and the side the correction steers to) --
# not just a part of it, because then the cart would keep turning right
# while the odometry reports left.
_saved = sr.MOTOR_TURN_SIGN
sr.MOTOR_TURN_SIGN = -1
fresh()
sr.rotate_deg(+90)
for m in sr._MOTORS:
    drain(m)
check("MOTOR_TURN_SIGN=-1: other motor goes backward",
      sr.MA.travel() > 0 and sr.MB.travel() < 0,
      "(A=%+d B=%+d)" % (sr.MA.travel(), sr.MB.travel()))
check("MOTOR_TURN_SIGN=-1: heading remains +90", abs(sr.heading() - 90.0) < 1.0,
      "(%.2f degrees)" % sr.heading())

fresh()
mv = sr.Move(30, sr.V_MAX_CM_S, correction=lambda: +10.0)
run_to_end(mv, 100000)
check("MOTOR_TURN_SIGN=-1: positive correction remains to the right",
      sr.heading() > 0, "(%.2f degrees)" % sr.heading())
check("MOTOR_TURN_SIGN=-1: the other wheel runs further",
      sr.MA.travel() > sr.MB.travel(),
      "(A=%+d B=%+d)" % (sr.MA.travel(), sr.MB.travel()))
sr.MOTOR_TURN_SIGN = _saved

# ----------------------------------------------------------------
print("\n--- reset_PIO_distance(): not during a movement ---")
fresh()
sr.mov('f', sr.V_MAX_CM_S, 50)
try:
    sr.reset_PIO_distance()
    check("reset during movement is refused", False)
except RuntimeError:
    check("reset during movement is refused", True)
sr.halt()
sr.reset_PIO_distance()
check("reset at standstill is allowed", sr.MA.pulses() == 0)

# ----------------------------------------------------------------
print("\n================================")
if _fails:
    print("%d TEST(S) FAILED:" % len(_fails))
    for f in _fails:
        print("  - %s" % f)
    sys.exit(1)
print("All tests passed.")
