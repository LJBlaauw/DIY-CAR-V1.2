# ================================================================
# tests/test_gripper_geometry.py
#
# Pure-Python tests voor lib/gripper/geometry.py. Die module heeft bewust GEEN
# hardware-imports, dus dit draait zonder stubs:
#
#     python3 tests/test_gripper_geometry.py
#
# De geometrie stond eerder in stepper_ramp.py; deze tests leggen de getallen
# vast die in globale_specificatie.md en in de missiestrategie genoemd worden.
# ================================================================

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "lib", "gripper"))

import geometry as g          # noqa: E402

_fails = []


def check(naam, conditie, detail=""):
    if conditie:
        print("  ok   %s" % naam)
    else:
        print("  FOUT %s %s" % (naam, detail))
        _fails.append(naam)


print("\n--- tip_pos_cm: rechte door de twee meetpunten ---")
check("maximaal open -> TIP_NEAR_CM",
      abs(g.tip_pos_cm(g.GRIP_OPEN_CM) - g.TIP_NEAR_CM) < 1e-9)
check("bijna dicht -> TIP_FAR_CM",
      abs(g.tip_pos_cm(g.GRIP_MIN_CM) - g.TIP_FAR_CM) < 1e-9)
check("de toppen komen naar VOREN bij sluiten",
      g.tip_pos_cm(3.0) > g.tip_pos_cm(8.0))

print("\n--- stop_dist_cm: smaller voorwerp -> grotere stopafstand ---")
for w, verwacht in ((3.0, 14.57), (5.0, 13.71), (6.0, 13.29), (8.0, 12.43)):
    d = g.stop_dist_cm(w)
    check("%.0f cm breed -> %.2f cm" % (w, verwacht), abs(d - verwacht) < 0.01,
          "(%.3f)" % d)

check("monotoon dalend in de objectbreedte",
      all(g.stop_dist_cm(w) > g.stop_dist_cm(w + 0.5) for w in (3.0, 4.0, 5.0, 6.0, 7.0)))
check("default gebruikt OBJECT_W_CM",
      abs(g.stop_dist_cm() - g.stop_dist_cm(g.OBJECT_W_CM)) < 1e-9)

# 13,29 en niet 13,0: bij een gemikte eindnauwkeurigheid van ~0,3 cm zou die
# afronding de hele foutbegroting opeten.
check("STOP_DIST_CM is 13,29 cm, niet 13", abs(g.STOP_DIST_CM - 13.2857) < 0.001,
      "(%.4f)" % g.STOP_DIST_CM)

print("\n--- grip_window_cm: het venster is de vooruitgang van de toppen ---")
for w in (3.0, 5.0, 6.0, 7.0, 8.0):
    lo, hi = g.grip_window_cm(w)
    check("%.0f cm: doel ligt op de bovengrens" % w,
          abs(hi - g.stop_dist_cm(w)) < 1e-9)
    check("%.0f cm: ondergrens is TIP_NEAR_CM" % w, abs(lo - g.TIP_NEAR_CM) < 1e-9)
    check("%.0f cm: venster is positief" % w, hi > lo, "(%.2f-%.2f)" % (lo, hi))

lo, hi = g.grip_window_cm(3.0)
check("3 cm breed geeft ~2,6 cm venster", abs((hi - lo) - 2.571) < 0.01,
      "(%.3f)" % (hi - lo))
lo, hi = g.grip_window_cm(8.0)
check("8 cm breed geeft ~0,4 cm venster", abs((hi - lo) - 0.429) < 0.01,
      "(%.3f)" % (hi - lo))

print("\n--- lateral_tolerance_cm ---")
for w, verwacht in ((3.0, 3.0), (5.0, 2.0), (6.0, 1.5), (7.0, 1.0)):
    t = g.lateral_tolerance_cm(w)
    check("%.0f cm -> +/- %.1f cm" % (w, verwacht), abs(t - verwacht) < 1e-9,
          "(%.3f)" % t)
check("een voorwerp zo breed als de kaken heeft geen speling",
      abs(g.lateral_tolerance_cm(g.GRIP_OPEN_CM)) < 1e-9)

print("\n================================")
if _fails:
    print("%d TEST(S) GEFAALD:" % len(_fails))
    for f in _fails:
        print("  - %s" % f)
    sys.exit(1)
print("Alle tests geslaagd.")
