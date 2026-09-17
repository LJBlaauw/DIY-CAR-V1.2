# ================================================================
# tests/test_gripper_geometry.py
#
# Pure-Python tests for lib/gripper/geometry.py. This module intentionally has NO
# hardware imports, so this runs without stubs:
#
#     python3 tests/test_gripper_geometry.py
#
# The geometry was previously located in stepper_ramp.py; these tests lock down
# the numbers mentioned in globale_specificatie.md and in the mission strategy.
# in terminal: cd "/mnt/Intenso/Micropython/DIY CAR v1.2"
# python3 tests/test_gripper_geometry.py
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
        print("  FAIL %s %s" % (naam, detail))
        _fails.append(naam)


print("\n--- tip_pos_cm: straight line through the two measurement points ---")
check("maximum open -> TIP_NEAR_CM",
      abs(g.tip_pos_cm(g.GRIP_OPEN_CM) - g.TIP_NEAR_CM) < 1e-9)
check("almost closed -> TIP_FAR_CM",
      abs(g.tip_pos_cm(g.GRIP_MIN_CM) - g.TIP_FAR_CM) < 1e-9)
check("the tips move FORWARD when closing",
      g.tip_pos_cm(3.0) > g.tip_pos_cm(8.0))

print("\n--- stop_dist_cm: narrower object -> larger stopping distance ---")
for w, verwacht in ((3.0, 14.57), (5.0, 13.71), (6.0, 13.29), (8.0, 12.43)):
    d = g.stop_dist_cm(w)
    check("%.0f cm wide -> %.2f cm" % (w, verwacht), abs(d - verwacht) < 0.01,
          "(%.3f)" % d)

check("monotonically decreasing with object width",
      all(g.stop_dist_cm(w) > g.stop_dist_cm(w + 0.5) for w in (3.0, 4.0, 5.0, 6.0, 7.0)))
check("default uses OBJECT_W_CM",
      abs(g.stop_dist_cm() - g.stop_dist_cm(g.OBJECT_W_CM)) < 1e-9)

# 13.29 and not 13.0: with a targeted final accuracy of ~0.3 cm, that
# rounding would consume the entire error budget.
check("STOP_DIST_CM is 13.29 cm, not 13", abs(g.STOP_DIST_CM - 13.2857) < 0.001,
      "(%.4f)" % g.STOP_DIST_CM)

print("\n--- grip_window_cm: the window is the progression of the tips ---")
for w in (3.0, 5.0, 6.0, 7.0, 8.0):
    lo, hi = g.grip_window_cm(w)
    check("%.0f cm: target lies on the upper limit" % w,
          abs(hi - g.stop_dist_cm(w)) < 1e-9)
    check("%.0f cm: lower limit is TIP_NEAR_CM" % w, abs(lo - g.TIP_NEAR_CM) < 1e-9)
    check("%.0f cm: window is positive" % w, hi > lo, "(%.2f-%.2f)" % (lo, hi))

lo, hi = g.grip_window_cm(3.0)
check("3 cm width gives ~2.6 cm window", abs((hi - lo) - 2.571) < 0.01,
      "(%.3f)" % (hi - lo))
lo, hi = g.grip_window_cm(8.0)
check("8 cm width gives ~0.4 cm window", abs((hi - lo) - 0.429) < 0.01,
      "(%.3f)" % (hi - lo))

print("\n--- lateral_tolerance_cm ---")
for w, verwacht in ((3.0, 3.0), (5.0, 2.0), (6.0, 1.5), (7.0, 1.0)):
    t = g.lateral_tolerance_cm(w)
    check("%.0f cm -> +/- %.1f cm" % (w, verwacht), abs(t - verwacht) < 1e-9,
          "(%.3f)" % t)
check("an object as wide as the jaws has no tolerance",
      abs(g.lateral_tolerance_cm(g.GRIP_OPEN_CM)) < 1e-9)

print("\n================================")
if _fails:
    print("%d TEST(S) FAILED:" % len(_fails))
    for f in _fails:
        print("  - %s" % f)
    sys.exit(1)
print("All tests passed.")
