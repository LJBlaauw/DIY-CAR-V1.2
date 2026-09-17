# ================================================================
# Final Approach — the last correction moment for the gripper
# ================================================================
#
# Previously located as finetune() in lib/stepper/stepper_ramp.py. It combines
# three things that do not belong in a motor driver: gripper geometry,
# ultrasonic averaging, and a stopping strategy. The stepper now only provides
# creep/drive/brake/busy; this module determines WHERE TO GO.
# ================================================================

import sys

for _p in ("/lib/gripper", "/lib/stepper"):
    if _p not in sys.path:
        sys.path.append(_p)

import geometry                                     # noqa: E402
import stepper_ramp as stepper                      # noqa: E402


def mean_dist_cm(read_cm, n=8, wacht_ms=60):
    """Average distance from n INDEPENDENT ultrasonic measurements.

    wacht_ms must be greater than ultrasoon.INTERVAL_MS (50 ms), otherwise you
    read the same buffered measurement multiple times and the averaging does nothing.

    Returns None if no single measurement was usable.
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
    """Measure while stationary and creep to the exact stopping distance.

    This is the LAST correction moment. As soon as the arm unfolds, the ultrasonic
    sensor looks at its own fingers and there is no more feedback -- everything after is
    open-loop. Measuring while stationary removes the driving speed and measurement latency from the
    error, reducing the uncertainty from ~0.74 cm to ~0.3 cm.

    `read_cm` is a callable that returns the ultrasonic distance in cm (e.g.,
    ultrasoon.read_cm), keeping this module sensor-agnostic.

    A new measurement is ALWAYS taken after a correction, even after the final attempt.
    Otherwise, the judgment would be based on the measurement from before that correction,
    and the cart could be positioned correctly while reporting "OUTSIDE WINDOW".

    Returns (measured_distance, target, success).
    """
    doel = geometry.stop_dist_cm(object_w_cm)
    lo, hi = geometry.grip_window_cm(object_w_cm)

    for _ in range(max(1, int(pogingen))):
        d = mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel, False
        fout = d - doel
        if abs(fout) <= tol_cm:
            break
        stepper.creep(fout, speed_cm_s)
        while stepper.busy():
            pass
    else:
        # The loop finished without a break: no measurement has been taken
        # after the last correction yet. Do it now, otherwise we judge an old position.
        d = mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel, False

    gelukt = lo <= d <= hi + tol_cm
    print("finetune: measured %.2f cm, target %.2f cm, window %.2f-%.2f -> %s"
          % (d, doel, lo, hi, "OK" if gelukt else "OUTSIDE WINDOW"))
    return d, doel, gelukt


def brake_target_cm(speed_cm_s, object_w_cm=None):
    """Ultrasonic distance at which you should call Move.finish().

    The cart still covers stopping_distance_cm() after finish() (deceleration ramp plus the
    slices already in the FIFOs), so the braking moment lies that distance ahead of
    the target distance:

        target = brake_target_cm(speed)
        if ultrasoon.read_cm() <= target:
            mv.finish()

    NOT included: the measurement latency of the ultrasonic sensor itself (INTERVAL_MS = 50 ms,
    so up to 0.96 cm at full speed and 0.25 cm at 5 cm/s).
    """
    return (geometry.stop_dist_cm(object_w_cm)
            + stepper.stopping_distance_cm(speed_cm_s))
