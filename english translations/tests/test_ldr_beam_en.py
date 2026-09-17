# ================================================================
# tests/test_ldr_beam.py
#
# Measurement script for the LDR characteristic, the beam profile of the light source,
# and with that the lateral deviation y of the cart relative to the beam axis.
#
# Three measurements, which can be called separately from the REPL:
#
#   gamma()    - determines the LDR exponent gamma. Bench measurement: source on the axis,
#                a series of distances. R ~ E^-gamma, so ln R = 2*gamma*ln d + c.
#   bundel()   - determines the beam profile I(phi). Bench measurement: LDR on an arc
#                with a FIXED radius around the source, always aimed TOWARDS the source.
#                This fixes the tunnel angle and distance, allowing you to pure-measure the beam.
#   meet_y()   - on the cart: drive a straight line and calculate how far you are from the
#                beam axis based on the drop in normalized brightness.
#
# WHY THIS WORKS
# --------------
# Received light E = I(phi) / d^2, with phi = atan(y/d) being the angle at which the
# source sees you. Normalize the distance out of it:
#
#     Q = -(1/gamma)*ln(R) + 2*ln(d)      (= ln I(phi) up to a constant)
#
# Q remains CONSTANT if you are on the beam axis (phi = 0 at any distance), and
# DROPS if you are off-center, because phi grows as you approach. The value of y follows
# from the drop over a straight run. No sidesteps are needed for the magnitude; only
# the sign requires an additional dither.
#
# PLEASE NOTE - check once:
#   After lowering R29/R30 to 1 kOhm, LDR_R_FIXED_OHM in lib/LDR/ldr_scan_isr.py
#   MUST also be set to 1000, otherwise all resistance values are off by a factor of 10.
#   This script verifies that.
# ================================================================

import math
import time
import sys
from machine import ADC, Pin

sys.path.append("/lib/LDR")
sys.path.append("/lib/stepper")
sys.path.append("/lib/ultrasoon")

# ----------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------
LDR_PIN_A = 26
LDR_PIN_B = 27

R_FIXED_VERWACHT = 1000.0     # pull-up to 3V3, after the hardware modification
ADC_MAX = 65535.0

N_SAMPLES = 32                # average per measurement; suppresses ADC noise
CSV_DIR = "/"

# Initial values; will be overwritten by gamma() and bundel()
GAMMA = 0.7                   # LDR exponent, R ~ E^-GAMMA
BEAM_W_DEG = 33.0             # 1/e half-angle of the beam in degrees

# Assumptions for error estimation in meet_y()
US_FOUT_CM = 0.3              # ultrasonic accuracy
DRIFT_Q = 0.01                # drift in Q over the duration of one measurement leg

_adc_a = ADC(Pin(LDR_PIN_A, Pin.IN))
_adc_b = ADC(Pin(LDR_PIN_B, Pin.IN))

_R_FIXED = R_FIXED_VERWACHT
try:
    import ldr_scan_isr
    _R_FIXED = float(ldr_scan_isr.LDR_R_FIXED_OHM)
except Exception:
    print("! ldr_scan_isr not importable; falling back to R_FIXED =",
          R_FIXED_VERWACHT)


def controleer_config():
    """Warn if the code does not match the modified hardware."""
    ok = True
    if abs(_R_FIXED - R_FIXED_VERWACHT) > 1.0:
        print("! LDR_R_FIXED_OHM = %.0f, expected %.0f." % (_R_FIXED,
                                                            R_FIXED_VERWACHT))
        print("  Adjust lib/LDR/ldr_scan_isr.py, otherwise every resistance is")
        print("  off by a factor of %.1f." % (_R_FIXED / R_FIXED_VERWACHT))
        ok = False
    try:
        if ldr_scan_isr.LDR_R_MIN_OHM > 20.0:
            print("! LDR_R_MIN_OHM = %.0f clamps the percentage scale close to the"
                  % ldr_scan_isr.LDR_R_MIN_OHM)
            print("  source firmly at 100%. Set it to ~10 for the final phase.")
            ok = False
    except Exception:
        pass
    if ok:
        print("Configuration OK. R_FIXED = %.0f ohm (pull-up to 3V3)."
              % _R_FIXED)
    return ok


# ----------------------------------------------------------------
# Base Measurement
# ----------------------------------------------------------------
def _raw(adc, n=N_SAMPLES):
    acc = 0
    for _ in range(n):
        acc += adc.read_u16()
    return acc / n


def _res(adc, n=N_SAMPLES):
    """LDR resistance in ohm. Topology: R_FIXED to 3V3, LDR to GND."""
    a = _raw(adc, n)
    if a < 1.0:
        a = 1.0
    elif a > ADC_MAX - 1.0:
        a = ADC_MAX - 1.0
    return _R_FIXED * a / (ADC_MAX - a)


def lees(n=N_SAMPLES):
    """Returns (R_A, R_B, raw_A, raw_B)."""
    ra = _res(_adc_a, n)
    rb = _res(_adc_b, n)
    return ra, rb, _raw(_adc_a, 4), _raw(_adc_b, 4)


def _ln_E(ra, rb, gamma=None):
    """ln of the combined light intensity of both LDRs, up to a constant.

    E ~ R^(-1/gamma), so the SUM of the light intensities is
    R_A^(-1/g) + R_B^(-1/g). Do not sum the resistances: that scale is
    logarithmic and non-linear with respect to light.
    """
    g = GAMMA if gamma is None else gamma
    e = ra ** (-1.0 / g) + rb ** (-1.0 / g)
    return math.log(e)


def Q(d_cm, ra=None, rb=None, gamma=None):
    """Normalized brightness: ln I(phi) up to a constant.

    Constant over distance if you are on the beam axis; dropping if you are off-center.
    This is the quantity that meet_y() relies on.
    """
    if ra is None:
        ra, rb, _, _ = lees()
    return _ln_E(ra, rb, gamma) + 2.0 * math.log(d_cm)


def meting(label=""):
    """Prints a single snapshot; useful for manual testing."""
    ra, rb, xa, xb = lees()
    print("%-12s R_A=%8.1f  R_B=%8.1f  adc=%6.0f/%6.0f  A-B=%+.3f"
          % (label, ra, rb, xa, xb, math.log(rb / ra)))
    return ra, rb


# ----------------------------------------------------------------
# 1. GAMMA — LDR Exponent
# ----------------------------------------------------------------
def gamma(afstanden=(15, 20, 30, 40, 60), csv=None):
    """Determine the LDR exponent gamma from a series of distances ON THE BEAM AXIS.

    On the axis, I(phi) is constant, so E ~ 1/d^2 and the following holds:
        ln R = 2*gamma*ln d + c
    The slope of that line is 2*gamma. Multiple points are used instead of two,
    so you can also see IF the inverse-square law holds: a slit source is not
    a point source at close range, which causes the line to bend.

    Always aim the LDR directly at the source and keep it exactly on the axis.
    """
    controleer_config()
    print("\n=== GAMMA Measurement ===")
    print("Always keep the LDR directly AIMED at the source and EXACTLY on the beam axis.")
    xs, ys, rijen = [], [], []
    for d in afstanden:
        input("  Set the source at %d cm and press Enter... " % d)
        ra, rb, xa, xb = lees()
        r = math.sqrt(ra * rb)          # geometric mean of both LDRs
        xs.append(math.log(d))
        ys.append(math.log(r))
        rijen.append((d, ra, rb, r, xa, xb))
        print("    d=%3d cm  R_A=%8.1f  R_B=%8.1f  R_avg=%8.1f"
              % (d, ra, rb, r))
        if xa > ADC_MAX * 0.97 or xb > ADC_MAX * 0.97:
            print("    ! ADC nearly full -> too dark, or R_FIXED too high")
        if xa < ADC_MAX * 0.03 or xb < ADC_MAX * 0.03:
            print("    ! ADC nearly zero -> possible LDR saturation; dim the source")

    helling, offset, r2 = _fit(xs, ys)
    g = helling / 2.0
    print("\n  slope ln R vs ln d = %.4f   ->  gamma = %.4f" % (helling, g))
    print("  R^2 = %.5f %s" % (r2, "(good)" if r2 > 0.99 else
                               "(PLEASE NOTE: inverse-square law does not hold)"))
    if not (0.3 < g < 1.2):
        print("  ! gamma outside the usual 0.4-1.0 range for CdS.")
    print("\n  Set in this script:  GAMMA = %.3f" % g)
    if csv:
        _csv(csv, "d_cm,R_A,R_B,R_avg,adc_A,adc_B", rijen)
    return g


# ----------------------------------------------------------------
# 2. BEAM — Profile I(phi)
# ----------------------------------------------------------------
def bundel(radius_cm=40, hoeken=(0, 5, 10, 15, 20, 25, 30, 40, 50), csv=None):
    """Determine the beam profile I(phi) on an arc with a FIXED radius.

    Keep the LDR constantly AIMED TOWARDS the source and at a fixed distance. Then
    the tunnel angle and distance are constant, and you exclusively measure how much
    light the source emits in that direction.

    Next, fit a Gaussian profile I(phi)/I(0) = exp(-(phi/w)^2) and return the
    1/e half-angle w. This w is used in every y-calculation.

    Tip: attach a string of radius_cm to the source and mark the angles on the
    floor; this is accurate enough and much faster than measuring point by point.
    """
    controleer_config()
    print("\n=== BEAM PROFILE ===")
    print("Radius %d cm. LDR always AIMED TOWARDS the source." % radius_cm)
    ln_e0 = None
    rijen, ws = [], []
    for phi in hoeken:
        input("  Set the LDR at %d degrees off-axis and press Enter... " % phi)
        ra, rb, xa, xb = lees()
        ln_e = _ln_E(ra, rb)
        if ln_e0 is None:
            ln_e0 = ln_e
        rel = math.exp(ln_e - ln_e0)     # I(phi)/I(0)
        w = None
        if phi > 0 and 0.0 < rel < 1.0:
            w = phi / math.sqrt(-math.log(rel))
            ws.append(w)
        rijen.append((phi, ra, rb, rel, w if w else 0.0))
        print("    phi=%3d  R_A=%8.1f  R_B=%8.1f  I/I0=%.4f%s"
              % (phi, ra, rb, rel,
                 "  w=%.1f degrees" % w if w else ""))

    if ws:
        w_gem = sum(ws) / len(ws)
        spreiding = max(ws) - min(ws)
        print("\n  1/e half-angle w = %.1f degrees  (spread %.1f)"
              % (w_gem, spreiding))
        print("  half-value half-angle = %.1f degrees" % (w_gem * 0.8326))
        if spreiding > 0.3 * w_gem:
            print("  ! large spread -> the profile is not Gaussian;")
            print("    use the table instead of the fit in that case.")
        print("\n  Set in this script:  BEAM_W_DEG = %.1f" % w_gem)
    else:
        print("\n  ! no usable points; is the source turned on?")
        w_gem = None
    if csv:
