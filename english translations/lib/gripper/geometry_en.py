# ================================================================
# Gripper Geometry — where should the cart stop to make a catch?
# ================================================================
#
# Previously located in lib/stepper/stepper_ramp.py. It didn't belong there: a
# generic motor driver became dependent on the dimensions of one gripper
# and the stopping strategy of one mission. This module is pure math —
# no hardware, no imports — so that it can be tested anywhere, including on the PC.
#
# The jaws close HORIZONTALLY, but the fingertips move FORWARD during this process.
# Measured: maximum open (9 cm) -> tips 12 cm ahead of the ultrasonic sensor;
#           almost closed  (2 cm) -> tips 15 cm ahead of the ultrasonic sensor.
#
# PLEASE NOTE: tip_pos_cm() is a STRAIGHT line through those two measurement points. A
# four-bar mechanism actually creates a curve; one extra measurement at
# ~5 cm opening shows how much that deviates.
#
# While driving, the servos are in their rest positions: the jaws are then BEHIND
# the ultrasonic sensor and fall outside its beam, so the distance measurement is clean.
# As soon as the arm unfolds, the jaws (9 cm open) are in a beam that is approximately
# 8 cm wide at 12-15 cm -> from that moment on, the sensor looks at its own
# fingers and there is NO MORE feedback.
# ================================================================

GRIP_OPEN_CM  = 9.0      # jaw opening maximum open
GRIP_MIN_CM   = 2.0      # jaw opening almost closed
TIP_NEAR_CM   = 12.0     # tips relative to ultrasonic sensor at maximum open
TIP_FAR_CM    = 15.0     # tips relative to ultrasonic sensor at almost closed
OBJECT_W_CM   = 6.0      # assumed object width; can be overwritten per mission


def tip_pos_cm(opening_cm):
    """Distance from the fingertips to the ultrasonic sensor at a given jaw opening."""
    f = (GRIP_OPEN_CM - opening_cm) / (GRIP_OPEN_CM - GRIP_MIN_CM)
    return TIP_NEAR_CM + f * (TIP_FAR_CM - TIP_NEAR_CM)


def stop_dist_cm(object_w_cm=None):
    """Target distance (ultrasonic) for an object of this width.

    The jaws touch the object the moment the opening equals the
    object width; at that point, the tips are at tip_pos_cm(width).

    Contra-intuitive but correct: a NARROWER object requires a LARGER
    stopping distance. Narrower means closing further, thus more forward progress of the
    tips, so the cart must stay further back.

        3 cm -> 14.6 cm      6 cm -> 13.3 cm
        5 cm -> 13.7 cm      8 cm -> 12.4 cm
    """
    return tip_pos_cm(OBJECT_W_CM if object_w_cm is None else object_w_cm)


def grip_window_cm(object_w_cm=None):
    """(min, max) ultrasonic distance where the object is still gripped.

    The object is gripped as long as the tips sweep past it while the
    opening is still wider than the object. The 3 cm forward progress is therefore a
    free window on top of the stopping accuracy:

        3 cm wide -> 2.6 cm window      7 cm wide -> 0.9 cm window
        5 cm wide -> 1.7 cm window      8 cm wide -> 0.4 cm window

    The lower limit is conservatively TIP_NEAR_CM. The actual lower limit lies
    lower and is determined by the JAW DEPTH (palm relative to tips), which has not
    yet been measured.
    """
    return TIP_NEAR_CM, stop_dist_cm(object_w_cm)


def lateral_tolerance_cm(object_w_cm=None):
    """Maximum lateral deviation where the object still fits between the jaws.

    The jaws sweep through the space where the object stands during unfolding.
    With a larger deviation, one jaw will hit the object and KNOCK it
    over -- a more troublesome failure mode than just a miss. Therefore, unfold the arm
    ABOVE the object and lower it; then the jaws close around it
    from above instead of entering horizontally.

        3 cm -> +/- 3.0 cm      6 cm -> +/- 1.5 cm
        5 cm -> +/- 2,0 cm      7 cm -> +/- 1.0 cm
    """
    w = OBJECT_W_CM if object_w_cm is None else object_w_cm
    return 0.5 * (GRIP_OPEN_CM - w)


# 13.29 cm for an object of 6 cm. Do NOT round this to 13 cm: with a
# targeted final accuracy of ~0.3 cm, a 0.3 cm rounding would consume the entire error budget.
STOP_DIST_CM = tip_pos_cm(OBJECT_W_CM)
