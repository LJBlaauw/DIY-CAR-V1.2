"""
fusion.py - Sensor fusion voor MPU9250 (accelerometer + gyro + magnetometer).

Complementair filter met kantelcompensatie:
- roll/pitch: fusie van accelerometer (stabiel op lange termijn) en
  gecumuleerde gyro-rotatie (ruisvrij op korte termijn).
- heading: magnetometer wordt met roll/pitch kantelgecompenseerd en
  vervolgens gefuseerd met de gyro-z rotatiesnelheid, zodat trillingen en
  kortstondige magnetische storing (bv. van steppermotoren) worden
  weggefilterd.

Gebruikt deltat.DeltaT voor de tijdmeting tussen twee update()-aanroepen.
"""

from math import atan2, sqrt, degrees, radians, sin, cos
from deltat import DeltaT


class Fusion:
    """Complementair filter dat roll, pitch en (kantelgecompenseerde) heading bijhoudt."""

    # Magnetische declinatie in graden. Pas aan voor uw locatie indien een
    # nauwkeurige koers t.o.v. het ware noorden nodig is.
    declination = 0.0

    def __init__(self, timediff=None, gyro_weight=0.98):
        self.gyro_weight = gyro_weight
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.deltat = DeltaT(timediff)
        self._started = False

    def update(self, accel, gyro, mag, ts=None):
        ax, ay, az = accel
        gx, gy, gz = gyro
        mx, my, mz = mag

        dt = self.deltat(ts)

        # Roll/pitch uit de accelerometer (in graden)
        acc_roll = degrees(atan2(ay, az))
        denom = sqrt(ay * ay + az * az)
        acc_pitch = degrees(atan2(-ax, denom)) if denom else 0.0

        was_started = self._started

        if not was_started:
            # Eerste meting: nog geen dt beschikbaar om de gyro te integreren
            self.roll = acc_roll
            self.pitch = acc_pitch
            self._started = True
        else:
            gyro_roll = self.roll + degrees(gx) * dt
            gyro_pitch = self.pitch + degrees(gy) * dt
            w = self.gyro_weight
            self.roll = w * gyro_roll + (1 - w) * acc_roll
            self.pitch = w * gyro_pitch + (1 - w) * acc_pitch

        # Kantelcompensatie van de magnetometer met de huidige roll/pitch
        roll_r = radians(self.roll)
        pitch_r = radians(self.pitch)

        mx_c = mx * cos(pitch_r) + mz * sin(pitch_r)
        my_c = (
            mx * sin(roll_r) * sin(pitch_r)
            + my * cos(roll_r)
            - mz * sin(roll_r) * cos(pitch_r)
        )

        mag_heading = (degrees(atan2(-my_c, mx_c)) + self.declination) % 360.0

        if not was_started:
            # Eerste meting: heading direct op de magnetometer-uitlezing zetten
            # i.p.v. langzaam vanaf 0 te laten toegroeien.
            self.heading = mag_heading
        else:
            # Circulaire fusie van gyro-z-integratie en magnetometer-heading
            gyro_heading = (self.heading + degrees(gz) * dt) % 360.0
            diff = (mag_heading - gyro_heading + 540) % 360 - 180  # kortste hoekverschil
            self.heading = (gyro_heading + (1 - self.gyro_weight) * diff) % 360.0

        return self.heading
