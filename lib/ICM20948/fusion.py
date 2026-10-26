"""
fusion.py - Sensor fusion voor de ICM-20948 (accelerometer + gyro + magnetometer).

Verwacht alle drie de sensoren in het KAR-assenstelsel: X naar voren, Y naar
links, Z omhoog (rechtshandig). Dat levert icm20948.ICM20948 met de "mount"
uit config.json (zie load_mag_calibration()).

KOERS-CONVENTIE: zoals een kompas en de rest van de code (stepper_ramp,
rotate_deg_gyro): 0 = neus naar het noorden, RECHTSOM positief, 90 = oost.
(Tot 2026-10-26 telde dit bestand linksom: atan2(-my, mx) met +gz.)

Complementair filter met kantelcompensatie:
- kanteling: de richting van de zwaartekracht ("omhoog", eenheidsvector in het
  karstelsel) wordt met de gyro meegedraaid en langzaam naar de accelerometer
  getrokken. Alleen als de accelerometer te vertrouwen is: |a| dicht bij de
  rust-waarde en de kar draait nauwelijks. Bij optrekken/afremmen en in
  draairamps meet de accelerometer anders een schijnkanteling (tot ~0,2 g
  door de hefboomarm van ~146 mm tot het draaipunt), en bij een inclinatie
  van 66 graden wordt een kantelfout ~2x vergroot tot koersfout.
- koers: de magnetometer wordt op het horizontale vlak geprojecteerd (met de
  gefilterde "omhoog"-vector) en gefuseerd met de gyro-draaisnelheid om de
  verticaal, zodat trillingen en kortstondige magnetische storing worden
  weggefilterd.

Tijdconstanten in SECONDEN, dus onafhankelijk van de updatefrequentie.

Gebruikt deltat.DeltaT voor de tijdmeting tussen twee update()-aanroepen.
"""

from math import atan2, sqrt, degrees, radians, asin
from deltat import DeltaT


class Fusion:
    """Complementair filter dat roll, pitch en (kantelgecompenseerde) heading bijhoudt.

    roll:  positief = linkerkant omhoog (kar helt naar rechts), graden
    pitch: positief = neus omhoog, graden
    heading: 0..360, rechtsom vanaf (magnetisch + declination) noord
    """

    # Magnetische declinatie in graden (oost positief): magnetisch noord ligt
    # in Nederland ~2-3 graden oostelijk van het ware noorden. Laat op 0 voor
    # een magnetische koers.
    declination = 0.0

    def __init__(self, timediff=None, tau_tilt=1.0, tau_heading=2.0,
                 accel_gate=0.05, rate_gate_deg_s=20.0):
        """
        tau_tilt:        s, hoe snel de kanteling naar de accelerometer trekt
        tau_heading:     s, hoe snel de koers naar de magnetometer trekt
        accel_gate:      accelerometer alleen gebruiken als |a| minder dan deze
                         FRACTIE afwijkt van de rust-waarde (eerste meting)
        rate_gate_deg_s: ... en de kar langzamer draait dan dit
        """
        self.tau_tilt = tau_tilt
        self.tau_heading = tau_heading
        self.accel_gate = accel_gate
        self.rate_gate = radians(rate_gate_deg_s)
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.mag_heading = 0.0      # ongefilterd, voor diagnose
        self.accel_used = False     # voor diagnose: telde de accelerometer mee?
        self.deltat = DeltaT(timediff)
        self._up = (0.0, 0.0, 1.0)
        self._g_ref = None
        self._started = False

    def update(self, accel, gyro, mag, ts=None):
        """accel in m/s^2 (of g), gyro in RAD/s, mag in uT. Geeft de koers terug."""
        ax, ay, az = accel
        wx, wy, wz = gyro
        mx, my, mz = mag

        dt = self.deltat(ts)
        g = sqrt(ax * ax + ay * ay + az * az)

        if not self._started:
            # Eerste meting: kanteling direct uit de accelerometer
            ux, uy, uz = (ax / g, ay / g, az / g) if g else (0.0, 0.0, 1.0)
            self._g_ref = g if g else 1.0
        else:
            # Een vector die vast staat in de wereld draait in het karstelsel
            # tegen de kar in: d(up)/dt = up x omega
            ox, oy, oz = self._up
            ux = ox + (oy * wz - oz * wy) * dt
            uy = oy + (oz * wx - ox * wz) * dt
            uz = oz + (ox * wy - oy * wx) * dt
            rate = sqrt(wx * wx + wy * wy + wz * wz)
            self.accel_used = (g > 0 and abs(g - self._g_ref) < self.accel_gate * self._g_ref
                               and rate < self.rate_gate)
            if self.accel_used:
                a = dt / (self.tau_tilt + dt)
                ux += a * (ax / g - ux)
                uy += a * (ay / g - uy)
                uz += a * (az / g - uz)
        n = sqrt(ux * ux + uy * uy + uz * uz)
        ux, uy, uz = ux / n, uy / n, uz / n
        self._up = (ux, uy, uz)

        self.roll = degrees(atan2(uy, uz))
        self.pitch = degrees(asin(max(-1.0, min(1.0, ux))))

        # Kantelcompensatie: projecteer m op het horizontale vlak.
        #   vooruit-horizontaal f = x - (x.up) up,  links-horizontaal l = up x x
        #   (even lang, beide loodrecht op up). Koers = atan2(m.l, m.f): met Y
        #   naar links is m.l > 0 als het noorden LINKS ligt, dus bij een koers
        #   tussen 0 en 180 (rechtsom gerekend).
        mup = mx * ux + my * uy + mz * uz
        m_f = mx - ux * mup
        m_l = my * uz - mz * uy
        self.mag_heading = (degrees(atan2(m_l, m_f)) + self.declination) % 360.0

        if not self._started:
            # Eerste meting: koers direct op de magnetometer zetten
            self.heading = self.mag_heading
            self._started = True
        else:
            # Draaisnelheid om de VERTICAAL (niet alleen gz: klopt ook gekanteld).
            # Linksom (om +Z) is positief, de koers telt rechtsom: dus min.
            w_up = wx * ux + wy * uy + wz * uz
            gyro_heading = (self.heading - degrees(w_up) * dt) % 360.0
            diff = (self.mag_heading - gyro_heading + 540) % 360 - 180  # kortste hoekverschil
            self.heading = (gyro_heading + dt / (self.tau_heading + dt) * diff) % 360.0

        return self.heading
