# servo_ctrl.py
# RP2040 MicroPython
# - Servos: 1 (GPIO2), 2 (GPIO3), 4 (GPIO5)
# - Laser-PWM op GPIO4 (de voormalige "servo3" pin), deelt PWM-slice => zelfde 50 Hz
# - PWM-tick via GPIO7 (INPUT, FALLING), fysiek verbinden met PWM van servo1 (GPIO2)
# - Mapping servo: 0° -> 2.5% duty, 180° -> 12.5% duty
# - Per-servo rustposities in duty-% (servo1: 4.1%, servo2: 3.6%, servo4: 2.0%)
# - Bij __init__ direct alle servo's naar hun rustpositie
# - servo_pos() is RELATIEF t.o.v. rust en nooit onder rust
# - Servo4: stroom-SETPOINT (PI) én stroom-LIMIT; stroommeting op ADC2 (GPIO28) + EMA filter
# - servo_rest() zonder argument: geordend 4 -> 1 -> 2 -> 3, allemaal @ 20 °/s
#
# Let op:
#   * Verbind GPIO2 (servo1 PWM) met GPIO7 (INPUT met PULL_DOWN) voor de 50 Hz "tick".
#   * Laser via low-side MOSFET (GPIO4 -> gate, 100R in serie, 100k naar GND). Laser en Pico GND gemeenschappelijk.
#   * Laser-PWM deelt slice met servo's: frequentie altijd 50 Hz (set_laser_freq() is no-op).
#
from machine import Pin, PWM, ADC
import micropython
import time

class ServoController:
    def __init__(self, irq_gpio=7):
        # --- Servo pin mapping (PWM outputs) ---
        self.servo_pins = {1: 2, 2: 3, 4: 5}  # GPIO2, GPIO3, GPIO5 (servo3 is laser geworden)

        # --- PWM/mapping ---
        self.pwm_freq = 50           # 50 Hz
        self._dt = 0.020             # 20 ms per tick (aanname bij PWM flank)
        self.min_deg, self.max_deg = 0.0, 180.0

        # Duty (%) mapping: 0°->2.5%, 180°->12.5%
        self._pct_min = 2.5
        self._pct_max = 12.5
        self._pct_span = self._pct_max - self._pct_min  # 10.0%
        self._u16_per_pct = 65535.0 / 100.0

        # --- Per-servo rustposities (% duty) ---
        self.rest_pct = {
            1: 4.1,  # onderste scharnier
            2: 3.6,
            4: 2.0,  # grijper
        }

        # --- Servo4: stroommeting (ADC2 = GPIO28) ---
        self.adc_servo4 = ADC(2)
        self.vref = 3.3
        self.adc_max = 65535.0
        self.shunt = 0.1     # ohm
        self.gain = 14.0     # opamp versterking
        # extra software-EMA op stroom
        self.ema_alpha = 0.2
        self.i_filt_mA = 0.0

        # --- PI voor SETPOINT-modus (stroom) ---
        self.Kp = 0.10
        self.Ki = 0.01           # klassieke "per tick" integratie (eerste versie)
        self.deadband = 20.0     # mA
        self.integral = 0.0
        self.mA_to_deg = 1.0 / 100.0  # 100 mA ≈ 1°

        # --- PI-LIMITER voor LIMIT-modus (positie blijft leidend) ---
        self.LKp = 0.05
        self.LKi_s = 0.3         # [1/s] -> vermenigvuldigd met dt
        self.limit_deadband = 30.0  # mA
        self.limit_integral = 0.0
        self.limit_int_clamp = 6000.0  # mA·s

        # --- Servo-objecten: direct op RUST bij init ---
        self.servos = {}
        for nr, gpio in self.servo_pins.items():
            pwm = PWM(Pin(gpio, Pin.OUT))
            pwm.freq(self.pwm_freq)

            rest_pct = self.rest_pct.get(nr, 4.1)
            rest_deg = self._pct_to_deg(rest_pct)

            state = {
                "pwm": pwm,
                "target_deg": rest_deg,  # doel = rust
                "pos_deg": rest_deg,     # actuele pos = rust
                "speed": 60.0,           # default °/s
                "cur_target": None,      # SETPOINT-modus (mA)
                "cur_limit": None,       # LIMIT-modus (mA)
            }
            self.servos[nr] = state
            pwm.duty_u16(self._pct_to_u16(rest_pct))  # direct naar rust

        # --- Laser PWM op GPIO4 (deelt slice met servo's) ---
        self.laser_pin = 4
        self.laser_pwm = PWM(Pin(self.laser_pin, Pin.OUT))
        self.laser_pwm.freq(self.pwm_freq)  # MOET gelijk zijn aan servo's (zelfde slice)
        self.laser_power(0.0)               # start uit

        # --- Tick via externe PWM op GPIO7 (input) ---
        self._irq_pin = Pin(int(irq_gpio), Pin.IN, Pin.PULL_DOWN)
        self._irq_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._pwm_irq)

    # ---------- Publieke API: Servo ----------
    def servo_pos(self, nr, graden, graden_per_sec=60.0):
        """
        RELATIEF t.o.v. rustpositie. Onder rust NIET toegestaan (clamp).
        Voorbeeld: servo_pos(1, 30) -> ga naar (rust_1 + 30°).
        """
        s = self.servos[nr]
        rest_deg = self._pct_to_deg(self.rest_pct.get(nr, 4.1))
        rel = float(graden)
        if rel < 0:
            rel = 0.0
        target_abs = self._clamp_deg(rest_deg + rel)
        s["target_deg"] = target_abs
        s["speed"] = float(graden_per_sec)

    def servo_rest(self, nr=None, graden_per_sec=60.0):
        """
        - nr = None  -> geordend 4 -> 1 -> 2 -> 3, allemaal @ 20 °/s (laser wordt niet aangeraakt).
        - nr = 1,2,4 -> alleen die servo naar rust @ opgegeven snelheid.
        """
        if nr is not None:
            self._set_rest_target(nr, graden_per_sec)
            return

        # Geordend: 4 -> 1 -> 2 -> 3
        sequence = [4, 1, 2, 3]
        for sid in sequence:
            if sid in self.servos:
                self._set_rest_target(sid, 20.0)
                # wacht (blokkerend) tot 'bijna' target, met timeout
                t0 = time.ticks_ms()
                while not self._at_target(sid, tol_deg=1.0):
                    if time.ticks_diff(time.ticks_ms(), t0) > 5000:  # 5 s safety
                        break
                    time.sleep_ms(10)

    def set_rest_pct(self, nr, pct):
        """Stel rustpositie (in % duty) per servo in (2.5..12.5%)."""
        pct = self._clamp_pct(pct)
        self.rest_pct[nr] = pct

    # SETPOINT-modus (exclusief met LIMIT)
    def servo_cur(self, mA, graden_per_sec=60.0):
        """Regel servo 4 op ingestelde stroom (mA); positie wordt genegeerd door PI."""
        s = self.servos[4]
        s["cur_limit"] = None
        s["cur_target"] = float(mA)
        s["speed"] = float(graden_per_sec)
        self.integral = 0.0

    def stop_cur(self):
        """Zet stroom-SETPOINT uit voor servo 4 (PI uit, integrator reset)."""
        s = self.servos[4]
        s["cur_target"] = None
        self.integral = 0.0

    # LIMIT-modus (exclusief met SETPOINT)
    def servo_cur_limit(self, mA):
        """Schakel LIMIT-modus in op servo 4 met nieuwe limiet (reset limiter-integrator)."""
        s = self.servos[4]
        s["cur_target"] = None
        s["cur_limit"] = float(mA)
        self.limit_integral = 0.0

    def update_cur_limit(self, mA, reset_integrator=False):
        """
        Pas de stroomlimiet aan tijdens runtime (zonder of met reset van limiter-integrator).
        Default: geen reset.
        """
        s = self.servos[4]
        s["cur_target"] = None
        s["cur_limit"] = float(mA)
        if reset_integrator:
            self.limit_integral = 0.0

    def clear_cur_limit(self):
        """Schakel LIMIT-modus uit voor servo 4."""
        s = self.servos[4]
        s["cur_limit"] = None
        self.limit_integral = 0.0

    # ---------- Publieke API: Laser ----------
    def laser_power(self, percent):
        """
        Stel laser-vermogen in via duty-cycle (0..100%). PWM deelt slice met servo's -> 50 Hz.
        Gebruik MOSFET low-side, niet direct aan GPIO hangen.
        """
        if percent < 0.0: percent = 0.0
        if percent > 100.0: percent = 100.0
        duty = int((percent / 100.0) * 65535)
        self.laser_pwm.duty_u16(duty)

    def laser_off(self):
        self.laser_power(0.0)

    def set_laser_freq(self, hz):
        """No-op: laser deelt PWM-slice met servo's en moet dezelfde freq houden (50 Hz)."""
        return

    # ---------- Opruimen ----------
    def close(self):
        try:
            self._irq_pin.irq(handler=None)
        except:
            pass
        for s in self.servos.values():
            try:
                s["pwm"].deinit()
            except:
                pass
        try:
            self.laser_pwm.deinit()
        except:
            pass

    # ---------- IRQ → scheduled update ----------
    def _pwm_irq(self, pin):
        try:
            micropython.schedule(self._scheduled_update, 0)
        except Exception:
            self.update_all(None)

    def _scheduled_update(self, _):
        self.update_all(None)

    # ---------- Kernupdate ----------
    def update_all(self, _t):
        # één keer per tick stroom lezen & filteren (servo 4)
        raw = self.adc_servo4.read_u16()
        i_meas = self.adc_to_current_mA(raw)
        self.i_filt_mA = (1 - self.ema_alpha) * self.i_filt_mA + self.ema_alpha * i_meas

        for nr, s in self.servos.items():
            pos = s["pos_deg"]
            target = s["target_deg"]

            if nr == 4:
                # --- SETPOINT-modus: PI op stroom ---
                if s["cur_target"] is not None:
                    err = s["cur_target"] - self.i_filt_mA
                    if abs(err) < self.deadband:
                        err = 0.0
                    self.integral += err  # "per tick" integratie
                    correction_mA = self.Kp * err + self.Ki * self.integral
                    target = self._clamp_deg(pos + correction_mA * self.mA_to_deg)

                # --- LIMIT-modus: positie leidend, begrenzen boven limiet ---
                elif s["cur_limit"] is not None:
                    over = self.i_filt_mA - s["cur_limit"]
                    if over > self.limit_deadband:
                        self.limit_integral += over * self._dt  # Ki op 1/s
                        if self.limit_integral > self.limit_int_clamp:
                            self.limit_integral = self.limit_int_clamp
                        lim_out = self.LKp * over + self.LKi_s * self.limit_integral
                        bias_deg = lim_out * self.mA_to_deg
                        if target > pos:
                            target = max(pos, target - bias_deg)
                    else:
                        # onder limiet: integrator langzaam ontladen
                        self.limit_integral *= 0.9

            # --- Snelheidsbeperking (ramp) ---
            step = s["speed"] * self._dt
            if step > 0:
                delta = target - pos
                if   delta >  step: new_pos = pos + step
                elif delta < -step: new_pos = pos - step
                else:               new_pos = target
            else:
                new_pos = pos

            s["pos_deg"] = new_pos
            s["pwm"].duty_u16(self._pct_to_u16(self._deg_to_pct(new_pos)))

    # ---------- Helpers ----------
    def _clamp_deg(self, d):
        return self.min_deg if d < self.min_deg else self.max_deg if d > self.max_deg else d

    def _clamp_pct(self, p):
        if p < self._pct_min:  return self._pct_min
        if p > self._pct_max:  return self._pct_max
        return p

    def _deg_to_pct(self, deg):
        deg = self._clamp_deg(deg)
        pct = self._pct_min + (deg / 180.0) * self._pct_span
        return self._clamp_pct(pct)

    def _pct_to_deg(self, pct):
        pct = self._clamp_pct(pct)
        return 180.0 * ((pct - self._pct_min) / self._pct_span)

    def _pct_to_u16(self, pct):
        return int(self._clamp_pct(pct) * self._u16_per_pct)

    def _set_rest_target(self, nr, speed_deg_s):
        s = self.servos[nr]
        rest_deg = self._pct_to_deg(self.rest_pct.get(nr, 4.1))
        s["target_deg"] = self._clamp_deg(rest_deg)
        s["speed"] = float(speed_deg_s)

    def _at_target(self, nr, tol_deg=1.0):
        s = self.servos[nr]
        return abs(s["pos_deg"] - s["target_deg"]) <= tol_deg

    def adc_to_current_mA(self, raw_u16):
        v = (raw_u16 / self.adc_max) * self.vref
        i_a = v / (self.shunt / self.gain)
        return i_a * 1000.0
