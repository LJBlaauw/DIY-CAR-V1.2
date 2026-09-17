import machine
import time
import math

class ServoController:
    """Driver for PCA9685 via I2C bus 0 (SDA0=GPIO0, SCL0=GPIO1).

    Includes current measurement on GPIO28 (ADC2) and a software shutdown
    if the configurable current limit is exceeded.
    """

    def __init__(self, addr=0x40, i2c_bus=0, sda_pin=0, scl_pin=1):
        self.addr = addr
        self.i2c = machine.I2C(i2c_bus, sda=machine.Pin(sda_pin), scl=machine.Pin(scl_pin), freq=400000)

        # PCA9685 Registers
        self.MODE1 = 0x00
        self.PRESCALE = 0xFE
        self.LED0_ON_L = 0x06

        # ADC configuration for current sensing (GPIO28)
        self.current_adc = machine.ADC(28)
        self.max_current_ma = 500  # Default software limit

        self.init_pca9685()

        # Internal state for position and speed calculations (4 servos supported)
        self.current_angles = [90.0, 90.0, 90.0, 90.0]

        # Map specific rest configurations for servos (IDs 1 to 4)
        # Format: (rest_angle, min_angle, max_angle, reverse_flag)
        self.servo_cfg = {
            1: (90.0,  0.0, 180.0, False),
            2: (30.0,  0.0, 150.0, False),
            3: (45.0,  10.0, 170.0, True),
            4: (10.0,  0.0,  120.0, False)
        }

    def init_pca9685(self):
        # Reset and configure PWM frequency to 50 Hz for analog/digital servos
        self.i2c.writeto_mem(self.addr, self.MODE1, b'\x00')
        time.sleep_ms(5)

        # Set frequency to 50 Hz: prescale = round(25MHz / (4096 * 50)) - 1 = 121
        self.i2c.writeto_mem(self.addr, self.MODE1, b'\x10') # Sleep bit high
        self.i2c.writeto_mem(self.addr, self.PRESCALE, b'\x79') # 121 in hex
        self.i2c.writeto_mem(self.addr, self.MODE1, b'\x00') # Sleep bit low
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.addr, self.MODE1, b'\xa1') # Auto-increment enabled

    def _angle_to_ticks(self, angle, servo_id):
        # Read constraints and direction orientation from config
        rest, amin, amax, rev = self.servo_cfg.get(servo_id, (90.0, 0.0, 180.0, False))

        # Clamp input angle to physical boundaries
        angle = max(amin, min(amax, angle))

        if rev:
            angle = 180.0 - angle

        # 50 Hz PWM means a 20 ms period. 12-bit resolution = 4096 steps.
        # 1 ms pulse width (0 deg)  = round((1/20) * 4096) = 205 ticks
        # 2 ms pulse width (180 deg) = round((2/20) * 4096) = 410 ticks
        ticks = 205 + int((angle / 180.0) * 205)
        return ticks

    def _set_pwm(self, channel, on_tick, off_tick):
        reg = self.LED0_ON_L + (channel * 4)
        buf = bytearray([
            on_tick & 0xFF,
            (on_tick >> 8) & 0xFF,
            off_tick & 0xFF,
            (off_tick >> 8) & 0xFF
        ])
        self.i2c.writeto_mem(self.addr, reg, buf)

    def read_current(self):
        """Read instantaneous current consumption from the shunt amplifier via ADC."""
        raw = self.current_adc.read_u16()
        voltage = (raw / 65535.0) * 3.3
        # Conversion logic for shunt amplifier (e.g., 1V = 200mA)
        current_ma = voltage * 200.0

        if current_ma > self.max_current_ma:
            print(f"Warning: Current limit exceeded! ({current_ma:.1f} mA)")
            self.emergency_detach()

        return current_ma

    def servo_cur_limit(self, limit_ma):
        """Set a threshold current limit to prevent motor damage or gear binding."""
        self.max_current_ma = max(50, limit_ma)

    def servo_pos(self, servo_id, angle, speed_deg_s=0):
        """Set individual servo positioning with an optional velocity limit constraint."""
        channel = servo_id - 1
        if channel < 0 or channel >= 4:
            return

        if speed_deg_s <= 0:
            ticks = self._angle_to_ticks(angle, servo_id)
            self._set_pwm(channel, 0, ticks)
            self.current_angles[channel] = angle
            self.read_current()
        else:
            # Simple linear ramp trajectory interpolation
            start_angle = self.current_angles[channel]
            delta = angle - start_angle
            duration = abs(delta) / speed_deg_s
            steps = max(1, int(duration * 20)) # 20 Hz update loop

            for s in range(1, steps + 1):
                interp_angle = start_angle + (delta * (s / steps))
                ticks = self._angle_to_ticks(interp_angle, servo_id)
                self._set_pwm(channel, 0, ticks)
                self.current_angles[channel] = interp_angle
                self.read_current()
                time.sleep_ms(50)

    def servo_rest(self, first_id=None, speed_deg_s=20):
        """Return all servos smoothly to predefined secure default rest positions."""
        ids = [1, 2, 3, 4]
        if first_id in ids:
            ids.remove(first_id)
            ids.insert(0, first_id)

        for sid in ids:
            rest_angle = self.servo_cfg[sid][0]
            self.servo_pos(sid, rest_angle, speed_deg_s)
            time.sleep_ms(100)

    def laser_power(self, duty_cycle_pct):
        """Control peripheral expansion laser output on PCA9685 auxiliary channel 7."""
        duty_cycle_pct = max(0.0, min(100.0, duty_cycle_pct))
        off_tick = int((duty_cycle_pct / 100.0) * 4095)
        self._set_pwm(7, 0, off_tick)

    def emergency_detach(self):
        """Force turn off PWM generation signals to instantly stop all current draw."""
        for channel in range(16):
            self._set_pwm(channel, 0, 0)
