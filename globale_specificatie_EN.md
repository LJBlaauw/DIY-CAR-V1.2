# DIY Robot Car — MicroPython on RP2350 (Pico 2 W)

## System description

An autonomously moving robot cart that searches for a light source, drives towards it, picks up an object with a gripping arm and then returns to its starting position.

### Status of values ​​

In this document, design values ​​are indicated where necessary as follows:

- **[MEASURED]** — measured directly on the current robot/hardware.
- **[CALCULATED]** — derived from measured values, datasheet values ​​or software constants.
- **[ASSUMPTION]** — used in the design model, but not yet sufficiently measured.
- **[TO BE VERIFIED]** — hardware or software behavior that has yet to be explicitly tested.

High numerical resolution is not automatically the same quantity as physical accuracy. Wheel slip, tire deformation, sensor bias, mechanical play and calibration remain separate sources of error.

### Work order

1. **LDR scan** — Turn one full turn + overlap (**370°**) in one direction and measure light intensity with two paired LDRs. A complete revolution always contains the maximum; the 10° overlap prevents a peak around 0°/360° from falling on the edge. So no pre-roll (rolling back before the scan) is needed anymore.
2. **Align** — Turn to the found direction via the **shortest path** (turn forward or back, depending on which angle is smaller). The last degrees are always approached in the same direction so that the backlash/backlash remains constant. The end position is adjusted in a closed-loop manner via the LDRs (see below). This corrects angle/alignment errors due to missed steps or mechanical deviation; this is not a detection or correction of general linear wheel slip.
3. **Driving with LDR correction** — Drive to the light source and continuously correct for course deviation along the way: as long as there is a difference between the two LDR values ​​(after gain correction), the cart will adjust towards the brightest side until A ≈ B. Stop at the set distance via the ultrasonic sensor (runs independently in the background, SM4).
4. **Grab** — Control two hinge servos so that the arm reaches the object.
5. **Close gripper** — Servo 4 with current limitation (PI limiter) so that the servo is not overloaded.
6. **Return** — Turn around and return to the starting position via stored cycling odometry/route information; place object. Its accuracy is limited by cumulative odometry error and wheel slip.
7. **GY9250 support** — gyro and magnetometer can support orientation/heading. They do not provide an absolute X/Y position. Software is present, but integration and hardware calibration have not yet been validated.

---

## Hardware

| Component | Quantity | Type | Connection |
|---|---|---|---|
| Stepper motor | 2 | — | Driver: TMC2209 |
| Servo motor (arm) | 2 | MG996R | GPIO2, GPIO3 |
| Servo motor (gripper) | 1 | MG996R | GPIO22 — with current sensor |
| Servo motor (optional/spare) | 1 | MG996R | GPIO4 — wired to PCB, not yet connected/deployed |
| Ultrasonic sensor | 1 | RCWL-1601 | GPIO19 (Echo), GPIO20 (Trig) |
| LDR | 2 | 1 kΩ voltage divider (pull-up to 3V3) | GPIO26, GPIO27 |
| Laser (cross hair) | 1 | — | GPIO15 via MOSFET |
| OLED | 1 | SSD1306 | I2C0: SDA=GPIO0, SCL=GPIO1 |
| Gyroscope/compass | 1 | GY9250 | I2C1: SDA=GPIO10, SCL=GPIO11 — *software provided (`lib/GY9250`, fusion, stepper link); integration and hardware calibration not yet validated* |
| Current sensor gripper | — | 0.1 Ω shunt + op amp (gain 14×) | ADC2 = GPIO28 |
| LED | 1 | WS2812B | GPIO6 |
| WiFi/Bluetooth | 1 | Infineon CYW43439 (onboard Pico 2 W) | internal board connection |

The robot controller is a **Raspberry Pi Pico 2 W** with a **RP2350** microcontroller and an onboard **Infineon CYW43439** wireless controller. In MicroPython this is controlled via the CYW43 driver. The web server/web socket control uses WiFi; the planned software architecture keeps networking and main control tasks on core 0 and reserves core 1 for GY9250/display tasks. This core distribution still needs to be validated as a whole on hardware.

See [hardware/gpio_pinout.md](hardware/gpio_pinout_EN.md) for the full GPIO table and verification status.

### Development environment

| Part | Configuration |
|---|---|
| Robot board | Raspberry Pi Pico 2W |
| Microcontroller | RP2350 |
| Robot firmware | MicroPython for Pico 2 W — **exact version/build yet to be determined** |
| Development PC | Fedora Linux 44 |
| Desktop | KDE Plasma |
| PC Tests | CPython; pure-Python tests from `tests/` |
| Hardware Validation | logic analyzer for PIO/STEP timing; multimeter/measuring setup for power supply and sensor calibration |

> **[TO BE VERIFIED] Reproducibility:** Before final hardware testing, record the exact MicroPython version/build and the CPython version used. The software uses RP2350 specific features such as PIO2 and `rp2.DMA()`.

---

## Software modules

### PIO State Machine assignment

| SM | Module | Function |
|---|---|---|
| SM0 | stepper/stepper.py *or* stepper_ramp.py | Step generator motor A |
| SM1 | stepper/stepper.py *or* stepper_ramp.py | Step generator motor B |
| SM2 | stepper/stepper.py *or* stepper_ramp.py | Pedometer motor A |
| SM3 | stepper/stepper.py *or* stepper_ramp.py | Pedometer motor B |
| SM4 | ultrasonic/ultrasonic.py | Ultrasonic sensor trigger + echo measurement |
| SM8 | LDR/ldr_scan_isr.py | Variable clock for LDR sample timing (PIO2 SM0) |

PIO0 is now full (4 of 4 SMs). `stepper.py` and `stepper_ramp.py` are **alternatives** — they claim the same SMs and the same GPIOs, so always import only one.

### DMA channel assignment

| Channel | Module | Function |
|---|---|---|
| 2 pieces (dynamically claimed via `rp2.DMA()`) | stepper/stepper_ramp.py | Rampatabel → TX-FIFO of SM0 / SM1 |

The RP2350 has 16 DMA channels; `treq_sel = (pio_num << 3) + sm_num` (also correct on RP2350, `DREQ_PIO2_TX0 = 16`).

---

### `lib/stepper/stepper.py`

Dual stepper motor controller based on PIO. Both motors run independently via their own SM and counter SM.

**Constants:**
- **[MEASURED]** `WHEEL_CIRC = 19,1 cm` — wheel circumference. Was 20.94 cm; that value gave **8.8% too short distances** (50 cm commanded → 45.6 cm driven). The loaded rolling circumference of a rubber wheel is slightly smaller than the free circumference, so calibrate this value (see *Odometry calibration* below).
- **[MEASURED]** `TRACK_WIDTH = 13,6 cm` — geometric track gauge center-to-center. Necessary for every rotation and rate calculation.
- `STEPS_REV = 12800` — steps per revolution (**1/64 microstepping**, TMC2209: 200 full steps × 64). MS pins hardwired: **MS1 → GND, MS2 → VCC_IO** *(denoted as +5 V in current documentation; physically verify)* (see [hardware/gpio_pinout.md](hardware/gpio_pinout_EN.md)).
- **[CALCULATED]** `CM_PER_STEP ≈ 14,9 µm/stap`
- **[CALCULATED]** `STEPS_PER_DEG ≈ 159` — steps difference between the wheels per degree change in heading. So resolution is **0.0063°**.
- **[CALCULATED/CONFIG]** `F_PIO = 15 MHz` — 150 MHz sysclk / 10, so an **integer clock divider** (no jitter from the fractional divider). `speed_to_delay()` gives the value **576** at the top speed of 12800 steps/s. The delay loop of this PIO is two instructions (`nop().side(0)` + `jmp(y_dec)`), so each unit is 2 cycles and the **speed resolution is 0.17%**. The STEP pulse remains for 11 cycles = **733 ns**, well above the TMC2209 minimum of ~100 ns.
> The often mentioned 1167 cycles and 0.085% do not belong here but with `_delay_for()` in `stepper_ramp.py`, where the delay loop is one cycle per unit. Don't confuse the two.

> **`OVERHEAD` in `speed_to_delay()`:** remains at the hardware-measured 9. The PIO loop actually costs 13 cycles of fixed overhead while the formula assumes 18; at 15 MHz, that **systematic** deviation at top speed automatically drops from ~2.3% to ~0.43%, so re-measurement is no longer necessary. (Previously this was ~11%; that was estimated too high.) In addition, the cutoff of `int()` is added, a maximum of 2 cycles ≈ 0.17%.

**Public functions:**

| Function | Description |
|---|---|
| `mov(dir, speed, dist)` | Both engines simultaneously. `dir='f'/'b'`, speed in cm/s, dist in cm |
| `s1(dir, speed, dist)` | Engine A | only
| `s2(dir, speed, dist)` | Engine B | only
| `rotate(dir, speed, dist)` | Turn on the axle. `dir='l'/'r'`, dist in cm (wheel arc length) |
| `stop()` | Stop both SMs immediately |
| `enable()` / `disable()` | Turn driver enable on/off |
| `status()` | Print position and status of both engines |
| `distance()` | Print and return PIO step distance (cm) |
| `reset_PIO_distance()` | Reset hardware counters to zero |
| `pio_pos1()` / `pio_pos2()` | Read number of steps motor A/B from PIO |

**Step Generator (PIO) Operation:**
- `pull(noblock)` takes a delay value from the TX FIFO (or reuses the previous one).
- The delay determines the step frequency → speed.
- The counter SM counts STEP edges and generates an IRQ when the target is reached.
- IRQ handler stops the associated engine SM and updates the software position.

**Limitation:** there is no disaster. The speed is commanded in one go, so from a standstill the rotor must jump to the final speed within one microstep (78 µs). That is infinite acceleration: the rotor cannot follow the field, loses synchronism and the motor remains humming. This is a **synchronism error, not a torque error** — see the torque budget below. `stepper_ramp.py` solves this.

---

### `lib/stepper/stepper_ramp.py`

Dual stepper motor controller **with ramp** (PIO + DMA) and continuous course correction. Replaces `stepper.py`.

It is an **API largely compatible successor, not a drop-in replacement.** Named `mov`, `s1`, `s2`, `rotate`, `stop`, `enable`, `disable`, `status`, `distance`, `pio_pos1/2` and `reset_PIO_distance` still exist with the same meaning, but during migration these are the breakpoints:

| Difference | Consequence |
|---|---|
| No more global `sm0`..`sm3` | code that comes directly there breaks |
| The generator SMs remain **permanently active** and reside on an empty FIFO | waiting for `sm0.active()` no longer works; use `busy()` |
| `distance()` prints nothing and returns **one signed center distance** instead of two motor distances | callers that extract two values ​​break |
| The motion functions return `True`/`False` instead of implicitly `None` | only relevant if the return value was used |
| Other stop and completion semantics | `halt()` / `brake()` / `emergency_stop()` are now three different things |
| `reset_PIO_distance()` refuses during a movement | first call `halt()` or `brake()` |
| Invalid input returns `ValueError` instead of silently calculating | a negative `dist` was previously made quietly positive |

#### Design basis

| Greatness | Value |
|---|---|
| Mass cart | **[MEASURED]** 1636 g |
| Wheel circumference / radius | **[MEASURED/CALCULATED]** 19.1 cm / 3.04 cm |
| Track width (center-to-center) | **[MEASURED]** 13.6 cm geometric |
| Engine | 17HS8401 — NEMA 17, 1.7 A, 52 N·cm, 1.8 Ω, 3.2 mH, rotor 68 g·cm² |
| Driver | TMC2209, standalone, **stealthChop**, 1/64 microstepping |
| VREF / motor current | **[CALCULATED, TO BE VERIFIED]** 1.0 V → approx. **0.71 A RMS** according to current driver assumptions; check sense resistor/driver version |
| Motor voltage | 24 V (6S Li-ion, 25.2 V full → ~18 V empty) |
| Achieve speed | **[DESIGN VALUE, TO BE VERIFIED]** start 0.1 rev/s = 1.91 cm/s · max 1.0 rev/s = 19.1 cm/s |

**Linking budget [CALCULATED]** — this budget supports the choice for a disaster and shows a large linking margin on paper; the actual margin should be confirmed with the current motor current and load on hardware:

| Mail | Torque per wheel |
|---|---|
| Acceleration of 1636 g at 55 cm/s² | 1.37 N·cm |
| Rotor inertia (68 g cm²) | 0.013 N·cm |
| Rolling resistance (well estimated) | ~1.2 N·cm |
| **Total required** | **~2.6 N·cm** |
| **Available at VREF 1 V** | **~22 N·cm** → factor 8 margin |

The item "accelerate" is `m·a/2 × r` = 1.636 kg × 0.55 m/s² ÷ 2 wheels × 0.0304 m; the wheels' own inertia is not yet present, but that is not interesting at this margin.

At 22 N·cm and wheel radius 3.04 cm, the pulling force is 7.2 N per wheel (14.5 N total) against a weight of 16.1 N: the **wheels slip rather than the engine lacking torque**. Therefore, VREF = 1.0 V is an appropriate design setting for the time being (1.8 W dissipation instead of 10.4 W at full power). stealthChop is sufficient: the practical ceiling is ~300 rpm and the cart rotates at 60 rpm. The SPREAD pin does not need to be connected.

#### FIFO word format

Each 32-bit word in the TX-FIFO encodes a whole **segment** instead of one step:

| Bits | Contents |
|---|---|
| 15..0 | number of steps in this segment − 1 (max 65536) |
| 31..16 | delay per step in PIO cycles (max 65535) |

As a result, a ramp of 2200 steps only costs 256 words = **1 KB**. One 32-bit word per step would cost 2200 × 4 = **8800 bytes ≈ 8.6 KiB per ramp**. Four separate tables (up/down for two engines) would together require approximately **34.4 KiB**, excluding Python object overhead. Segment coding greatly reduces this. A cross phase up to **97.8 cm** fits in a single word.

The PIO loop is 7 instructions; fixed overhead **5 cycles per step** (0.43% at top speed).

#### Phases of a Movement

| Phase | Source | CPU costs |
|---|---|---|
| Disaster on | DMA table, 256 words | **0** — ~2 ms segments are too fast for MicroPython; DMA is required here and immune to GC breaks |
| Cross phase **without** adjustment | same DMA transfer, 1 word | **0** |
| Cross phase **with** adjustment | CPU pushes 1 word per 20 ms per engine | ~0.1% (≈100 `put()`/s) |
| Disaster | DMA table, 256 words | **0** |

A complete 50 cm move without adjustment is **513 words = 2052 bytes in one DMA transfer** — the same zero-overhead behavior as the old concept, with ramp. There is **no interrupt**: `stepper_ramp.py` does not configure a DMA or PIO IRQ anywhere. During execution, the CPU does nothing, and completion is polled with `busy()`. (The old `stepper.py` above did have `sm2.irq`/`sm3.irq`.)

**With an empty FIFO, `pull(block)` stalls with STEP low.** Consequences:
- the motor holds its position, no step is lost;
- CPU latency (also a GC pause of tens of ms) affects **step timing** — the PIO generates with hardware precision. Late CPU means the correction comes one slice later, not a timing error;
- if the FIFO is really empty, the result is a short pause, not a glitch. This is neatly degrading failure behavior, unlike a CPU timed pulse generator.

With 3 slices ahead in the FIFO, the runway is **60 ms** and the control latency is also ≤60 ms.

**DMA and CPU should never write to the same FIFO simultaneously** — the order would be mixed up and the two engines could have different segment orders, making both the ramp and the course unreliable. Therefore:

- during ramp-up the CPU does not push anything; `Move.service()` waits until `dma.active()` of **both** engines is `False`. The DMA has then stopped *writing* while the FIFO still contains data — just the head start the CPU needs.
- the ramp-off DMA will not be started until the CPU has stopped pushing.

> Waiting until the ramp steps have also been *executed* would be wrong: then the FIFO will run empty and the engine will pause between ramp and cruise phase.

**Bridge segment.** When the ramp-up DMA has finished writing, there are a maximum of 4 words left in the FIFO. At the end of the ramp we are at top speed, so those 4 words together are only ~3 ms — with `service()` every 10 ms the FIFO would still empty. Therefore, the ramp-up table ends with one **bridge segment** at a cruise speed of `BRIDGE_SLICES × SLICE_MS` = 40 ms. These steps are part of the cross phase but are not adjusted; a correction 40 ms earlier or later makes no difference.

**S-curve:** the speed follows a smooth step (3p²−2p³) in the distance traveled, so the acceleration is zero at the beginning and end of the ramp — no torque shock at the transitions. Doesn't cost anything extra, because the table is generated in Python and then only played back by DMA. `ACCEL_CM_S2 = 55` determines the ramp *distance* (3.28 cm); the peak acceleration is 1.5× = 82 cm/s² = 0.084 g and the ramp *lasts* 0.55 s.

A movement shorter than 2 × 3.28 = 6.6 cm does not reach top speed and automatically gets a triangular profile.

#### Exact distance

Adjustment changes **when** steps occur, not **how many**. `committed` is kept for each engine: the sum of all written `repeat` values. The braking point is determined at `committed`, not at a measured position, so the **commanded average step total is exact** — regardless of when the control loop happens to call. The *physical* distance is not: microstepping accuracy, wheel slip, tire deformation and the calibration of `WHEEL_CIRC` are still above that. `repeat_A + repeat_B = 2 × base` applies to each slice, causing the center of the cart to move exactly `base` steps while the difference changes the course.

Both motors receive the same slice duration** (20 ms) and a different number of steps per slice; as a result, they remain synchronous in time (**calculated** rounding error < 20 µs at 20 ms; not yet measured with a logic analyzer).

#### Odometry — signed, because the counter SM does not know the DIR pin

The counter SMs count **STEP edges** and know nothing about the direction. A rough pulse counter is therefore not a position: when rotating on the spot, both counters increase positively while the wheels rotate in opposite directions. In addition to the monotonous pulse counter, each motor therefore maintains a **signed position** that takes the sign with each change of direction.

| Function | Meaning |
|---|---|
| `pio_pos1()` / `pio_pos2()` | monotone pulse counter (unsigned), used by `busy()` |
| `_Motor.travel()` | signed position in steps: forward positive, backward negative |
| `distance()` | `(travel_A + travel_B) / 2 × CM_PER_STEP` — a rotation gives ~0 cm, backwards counts negative |
| `heading()` | `(travel_A − travel_B) / STEPS_PER_DEG` — also works with a rotation at the position |

> **Note:** the counters count STEP pulses/commands, not guaranteed physical wheel movement. In the event of slip or a missed motor step, the odometry may therefore deviate. The GY9250 provides independent measurement of rotational speed and orientation, but not absolute linear X/Y position; linear slip of both wheels requires an external position reference to detect directly.

#### Course correction — cascade LDR + gyro

Change in direction comes from a **difference in number of steps** between the wheels. In fire-and-forget, where both engines are locked to the same total, a difference in speed gives a net **zero** change in course (the cart arcs and returns in the same direction). In the cruising phase this is not fixed at all, so a speed difference does integrate into a permanent change in course.

| Loop | Sensor | Frequency | Roll |
|---|---|---|---|
| Outside | LDR A/B difference | single Hz (1× per 5 slices) | determines **where** we should go; A ≈ B = right to the source. Provides the setpoint for the rotation speed |
| Inside | GY9250 gyro-Z | each slice (~50 Hz) | suppresses **disruptions**: bumps, grooves, uneven floor, wheel slip. Adjusts the difference between desired and measured rotational speed |

This is **not dual control**: the LDR determines the direction, the gyro only the interference suppression.

Pay attention to what the inner loop does and doesn't do. It compensates for the difference between desired and measured **turning speed**, so it dampens a disturbance *as long as it occurs*. It does not integrate an angular error, so the twist that comes out at the bottom of the line is not subsequently reversed; the LDR outer loop does that. Without LDR (`damp_yaw_rate()`) there is **no course keeping** — only damping, with permanent drift due to gyro bias. With a briefly covered light source, the cart continues to go approximately straight, but it does not return to the old course.

The **magnetometer/compass** is deliberately not used while driving - the stepper motors disturb the field (see the GY9250 stepper motor calibration). The compass is for the return journey, where an absolute heading is needed.

The **accelerometer** is not used for slip detection. That idea — "stepping without acceleration means slipping" — doesn't work: At constant speed, forward acceleration is zero by definition, so normal driving and full skidding look identical.

What it does contain is a **course tracking error** on gyro-Z: if the measured rotational speed persistently deviates from what was actually written, then `HeadingController.yaw_tracking_error` is set to `True`. That's a *clue*, not slip proof — control dynamics, gyro delay, saturation and wrong gain give the same picture. Linear slip (both wheels straight ahead) cannot be seen anyway; that requires an external position reference.

**Steering authority:** at a speed differential of ±20%, the turning speed is ±32 °/s; at ±5% this is ±8 °/s. Resolution 0.0063° (1 step difference). That 32 °/s applies **at top speed**: the authority is a fraction of the driving speed and therefore scales proportionally (at 5 cm/s still 8.4 °/s). `turn_authority_deg_s(rate)` calculates that and `Move()` sets the ceiling of `HeadingController`.

#### Why not vary the PIO clock

The alternative — modify `SMn_CLKDIV` at runtime — is deliberately **not** used, even though it is technically possible via `machine.mem32`:
- it is outside the data path, so out of sync with the segment boundaries;
- the step numbers are then no longer known exactly;
- the delay values ​​are in PIO cycles, so a clock change rescales the ramp table along the way (and the acceleration scales by f²).

The clock divider remains usable as a global **speed override** (everything slower, e.g. near an obstacle).

#### Public API

| Function | Description |
|---|---|
| `mov(dir, speed, dist)` | Both engines, fire-and-forget with ramp, zero CPU overhead |
| `s1(dir, speed, dist)` / `s2(...)` | Ditto, one engine |
| `rotate(dir, speed, dist)` | Turn on the axle, `dist` = arc length per wheel in cm |
| `rotate_deg(graden, speed)` | Turn on the shaft through an angle. Positive = right |
| `drive(dist, speed, correction=fn)` | Movement **with** continuous adjustment; returns a `Move` |
| `adrive(...)` | asyncio variant of `drive()` |
| `Move.service()` | Complete the FIFOs and apply the correction. Call ≥1× every 10 ms. `False` = done |
| `Move.finish()` | Abort the movement and brake properly — also during the ramp, where the running DMA is first transactionally deleted and then braked from the actual speed |
| `HeadingController(ldr_diff, gyro_rate)` | Cascade course controller. Give the **controller itself** to `correction=`: then `Move()` sets his steering authority to the actual driving speed and he receives feedback about what has been written |
| `gyro_z_deg_s(sensor, sign=None)` | **Required** around the GY9250: the driver delivers radians/s, the controller calculates in degrees/s. Applies `GYRO_Z_SIGN` |
| `damp_yaw_rate(gyro_rate)` | Rotation speed damping on the gyro only. **Does not hold course** — therefore previously incorrectly called `hold_heading()`; see the cascade explanation above |
| `turn_authority_deg_s(rate)` | Maximum turning speed at a given driving speed |
| `halt()` (= `stop()`) | Immediate stop, drivers remain **on**: the motors hold their position. This is the behavior of `stepper.stop()` |
| `brake()` | Neat stop: slows down from the **estimated** current speed (from progress through the profile) |
| `emergency_stop()` | Emergency stop: drivers **off**, DMA silent, FIFOs empty. Motors run freely, the cart can roll
| `distance()` / `heading()` | Signed odometry from the hardware counters. `heading()` is positive to the right, the same sign as `rotate_deg()` and as the adjustment |
| `creep(dist, speed)` | Small correction movement; the only function with a **signed** distance |
| `busy()` | True as long as not all written steps have been sent |
| `stopping_distance_cm(speed)` | Distance that the cart still travels after `finish()`: braking ramp + what is already in the FIFOs |
| `info()` | Print all derived design numbers (no hardware required) |
| `meet_frequentie()` | Measure actual STEP frequency against `_delay_for()`, to verify `CYCLES_FIXED` |

**Start braking in time.** `finish()` does not stop immediately — the braking ramp and the slices already written are fixed:

| Speed ​​| braking disaster | in FIFO | **committed** | + ultrasound latency (50 ms) |
|---|---|---|---|---|
| 19.1 cm/s | 3.28cm | 1.15cm | **4.43cm** | 0.96cm |
| 10 cm/s | 0.88cm | 0.60cm | **1.48cm** | 0.50cm |
| 5cm/s | 0.19cm | 0.30cm | **0.49cm** | 0.25cm |

```python
doel = approach.brake_target_cm(snelheid)   # lib/gripper/approach.py
cm, kind = ultrasoon.read_cm()
if kind == 'ok' and cm <= doel:
    mv.finish()
```

Therefore, slow down to 5 cm/s for the last ~25 cm: that brings the stopping uncertainty from 5.4 cm to 0.74 cm, and that is more robust than trying to predict 4.43 cm exactly.

> A full step-by-step explanation of PIO/DMA/FIFO operation and control can be found in [stepper_ramp.md](stepper_ramp_EN.md).

Invalid input is rejected with `ValueError` (unknown direction, speed ≤ 0, acceleration ≤ 0, non-finite numbers). A zero distance returns `False` without touching the hardware — a DMA transfer with `count=0` is firmware-dependent behavior and is avoided. A new command **overwrites** an ongoing movement (no blocking until it is completed).

A requested speed below `V_START_CM_S` is respected and does not cause disaster: the starting speed is an *upper limit* for safe starting, not a minimum.

#### Key figures

| | |
|---|---|
| Top speed 12800 steps/s | delay 1167 cycles, resolution 0.085 % |
| Starting speed 1280 steps/s | delay 11714 cycles |
| Disaster | 2200 steps = 3.28 cm, 256 segments, max speed jump 1.78% per segment |
| Cross-slice | 20 ms = 256 steps |
| 360° on site | 28632 steps per wheel = 2.24 wheel revolutions = 2.24 s at 1 rev/s |
| Emergency stop | `ENA` high + `dma.active(0)` + `sm.init()` to clear the FIFO |

#### Odometry calibration (to be performed)

Two calibrations, to be included in the calibration session. Without this, the cart will steer structurally askew, no matter how good the disaster is:

1. **Distance scale** — drive a measured 1.00 m, measure the actual distance traveled, correct `WHEEL_CIRC`. Captures both the residual measurement error and the loaded roll circumference.
2. **Rotation scale** — command exactly 360° (28632 steps per wheel, opposite), measure the residual angle with the GY9250, correct `TRACK_WIDTH`. The effective track width is usually 1–5% larger than the geometric 136 mm due to tire scrub.

#### Course tracking error (formerly "slip detection")

`HeadingController.yaw_tracking_error` is set if the **measured** rotational speed deviates persistently (10 ticks) by more than 8 °/s from the rotational speed that was **actually written**.

Two things where this went wrong before:

- **Compare to what actually happened, not to the setpoint.** Steering authority is a fraction of the vehicle speed, so at low speed there is much less authority than the ±32°/s at top speed. When compared against the clamped setpoint, the controller could "command" 25°/s while the wheels could only physically deliver ~8°/s — and that difference read as slip. `Move()` therefore sets the ceiling at `turn_authority_deg_s(r1)` and reports back via `note_applied()` what has actually been written after rounding and capping.
- **A deviation is not proof of slip.** Control dynamics, gyro delay, saturation and an incorrectly adjusted gain give exactly the same picture. The flag is therefore called `yaw_tracking_error` and not `slipping`.

> What doesn't work: "motor active + little forward acceleration". After all, at constant speed the forward acceleration is zero, so that test would report the entire cruise phase as a slip. **Linear** slip (both wheels slip straight ahead) is basically not visible with wheel odometry and an IMU; this requires an external position reference.

#### Tests

[`tests/test_stepper_ramp_math.py`](tests/test_stepper_ramp_math.py) — **226 pure-Python tests, no hardware required** (`machine` and `rp2` are stubbed, also runs on PC with CPython):

```
python3 tests/test_stepper_ramp_math.py
```

Covered: exact step total of `ramp_words()` / `cruise_words()` / `profile_words()` over the entire distance range (0 to 200,000 steps), monotonic speed in both ramp directions, delay and repeat fields within range, triangle profile, zero distances, input validation, the slice arithmetic (equal duration, exact center), the signed odometry in rotation and reverse, `Move.finish()`, and the DMA → CPU transition (that the CPU does not push while the DMA is still writing).

Also regressed were the three state errors included: `finish()` during the ramp-up DMA (where `busy()` remained `True` forever), a new command during an ongoing move (DIR and counters cycled without stopping first), and the sign convention (`rotate_deg(+90)` gave `heading() = −90`).

[`tests/test_gripper_geometry.py`](tests/test_gripper_geometry.py) — 32 gripper geometry tests, also without hardware.

#### Still to be verified on hardware

- **Three separate sign conventions.** They correct different errors and therefore do not belong in one constant — a reverse-mounted GY9250 requires a different gyro sign without anything wrong with the motor wiring. Public applies everywhere: **positive = to the right**.
- `MOTOR_TURN_SIGN` (+1 / −1): which physical engine is on the left or right. Test: `rotate_deg(+90)` should turn to **right** and `heading()` should then give ≈ +90.
- `GYRO_Z_SIGN` (+1 / −1): wheels raised, turn cart clockwise by hand — `gyro_z_deg_s(sensor)()` must be **positive**.
- `LDR_DIFF_SIGN` (+1 / −1): keep a lamp to the right of the cart — `ldr_diff()` must be **positive**.
- `CYCLES_FIXED = 5`: the fixed PIO cycle overhead **per step within a segment** (`mov[2]` = 3, delay loop = delay + 1, `jmp(y_dec)` = 1). Per **segment transition**, 4 more cycles are added for `pull`/`out`/`out`/`mov`, which are not included in the formula; the effective overhead is therefore `5 + 4/repeat`. For the 256 disaster segments this is 0.02%, for segments of 1 step it is 0.4%. The value is **calculated, not yet measured**. `meet_frequentie()` compares the actual STEP frequency with `_delay_for()`; measure long segments as well as segments of 1, 2, 8 and 256 steps, and also check the pulse width (expected 200 ns) and whether any steps are lost with a **logic analyzer**.
- Maximum take-off speed and maximum acceleration, with the GY9250 as independent reference (the PIO counter cannot see a stall). The assumed 0.1 rev/s is conservative — 20 full steps/s is well within the pull-in range of any NEMA 17.
- Control Gains `kp_ldr` and `kp_gyro`. The standard `kp_ldr = 25` gives 25°/s difference at full LDR, just below the ceiling of 32.2°/s, so the controller normally does not saturate.
- `fifo_join=PIO.JOIN_TX` is **not** used (not verified in this MicroPython version). If it works, the FIFO runway doubles from 60 to 140 ms.

---

### `lib/servo/servo_crl.py`

`ServoController` class for 3× MG996R servo (servo 1, 2, 4) + cross hair laser.

**Init:** `sc = ServoController()` — all servos immediately go to rest position.

**Rest positions (duty %):**

The final rest positions are set automatically or manually and stored in `config.json`. The historical fixed values ​​below are for reference only.

<!-- Historical, no longer normative:
| Servo | GPIO | Rest (duty%) | Function |
|---|---|---|---|
| 1 | GPIO2 | 4.1% | Lower hinge arm |
| 2 | GPIO3 | 3.6% | Upper hinge arm |
| 4 | GPIO22 | 2.0% | Grab |
-->

| Servo | GPIO | Function |
|---|---|---|
| 1 | GPIO2 | Lower hinge arm |
| 2 | GPIO3 | Upper hinge arm |
| 4 | GPIO22 | Grab |

**PWM tick:** GPIO5 (INPUT) must be physically connected to GPIO2 (servo1 PWM). The falling edge triggers the servo update ISR @ 50 Hz.

**Public Methods:**

| Method | Description |
|---|---|
| `servo_pos(nr, graden, graden_per_sec)` | Relative to rest (never at rest). Example: `servo_pos(1, 30)` → rest+30° |
| `servo_rest(nr=None)` | Back to rest. Without argument: order 4→1→2 @ 20°/s |
| `set_rest_pct(nr, pct)` | Set rest position as duty-% |
| `servo_cur(mA, graden_per_sec)` | **SETPOINT mode**: PI controller keeps servo 4 at set current |
| `servo_cur_limit(mA)` | **LIMIT mode**: position is leading, current is limited by PI limiter |
| `update_cur_limit(mA, reset_integrator)` | Adjust power limit at runtime |
| `clear_cur_limit()` | Turn off LIMIT mode |
| `stop_cur()` | Turn off SETPOINT mode |
| `laser_power(percent)` | Laser duty cycle 0–100% via MOSFET |
| `laser_off()` | Laser off |
| `close()` | Disable all PWM and remove IRQ |

**Current control servo 4 (gripper):**
- Shunt: 0.1 Ω, op amp gain 14×, measured on ADC2 (GPIO28).
- EMA filter (α=0.2) on the measured current.
- SETPOINT mode: PI adjusts position so that current remains at setpoint.
- LIMIT mode: position is leading; PI limiter pulls back if current exceeds limit.

---

### `lib/ultrasoon/ultrasoon.py`

Ultrasonic sensor RCWL-1601 via PIO (SM4). Measure asynchronously in the background with ping-pong buffer.

**Config:**
- `PIO_FREQ_HZ = 2 MHz`
- `INTERVAL_MS = 50` — measurement interval
- `TIMEOUT_US = 30 000` — maximum echo latency (≈ 5 m)

**Public functions:**

| Function | Return value | Description |
|---|---|---|
| `read_us()` | `(us, kind)` | child: `'ok'`, `'timeout'`, `'overflow'` |
| `read_cm()` | `(cm, kind)` | Distance in cm, rounded to 0.1 cm |
| `stop()` | — | Stop the SM |

> **API convention:** `read_us()` and `read_cm()` always return a tuple `(waarde, kind)`. Check `kind == 'ok'` before using the value numerically; with `timeout` or `overflow` the distance may not be entered into the controller as a valid number.

The ldr scan routine now uses its own PIO block (SM8, see table above) instead of the same block as the ultrasonic sensor (SM4) — the RP2350 has 3 independent PIO blocks. This resolves the previously reported PIO conflict between ultrasound and LDR scan; SM4 no longer needs to be stopped during the LDR scan.

---

### `lib/LDR/ldr_scan_isr.py`

LDR scan module. Runs the cart via `stepper.rotate()`, samples LDR A and B synchronously with a PIO clock (SM8 / PIO2 SM0) via ISR.

**Status: still contains known bugs — under development.**

**Scan algorithm (revised):**

1. **Full rotation scan (370°).** Rotate 370° in one direction and sample both LDRs separately. No pre-roll/reverse: a full rotation always contains the maximum and the 10° overlap keeps a peak around 0°/360° from the edge. The 0–10° region appears twice in the buffer; `_find_peak` takes the first (earliest) index.
- *Precondition:* the cart must be able to rotate freely on its axis (differential, `WHEEL_BASE_CM`) without cables that twist at 360°+.
2. **Coarse peak determination per LDR.** The LDRs are horizontally spaced, so two brightness maxima arise; the direction of the source is at the **intersection A ≈ B** (after gain correction), not at the maximum of the sum. See also the “Driving to the Light Source” section.
3. **Shortest way back (idea 2).** After 370° the cart is at the end angle 370°; to peak angle θ is turning back `370 − θ` and turning `(θ − 10) mod 360`. Choose the smallest. Always let the last degrees end in the same direction (e.g. always in the scanning direction; when turning back, go a little past and then go back forward), so that the backlash is constant and calibrated.
4. **Closed-loop end position via null-seek A−B (idea 3).** Rough to the expected peak position in steps (open-loop), then adjust in small steps with `measure_now()`. Do not use the stored maximum magnitude as a threshold (the motion scan smears the peak, and the absolute brightness may have drifted in the meantime), but look for the zero crossing of the difference A−B (after gain correction):
- The difference signal is sharper than the flat sum peak → better angular resolution.
- It is insensitive to absolute brightness drift (if the light dims, the sum drops but the zero remains at the same angle).
- Limit the fine adjustment to a window around the expected position (e.g. ±15°); If you do not find a clear zero crossing within the window, fall back on the step target. This prevents running away due to light changes/noise.
This corrects stepper slip: the cart turns until the LDRs actually measure the aligned condition, not to a counted step count.

**Config:**
- `LDR_PIN_A = GPIO26`, `LDR_PIN_B = GPIO27`
- `WHEEL_BASE_CM = 13,6 cm` — center-to-center track width, for degrees→cm conversion. **Standed at 18.5 cm**, while the measured track width is 13.6 cm; a commanded 370° scan actually rotated **503°**. Must remain the same as `TRACK_WIDTH` in `stepper_ramp.py`; empirical correction (tyre scrub, backlash) belongs in `ROT_SCALE`.
- **[CONFIG]** `LDR_R_FIXED_OHM = 1000` — pull-up to 3V3 (R29/R30). **Was 10 000.** For a 100–200 Ω LDR, 10 kΩ gives approximately **0.99–1.96% full-scale** (approx. 41–80 true 12-bit counts); 1 kΩ gives approximately **9.1–16.7% full-scale** (approx. 372–683 real 12-bit counts). MicroPython `read_u16()` scales this raw ADC value to 16 bits, so 12-bit ADC counts and `read_u16()` counts should not be used interchangeably.
- **[CONFIG]** `LDR_R_MIN_OHM = 20` — percent scale lower limit. **Was 60**, causing the scale close to the source to be stuck at 100% and leaving the end stage with no information.
- **[CONFIG]** `LDR_R_MAX_OHM = 20 000` — unchanged. Over 20 Ω … 20 kΩ the resolution remains 79–1024 actual codes per e-fold, so the entire operating range from 5 m to 5 cm is usable.
- **[TO CALIBRATE]** `LDR_GAIN_B = 1,136` — current calibration factor for differential measurement. **Needs to be recalibrated**: This factor also compensated for the tolerance of the old 10 kΩ resistor pair.
- `TARGET_SAMPLES_PER_DEG = 3` — at 370° ≈ 1110 samples. The scan maintains three `array`s of 1110 words (LDR A, LDR B, stepper mode), together ~13 KB; well within RAM.

**Divider topology:** `R_FIXED` is the **pull-up** to 3V3, the LDR the pull-down to GND. Bright light → low LDR resistance → **low** ADC value. `_adc_to_res_ohm()` is counting on that. Check: Shine light on LDR A and read the raw ADC; if it goes to **zero**, then the assumption is correct.

**Public functions:**

| Function | Description |
|---|---|
| `scan(dir, speed_cm_s, graden, start_graden, go_max, excel, out_csv)` | Run scan. Returns dict with results and peak position |
| `measure_now(n=8)` | Directly read LDR value (%, tuple A/B) |
| `attach_stepper_reader(fn)` | Link stepper.pio_pos1 as position source for the scan |

**`scan()` returns:**
```python
{
  'samples': int,        # aantal gemeten samples
  'tick_ms': int,        # gebruikte sample-interval
  'est_time_s': float,   # geschatte scanduur
  'total_deg': float,
  'dist_cm': float,
  'peak_index': int,     # index of light maximum
  'peak_percent': float, # lichtsterkte op maximum (%)
  's1_at_peak': int,     # stapperstand op maximum
  'backtrack': dict,     # terugrijinformatie
  'csv_path': str,       # path to CSV (None if not written)
  'csv_error': str,      # error message (None if OK)
}
```

---

### Driving to the light source — positioning on the beam axis

**Objective: the cart positions itself *right in front* of the light source**, `STOP_DIST_CM = 13,3 cm` (ultrasonic) from the object — at the default `OBJECT_W_CM = 6,0 cm`. Do not round this off to 13 cm: with a **design goal** for the final accuracy of ~0.3 cm, that rounding eats up the entire error budget. That's more than looking at it, and that difference determines the entire algorithm.

#### Two independent quantities, two signals

The light source is a **beam source** (slit), not an omnidirectional radiator. As a result, there are two degrees of freedom that you have to arrange separately:

| Signal | Measure | Target | Controller |
|---|---|---|---|
| **A − B** | the *poll* to the source | zero → look straight at it | `HeadingController`, per slice |
| **Q** (see below) | the *position* relative to the beam axis | maximum → stand right in front of it | side step between legs |

> **A − B says nothing about your position relative to the beam axis.** You can be perfectly aligned (A = B) and still receive almost no light because you view the source diagonally from the side. If you just zero the bearing, the cart drives a chase curve: it always looks at the source but keeps its lateral deviation. That is not a control problem but an **observability problem** — no amount of reinforcement on A − B will solve it, because the information is not there.

#### Normalized brightness Q

Received light is `E = I(φ)/d²`, with `φ = atan(y/d)` the angle at which the source sees the cart and `y` the lateral deviation from the beam axis. Normalize the distance with the ultrasound:

```
Q = −(1/γ)·ln(R) + 2·ln(d)          ( = ln I(φ) up to a constant )
```

where the light intensities of both LDRs are **added as `R_A^(−1/γ) + R_B^(−1/γ)`** — not the resistances averaged, because that scale is logarithmic and not linear in light.

- **Q constant as you approach** → you are on the beam axis (φ = 0 at any distance).
- **Q decreases as you approach** → you are wrong, because φ is growing. The decline results in `y`.

Without this normalization this cannot be measured: from 100 to 12 cm the 1/d² term already increases by a factor of ~19 (see the table below), and that overshadows any lateral gradient.

#### Why the light close by actually decreases

With the current **[ASSUMPTION/PROVISIONAL ESTIMATE]** `w ≈ 33°` (1/e half angle) and a lateral deviation of 20 cm:

| Distance | φ | `I(φ)` | 1/d² | received light |
|---|---|---|---|---|
| 100cm | 11.3° | 0.89 | 1.0× | 0.33 |
| 60cm | 18.4° | 0.73 | 2.6× | 0.70 |
| **40cm** | 26.6° | 0.52 | 5.2× | **1.00 ← peak** |
| 20cm | 45.0° | 0.16 | 13× | 0.74 |
| 12cm | 59.0° | 0.041 | 19× | 0.29 |

The column `1/d²` is standardized to the row of 100 cm and calculates with the **oblique** distance √(d² + y²), not with `d` alone; the last column is `I(φ)` × that factor, normalized to the peak.

The beam drop beats 1/d². Rule of thumb: **the received light peaks around `d ≈ 2y` and then collapses.**

Conversely, **if you are on the beam axis, the signal actually grows as you approach.** The blindness problem only exists as long as you are at an angle — so getting on the axis at medium distance solves it completely.

#### Calculate `y` from one straight measuring leg

```
Q₁ − Q₂ = (atan(y/d₂)/w)² − (atan(y/d₁)/w)²        →  solve for y (bisection)
```

Small-angle approximation for the feeling: `y ≈ w·√( ΔQ / (1/d₂² − 1/d₁²) )`.

One straight ride therefore provides the **size** of the deviation; only the **sign** costs one more dither (arc left/right, see which side increases Q). That is considerably cheaper than iterative gradient climbing.

The leg length comes from the **odometer** (nominal step resolution 14.9 µm — that's the resolution, not the accuracy: microstepping, slip and the calibration of `WHEEL_CIRC` are an order coarser), not from the ultrasound; this is only used for the absolute starting distance. This means that the distance error takes effect once instead of twice — a factor of 1.4 gain.

#### Phase structure

| Phase | Distance | Speed ​​| Signal | Action |
|---|---|---|---|---|
| 1 Search | — | — | A − B, 370° scan | poll to the source |
| 2 Approach + measure | 60 → 45 cm | 19.1 cm/s | ΔQ | calculate `y` |
| 3 Sign | 45cm | 8cm/s | dither | left or right of the axis |
| 4 Sidestep | 45cm | 8cm/s | odometry | calculated correction `y` |
| 5 Repeat | 45→30, 30→20 cm | 19.1 cm/s | ΔQ | refine, `Q` should become flat |
| 6 Braking | 25cm | → **5 cm/s** | — | stopping distance of 4.4 → 0.5 cm |
| 7 Inwards | 20 → 13.3 cm | 5cm/s | A − B zero, Q guard | the signal | grows on the axis
| 8 Stop | 13.3cm | — | ultrasonic | `mv.finish()` on `approach.brake_target_cm()` |

Phases 2 and 5 do not take any extra time — you have to drive that distance anyway.

#### Error budget

Two very different quantities:

- **Heading (where the cart is looking): well better than ±1°.** The A − B zero crossing is sharp, the gyro suppresses interference, and the step resolution is 0.0063°.
- **Lateral position relative to the beam axis: this is the limitation.**

Detection threshold for `y` per measuring leg (leg length from the odometer, ultrasonic ±3 mm, Q-drift 0.01):

| Leg | preliminary model (w ≈ 33°) | model with halved slit (w ≈ 16.5°) |
|---|---|---|
| 60 → 45 cm | 4.55cm | 2.27cm |
| 45 → 30 cm | 3.01cm | 1.50 cm |
| 30 → 20 cm | 2.20 cm | 1.09cm |
| 20 → 14 cm | 1.72cm | 0.86cm |
| **final deviation, one pass** | **≈ 2cm** | **≈ 1cm** |

These numbers come from `nauwkeurigheid()` in [`tests/test_ldr_beam.py`](tests/test_ldr_beam.py).

The final value is set by the last leg on which you can still **adjust**, and that is the leg 30 → 20 cm (2.20 cm, rounded ≈ 2 cm). The leg 20 → 14 cm is only included for information: it can be measured there but cannot be corrected, because the side step must be completed before 20 cm (see *Standing re-measurement*). **The limitation is the ultrasonic and light drift, not the LDR or the ADC** — ADC noise contributes ~0.02 percentage points and is negligible.

How that lateral deviation relates to what the gripper can tolerate is shown in *Grip geometry and final positioning* below. Short: **The current setup is sufficient for an object width of 4 cm; above that the halved gap is needed.**

#### Slit of the light source

| Effect of halving | Factor | Review |
|---|---|---|
| `I(φ)` loses weight 2x faster → Q signal ×4 | **y-precision ×2** | main advantage |
| luminous flux ×0.5 → R of 100 → 162 Ω | | bonus: away from LDR saturation |
| search cone ×0.5 (half value 27.5° → 13.7°) | | risk for phase 1 |

**Set the slit vertically** (high and narrow): then you narrow the beam horizontally, exactly the axis on which you need precision, while remaining vertically wide and forgiving height differences. A horizontal crack does the opposite.

For comparison: halving the gap gains a factor of 2, halving the ultrasonic error only √2. So the gap is the cheapest gain.

#### Preconditions / dependencies

- **`γ` (LDR exponent) and `w` (beam half angle) must be measured** — both are in each formula above. See [`tests/test_ldr_beam.py`](tests/test_ldr_beam.py). The current `w ≈ 33°` comes from one measurement point with an *assumed* `γ = 0,7`; With `γ = 0,9` the beam is wider, with `γ = 0,5` it is narrower.
- **LDR gain calibration** (sensitivity difference A/B); without that correction the cart will steer crookedly.
- **Odometry Calibration** (`WHEEL_CIRC`, `TRACK_WIDTH`).
- **Hardware/code link:** after lowering R29/R30 to **1 kΩ**, `LDR_R_FIXED_OHM` in [`lib/LDR/ldr_scan_isr.py`](lib/LDR/ldr_scan_isr.py) must also become 1000. If it remains at 10000, then each resistance value is a factor of 10 error without anything failing. With a 100 Ω LDR, the signal with 1 kΩ is at **9.1%** of the ADC scale instead of 0.99% — a factor of **9.2** (see the numbers at `LDR_R_FIXED_OHM` above). More important than the level is the sensitivity: the `y` measurement should see a ΔQ of ~0.01, which at 1 kΩ is about **38 u16 LSBs** versus ~4.5 LSBs at 10 kΩ — a factor of 8.4. With 10 kΩ this disappears into the noise.
- `LDR_R_MIN_OHM` is now at **20 Ω** (was 60 Ω). For the beam axis calculation and final phase, working in `ln R` is preferable to a truncated percent scale; check on hardware whether 20 Ω gives sufficient margin against saturation.
- **Grey filter** over the gap when the LDR physically saturates (100 Ω is very low for CdS). No diffuser and no smaller opening — they ruin the directivity.
- **Arbitration with the compass:** during the approach, the **LDR is leading** for the direction; the gyro-Z only does interference suppression (so it is not dual control). The **magnetometer** is not used while driving because the stepper motors disturb the field; that is for the return journey, where an absolute course is needed.

> **Why not more segment-by-segment:** the previous approach (row short segment → stop → measure → `rotate()` → repeat) has been replaced. With segments of 1–2 cm the top speed is never achieved, because the ramp alone is 3.28 cm on plus 3.28 cm off. Furthermore, stop-turn-drive costs orders more CPU and wall clock time than adjusting per slice (~0.1% CPU), and a stepwise rotation cannot suppress a bump or groove.

---

### Gripper geometry and end positioning

The jaws close **horizontally**, but the fingertips move forwards**. Measured:

| | Jaw opening | Tops compared to ultrasonic |
|---|---|---|
| maximum open | 9cm | 12cm |
| almost closed | 2cm | 15cm |

```
tip_pos_cm(opening) = 12 + (9 − opening) × 3/7        →  0.43 cm forward per cm of closing
```

> This is a **straight through two measuring points**. A four-bar mechanism actually produces a curve; one additional measurement at ~5 cm opening shows how much this differs.

#### The ultrasound can only be used with the arm at rest

While driving, the servos are in rest position: the jaws are then **behind** the ultrasound and fall outside the beam, so the distance measurement is accurate. At a half opening angle of 15°, that beam is 6.4 cm wide at 12 cm and **8 cm wide at 15 cm**, and the jaws are open 9 cm — **as soon as the arm unfolds, the sensor looks at your own fingers.** From that moment on, there is no more distance feedback.

#### Three derivative quantities

This geometry was previously in `stepper_ramp.py`. That made a generic motor driver dependent on the dimensions of one grabber and the stopping strategy of one mission; it is now in [`lib/gripper/geometry.py`](lib/gripper/geometry.py) (pure calculations, no imports) and [`lib/gripper/approach.py`](lib/gripper/approach.py) (`finetune()`, `mean_dist_cm()`, `brake_target_cm()`).

| Object width | Stopping distance `stop_dist_cm()` | Grab window `grip_window_cm()` | Lateral tolerance `lateral_tolerance_cm()` |
|---|---|---|---|
| 3cm | 14.6cm | 12.0 – 14.6 (**2.6cm**) | ± 3.0 cm |
| 4cm | 14.1cm | 12.0 – 14.1 (**2.1 cm**) | ± 2.5 cm |
| 5cm | 13.7cm | 12.0 – 13.7 (**1.7cm**) | ± 2.0 cm |
| 6cm | 13.3cm | 12.0 – 13.3 (**1.3 cm**) | ± 1.5 cm |
| 7cm | 12.9cm | 12.0 – 12.9 (**0.9 cm**) | ± 1.0 cm |
| 8cm | 12.4cm | 12.0 – 12.4 (**0.4 cm**) | ± 0.5 cm |

**Stopping distance:** counterintuitive but correct — a *narrower* object requires a *greater* stopping distance. Narrower means closing further, so more progress from the tops, so the cart must remain further back. The default `OBJECT_W_CM = 6.0` gives `STOP_DIST_CM = 13,3 cm`.

**Grab window:** the 3 cm advance is a *free* window on top of the stopping accuracy. The lower limit is conservatively 12 cm; the actual lower limit is determined by the **jaw depth** (palm to tops), which has not yet been measured.

**Lateral tolerance:** the jaws sweep through the space where the object is during unfolding. With a larger deviation, one jaw hits the object and **knocks it over** — a more annoying failure mode than just misgrasping.

> **Suggestion: extend the arm above the object and then lower it.** With two arm hinges (servos 1 and 2) the same end point can be reached via different paths. From top to bottom, the jaws encircle the object instead of entering it horizontally; that completely eliminates the swipe collision and costs no hardware.

#### Lateral accuracy versus gripper tolerance

| Object width | Gripper Tolerance | Preliminary model current beam (≈ ± 2 cm) | Model halved slit (≈ ± 1 cm) |
|---|---|---|---|
| 3cm | ± 3.0 cm | ✓ spacious | ✓ spacious |
| 4cm | ± 2.5 cm | ✓ | ✓ |
| 5cm | ± 2.0 cm | ⚠ on the border | ✓ |
| 6cm | ± 1.5 cm | ✗ too tight | ✓ |
| 7cm | ± 1.0 cm | ✗ too tight | ⚠ on the border |

**Design expectation:** With current model parameters, a halved gap is required for objects wider than 4 cm. This will only become a final system requirement after `γ` and `w` have been measured with `tests/test_ldr_beam.py` and the lateral error on the robot has been validated.

#### Taking a stationary measurement — the last correction moment

Because the beam is free as long as the arm is at rest, a fresh, average measurement can be taken after stopping but before unfolding. Then the driving speed and the measurement latency fall out of error:

| | Uncertainty in distance |
|---|---|
| stop at 5 cm/s (committed + latency) | 0.74cm |
| **stationary re-measurement + creep correction** | **design target ≈ 0.3 cm**; hardware validation needed |

`approach.finetune(read_cm, object_w_cm)` does this (in [`lib/gripper/approach.py`](lib/gripper/approach.py)): average over 8 **independent** measurements (wait time > `INTERVAL_MS` = 50 ms, otherwise you read the same buffered value) and then `creep()` to the target. At 2 cm/s there is virtually no ramp required, so the crawling movement is immediately precise and gentle. After a correction, measurements are **always** taken again, even after the last attempt — otherwise the judgment would be based on the measurement before that correction.

The **lateral** deviation cannot be measured again: the `y` determination from the Q drop requires movement over two distances. Standing still, it can only be verified that A − B is zeroed, and that is the *bearing*, not the lateral position. **The lateral correction must therefore be completed at 20–45 cm.**

#### End sequence

| Step | Action | Ultrasonic | Feedback |
|---|---|---|---|
| 1–7 | approaching, beam axis, braking to 5 cm/s | usable | LDR + gyro + ultrasonic |
| 8 | stop around `STOP_DIST_CM` | usable | |
| 9 | **`finetune()`** — stationary re-measurement, average | usable | last correction moment; design target ± 0.3 cm |
| 10 | **`creep()`** to `stop_dist_cm(breedte)` | usable | |
| 11 | extend arm **above** the object, then lower | **blind** | open-loop |
| 12 | to grasp; tops extend 12 → 15 cm | **blind** | current limitation via PI on ADC2 |

From step 11 onwards everything is open-loop; **steps 9–10 are the last correction moment.** In step 12 there is still feedback via the gripper current sensor: a grip in the air produces a different current curve than a grip around an object. That's the only "success or not" signal, and it's already there.

---

### `tests/test_ldr_beam.py` — LDR and beam characterization

Measurement script for the constants on which the beam axis positioning rests. Everything can be called from the REPL.

| Function | What | Where |
|---|---|---|
| `controleer_config()` | whether code and modified hardware match (runs on import) | — |
| `gamma()` | LDR exponent from 5 distances on the axis; fit `ln R = 2γ·ln d`, reports R² | bank |
| `bundel()` | profile `I(φ)` on an arc with a fixed radius, LDR always pointed at the source | bank |
| `nauwkeurigheid()` | achievable `y` resolution per leg, with and without halved slit | no hardware |
| `meet_y()` | lateral deviation from the Q-descent over one straight journey | on the cart |
| `dither_teken()` | on which side of the beam axis the cart sits | on the cart |

The two bench measurements are set up so that they each **isolate one factor**: `gamma()` keeps φ constant (on axis) and varies only the distance; `bundel()` keeps the distance and tunnel angle constant and only varies φ. `meet_y()` warns if A − B is not zeroed — then you measure tunnel vignetting instead of the beam.

---

### `webserver` — microdot web socket (Pico 2 W)

Web server based on **microdot** (asyncio) with a web socket, so that the cart can be operated and read from a standard browser. Is designed to run on **core 0** in addition to the main control tasks; core 1 is reserved for GY9250+ display. The interaction between MicroPython, CYW43/lwIP, `_thread` and the control loop still needs to be validated under load.

**Functionality:**
- **Read sensors** (browser, ~5–10 Hz): LDR A/B in %, ultrasonic distance in cm (or timeout), compass direction in degrees, servo positions in %, stepper speed/direction/distance traveled.
- **Direct driving control:** forward/backward/left/right + speed, with an **emergency stop** and a **deadman/heartbeat**: in the event of a broken web socket or missing heartbeat, the cart stops automatically.
- **High-level commands:** LDR scan, drive-to-light, grab, return, start/stop calibration mode.

**Network:** AP mode (cart as own access point) is preferred for mobile use; station mode optional. SSID/password in the control panel `config.json`.

**Notes / limitations (yet to be implemented):**
- **Async refactor required.** The control is now fully blocking: busy-wait `while sm.active(): pass` in `_wait_stepper_done`, blocking `scan()`/ADC loops, and `ServoController.servo_rest()` without argument, which waits per servo with `time.sleep_ms(10)` until the position is reached with a timeout of 5 s — so up to ~15 s in a row. They all starve the asyncio event loop. Needed: cooperative tasks with `await asyncio.sleep`, or a command queue + shared state between web server and a control task.
- **Memory:** microdot + asyncio + lwIP + existing modules on ~520 KB RAM is feasible but tight — monitoring.
- **Safety:** command validation and a mandatory deadman stop (see above), so that a moving cart does not continue driving in the event of a loss of connection.

---

## Known issues / TODO

- [ ] `lib/LDR/ldr_scan_isr.py` — implement revised scan algorithm: 370° scan (no pre-roll), peak at A≈B intersection instead of sum-maximum, shortest path back, and closed-loop alignment via null seek at A−B
- [ ] Gyroscope/compass (GY9250) — basic and fusion code is present; still testing integration, sign convention, bias, magnetometer calibration and hardware behavior with running stepper motors.
- [ ] OLED (SSD1306) — integration not yet implemented; proposal is in `te doen.md`.
- [x] GPIO table checked and verified against KiCad netlist V1.2 (see [hardware/gpio_pinout.md](hardware/gpio_pinout_EN.md))
- [ ] Convert `test_all.py` to separate test functions per module; Keep `test_all.py` as a quick check test.
- [x] Stepper 1/64: `STEPS_REV = 12800` coded. `F_PIO` is set to 15 MHz; the calculated residual velocity deviation is ~0.43%.
- [ ] Physically verify TMC2209 MS wiring: `MS1→GND`, `MS2→VCC_IO` (in current documentation +5 V).
- [x] `WHEEL_CIRC` corrected to the measured 19.1 cm (was 20.94 → 8.8% too short distances). Added `TRACK_WIDTH = 13,6 cm`.
- [ ] Test `lib/stepper/stepper_ramp.py` on hardware: `MOTOR_TURN_SIGN` / `GYRO_Z_SIGN` / `LDR_DIFF_SIGN`, `CYCLES_FIXED` with a logic analyzer, measure maximum starting speed and acceleration, adjust control gains `kp_ldr`/`kp_gyro`. Then decide whether `stepper.py` is deleted.
- [ ] **Odometry Calibration** — include distance scale (`WHEEL_CIRC`) and rotation scale (`TRACK_WIDTH`) in the calibration session. Without this, the cart steers structurally askew.
- [ ] Drive-to-light: the segment-by-segment approach has been **replaced** by the continuous cross phase with adjustment per slice in `stepper_ramp.py`, plus positioning on the **beam axis** via the normalized brightness `Q`. See the section *Driving to the light source*.
- [x] `LDR_R_FIXED_OHM` to 1000 and `LDR_R_MIN_OHM` to 20 in `lib/LDR/ldr_scan_isr.py`, to match the new 1 kΩ pull-ups. Divider topology documented in the code.
- [x] `WHEEL_BASE_CM` in `ldr_scan_isr.py` corrected from 18.5 to 13.6 cm — a 370° scan actually rotated 503°.
- [ ] **Recalibrate `LDR_GAIN_B`** after exchanging R29/R30: that factor also compensated for the tolerance of the old 10 kΩ pair.
- [ ] **Verify divider topology on hardware once:** shine light on LDR A and read the raw ADC. If it goes to zero, then `_adc_to_res_ohm()` is correct; if it goes to 65535, the entire scale is reversed and the formula must become `R_FIXED × (65535 − adc)/adc`.
- [ ] **Recheck 370° scan** with the corrected `WHEEL_BASE_CM`. While previous scans were empirically tuned to 18.5, peak positions now deviate.
- [ ] **Measure `γ` and `w`** with `tests/test_ldr_beam.py` (`gamma()` and `bundel()`). Both constants are included in every `y` calculation; the current `w ≈ 33°` comes from one measurement point with an assumed `γ = 0,7`.
- [ ] **Cut light source slit in half — preliminary design proposal for objects wider than 4 cm.** The gripper only tolerates ±(9 − width)/2 cm lateral deviation, and the current beam achieves ≈ ±2 cm. Set the slit **vertically** (narrow horizontally). First run `bundel()`, then adjust, then check whether the 370° scan still finds the source from the actual starting position.
- [x] Lateral tolerance of the gripper determined: **±(9 − object width)/2 cm**. See *Gripper geometry and end positioning*.
- [ ] **Measure jaw depth** (palm to fingertips). This sets the actual lower limit of the grab window; now kept conservatively at 12 cm, which makes each window ~2 cm too narrow.
- [ ] **Third measuring point of the gripper** at ~5 cm opening. `tip_pos_cm()` is now a straight line through two points; a four-bar mechanism produces a curve.
- [ ] **Extend arm above the object and then lower it**, instead of sweeping forward horizontally. Prevents a jaw from knocking over the object in the event of a lateral deviation. Requires a coordinated movement of servo 1 and 2.
- [ ] Gray filter over the LDR openings when the cell physically saturates (100 Ω is very low for CdS). No diffuser and no smaller opening — they ruin the directivity.
- [ ] **Stepper motor naming inconsistent**: `stepper.py`/`stepper_ramp.py` call GPIO16/17/18 "motor A", but [hardware/gpio_pinout.md](hardware/gpio_pinout_EN.md) maps GPIO16/17/18 to stepper **B** (U2) and GPIO12/13/14 to stepper A (U1). Functionally no problem, but code and schematic contradict each other. Choose which one is the truth.
- [ ] **BLOCKER before full 6S battery operation — power supply U5 (DSN-MINI-360, MP2307) out of spec**: specified input range 4.75–23 V, battery supplies 22.2 V nominal and 25.2 V fully charged. A 28 V pin compatible version has been found; that build in. When a high-side switch is blown, 25 V is applied to the 5 V rail, which takes Pico, servos, OLED and ultrasonic in one go.
- [ ] **Separate servo rail from logic rail** (optional): all servos are connected to +5 V from U5 (1.8 A continuous / 3 A peak). Three MG996R together can draw 3–4.5 A peak. The gripper already has current limitation via the PI controller on ADC2, and the servos are slowly driven to their end position, so in practice the current remains low. If you have an unexplained Pico reset while grabbing, this is the first place to look.
- [ ] **BLOCKER before long-term motor test:** verify bulk electrolytic capacitor at the VM pin of U1/U2 (≥100 µF, short traces). At 24 V, a voltage spike from motor cable induction is the classic cause of death of a TMC2209. Never disconnect the motor plug while VM is on.
- [ ] `webserver` (microdot web socket, Pico 2 W): sensor readout, direct driving control with deadman, high-level commands; requires async refactor of the blocking control.
- [ ] Record exact MicroPython build and CPython version in this specification.


---

## Dependencies

MicroPython for **Raspberry Pi Pico 2 W (RP2350 + CYW43439)**. **Exact firmware version/build yet to be recorded** for reproducibility. External libraries: `mpu9250`, `mpu6500`, `ak8963` (Tuupola, via awesome-micropython), `ssd1306` (SSD1306 OLED driver), `microdot` (asyncio web server with websocket support).
