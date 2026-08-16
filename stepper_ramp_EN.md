# Operation of `stepper_ramp.py` — ramp via PIO/DMA and control per slice

Step-by-step explanation of [`lib/stepper/stepper_ramp.py`](lib/stepper/stepper_ramp.py). For the constants, the API table and the location in the system: see [global_specification.md](globale_specificatie_EN.md).

> `stepper_ramp.py` is a **replacement** of `stepper.py`, not an addition. Both claim PIO0 SM0..SM3 and the same GPIOs. Always import only one.

---

## 1. The problem that solves the disaster

The old module commands a speed at once. From a standstill this means: the rotor must jump to the final speed within one microstep of 78 µs. That's infinite acceleration — the rotor can't keep up with the rotating field, loses synchronism, and the motor is left whirring.

So it is **not a coupling problem**. The torque budget shows a factor of 8 margin at VREF = 1.0 V:

| Mail | Torque per wheel |
|---|---|
| Acceleration of 1636 g at 55 cm/s² | 1.43 N·cm |
| Rotor inertia (68 g cm²) | 0.013 N·cm |
| Rolling resistance (well estimated) | ~1.2 N·cm |
| **Total required** | **~2.7 N·cm** |
| **Available** | **~22 N·cm** |

At 22 N·cm and wheel radius 3.04 cm, the pulling force is 14.5 N against a weight of 16.1 N: **the wheels slip before the engine lacks torque.** So more power or spreadCycle does not help - a disaster does.

The additional requirement: the old concept ran 100% in the PIO without CPU overhead. We want to keep that.

---

## 2. The core trick: one FIFO word is a whole segment

The naive approach — one FIFO word per step — gets stuck on RAM. A ramp of 2200 steps would be 8.8 KB, and with a finer ramp this increases to tens of KB per ramp × 2 ramps × 2 engines. That doesn't fit reliably into the MicroPython heap.

Therefore, each 32-bit word encodes a **segment**:

```
 31                          16 15                           0
+------------------------------+------------------------------+
|   delay in PIO-cycles        |   aantal stappen - 1         |
|   (max 65535)                |   (max 65536)                |
+------------------------------+------------------------------+
```

A ramp of 2200 steps in 256 segments is therefore **1 kB**. A cross phase of up to 65536 steps (97.8 cm) fits in **one word**.

Packaging is done in `_word(repeat, delay)`; both fields are clamped, not masked, so that too large a value does not silently flip.

---

## 3. The PIO program, instruction by instruction

```python
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT)
def ramp_stepper():
    pull(block)             .side(0)        # 0
    out(y, 16)              .side(0)        # 1
    out(x, 16)              .side(0)        # 2
    mov(isr, x)             .side(0)        # 3
    label("pulse")
    mov(x, isr)             .side(1)  [2]   # 4
    label("wait")
    jmp(x_dec, "wait")      .side(0)        # 5
    jmp(y_dec, "pulse")                     # 6
```

| # | Instruction | What's happening | Cycles |
|---|---|---|---|
| 0 | `pull(block)` | remove the next segment from the FIFO. **If the FIFO is empty, the SM stalls here** — with STEP low, because side-set is also used during a stall. | 1 |
| 1 | `out(y, 16)` | lowest 16 bits → Y = `repeat − 1` (shift direction is RIGHT) | 1 |
| 2 | `out(x, 16)` | highest 16 bits → 1 |
| 3 | `mov(isr, x)` | set aside delay in ISR. **ISR is scratch register here**, no input FIFO — autopush is disabled. Necessary because X will soon be counted empty and there are only two scratch registers. | 1 |
| 4 | `mov(x, isr) [2]` | STEP **high**, restore delay from ISR. `[2]` makes the pulse 3 cycles = **200 ns** at 15 MHz, well above the TMC2209 minimum of ~100 ns and wide enough for the counter SM. | 3 |
| 5 | `jmp(x_dec, "wait")` | STEP **low**, X countdown. Jumps to itself, so this is the delay loop. | delay + 1 |
| 6 | `jmp(y_dec, "pulse")` | next step in this segment; if Y runs out, it falls through and **automatically wraps to 0** for the next segment. | 1 |

**Fixed overhead per step = 3 + 1 + 1 = 5 cycles.** That is `CYCLES_FIXED`, and therefore:

```
delay = round(F_PIO / stapfrequentie) − 5
```

Note: that is the overhead *within* a segment. At each **segment transition**, the four instructions 0–3 (`pull`/`out`/`out`/`mov`) are added, and they are not included in the formula. So the effective overhead is:

```
5 + 4/repeat   cycles per stap
```

For the 256 ramp segments (~22 steps each) this is 0.18 cycle per ~1000, or 0.02%. For segments of 1 step it increases to 0.4%. Negligible for the profiles used, but measure it on both long segments and segments of 1, 2, 8 and 256 steps — the value is **calculated, not yet measured**.

With `F_PIO = 15 MHz` and the top speed of 12800 steps/s, that delay = 1167 cycles. The overhead is then 5 of 1172 = **0.43%**, compared to ~2.3% with the old 3 MHz. `F_PIO = 15 MHz` is also 150 MHz sysclk / 10 and therefore an **integer clock divider**, without jitter of the fractional divider.

The program is 7 instructions. The counter SM is 3, so PIO0 uses 10 of the 32 instruction places.

### The step generator is always on

There is no start/stop condition. The SMs are activated at init and remain active. No data in the FIFO means stalling at `pull(block)` with STEP low — and that is the idle state in which the engine holds its position. This eliminates an entire class of status errors.

---

## 4. The counter-SM: hardware odometer

```python
@asm_pio()
def step_counter():
    label("loop")
    wait(0, pin, 0)          # wait until STEP is low
    wait(1, pin, 0)          # wait for the rising edge
    jmp(y_dec, "loop")
```

Y starts from `0xFFFFFFFF`; the number of pulses is `0xFFFFFFFF − Y`. Reading is done with `exec()`:

```python
self.cnt.exec("mov(isr, y)")
self.cnt.exec("push()")
return 0xFFFFFFFF - self.cnt.get()
```

Two things to know:

- **The counter does not know the DIR pin.** It counts edges. When rotating in place, both counters increase positively while the wheels rotate in opposite directions. See §8.
- **The counter counts commands, not movements.** It continues to run in case of wheel slip. The GY9250 only sees the **rotation** component of this; An IMU cannot provide actual *linear* displacement, because double integration of the accelerometer drifts too quickly. What which source does provide:

| Source | What you get out of it |
|---|---|
| PIO counter | commanded wheel steps |
| gyro-Z | actual rotational speed and short-term relative rotation |
| magnetometer | absolute orientation, provided it is not magnetically disturbed |
| actual linear position | requires an **external** reference: wheel coder, optical flow, beacon or map observation |

---

## 5. DMA to the TX-FIFO

One DMA channel per engine (of the 16 that the RP2350 has):

```python
self.dma = rp2.DMA()
self._ctrl = self.dma.pack_ctrl(size=2,            # 32-bit transfers
                                inc_read=True,     # walk through the table
                                inc_write=False,   # always to the same FIFO
                                treq_sel=(0 << 3) + sm_id)
```

Three details:

- **`treq_sel = (pio_num << 3) + sm_num`** links the DMA to the DREQ of that TX-FIFO, so that writing only takes place when there is space. This formula is also correct on the RP2350: `DREQ_PIO2_TX0 = 16 = 2 << 3`.
- **`write=self.sm`** is allowed directly: a `StateMachine` supports the buffer protocol, so no `mem32` address fiddling.
- **The buffer must remain alive** as long as the transfer is running; therefore `self._buf = array('I', words)`.

`start_table()` rejects an empty list, because a transfer with `count = 0` is firmware-dependent behavior.

**Add and replace are two different operations**, and that distinction does not exist for the form:

| | When | What it does |
|---|---|---|
| `start_table(words, n)` | the DMA is silent | the table hangs behind what is still in the FIFO; **refuses** (`False`) as long as the DMA writes |
| `replace_table(words, n)` | a movement is still ongoing | `_clear()` first: DMA silent, FIFO cleared, signed odometry updated, `committed` back to the actual pulse position — then the new table |

Previously, `start_table()` cut off a pending transfer itself. That looked useful, but `committed` had already been increased by the **entire** table at that time, while the truncated DMA had not written everything to the FIFO. `busy()` then waited forever for pulses that never came — exactly what happened to `Move.finish()` during the ramp. In addition, the old FIFO content remained, so that old and new profile words appeared one after the other.

Each movement command (`mov`, `s1`, `s2`, `rotate`, `Move`) therefore starts with `halt()` on the motors involved: changing direction, resetting counters and starting a new profile is always done from a standstill.

---

## 6. The four phases of a movement

```
snelheid
   ^
19 |         ______________________________
   |        /                              \
   |       /                                \
   |      /                                  \
 2 |_____/                                    \_____
   +---------------------------------------------------> tijd
      ^        ^                          ^        ^
      |        |                          |        |
   ramp op   brug                      ramp af   stil
   (DMA)     (DMA)     kruisfase (CPU)   (DMA)   (stall,
   256 wrd   1 wrd     1 woord per 20 ms 256 wrd  STEP laag)
   0,55 s    40 ms                       0,55 s
```

| Phase | Source | CPU costs | Why so |
|---|---|---|---|
| Disaster on | DMA, 256 words | **0** | ~2 ms segments are too fast for MicroPython; DMA is immune to GC breaks |
| Bridge | part of the same DMA transfer | **0** | see §7 |
| Cross phase **without** adjustment | same DMA transfer, 1 word | **0** | entire movement in one transfer |
| Cross phase **with** adjustment | CPU, 1 word per 20 ms per engine | ~0.1% | this is where the control loop |
| Disaster | DMA, 256 words | **0** | |

A movement of 50 cm without adjustment is **513 words = 2052 bytes in one DMA transfer**: exactly the zero-overhead behavior of the old concept, with ramp.

### The S-curve

`ramp_words()` divides the ramp steps over 256 segments of the same number of steps, and sets the speed per segment according to a **smooth step in the distance traveled**:

```python
p = (i + 0.5) / n_seg
s = p * p * (3.0 - 2.0 * p)
rate = rate0 + (rate1 - rate0) * s
```

The derivative of `3p² − 2p³` is zero at both ends, so the **acceleration is zero at the beginning and end** of the ramp. No torque shock on the transitions. This costs nothing extra, because the table is calculated in Python and then only played back by DMA.

`ACCEL_CM_S2 = 55` determines the ramp **distance** via `(v₁² − v₀²)/(2a)` = 3.28 cm. The peak acceleration is 1.5× = 82 cm/s² = 0.084 g. Note: because the S-curve is slow at the beginning and end, the ramp **lasts** 0.55 s and not the 0.31 s of a linear ramp over the same distance.

If the movement is shorter than 2 × 3.28 = 6.6 cm, `plan()` reduces the top speed to + exactly fit (triangular profile).

---

## 7. The DMA → CPU transition, and why the bridge is there

**Rule: DMA and CPU should never write to the same FIFO simultaneously.** If they do, the segments will be mixed up and the two engines may even have a different order — then both the ramp and the course will be unreliable.

Therefore, `service()` waits in the `_RAMP_UP` state:

```python
if MA.dma.active() or MB.dma.active():
    return True                  # DMA is still writing; CPU stays out
self._state = self._CRUISE
```

Notice what is **not** here: there is no waiting for the disaster steps to be *executed*. That would be wrong — the FIFO would run dry and the engine would pause between ramp and cruise. `dma.active() == False` just means that the DMA has stopped **writing**, while the FIFO still contains data. Just the edge the CPU needs, and the FIFO preserves the order.

### Why that's not enough

When the DMA has finished writing, there is a maximum of 4 words left in the FIFO. But at the end of the disaster we are at top speed, so those last segments are short: together only **~3 ms**. With `service()` every 10 ms the FIFO would still empty and you would get a stutter.

Therefore the ramp-up table ends with one **bridge segment** at cruising speed of `BRIDGE_SLICES × SLICE_MS = 40 ms`:

```python
up = ramp_words(self.n_ramp, self.r0, self.r1)
up.extend(cruise_words(bridge, self.r1))
```

These steps are part of the cross phase but are not adjusted — a correction 40 ms earlier or later makes no difference.

It's simpler the other way: the ramp-off DMA is only started after the CPU has **stopped** pushing, so the order can't be mixed up there.

### Why an empty FIFO isn't a disaster

If the FIFO is really empty, the PIO stalls at `pull(block)` with STEP low. Consequences:

- the motor **holds its position**, no step is lost;
- CPU latency — also a GC pause of tens of ms — does not affect the **step timing**, because the PIO generates with hardware precision. Being late means the correction comes one slice later, not a timing error;
- the result is a short **pause**, not a glitch.

That is neatly degrading failure behavior. With a CPU-timed pulse generator, a GC pause of 40 ms immediately results in lost steps. With 3 slices ahead in the FIFO the runway is 60 ms.

---

## 8. Steering: How a Slice Becomes a Course Change

### Why speed alone doesn't drive

Change in direction comes from a **difference in number of steps** between the wheels. If both engines are locked to the same total — as with fire-and-forget — then a speed difference gives a net **zero** change in course: the cart makes an arc and returns in the same direction.

In the cross phase this is not fixed at all. There, a speed difference does integrate into a permanent change in course. That is exactly why steering belongs in the cruising phase.

### From degrees/s to two FIFO words

Per slice with duration `t`:

```python
omega = self.correction()                             # degrees/s, + = right
delta = int(omega * STEPS_PER_DEG * t / 2.0 + 0.5)    # extra stappen LINKER wiel
delta = klem(delta, ±base * MAX_DIFF_FRAC)            # max ±20 %

ra = base - MOTOR_TURN_SIGN * delta                   # A = rechter wiel bij +1
rb = base + MOTOR_TURN_SIGN * delta

cyc = F_PIO * base / self.r1                          # PIO cycles for this slice
MA.push(ra, _clamp_delay(int(cyc / ra + 0.5) - CYCLES_FIXED))
MB.push(rb, _clamp_delay(int(cyc / rb + 0.5) - CYCLES_FIXED))
```

Two things are deliberately this way:

1. **The slice has a fixed DURATION, not a fixed number of steps.** Both engines receive the same `cyc`, but a different number of steps and therefore a different delay. This way they remain synchronized in time; the rounding error of `int(cyc/ra + 0.5)` is **calculated** at < 20 µs at 20 ms (not yet measured with a logic analyzer). With a fixed number of steps they would be out of sync.

This applies to slices once in the FIFO, **not before the start**. The two DMAs are configured one after the other and the SMs are already active, so engine A can start while another array is created for engine B. The skew is order 1 ms, and the take-off occurs at `V_START_CM_S` = 1.91 cm/s, so that is ~0.002 cm path difference ≈ 0.008° heading error — negligible, but it is an estimated value. If you want it exactly: measure the skew with a logic analyzer, or prepare both FIFOs with stopped SMs and start them together.
2. **`ra + rb = 2 × base` exactly.** The center of the cart therefore shifts exactly `base` steps, no matter how large the steering difference is.

With `STEPS_PER_DEG ≈ 159` and `base = 256` steps per 20 ms:

| Differentiation | Δ steps | per slice | rotation speed |
|---|---|---|---|
| ±5% | 25 | 0.16° | 8°/s |
| ±20% (maximum) | 102 | 0.64° | 32°/s |

That 32 °/s applies **at top speed**. The steering authority is a fraction of the driving speed, so it scales proportionally: at 5 cm/s it is only 8.4 °/s. `turn_authority_deg_s(rate)` calculates that and `Move()` sets the ceiling of the `HeadingController` — otherwise the controller commands a turning speed at low speed that the wheels cannot deliver, and then reads that difference as a fault.

Resolution: 1 step difference = **0.0063°**.

### Why not vary the PIO clock

`SMn_CLKDIV` is writeable at runtime via `machine.mem32`, and that would also give a speed difference. Deliberately not done:

- it is **outside the data path**, so not synchronous with the segment boundaries;
- the step numbers are then no longer known exactly;
- the delays are in PIO cycles, so a clock change **rescales the ramp table along the way** (and the acceleration scales by f²).

The clock divider remains usable as a global speed override (everything slower, for example near an obstacle).

---

## 9. Exact distance despite steering

Adjustment changes **when** steps occur, not **how many**. `committed` is kept for each engine: the sum of all written `repeat` values.

The braking point is determined at `committed`, **not** at a measured position:

```python
remaining = self.n_total - self.n_ramp - self._centre
base = min(self._slice_base, remaining)
if base < 1:
    self._state = self._RAMP_DOWN
```

This means that the **commanded average step total is exact**, regardless of when the control loop happens to be called. No poll uncertainty; the final slice is made to measure exactly. The *physical* distance is not: microstepping accuracy, wheel slip, tire deformation and the calibration of `WHEEL_CIRC` are all on top of that. `CM_PER_STEP` is 14.9 µm nominal resolution, no accuracy.

### Signed odometry

Because the counter-SM does not know the DIR pin, each motor maintains a **signed position** in addition to the monotone pulse counter:

```python
def travel(self):
    p = self.pulses()
    d = p - self._seen
    if d:
        self._pos += self._sign * d
        self._seen = p
    return self._pos
```

`set_dir()` first calls `travel()` with the **old** character and then switches `_sign`. This way the position remains correct even after a change in direction.

| Function | Meaning |
|---|---|
| `pio_pos1()` / `pio_pos2()` | monotonic pulse counter, used by `busy()` |
| `distance()` | `(travel_A + travel_B)/2 × CM_PER_STEP` — rotation gives ~0 cm, reverse negative |
| `heading()` | `(travel_A − travel_B)/STEPS_PER_DEG` — also works when rotating in place |

---

## 10. Stopping — three types

| Function | DMA | FIFO | Driver | Engine |
|---|---|---|---|---|
| `halt()` (= `stop()`) | quiet | cleared | **on** | stops immediately, **holds position** |
| `brake()` | quiet | cleared, then braking ramp | on | slows down from the estimated speed |
| `emergency_stop()` | quiet | cleared | **off** | runs freely, cart can roll |

Clearing the FIFO is done with `sm.init(...)`: MicroPython has no separate API for that, and `init()` does `clear_fifos + restart` internally. That is safer than writing `SHIFTCTRL` yourself. After that, `committed` no longer matches what is about to happen, so it is set to the actual pulse setting.

`brake()` estimates the current speed from the progress through the profile (`current_rate()`), because braking from an assumed speed that is too high would first cause a speed jump upwards — and thus a loss of synchronization.

### `finish()` during the
ramp
If `finish()` falls into the cross phase, it is simple: the CPU stops pushing and the deceleration ramp follows the remaining slices via DMA.

If he falls during the **ramp**, that is not possible, for two reasons at the same time:

- `committed` has already been increased at the start with the entire ramp plus the bridge segment, but the DMA has not yet written everything to the FIFO. If you simply cut off the transfer, `busy()` waits for pulses that no longer come;
- the regular braking table starts at `r1`, while the engine is still somewhere between `r0` and `r1`. That would initially give a jump **up** — exactly what the disaster is intended for.

`_brake_from_ramp_up()` therefore does it in the following order: first read `current_rate()`, then `halt()` on both engines (DMA silent, FIFO cleared, `committed` back to the actual pulse position), and only then a fresh braking ramp from that speed. The estimate is slightly behind reality, and that is the safe direction: a small decline instead of a jump up.

### Start braking on time

`finish()` does not stop immediately. The braking ramp and the slices already written are fixed:

| Speed ​​| braking disaster | in FIFO | **committed** | + ultrasound latency (50 ms) |
|---|---|---|---|---|
| 19.1 cm/s | 3.28cm | 1.15cm | **4.43cm** | 0.96cm |
| 10 cm/s | 0.88cm | 0.60cm | **1.48cm** | 0.50cm |
| 5cm/s | 0.19cm | 0.30cm | **0.49cm** | 0.25cm |

```python
doel = approach.brake_target_cm(snelheid, object_w_cm=6.0)
if ultrasoon.read_cm() <= doel:
    mv.finish()
```

**For the last ~25 cm, slow down to 5 cm/s.** That brings the stopping uncertainty from 5.4 cm to 0.74 cm, and that is more robust than trying to predict 4.43 cm exactly.

### And then: standing still, measure

When driving, the servos are in rest position, so the jaws are behind the ultrasound and the beam is free. After stopping but **before** unfolding, you can therefore take a fresh average measurement — then the driving speed and the measurement latency will be out of error:

| | Uncertainty |
|---|---|
| stop at 5 cm/s | 0.74cm |
| **`finetune()`: re-measure while stationary + `creep()`** | **≈ 0.3cm** |

```python
import sys; sys.path.append("/lib/gripper")
import approach
approach.finetune(ultrasoon.read_cm, object_w_cm=5.0)
```

As soon as the arm unfolds, the jaws (9 cm open) are in a bundle that is approximately 8 cm wide at 12–15 cm — the sensor then looks at your own fingers. **`finetune()` is therefore the last correction moment**; everything after that is open-loop, except for the gripper current sensor.

`mean_dist_cm()` waits 60 ms between measurements, because `ultrasoon.INTERVAL_MS` is 50 ms: if you read faster, you get the same buffered value back and the average does nothing.

### Gripper geometry

The jaws close horizontally, but the tips move forward: `tip_pos_cm(o) = 12 + (9 − o) × 3/7`. Three things follow from this:

| Object width | `stop_dist_cm()` | `grip_window_cm()` | `lateral_tolerance_cm()` |
|---|---|---|---|
| 3cm | 14.6cm | 2.6cm wide | ± 3.0 cm |
| 5cm | 13.7cm | 1.7cm wide | ± 2.0 cm |
| 7cm | 12.9cm | 0.9cm wide | ± 1.0 cm |

A *narrower* object requires a *larger* stopping distance: narrower means closing further, so more progress from the tops. The complete table and the substantiation can be found in [global_specification.md](globale_specificatie_EN.md) under *Grip geometry and final positioning*.

This geometry was previously in `stepper_ramp.py`, but does not belong in a motor driver: it therefore knew the dimensions of one gripper and the stopping strategy of one mission. Now in [`lib/gripper/geometry.py`](lib/gripper/geometry.py) (pure calculations, no imports) and [`lib/gripper/approach.py`](lib/gripper/approach.py) (`finetune()`, `mean_dist_cm()`, `brake_target_cm()`). The stepper only supplies `creep`, `drive`, `brake`, `halt` and `busy`.

---

## 11. Calculated example: 50 cm at 19.1 cm/s

| Greatness | Value |
|---|---|
| Total | 33 508 steps |
| Disaster on/off | 2200 steps each = 3.28 cm, 256 segments = 1024 bytes |
| Bridge segment | 512 steps = 40 ms, 1 word |
| Cross phase | 28 596 steps ≈ 112 slices of 20 ms |
| Delay at top speed | 1167 cycles |
| Delay at starting speed | 11 714 cycles |
| Duration | ramp 0.55 s + bridge 0.04 s + cross 2.23 s + ramp 0.55 s ≈ **3.37 s** |
| Average speed | 14.8 cm/s (the ramps are slow) |
| **Without** adjustment | 513 words = 2052 byte, **one** DMA transfer, zero CPU |
| **With** adjustment | 257 + 256 words DMA, plus ~224 `put()` calls (~0.1% CPU) |

---

## 12. Use

**Fire-and-forget, zero CPU overhead:**

```python
import stepper_ramp as sr
sr.mov('f', 19.1, 50)         # 50 cm forward, with ramp
sr.rotate_deg(90)             # 90 degrees to the right
while sr.busy():
    pass
```

**With continuous course correction:**

```python
import stepper_ramp as sr, ultrasoon, time
from stepper_ramp import HeadingController, gyro_z_deg_s
sys.path.append("/lib/gripper"); import approach

hc = HeadingController(ldr_diff=mijn_ldr_verschil,
                       gyro_rate=gyro_z_deg_s(sensor))

mv = sr.drive(200, 19.1, correction=hc)      # the controller itself, not hc.output
doel = approach.brake_target_cm(19.1)
while mv.service():
    if ultrasoon.read_cm() <= doel:
        mv.finish()
    time.sleep_ms(10)
```

**Mute the rotation speed only with the gyro:**

```python
mv = sr.drive(50, 19.1, correction=sr.damp_yaw_rate(gyro_z_deg_s(sensor)))
```

This was called `hold_heading()`, but that promised too much: the setpoint here is always zero, so the output is `−kp_gyro × gemeten`. An external twist is resisted *as long as it occurs*, but the angular error that remains is not reversed afterwards, and gyrobias produces persistent drift. **Really** keeping course requires integration of gyro-Z into an angle and correcting that error; that is not possible now.

`gyro_z_deg_s()` is **mandatory** around the GY9250: [`mpu6500.py`](lib/GY9250/mpu6500.py) has `gyro_sf=SF_RAD_S` as default and therefore delivers **radians/s**, while the controller calculates in degrees/s. Passing `sensor.gyro[2]` directly makes the correction 57.3× too small.

---

## 13. Tests and Verification

[`tests/test_stepper_ramp_math.py`](tests/test_stepper_ramp_math.py) — **226 pure-Python tests, no hardware required** (`machine` and `rp2` are stubbed, running on PC with CPython):

```
python3 tests/test_stepper_ramp_math.py
```

Covered: exact step total over the entire range (0 to 200,000 steps), monotonic speed in both ramp directions, field boundaries, triangular profile, zero distances, input validation, slice arithmetic, signed odometry in rotation and reverse, `Move.finish()`, and the DMA → CPU transition.

In addition, as a regression, because each of these three was wrong before:

- **`finish()` during the ramp-up DMA** (immediately after construction, halfway through, and just before the DMA end). `committed` was already incremented by the entire ramp at the start, while the truncated transfer never sent out those steps — `busy()` therefore remained `True` forever.
- **A new command during an ongoing movement.** `Move()` inverted DIR and the counters without stopping first, so that old FIFO words could come out with the new direction.
- **The sign convention.** `rotate_deg(+90)` gave `heading() = −90`: the direction of rotation was opposite to both the odometry and the adjustment. The test now checks both values ​​of `MOTOR_TURN_SIGN`.

[`tests/test_gripper_geometry.py`](tests/test_gripper_geometry.py) — 32 gripper geometry tests, also without hardware.

### Still to be verified on hardware

| What | How |
|---|---|
| `MOTOR_TURN_SIGN` (+1 / −1) | `rotate_deg(+90)` must turn the cart to **right**; `heading()` should then give ≈ +90 |
| `GYRO_Z_SIGN` (+1 / −1) | raised wheels: turn the cart by hand to the right, `gyro_z_deg_s(sensor)()` must be **positive** |
| `LDR_DIFF_SIGN` (+1 / −1) | keep a lamp to the right of the cart; `ldr_diff()` must be **positive** |
| `CYCLES_FIXED = 5` (+4/segment) | `meet_frequentie()` compares the actual STEP frequency with `_delay_for()`. Measure long segments as well as segments of 1, 2, 8, and 256 steps to reveal per-segment overhead. Also check the pulse width (expected 200 ns) and whether any steps are missing with a logic analyzer |
| Starting speed and max. acceleration | with the GY9250 as an independent reference — the PIO counter cannot see a stall |
| `kp_ldr`, `kp_gyro` | the standard `kp_ldr = 25` gives 25 °/s at full LDR difference, just below the ceiling of 32.2 °/s, so the controller normally does not saturate |
| Odometry Calibration | 1.00 m drive (`WHEEL_CIRC`) and 360° turn (`TRACK_WIDTH`) |
| `fifo_join=PIO.JOIN_TX` | not used because not verified in this MicroPython version. If it works, the runway doubles from 60 to 140 ms |
