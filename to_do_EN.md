# Updating the code base
**All steps are implemented one after the other and first checked for correctness, problem analysis and solution proposals**
1. Keep the PIO-asm code as far as possible.
2. Order of implementation (due to interdependencies): first GY9250 compass, then display (shows, among other things, the compass direction), then WS2812B status, then the laser cross-control, then the LDR scan fixes, then servo calibration, then LDR calibration, then the stepper ramp (proposal). "Update global_specification.md" happens after each step, not just at the end.
3. Do not store calibration values ​​(GY9250, servos, LDRs) in separate ad-hoc files, but in one central configuration file on the RP2350 (e.g. `config.json`) with a separate section per module.

# The micropython code for the GY9250 was downloaded from the awesome-micropython website (author Tuupola).

1. This micropython code for the GY9250 will run on core 1 of the RP2350, so that the other modules are not affected (computation intensive).
2. In the main program you can choose whether we import the `GY9250_basic.py` library function (compass function only) or the `GY9250_fusion.py` library function (will use compass plus accelerator correction). **Note:** these files do not yet exist in the repo — when creating them, keep the naming consistent with the references in this document and in `globale_specificatie.md`.
3. Provide secure data exchange between core 0 and core 1 (e.g. a lock or atomic read/write) for the shared compass value — this is not automatically secure with `_thread` on the RP2350.
4. The code still needs to be reviewed and tested.
5. The purpose of the compass is to keep the cart moving in the right direction. The surface may be uneven and the wheel circumference may vary slightly. The compass can then be used to correct the course.
6. The calibration of the GY9250 (hard-iron offset + soft-iron scale) is performed via `tests/test_gy9250_auto_calibrate.py`. This script spins the cart and determines the magnetometer offsets. The found calibration is stored in the central configuration (`config.json`) and applied during normal operation by `GY9250_basic.py`/`GY9250_fusion.py` to compensate the compass value.
7. Save the calibration values ​​(the correction file from point 6) to the RP2350 (see central configuration, section above).
8. The compass routine is only called once every 100/200 ms, depending on the time needed to complete the calculations; As soon as we know how long the calculation will take, we will determine what the cycle time will be. The remaining time within that cycle on core 1 is available for display handling (see next section).

# add display routine for the 128x64 0.96 inch OLED display with SSD1306 driver chip
**The display is used to show real-time sensor and motion information**
1. The display runs on core 1 just like the GY9250. The routines for the GY9250 and the display always run one after the other, at the pace of the GY9250 routine: the remaining time of each compass cycle (see point 8 above) is used for display handling.
2. Record the minimum acceptable refresh rate: If the GY9250 fusion is slow, "real-time" stepper speed/distance traveled information may lag noticeably.
3. As far as possible, read the information directly from the PIO.

**information to be displayed**
1. light values of both LDRs in %
2. distance measurement of the ultrasonic sensor in cm, or time-out
3. compass direction in degrees relative to north (max 180 can be + or minus relative to north)
4. servo positions in % (rest = 0%, max = 100%)
5. stepper motors speed, direction and distance

# add control WS2812B multi-color LED
**the LED is used for general status of the system**
1. green: everything ok (rest)
2. red (solid): catastrophic error
3. white: cart on the way to target
4. red flashing: non-fatal warning (e.g. servo current limit reached) — separate from solid red, which remains reserved for a real stop
5. flashing blue: LDR scan in progress
6. yellow/orange: cart returns to starting position
7. purple: calibration mode (servo/LDR/compass)
8. off: system off / sleep mode
9. maximum power consumption multi-color LED 25%

# add laser cross control
1. The laser cross can be controlled from the main program with PWM control. pwm 0 is off and pwm max is 100%

# fix errors in the ldr-scan routine
1. ~~Now before the ldr_scan starts the ultrasonic routine must be stopped.~~ **Fixed**: ldr_scan now uses SM8 (PIO2 SM0), ultrasonic uses SM4 (PIO1 SM0) — separate PIO blocks, no more conflict.
2. The LDR scan must record the two LDR values ​​separately during the scan and then determine the direction of the light source (now it peaks at the average of both LDRs, that's the bug). The LDRs are spaced apart in the horizontal plane, so two maxima are created during scanning. The desired direction therefore lies between the two maximums. The intersection where LDR A ≈ LDR B is the center, after correction of the gain difference between the two LDRs.
3. Save the measured values ​​in a CSV file on the RP2350.
4. After turning the cart back to the light source, check whether this corresponds to the calculated values.
5. **Full rotation scan (370°) instead of pre-roll.** Do not turn back first, but immediately turn 370° in one direction. A full revolution always contains the maximum; the 10° overlap maintains a peak around 0°/360° from the edge. The pre-roll (`start_graden`/roll back before scan) is deleted. Precondition: the cart must be able to rotate freely on its axis without cables that twist at 360°+.
6. **Shortest route to the found direction.** After 370° at end angle 370°, turning back to peak angle θ is equal to `370 − θ` and continuing to turn is `(θ − 10) mod 360`; choose the smallest. Always let the last degrees end in the same direction (when turning back a little past and then back forward), so that the backlash remains constant.
7. **Closed-loop alignment via null-seek on A−B.** Coarse to the expected peak position on steps, then fine-adjust with `measure_now()`. Find the **zero crossing of A−B** (after gain correction), not the stored maximum magnitude: the difference signal is sharper and insensitive to brightness drift, while a magnitude threshold stops too early because the motion scan smears the peak. Limit the fine adjustment to a window (e.g. ±15°) around the expected position and fall back to the step target if there is no clear zero crossing. This corrects stepper slip (replaces/refines the current step-based `backtrack` and point 4 above).

# routine for the calibration of servomotors
**The servomotors now have fixed idle values, this must be done via a calibration routine**
1. The calibration is carried out via the REPL.
2. Automatic: Calibration is performed by placing the arm servos one after the other on the gripper servo plug (this is the only connector with current measurement). By measuring the current it can be determined when the servo reaches its extreme position (current exceeds the limit value). Then make a small correction (reduce PWM so that there is margin) to prevent the servo from continuing to consume a lot of power when it gets stuck. Then check whether the servo can be rotated 180 degrees. All this only applies to the arm servos.
3. The gripper (servo 4, GPIO22) should never be opened more than 90 degrees (maximum position) — record how the gripper itself calibrates its rest and end positions, separate from the 180° test procedure of the arm servos above.
4. Servo 3 (GPIO4, connector J3) is an optional, spare servo connection — not part of the gripper. This is wired to the PCB (KiCad netlist V1.2 confirms the connection), but is not currently controlled and has not yet been implemented in `ServoController` (`lib/servo/servo_crl.py`). Clarify whether this connection should still be included in the calibration routine, or whether servo 1, 2 and 4 are sufficient for the time being.
5. Manually: by specifying PWM values.
6. Define all limit values ​​in the central configuration on the RP2350 (see above).
7. If the configuration is missing: set rest position to the current fixed default values ​​— the middle of the duty range (50% of the 0–180° angle range) per servo (servo1 7.5%, servo2 7.5%, servo4 7.5% duty). Servo 3 (optional, not yet implemented) is not included here for the time being.

# add routine for the calibration of the LDRs
**the LDR resistors have quite large deviations from each other and must therefore be compensated. Previously we paired them two by two. Now we are going to create a semi-automatic route**
1. The measurement commands are given via the REPL, starting at a distance of 5 meters between cart and light source (turn the cart so that the laser pointer points to the light source). The cart is placed by hand over the distances to be measured and aligned with the laser pointer.
2. Specify the following measurement distance and measure both LDRs.
3. Repeat this until a distance of 5 cm. Measure at distances 5m, 4m, 3m, 2m, 1m, 50cm, 15cm and 5cm.
4. Calculate a correction and save it on the RP2350 (see central configuration). A correction table that corrects the sensitivity between both LDRs.

# make a proposal to add a ramp for the stepper motors
**Now a speed is directly specified for the stepper motor; if the mass inertia is too great, the cart will not move. Now the control and measurement of the distance traveled runs 100% in the PIO without CPU overhead (except for a one-time interrupt when the target position is reached)**
1. ~~Is a linear acceleration control possible that requires no or minimal CPU overhead?~~ **Developed and implemented** in [`lib/stepper/stepper_ramp.py`](lib/stepper/stepper_ramp.py); working principle fully described in [global_specification.md](globale_specificatie_EN.md). Answer: yes. Each FIFO word encodes an entire segment of `(number of steps, delay)` instead of one step, allowing an S-curve ramp of 2200 steps to fit into 1 KB and played back via DMA. Without adjustment, a complete movement runs as **one DMA transfer with zero CPU overhead** — so exactly the old behavior, with ramp. (There is no IRQ: completion is polled with `busy()`.) With adjustment it costs ~0.1% CPU.
- Why previous attempts failed: it was not a coupling problem (there is factor 8 margin at VREF 1 V) but a **synchronism** error. Commanding a speed directly means that the rotor must jump to the final speed within one microstep of 78 µs; no engine can follow that.
- Still to do on hardware: verify `TURN_SIGN`, measure maximum take-off speed and acceleration using the GY9250 as reference, and tune `kp_ldr`/`kp_gyro`.

# calibrations
**All calibrations are performed in a separate code section, separate from the control program. The intention is that all individual calibrations (GY9250 stepper motor compensation, servos, LDRs) can be completed one after the other in one session; calibrations that are not necessary are skipped.**
1. After start-up, the REPL asks whether calibration is required. If no confirmation is given via the keyboard within 2 seconds, the control program will start.
2. Upon confirmation, the calibrations are offered step by step (one after the other), each with the question whether this step should be performed:
- **n**: this step is skipped, moving on to the next calibration.
- **y**: calibration is in progress; the result is shown with the question whether it should be saved (y/n).
- **x**: The entire calibration session is immediately ended, asking whether the calibrations performed so far should be saved.
3. The configuration is read at the start of the session. As soon as a change actually occurs during a calibration step, the still unchanged (session start) configuration is first written to `config_backup.json` (one-time backup per session), and only then the new value is saved in the configuration file (`config.json`).
4. During the calibration session, the WS2812B LED will show purple (see WS2812B section above) and the display will show text indicating that the system is in calibration mode.
5. This section is the overarching startup flow that calls the GY9250 stepper motor calibration (point 6 above), the servo calibration routine and the LDR calibration routine one after the other.

Convert # stepper to 1/64 microstepping (12800 steps/rev)
**The microstepping has been brought from 1/8 (1600 steps/rev) to 1/64 (12800 steps/rev), partly to make the stepper ramp smoother and easier to implement (smaller speed jump per pulse).**
1. Physically verify MS wiring: TMC2209 MS1 → GND, MS2 → +5V (10 kΩ pull-ups removed). Truth table: MS2=H/VIO, MS1=L/GND → 1/64.
2. ~~In `lib/stepper/stepper.py` put `STEPS_REV = 12800`.~~ **Done.** `CM_PER_STEP ≈ 14,9 µm/stap` (after correction of `WHEEL_CIRC` to the measured 19.1 cm).
3. Remeasure `OVERHEAD` in `speed_to_delay()`: with 8x more pulses the fixed overhead becomes a larger share of the delay (~11% at 30 cm/s) → speed deviation. Alternative/Additional: Increase `F_PIO` (finer resolution, negligible overhead).
4. Precondition when increasing `F_PIO`: the STEP pulse must remain high/low for ≥ ± 100 ns (TMC2209 minimum); if necessary, keep an extra `nop` in the PIO pulse.

# driving to the light source with LDR correction
**While driving to the light source, course deviation is continuously corrected: bringing the difference between the two LDR values ​​(after gain correction) to zero (A ≈ B = straight on the source).**
1. Chosen approach: segment-by-segment (fits the existing fire-and-forget + IRQ-stop stepper architecture). Drive a short segment → measure both LDRs → outside a deadband, make a small, angle-limited `rotate()` correction towards the brightest side → repeat.
2. Stop at the set distance via the ultrasonic sensor (runs independently in the background, SM4).
3. Depending on the LDR calibration (correction of sensitivity difference A/B) — without proper calibration the cart will steer crooked.
4. Arbitration with the compass: LDR is leading during the approach; the GY9250 course correction is for the return route. Both do not steer at the same time.

# microdot web socket for control via the browser (Pico 2 W)
**The cart (Pico 2 W, onboard CYW43-WiFi) is operated via a standard browser and read out with a microdot web server + web socket. Runs on core 0 next to the controls; core 1 remains for GY9250 + display.**
1. Read out sensors (~5–10 Hz): LDR A/B in %, ultrasonic distance in cm (or time-out), compass direction in degrees, servo positions in %, stepper speed/direction/distance traveled.
2. Direct driving control: forward/backward/left/right + speed, with emergency stop and a deadman/heartbeat (if the web socket is broken or the heartbeat is missing, the cart stops automatically).
3. High-level commands: LDR scan, drive-to-light, grab, return, start/stop calibration mode.
4. Async refactor required: control is now blocking (busy-wait `while sm.active(): pass`, blocking `scan()`/ADC loops) and starves the asyncio event loop. Needed: cooperative tasks (`await asyncio.sleep`) or a command queue + shared state between web server and control task.
5. Network: AP mode (cart as own access point) is preferred; station mode optional. SSID/password in the control panel `config.json`.
6. Monitor memory (microdot + asyncio + lwIP + existing modules on ~520 KB RAM is tight) and command validation/ensure safety.

# update global_specification.md
**This to-do list means that the global_specification.md must be updated after each successfully completed step in this list.**
