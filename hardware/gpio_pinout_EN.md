# GPIO Pinout — DIY Robot Car (Revision V1.2, RP2350)

> **Status: VERIFIED** — GPIO table has been checked against KiCad netlist V1.2 (export 2026-06-09).

## GPIO table

| GPIO | Direction | Function | Component / Net | Note |
|------|----------|-------------|-----------------|-----------|
| GPIO0 | OUT (SDA) | I2C0 SDA | U6 SSD1306 OLED — net `/SDA0` | Used as I2C0 SDA (GP0) |
| GPIO1 | OUT (SCL) | I2C0 SCL | U6 SSD1306 OLED — net `/SCL0` | Used as I2C0 SCL (GP1) |
| GPIO2 | OUT (PWM) | Servo 1 PWM | J1 (JST-XH 3) — MG996R lower hinge arm | |
| GPIO3 | OUT (PWM) | Servo 2 PWM | J2 (JST-XH 3) — MG996R upper hinge arm | |
| GPIO4 | OUT (PWM) | Servo 3 PWM | J3 (JST-XH 3) — third servo | Previously listed as NC; netlist confirms connection |
| GPIO5 | IN | PWM tick for servo ISR | Not connected on PCB (no-connect marker) | Connect externally to GPIO2 with wire |
| GPIO6 | OUT | WS2812B data | D2 WS2812B — net `Net-(D2-DIN)` | |
| GPIO7 | — | NC | — | |
| GPIO8 | — | NC | — | |
| GPIO9 | — | NC | — | |
| GPIO10 | BIDI (SDA) | I2C1 SDA | J7 pin 4 — GY9250 gyroscope/compass | *Not yet implemented* |
| GPIO11 | BIDI (SCL) | I2C1 SCL | J7 pin 3 — GY9250 gyroscope/compass | *Not yet implemented* |
| GPIO12 | OUT | DIR stepper A | U1 TMC2209 pin DIR — net `Net-(U1-DIR)` | |
| GPIO13 | OUT | STEP stepper A | U1 TMC2209 pin STEP — net `Net-(U1-STEP)` | |
| GPIO14 | OUT | ENABLE stepper A | U1 TMC2209 pin ~ENABLE — net `Net-(U1-~{ENABLE})` | Low = enabled |
| GPIO15 | OUT (PWM) | Laser PWM | Q1 BS170 MOSFET gate — net `Net-(Q1-G)` | Via R10 (100 Ω) and Q1 BS170 to J6 (laser) |
| GPIO16 | OUT | DIR stepper B | U2 TMC2209 pin DIR — net `Net-(U2-DIR)` | |
| GPIO17 | OUT | STEP stepper B | U2 TMC2209 pin STEP — net `Net-(U2-STEP)` | |
| GPIO18 | OUT | ENABLE stepper B | U2 TMC2209 pin ~ENABLE — net `Net-(U2-~{ENABLE})` | Low = enabled |
| GPIO19 | IN | Echo ultrasonic sensor | J13 pin 2 — net `/Echo` | |
| GPIO20 | OUT | Trigger ultrasonic sensor | J13 pin 3 — net `/Trig` | |
| GPIO21 | — | NC | — | |
| GPIO22 | OUT (PWM) | Servo 4 PWM (gripper) | J10 pin 1 — net `Net-(J10-Pin_1)` | With current limitation via PI; current sensor on J10 pin 3 |
| GPIO26 | IN (ADC0) | LDR A | J8 (JST-XH 2) — net `Net-(J8-Pin_1)` | Pull-up R29 (1 kΩ) to 3V3; filter C13 |
| GPIO27 | IN (ADC1) | LDR B | J9 (JST-XH 2) — net `Net-(J9-Pin_1)` | Pull-up R30 (1 kΩ) to 3v3; filter C15 |
| GPIO28 | IN (ADC2) | Current sensor gripper | R22 (1 kΩ) ← U3B output — net `/I_Servo` | Filter C10; see circuit below |

## Stepper drivers

The circuit uses **TMC2209** drivers (U1 and U2). The KiCad footprint is labeled as A4988 but the installed component is TMC2209.

### Connector overview

| Connector | GPIO side | Engine side | Engine |
|-----------|-----------|------------|-------|
| J4 (JST-XH 4) | U1 TMC2209 | 1A / 1B / 2A / 2B | Stepper A |
| J5 (JST-XH 4) | U2 TMC2209 | 1A / 1B / 2A / 2B | Stepper B |

### Microstepping setting (TMC2209)

MS pins are hardwired (TMC2209). The original 10 kΩ pull-ups have been removed; MS1 and MS2 are now fixed for **1/64 microstepping**:

| Pin | Connection | Condition |
|-----|-----------|---------|
| MS1 | → GND | Low |
| MS2 | → +5V | High |

TMC2209 truth table (MS2, MS1): GND/GND → 1/8 · GND/VIO → 1/32 · **VIO/GND → 1/64** · VIO/VIO → 1/16.

With MS2=H (VIO), MS1=L (GND) → **1/64 microstepping**

→ 200 steps/revolution × 64 = **12800 steps per revolution**

Same goes for U2.

> **History:** Previously documented as 1/8 (1600 steps/rev) based on A4988 footprint logic. The actual driver is a TMC2209 and the MS wiring has been adjusted to 1/64 to allow for a smoother implementation of the stepper ramp.

## Gripper servo circuit (GPIO28)

```
Servo 4 GND ── R31 (0,1 Ω shunt) ── GND
                        │
                  net /Isense
                  │           │
               R3 (20 kΩ)  R13 (2,4 kΩ)
                  │           │
              U3A (+)      U3B (+) ─── C9
              U3A (-) ← R2 (3,9 kΩ) ← GNDA
              U3A (-) ← R4 (47 kΩ) ← U3A output
              U3A output → R9 (16 kΩ) → net C8
              U3B (-) = U3B output (voltage follower trap)
              U3B output → R22 (1 kΩ) → GPIO28 (ADC2)
```

- IC: U3 OPA2705 (DIP-8 socket) — dual op amp
- Filter C10 (1 nF) on GPIO28
- Net `/I_Servo` connects R22 pin 1 to GPIO28

## Servo connectors (JST-XH 3-pin)

Pin order: **pin 1 = signal, pin 2 = +5V, pin 3 = GND**

| Connector | GPIO | Servo |
|-----------|------|-------|
| J1 | GPIO2 | Servo 1 — lower hinge arm |
| J2 | GPIO3 | Servo 2 — upper hinge arm |
| J3 | GPIO4 | Servo 3 |
| J10 | GPIO22 | Servo 4 — gripper (pin 3 = /Isense) |

## Ultrasonic sensor connector

J13 (JST-XH 4): **pin 1 = GND, pin 2 = Echo (GPIO19), pin 3 = Trig (GPIO20), pin 4 = +5V**

## Laser circuit

GPIO15 → R10 (100 Ω) → Q1 BS170 gate. Drain via J6 to laser. Source to GND.

## PWM tick connection

GPIO5 has **no PCB connection** (no-connect in schematic).
Connect externally: a wire from GPIO2 (servo1 PWM output) to GPIO5 (input).
The falling edge of the 50 Hz servo PWM signal triggers the servo update ISR.

## Verified items (netlist V1.2)

- [x] GPIO0 SDA / GPIO1 SCL → OLED U6 ✓
- [x] GPIO2 servo 1 (J1) ✓
- [x] GPIO3 servo 2 (J2) ✓
- [x] GPIO4 servo 3 (J3) — **new, was NC**
- [x] GPIO5 NC on PCB (external to GPIO2) ✓
- [x] GPIO6 WS2812B (D2) ✓
- [x] GPIO10–11 gyroscope J7 ✓
- [x] GPIO12–14 stepper A (U1 TMC2209) ✓
- [x] GPIO15 laser MOSFET Q1 BS170 ✓
- [x] GPIO16–18 stepper B (U2 TMC2209) ✓
- [x] GPIO19–20 ultrasonic sensor J13 ✓
- [x] GPIO22 servo 4 gripper (J10) ✓
- [x] GPIO26–27 LDR (J8, J9) ✓
- [x] GPIO28 current sensor via U3 OPA2705 ✓
- [x] Stepper driver type: **TMC2209** (KiCad footprint labeled A4988; installed component is TMC2209) ✓
- [x] Microstepping: **1/64** (TMC2209, MS2=H/VIO, MS1=L/GND) → 12800 steps/rev — MS wiring to be physically verified
