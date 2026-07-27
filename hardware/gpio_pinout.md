# GPIO Pinout — DIY Robot Car (Revisie V1.2, RP2350)

> **Status: GEVERIFIEERD** — GPIO-tabel is gecontroleerd tegen KiCad netlist V1.2 (export 2026-06-09).

## GPIO-tabel

| GPIO | Richting | Functie | Component / Net | Opmerking |
|------|----------|---------|-----------------|-----------|
| GPIO0 | OUT (SDA) | I2C0 SDA | U6 SSD1306 OLED — net `/SDA0` | Gebruikt als I2C0 SDA (GP0) |
| GPIO1 | OUT (SCL) | I2C0 SCL | U6 SSD1306 OLED — net `/SCL0` | Gebruikt als I2C0 SCL (GP1) |
| GPIO2 | OUT (PWM) | Servo 1 PWM | J1 (JST-XH 3) — MG996R onderste scharnier arm | |
| GPIO3 | OUT (PWM) | Servo 2 PWM | J2 (JST-XH 3) — MG996R bovenste scharnier arm | |
| GPIO4 | OUT (PWM) | Servo 3 PWM | J3 (JST-XH 3) — derde servo | Eerder als NC genoteerd; netlist bevestigt verbinding |
| GPIO5 | IN | PWM-tick voor servo ISR | Niet verbonden op PCB (no-connect marker) | Extern aansluiten op GPIO2 met draad |
| GPIO6 | OUT | WS2812B data | D2 WS2812B — net `Net-(D2-DIN)` | |
| GPIO7 | — | NC | — | |
| GPIO8 | — | NC | — | |
| GPIO9 | — | NC | — | |
| GPIO10 | BIDI (SDA) | I2C1 SDA | J7 pin 4 — GY9250 gyroscoop/kompas | *Nog niet geïmplementeerd* |
| GPIO11 | BIDI (SCL) | I2C1 SCL | J7 pin 3 — GY9250 gyroscoop/kompas | *Nog niet geïmplementeerd* |
| GPIO12 | OUT | DIR stepper A | U1 TMC2209 pin DIR — net `Net-(U1-DIR)` | |
| GPIO13 | OUT | STEP stepper A | U1 TMC2209 pin STEP — net `Net-(U1-STEP)` | |
| GPIO14 | OUT | ENABLE stepper A | U1 TMC2209 pin ~ENABLE — net `Net-(U1-~{ENABLE})` | Low = ingeschakeld |
| GPIO15 | OUT (PWM) | Laser PWM | Q1 BS170 MOSFET gate — net `Net-(Q1-G)` | Via R10 (100 Ω) en Q1 BS170 naar J6 (laser) |
| GPIO16 | OUT | DIR stepper B | U2 TMC2209 pin DIR — net `Net-(U2-DIR)` | |
| GPIO17 | OUT | STEP stepper B | U2 TMC2209 pin STEP — net `Net-(U2-STEP)` | |
| GPIO18 | OUT | ENABLE stepper B | U2 TMC2209 pin ~ENABLE — net `Net-(U2-~{ENABLE})` | Low = ingeschakeld |
| GPIO19 | IN | Echo ultrasoonsensor | J13 pin 2 — net `/Echo` | |
| GPIO20 | OUT | Trigger ultrasoonsensor | J13 pin 3 — net `/Trig` | |
| GPIO21 | — | NC | — | |
| GPIO22 | OUT (PWM) | Servo 4 PWM (grijper) | J10 pin 1 — net `Net-(J10-Pin_1)` | Met stroombeperking via PI; stroomsensor op J10 pin 3 |
| GPIO26 | IN (ADC0) | LDR A | J8 (JST-XH 2) — net `Net-(J8-Pin_1)` | Pull-down R29 (10 kΩ) naar GNDA; filter C13 |
| GPIO27 | IN (ADC1) | LDR B | J9 (JST-XH 2) — net `Net-(J9-Pin_1)` | Pull-down R30 (10 kΩ) naar GNDA; filter C15 |
| GPIO28 | IN (ADC2) | Stroomsensor grijper | R22 (1 kΩ) ← U3B output — net `/I_Servo` | Filter C10; zie stroomkring hieronder |

## Stepper drivers

De schakeling gebruikt **TMC2209** drivers (U1 en U2). De KiCad footprint is gelabeld als A4988 maar de geplaatste component is TMC2209.

### Connector-overzicht

| Connector | GPIO-kant | Motor-kant | Motor |
|-----------|-----------|------------|-------|
| J4 (JST-XH 4) | U1 A4988 | 1A / 1B / 2A / 2B | Stepper A |
| J5 (JST-XH 4) | U2 A4988 | 1A / 1B / 2A / 2B | Stepper B |

### Microstepping instelling (A4988)

MS-pinnen zijn hardwired (TMC2209). De oorspronkelijke 10 kΩ pull-ups zijn verwijderd; MS1 en MS2 worden nu vast gezet voor **1/64 microstepping**:

| Pin | Verbinding | Toestand |
|-----|-----------|---------|
| MS1 | → GND | Laag |
| MS2 | → +5V | Hoog |

TMC2209 waarheidstabel (MS2, MS1): GND/GND → 1/8 · GND/VIO → 1/32 · **VIO/GND → 1/64** · VIO/VIO → 1/16.

Met MS2=H (VIO), MS1=L (GND) → **1/64 microstepping**

→ 200 stappen/omw × 64 = **12800 stappen per omwenteling**

Zelfde geldt voor U2.

> **Historie:** eerder gedocumenteerd als 1/8 (1600 stappen/omw) op basis van de A4988-footprintlogica. De feitelijke driver is een TMC2209 en de MS-bedrading is aangepast naar 1/64 om de stepper-ramp gladder te kunnen implementeren.

## Stroomkring grijper-servo (GPIO28)

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

- IC: U3 OPA2705 (DIP-8 socket) — dual opamp
- Filter C10 (1 nF) op GPIO28
- Net `/I_Servo` verbindt R22 pin 1 met GPIO28

## Servo connectors (JST-XH 3-pin)

Pinvolgorde: **pin 1 = signaal, pin 2 = +5V, pin 3 = GND**

| Connector | GPIO | Servo |
|-----------|------|-------|
| J1 | GPIO2 | Servo 1 — onderste scharnier arm |
| J2 | GPIO3 | Servo 2 — bovenste scharnier arm |
| J3 | GPIO4 | Servo 3 |
| J10 | GPIO22 | Servo 4 — grijper (pin 3 = /Isense) |

## Ultrasoonsensor connector

J13 (JST-XH 4): **pin 1 = GND, pin 2 = Echo (GPIO19), pin 3 = Trig (GPIO20), pin 4 = +5V**

## Laser circuit

GPIO15 → R10 (100 Ω) → Q1 BS170 gate. Drain via J6 naar laser. Source naar GND.

## PWM-tick verbinding

GPIO5 heeft **geen PCB-verbinding** (no-connect in schematic).
Sluit extern aan: een draad van GPIO2 (servo1 PWM output) naar GPIO5 (input).
De falling edge van het 50 Hz servo-PWM signaal triggert de servo-update ISR.

## Geverifieerde items (netlist V1.2)

- [x] GPIO0 SDA / GPIO1 SCL → OLED U6 ✓
- [x] GPIO2 servo 1 (J1) ✓
- [x] GPIO3 servo 2 (J2) ✓
- [x] GPIO4 servo 3 (J3) — **nieuw, was NC**
- [x] GPIO5 NC op PCB (extern naar GPIO2) ✓
- [x] GPIO6 WS2812B (D2) ✓
- [x] GPIO10–11 gyroscoop J7 ✓
- [x] GPIO12–14 stepper A (U1 TMC2209) ✓
- [x] GPIO15 laser MOSFET Q1 BS170 ✓
- [x] GPIO16–18 stepper B (U2 TMC2209) ✓
- [x] GPIO19–20 ultrasoonsensor J13 ✓
- [x] GPIO22 servo 4 grijper (J10) ✓
- [x] GPIO26–27 LDR (J8, J9) ✓
- [x] GPIO28 stroomsensor via U3 OPA2705 ✓
- [x] Stepper driver type: **TMC2209** (KiCad footprint gelabeld A4988; geplaatste component is TMC2209) ✓
- [x] Microstepping: **1/64** (TMC2209, MS2=H/VIO, MS1=L/GND) → 12800 stappen/omw — MS-bedrading fysiek te verifiëren
