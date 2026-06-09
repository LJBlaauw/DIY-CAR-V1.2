# GPIO Pinout — DIY Robot Car (Revisie 2, RP2350)

> **Status: CONTROLEREN** — GPIO-nummers zijn overgenomen uit de software (`_nieuw` versies).
> Verificatie tegen KiCad netlist nog uitvoeren.

## GPIO-tabel

| GPIO | Richting | Functie | Component | Opmerking |
|------|----------|---------|-----------|-----------|
| GPIO0 | OUT (SDA) | I2C0 SDA | SSD1306 OLED | *Nog niet geïmplementeerd* |
| GPIO1 | OUT (SCL) | I2C0 SCL | SSD1306 OLED | *Nog niet geïmplementeerd* |
| GPIO2 | OUT (PWM) | Servo 1 PWM | MG996R — onderste scharnier arm | Ook gekoppeld aan GPIO5 (tick) |
| GPIO3 | OUT (PWM) | Servo 2 PWM | MG996R — bovenste scharnier arm | |
| GPIO4 | — | NC | — | |
| GPIO5 | IN | PWM-tick voor servo ISR | Verbinden met GPIO2 | Falling edge triggert servo-update |
| GPIO6 | OUT | WS2812B datapin | LED | |
| GPIO7–GPIO9 | — | NC | — | |
| GPIO10 | OUT (SDA) | I2C1 SDA | GY9250 Gyroscoop/kompas | *Nog niet geïmplementeerd* |
| GPIO11 | OUT (SCL) | I2C1 SCL | GY9250 Gyroscoop/kompas | *Nog niet geïmplementeerd* |
| GPIO12 | OUT | DIR stepper 2 | TMC2209 driver — motor B | |
| GPIO13 | OUT | STEP stepper 2 | TMC2209 driver — motor B | |
| GPIO14 | OUT | ENB stepper 2 | TMC2209 driver — motor B | Low = enabled |
| GPIO15 | OUT (PWM) | Laser PWM | Kruishaar laser via MOSFET | Deelt PWM-slice met servo's → 50 Hz |
| GPIO16 | OUT | DIR stepper 1 | TMC2209 driver — motor A | |
| GPIO17 | OUT | STEP stepper 1 | TMC2209 driver — motor A | |
| GPIO18 | OUT | ENB stepper 1 | TMC2209 driver — motor A | Low = enabled |
| GPIO19 | IN | Echo ultrasoonsensor | RCWL-1601 | PIO SM4 jmp_pin + in_base |
| GPIO20 | OUT | Trigger ultrasoonsensor | RCWL-1601 | PIO SM4 set_base |
| GPIO21 | — | NC | — | |
| GPIO22 | OUT (PWM) | Servo 4 PWM | MG996R — grijper | Met stroombeperking via PI |
| GPIO26 | IN (ADC0) | LDR A | 10 kΩ spanningsdeler | 10 kΩ naar 3V3, LDR naar GND |
| GPIO27 | IN (ADC1) | LDR B | 10 kΩ spanningsdeler | 10 kΩ naar 3V3, LDR naar GND |
| GPIO28 | IN (ADC2) | Stroomsensor grijper | 0,1 Ω shunt + opamp (gain 14×) | |

## TMC2209 microstepping instelling

Beide drivers zijn hardwired op 1/8 microstepping:

| Pin | Waarde | Resultaat |
|-----|--------|-----------|
| MS1 | 1 (vast) | |
| MS2 | 1 (vast) | 1/8 stap |
| MS3 | 0 (vast) | |

→ 200 stappen/omw × 8 = **1600 stappen per omwenteling**

## Stroomkring grijper-servo (GPIO28)

```
Servo4 GND ── 0,1 Ω shunt ── GND
                   │
              opamp (gain 14×)
                   │
               GPIO28 (ADC2)
```

Formule in software:
```python
V = (adc_raw / 65535) * 3.3
I_A = V / (shunt / gain)   # = V / (0.1 / 14)
I_mA = I_A * 1000
```

## PWM-tick verbinding

GPIO2 (servo1 PWM output) moet fysiek verbonden zijn met GPIO5 (input).
De falling edge van het 50 Hz servo-PWM signaal triggert de servo-update ISR.

## Verificatielijst

- [ ] GPIO2 servo1 → klopt met print?
- [ ] GPIO3 servo2 → klopt met print?
- [ ] GPIO5 tick-ingang → klopt met print?
- [ ] GPIO12–14 stepper 2 → klopt met print?
- [ ] GPIO15 laser → klopt met print?
- [ ] GPIO16–18 stepper 1 → klopt met print?
- [ ] GPIO19–20 ultrasoon → klopt met print?
- [ ] GPIO22 servo4 (grijper) → klopt met print?
- [ ] GPIO26–27 LDR → klopt met print?
- [ ] GPIO28 stroomsensor → klopt met print?
