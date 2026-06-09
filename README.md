# DIY Robot Car — MicroPython op RP2350

## Systeembeschrijving

Een autonoom rijdend robotkarretje dat een lichtbron opzoekt, er naartoe rijdt en met een grijparm een voorwerp oppakt.

### Werkvolgorde

1. **LDR-scan** — Draai (bijv. 360°) en meet lichtsterkte met twee gepaarde LDR's. Bepaal richting van de lichtbron.
2. **Uitlijnen** — Draai terug naar de gevonden richting.
3. **Rijden** — Rij naar de lichtbron; stop op ingestelde afstand via ultrasoonsensor.
4. **Grijpen** — Bestuur twee scharnier-servo's zodat de arm het voorwerp bereikt.
5. **Sluiten grijper** — Servo 4 met stroombeperking (PI-limiter) zodat de servo niet overbelast raakt.
6. **Terugkeren** — Omdraaien en terug naar startpositie; voorwerp neerzetten.
7. *(Optioneel)* Gyroscoop/kompas voor nauwkeurige terugnavigatie — nog niet geïmplementeerd.

---

## Hardware

| Component | Aantal | Type | Aansluiting |
|---|---|---|---|
| Stappenmotor | 2 | — | Driver: TMC2209 |
| Servomotor (arm) | 2 | MG996R | GPIO2, GPIO3 |
| Servomotor (grijper) | 1 | MG996R | GPIO22 — met stroomsensor |
| Ultrasoonsensor | 1 | RCWL-1601 | GPIO19 (Echo), GPIO20 (Trig) |
| LDR | 2 | 10 kΩ spanningsdeler | GPIO26, GPIO27 |
| Laser (kruishaar) | 1 | — | GPIO15 via MOSFET |
| OLED | 1 | SSD1306 | I2C0: SDA=GPIO0, SCL=GPIO1 |
| Gyroscoop/kompas | 1 | GY9250 | I2C1: SDA=GPIO10, SCL=GPIO11 — *nog niet geïmplementeerd* |
| Stroomsensor grijper | — | 0,1 Ω shunt + opamp (gain 14×) | ADC2 = GPIO28 |
| LED | 1 | WS2812B | GPIO6 |

Zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md) voor de volledige GPIO-tabel en verificatiestatus.

---

## Software modules

### PIO State Machine toewijzing

| SM | Module | Functie |
|---|---|---|
| SM0 | stepper.py | Stapgenerator motor A |
| SM1 | stepper.py | Stapgenerator motor B |
| SM2 | stepper.py | Stapenteller (doel-detectie) motor A |
| SM3 | stepper.py | Stapenteller (doel-detectie) motor B |
| SM4 | ultrasoon.py | Ultrasoonsensor trigger + echometing |
| SM8 | ldr_scan_isr.py | Variabele klok voor LDR-sampletiming (PIO2 SM0) |

---

### `lib/stepper.py`

Dual stappenmotor controller op basis van PIO. Beide motoren lopen onafhankelijk via eigen SM en teller-SM.

**Constanten:**
- `WHEEL_CIRC = 20.94 cm` — wielomtrek
- `STEPS_REV = 1600` — stappen per omwenteling (1/8 microstepping, TMC2209)
- `F_PIO = 3 MHz`

**Publieke functies:**

| Functie | Beschrijving |
|---|---|
| `mov(dir, speed, dist)` | Beide motoren gelijktijdig. `dir='f'/'b'`, speed in cm/s, dist in cm |
| `s1(dir, speed, dist)` | Alleen motor A |
| `s2(dir, speed, dist)` | Alleen motor B |
| `rotate(dir, speed, dist)` | Draai op de as. `dir='l'/'r'`, dist in cm (booglengte wiel) |
| `stop()` | Stop beide SM's direct |
| `enable()` / `disable()` | Zet driver-enable aan/uit |
| `status()` | Print positie en status van beide motoren |
| `distance()` | Print en geef PIO-stapafstand terug (cm) |
| `reset_PIO_distance()` | Reset hardware-tellers naar nul |
| `pio_pos1()` / `pio_pos2()` | Lees aantal stapjes motor A/B uit PIO |

**Werking stapgenerator (PIO):**
- `pull(noblock)` haalt een delay-waarde uit de TX FIFO (of hergebruikt de vorige).
- De delay bepaalt de stapfrequentie → snelheid.
- De teller-SM telt STEP-flanken en genereert een IRQ als het doel bereikt is.
- IRQ-handler stopt de bijbehorende motor-SM en werkt de softwarepositie bij.

---

### `lib/servo_crl.py`

`ServoController` klasse voor 3× MG996R servo + kruishaarlaser.

**Init:** `sc = ServoController()` — alle servo's gaan direct naar rustpositie.

**Rustposities (duty %):**
| Servo | GPIO | Rust (duty%) | Functie |
|---|---|---|---|
| 1 | GPIO2 | 4,1% | Onderste scharnier arm |
| 2 | GPIO3 | 3,6% | Bovenste scharnier arm |
| 4 | GPIO22 | 2,0% | Grijper |

**PWM-tick:** GPIO5 (INPUT) moet fysiek verbonden zijn met GPIO2 (servo1 PWM). De falling edge triggert de servo-update ISR @ 50 Hz.

**Publieke methoden:**

| Methode | Beschrijving |
|---|---|
| `servo_pos(nr, graden, graden_per_sec)` | Relatief t.o.v. rust (nooit onder rust). Voorbeeld: `servo_pos(1, 30)` → rust+30° |
| `servo_rest(nr=None)` | Terug naar rust. Zonder argument: volgorde 4→1→2 @ 20°/s |
| `set_rest_pct(nr, pct)` | Stel rustpositie in als duty-% |
| `servo_cur(mA, graden_per_sec)` | **SETPOINT-modus**: PI-regelaar houdt servo 4 op ingestelde stroom |
| `servo_cur_limit(mA)` | **LIMIT-modus**: positie is leidend, stroom wordt begrensd door PI-limiter |
| `update_cur_limit(mA, reset_integrator)` | Pas stroomlimiet aan tijdens runtime |
| `clear_cur_limit()` | Zet LIMIT-modus uit |
| `stop_cur()` | Zet SETPOINT-modus uit |
| `laser_power(percent)` | Laser duty-cycle 0–100% via MOSFET |
| `laser_off()` | Laser uit |
| `close()` | Deinit alle PWM en verwijder IRQ |

**Stroomregeling servo 4 (grijper):**
- Shunt: 0,1 Ω, opamp gain 14×, gemeten op ADC2 (GPIO28).
- EMA-filter (α=0,2) op de gemeten stroom.
- SETPOINT-modus: PI stuurt positie bij zodat stroom op setpoint blijft.
- LIMIT-modus: positie is leidend; PI-limiter trekt terug als stroom de limiet overschrijdt.

---

### `lib/ultrasoon.py`

Ultrasoonsensor RCWL-1601 via PIO (SM4). Meet asynchroon in de achtergrond met ping-pong buffer.

**Config:**
- `PIO_FREQ_HZ = 2 MHz`
- `INTERVAL_MS = 50` — meetinterval
- `TIMEOUT_US = 30 000` — maximale echo-wachttijd (≈ 5 m)

**Publieke functies:**

| Functie | Retourwaarde | Beschrijving |
|---|---|---|
| `read_us()` | `(us, kind)` | kind: `'ok'`, `'timeout'`, `'overflow'` |
| `read_cm()` | `(cm, kind)` | Afstand in cm, afgerond op 0,1 cm |
| `stop()` | — | Stop de SM |

**Let op:** SM4 moet gestopt worden tijdens de LDR-scan (zie `test_all.py`).

---

### `lib/ldr_scan_isr.py`

LDR-scan module. Draait het karretje via `stepper.rotate()`, samples LDR A en B synchroon met een PIO-klok (SM8 / PIO2 SM0) via ISR.

**Status: bevat nog bekende fouten — in ontwikkeling.**

**Config:**
- `LDR_PIN_A = GPIO26`, `LDR_PIN_B = GPIO27`
- `WHEEL_BASE_CM = 18,5 cm` — afstand tussen wielen (voor graden→cm omrekening)
- `LDR_GAIN_B = 1,136` — kalibratiefactor voor differentiële meting
- `TARGET_SAMPLES_PER_DEG = 3`

**Publieke functies:**

| Functie | Beschrijving |
|---|---|
| `scan(dir, speed_cm_s, graden, start_graden, go_max, exell, out_csv)` | Voer scan uit. Retourneert dict met resultaten en piekpositie |
| `measure_now(n=8)` | Direct LDR-waarde lezen (%, tuple A/B) |
| `attach_stepper_reader(fn)` | Koppel stepper.pio_pos1 als positiebron voor de scan |

**`scan()` retourneert:**
```python
{
  'samples': int,        # aantal gemeten samples
  'tick_ms': int,        # gebruikte sample-interval
  'est_time_s': float,   # geschatte scanduur
  'total_deg': float,
  'dist_cm': float,
  'peak_index': int,     # index van lichtmaximum
  'peak_percent': float, # lichtsterkte op maximum (%)
  's1_at_peak': int,     # stapperstand op maximum
  'backtrack': dict,     # terugrijinformatie
  'csv_path': str,       # pad naar CSV (None als niet geschreven)
  'csv_error': str,      # foutmelding (None als ok)
}
```

---

## Bekende issues / TODO

- [ ] `ldr_scan_isr.py` — scan werkt niet correct; bugs aanwezig
- [ ] Gyroscoop/kompas (GY9250) — nog niet geïmplementeerd
- [ ] OLED (SSD1306) — nog niet geïmplementeerd
- [ ] GPIO-tabel controleren op nieuwe RP2350 print (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md))
- [ ] `test_all.py` converteren naar afzonderlijke testfuncties per module

---

## Afhankelijkheden

Standaard MicroPython voor RP2350. Geen externe libraries vereist.
