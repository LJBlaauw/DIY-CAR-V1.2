# DIY Robot Car — MicroPython op RP2350 (Pico 2 W)

## Systeembeschrijving

Een autonoom rijdend robotkarretje dat een lichtbron opzoekt, er naartoe rijdt en met een grijparm een voorwerp oppakt en dan weer naar de begin positie terug keert.

### Werkvolgorde

1. **LDR-scan** — Draai in één richting een volledige omwenteling + overlap (**370°**) en meet lichtsterkte met twee gepaarde LDR's. Een volledige omwenteling bevat altijd het maximum; de 10° overlap voorkomt dat een piek rond 0°/360° op de rand valt. Er is dus géén pre-roll (terugdraaien vóór de scan) meer nodig.
2. **Uitlijnen** — Draai naar de gevonden richting via de **kortste weg** (door- of terugdraaien, afhankelijk van welke hoek kleiner is). De laatste graden worden altijd in dezelfde richting benaderd zodat de speling/backlash constant blijft. De eindpositie wordt closed-loop bijgeregeld via de LDR's (zie hieronder), zodat stepper-slip gecorrigeerd wordt.
3. **Rijden met LDR-correctie** — Rij naar de lichtbron en corrigeer onderweg continu op koersafwijking: zolang er een verschil tussen de twee LDR-waarden (na gain-correctie) optreedt, stuurt de kar bij richting de helderste kant tot A ≈ B. Stop op ingestelde afstand via de ultrasoonsensor (draait onafhankelijk in de achtergrond, SM4).
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
| Servomotor (optioneel/reserve) | 1 | MG996R | GPIO4 — bedraad op PCB, nog niet aangesloten/geïmplementeerd |
| Ultrasoonsensor | 1 | RCWL-1601 | GPIO19 (Echo), GPIO20 (Trig) |
| LDR | 2 | 10 kΩ spanningsdeler | GPIO26, GPIO27 |
| Laser (kruishaar) | 1 | — | GPIO15 via MOSFET |
| OLED | 1 | SSD1306 | I2C0: SDA=GPIO0, SCL=GPIO1 |
| Gyroscoop/kompas | 1 | GY9250 | I2C1: SDA=GPIO10, SCL=GPIO11 — *nog niet geïmplementeerd* |
| Stroomsensor grijper | — | 0,1 Ω shunt + opamp (gain 14×) | ADC2 = GPIO28 |
| LED | 1 | WS2812B | GPIO6 |
| WiFi | 1 | CYW43 (onboard Pico 2 W) | — (SPI intern, geen vrije GPIO's) |

De controller is een **Pico 2 W**: de RP2350 met onboard CYW43-WiFi. Deze WiFi wordt gebruikt voor de webserver/websocket-besturing (zie module hieronder). De CYW43-driver draait op core 0.

Zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md) voor de volledige GPIO-tabel en verificatiestatus.

---

## Software modules

### PIO State Machine toewijzing

| SM | Module | Functie |
|---|---|---|
| SM0 | stepper/stepper.py | Stapgenerator motor A |
| SM1 | stepper/stepper.py | Stapgenerator motor B |
| SM2 | stepper/stepper.py | Stapenteller (doel-detectie) motor A |
| SM3 | stepper/stepper.py | Stapenteller (doel-detectie) motor B |
| SM4 | ultrasoon/ultrasoon.py | Ultrasoonsensor trigger + echometing |
| SM8 | LDR/ldr_scan_isr.py | Variabele klok voor LDR-sampletiming (PIO2 SM0) |

---

### `lib/stepper/stepper.py`

Dual stappenmotor controller op basis van PIO. Beide motoren lopen onafhankelijk via eigen SM en teller-SM.

**Constanten:**
- `WHEEL_CIRC = 20.94 cm` — wielomtrek
- `STEPS_REV = 12800` — stappen per omwenteling (**1/64 microstepping**, TMC2209: 200 volle stappen × 64). Was 1600 (1/8). MS-pinnen hardwired: **MS1 → GND, MS2 → +5V** (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md)).
- `CM_PER_STEP ≈ 16,4 µm/stap` — 8× fijnere afstands- en positieresolutie dan bij 1/8; gekozen om de stepper-ramp gladder en eenvoudiger te kunnen implementeren (kleinere snelheidssprong per puls).
- `F_PIO = 3 MHz` — mag verhoogd worden voor fijnere delay-resolutie en een verwaarloosbare `speed_to_delay()`-overhead. **Randvoorwaarde:** de STEP-puls moet ≥ ± 100 ns hoog/laag blijven (TMC2209-minimum); bij hogere klok eventueel een extra `nop` in de puls houden.

> **Let op — kalibratie na de overstap naar 1/64:** de vaste `OVERHEAD` in `speed_to_delay()` is bij 8× meer pulsen een groter aandeel van de delay (bij ~30 cm/s ≈ 11% i.p.v. ~1,4%). Hermeet `OVERHEAD`, of verhoog `F_PIO`, zodat de ingestelde snelheid klopt.

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

### `lib/servo/servo_crl.py`

`ServoController` klasse voor 3× MG996R servo (servo 1, 2, 4) + kruishaarlaser.

**Init:** `sc = ServoController()` — alle servo's gaan direct naar rustpositie.

**Rustposities (duty %):**

- De onderstaande rust waarden worden vervangen door de automatisch of handmatig bepaalde rust toestanden.

#| Servo | GPIO | Rust (duty%) | Functie |
#|---|---|---|---|
#| 1 | GPIO2 | 4,1% | Onderste scharnier arm |
#| 2 | GPIO3 | 3,6% | Bovenste scharnier arm |
#| 4 | GPIO22 | 2,0% | Grijper |

Deze waarden worden vervangen door de automatisch of handmatige instelling van de rust posities en op geslagen in een config.json

**PWM-tick:** GPIO5 (INPUT) moet fysiek verbonden zijn met GPIO2 (servo1 PWM). De falling edge triggert de servo-update ISR @ 50 Hz.

**Publieke methoden:**

| Methode | Beschrijving |
|---|---|
| `servo_pos(nr, graden, graden_per_sec)` | Relatief t.o.v. rust (nooit onder rust). Voorbeeld: `servo_pos(1, 30)` → rust+30° |
| `servo_rest(nr=None)` | Terug naar rust. Zonder argument: volgorde 4→1→2→3 @ 20°/s |
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

### `lib/ultrasoon/ultrasoon.py`

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

De ldr-scan routine gebruikt inmiddels een eigen PIO-blok (SM8, zie tabel hierboven) in plaats van hetzelfde blok als de ultrasoonsensor (SM4) — de RP2350 heeft 3 onafhankelijke PIO-blokken. Hiermee is het eerder gemelde PIO-conflict tussen ultrasoon en ldr-scan opgelost; SM4 hoeft niet meer gestopt te worden tijdens de LDR-scan.

---

### `lib/LDR/ldr_scan_isr.py`

LDR-scan module. Draait het karretje via `stepper.rotate()`, samples LDR A en B synchroon met een PIO-klok (SM8 / PIO2 SM0) via ISR.

**Status: bevat nog bekende fouten — in ontwikkeling.**

**Scan-algoritme (herzien):**

1. **Volledige-omwentelingsscan (370°).** Draai in één richting 370° en sample beide LDR's apart. Geen pre-roll/terugdraaien vooraf: een volle omwenteling bevat altijd het maximum en de 10° overlap houdt een piek rond 0°/360° van de rand af. De 0–10° regio komt twee keer in de buffer voor; `_find_peak` neemt de eerste (vroegste) index.
   - *Randvoorwaarde:* de kar moet vrij op zijn as kunnen draaien (differentieel, `WHEEL_BASE_CM`) zonder kabels die bij 360°+ mee-twisten.
2. **Grove piekbepaling per LDR.** De LDR's staan horizontaal uit elkaar, dus er ontstaan twee helderheidsmaxima; de richting van de bron ligt op het **kruispunt A ≈ B** (na gain-correctie), niet op het maximum van de som. Zie ook de "Rijden naar de lichtbron"-sectie.
3. **Kortste weg terug (idee 2).** Na 370° staat de kar op eindhoek 370°; naar piekhoek θ is terugdraaien `370 − θ` en doordraaien `(θ − 10) mod 360`. Kies de kleinste. Laat de laatste graden altijd in dezelfde richting eindigen (bv. altijd in scan-richting; bij terugdraaien iets voorbij en dan vooruit terug), zodat de backlash constant en kalibreerbaar is.
4. **Closed-loop eindpositie via null-seek A−B (idee 3).** Grof naar de verwachte piekpositie op stappen (open-loop), daarna in kleine stapjes bijregelen met `measure_now()`. Gebruik hierbij **niet** de opgeslagen maximum-magnitude als drempel (de bewegings-scan smeert de piek uit, en de absolute helderheid kan intussen gedrift zijn), maar zoek de **nuldoorgang van het verschil A−B** (na gain-correctie):
   - Het verschilsignaal is scherper dan de vlakke som-piek → betere hoekresolutie.
   - Het is ongevoelig voor absolute helderheidsdrift (dimt het licht, dan zakt de som maar blijft de nul op dezelfde hoek).
   - Begrens de fijnregeling tot een venster rond de verwachte positie (bv. ±15°); vind je binnen het venster geen duidelijke nuldoorgang, val dan terug op de stappen-target. Dit voorkomt weglopen bij licht-verandering/ruis.
   Dit corrigeert stepper-slip: de kar draait tot de LDR's daadwerkelijk de uitgelijnde toestand meten, niet tot een geteld stappen-aantal.

**Config:**
- `LDR_PIN_A = GPIO26`, `LDR_PIN_B = GPIO27`
- `WHEEL_BASE_CM = 18,5 cm` — afstand tussen wielen (voor graden→cm omrekening)
- `LDR_GAIN_B = 1,136` — kalibratiefactor voor differentiële meting
- `TARGET_SAMPLES_PER_DEG = 3` (bij 370° ≈ 1110 samples; ~13 KB buffers, ruim binnen RAM)

**Publieke functies:**

| Functie | Beschrijving |
|---|---|
| `scan(dir, speed_cm_s, graden, start_graden, go_max, excel, out_csv)` | Voer scan uit. Retourneert dict met resultaten en piekpositie |
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

### Rijden naar de lichtbron met LDR-correctie

Gesloten stuur-regellus die tijdens het rijden de koers corrigeert op het verschil tussen de twee LDR's. Doel: het verschil `LDR_A − LDR_B` (na gain-correctie) naar nul brengen, want bij A ≈ B wijst de kar recht op de lichtbron (kruispunt tussen de twee LDR-maxima).

**Gekozen aanpak — segmentgewijs** (past op de bestaande fire-and-forget + IRQ-stop stepper-architectuur):

1. Rij een kort segment vooruit (`stepper.mov('f', …)`).
2. Meet beide LDR's (`ldr.measure_now()`).
3. Bepaal de afwijking `A − B`; buiten een **deadband** een kleine `stepper.rotate()`-correctie richting de helderste kant, begrensd in hoek per stap (anti-oscillatie).
4. Herhaal tot de ultrasoonsensor (achtergrond, SM4) de ingestelde stopafstand meldt.

**Randvoorwaarden / afhankelijkheden:**
- Afhankelijk van de **LDR-kalibratie** (correctie van het gevoeligheidsverschil A/B); zonder goede kalibratie stuurt de kar scheef.
- **Arbitrage met het kompas:** tijdens de nadering is de **LDR leidend**; de GY9250-koerscorrectie is bedoeld voor de terugweg. Beide sturen niet tegelijk.

---

### `webserver` — microdot-websocket (Pico 2 W)

Webserver op basis van **microdot** (asyncio) met een websocket, zodat het karretje vanuit een standaard browser bediend en uitgelezen kan worden. Draait op **core 0** naast de besturing (CYW43/lwIP draait daar ook); core 1 blijft voor GY9250 + display.

**Functionaliteit:**
- **Sensoren uitlezen** (browser, ~5–10 Hz): LDR A/B in %, ultrasoon-afstand in cm (of time-out), kompasrichting in graden, servoposities in %, stepper-snelheid/richting/afgelegde weg.
- **Directe rijbesturing:** vooruit/achteruit/links/rechts + snelheid, met een **noodstop** en een **deadman/heartbeat**: bij een verbroken websocket of uitblijvende heartbeat stopt de kar automatisch.
- **Hoog-niveau commando's:** LDR-scan, rijden-naar-licht, grijpen, terugkeren, kalibratiemodus starten/stoppen.

**Netwerk:** AP-modus (kar als eigen accesspoint) heeft de voorkeur voor mobiel gebruik; station-modus optioneel. SSID/wachtwoord in de centrale `config.json`.

**Aandachtspunten / beperkingen (nog te implementeren):**
- **Async-refactor vereist.** De besturing is nu volledig blokkerend (o.a. busy-wait `while sm.active(): pass` in `_wait_stepper_done`, blokkerende `scan()`/ADC-lussen). Die verhongeren de asyncio-event-loop. Nodig: coöperatieve taken met `await asyncio.sleep`, of een commandowachtrij + gedeelde state tussen webserver en een control-task.
- **Geheugen:** microdot + asyncio + lwIP + bestaande modules op ~520 KB RAM is haalbaar maar krap — bewaken.
- **Veiligheid:** commandovalidatie en een verplichte deadman-stop (zie boven), zodat een rijdende kar bij connectieverlies niet doorrijdt.

---

## Bekende issues / TODO

- [ ] `lib/LDR/ldr_scan_isr.py` — herzien scan-algoritme implementeren: 370°-scan (geen pre-roll), piek op A≈B-kruispunt i.p.v. som-maximum, kortste weg terug, en closed-loop uitlijnen via null-seek op A−B
- [ ] Gyroscoop/kompas (GY9250) — nog niet geïmplementeerd, Er is code toegevoegd voor de GY9250 (basic en fusion) moet getest worden
- [ ] OLED (SSD1306) — nog niet geïmplementeerd (voorstel van implementatie in de te doen.md file)
- [x] GPIO-tabel gecontroleerd en geverifieerd tegen KiCad netlist V1.2 (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md))
- [ ] `test_all.py` converteren naar afzonderlijke testfuncties per module, de test all bewaren voor een snelle controlle test.
- [ ] Stepper 1/64: `STEPS_REV = 12800` in code zetten, MS-bedrading (MS1→GND, MS2→+5V) fysiek verifiëren, `OVERHEAD` in `speed_to_delay()` hermeten (of `F_PIO` verhogen).
- [ ] Rijden-naar-licht met LDR-correctie (segmentgewijs) implementeren; afhankelijk van LDR-kalibratie en arbitrage met het kompas.
- [ ] `webserver` (microdot-websocket, Pico 2 W): sensoruitlezing, directe rijbesturing met deadman, hoog-niveau commando's; vereist async-refactor van de blokkerende besturing.

---

## Afhankelijkheden

Standaard MicroPython voor RP2350 (Pico 2 W, met CYW43-WiFi). Externe libraries: `mpu9250`, `mpu6500`, `ak8963` (Tuupola, via awesome-micropython), `ssd1306` (SSD1306 OLED driver), `microdot` (asyncio-webserver met websocket-ondersteuning).
