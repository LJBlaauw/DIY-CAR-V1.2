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
| SM0 | stepper/stepper.py *of* stepper_ramp.py | Stapgenerator motor A |
| SM1 | stepper/stepper.py *of* stepper_ramp.py | Stapgenerator motor B |
| SM2 | stepper/stepper.py *of* stepper_ramp.py | Stapenteller motor A |
| SM3 | stepper/stepper.py *of* stepper_ramp.py | Stapenteller motor B |
| SM4 | ultrasoon/ultrasoon.py | Ultrasoonsensor trigger + echometing |
| SM8 | LDR/ldr_scan_isr.py | Variabele klok voor LDR-sampletiming (PIO2 SM0) |

PIO0 is hiermee vol (4 van 4 SM's). `stepper.py` en `stepper_ramp.py` zijn **alternatieven** — ze claimen dezelfde SM's en dezelfde GPIO's, dus importeer er altijd maar één.

### DMA-kanaaltoewijzing

| Kanaal | Module | Functie |
|---|---|---|
| 2 stuks (dynamisch geclaimd via `rp2.DMA()`) | stepper/stepper_ramp.py | Rampatabel → TX-FIFO van SM0 / SM1 |

De RP2350 heeft 16 DMA-kanalen; `treq_sel = (pio_num << 3) + sm_num` (klopt ook op RP2350, `DREQ_PIO2_TX0 = 16`).

---

### `lib/stepper/stepper.py`

Dual stappenmotor controller op basis van PIO. Beide motoren lopen onafhankelijk via eigen SM en teller-SM.

**Constanten:**
- `WHEEL_CIRC = 19,1 cm` — **gemeten** wielomtrek. Was 20,94 cm; die waarde gaf **8,8 % te korte afstanden** (50 cm gecommandeerd → 45,6 cm gereden). De belaste rolomtrek van een rubberwiel is iets kleiner dan de vrije omtrek, dus deze waarde nog kalibreren (zie *Odometriekalibratie* hieronder).
- `TRACK_WIDTH = 13,6 cm` — spoorbreedte hart-op-hart. Nodig voor elke rotatie- en koersberekening.
- `STEPS_REV = 12800` — stappen per omwenteling (**1/64 microstepping**, TMC2209: 200 volle stappen × 64). MS-pinnen hardwired: **MS1 → GND, MS2 → +5V** (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md)).
- `CM_PER_STEP ≈ 14,9 µm/stap`
- `STEPS_PER_DEG ≈ 159` — stappenverschil tussen de wielen per graad koersverandering. Resolutie dus **0,0063°**.
- `F_PIO = 15 MHz` — 150 MHz sysclk / 10, dus een **integer klokdeler** (geen jitter van de fractionele deler). Bij de topsnelheid van 12800 stappen/s is de delay-waarde 1167 cycles → snelheidsresolutie 0,085 %. De STEP-puls blijft 733 ns, ruim boven het TMC2209-minimum van ~100 ns.

> **`OVERHEAD` in `speed_to_delay()`:** blijft op de op hardware gemeten 9. Bij 15 MHz zakt de resterende snelheidsafwijking bij topsnelheid automatisch van ~2,3 % naar ~0,43 %, dus hermeten is niet meer nodig. (Eerder stond hier ~11 %; dat was te hoog ingeschat.)

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

**Beperking:** er is geen ramp. De snelheid wordt in één keer gecommandeerd, dus vanuit stilstand moet de rotor binnen één microstap (78 µs) naar de eindsnelheid springen. Dat is een oneindige versnelling: de rotor kan het veld niet volgen, verliest synchronisme en de motor blijft zoemend staan. Dit is een **synchronisme-fout, geen koppel-fout** — zie de koppelbegroting hieronder. `stepper_ramp.py` lost dit op.

---

### `lib/stepper/stepper_ramp.py`

Dual stappenmotor controller **met ramp** (PIO + DMA) en doorlopende koerscorrectie. Vervangt `stepper.py`; de publieke API (`mov`, `s1`, `s2`, `rotate`, `stop`, `enable`, `disable`, `status`, `distance`, `pio_pos1/2`, `reset_PIO_distance`) is gelijk gehouden zodat het een drop-in is.

#### Ontwerpbasis (gemeten)

| Grootheid | Waarde |
|---|---|
| Massa kar | 1636 g |
| Wielomtrek / -radius | 19,1 cm / 3,04 cm |
| Spoorbreedte (hart-op-hart) | 13,6 cm |
| Motor | 17HS8401 — NEMA 17, 1,7 A, 52 N·cm, 1,8 Ω, 3,2 mH, rotor 68 g·cm² |
| Driver | TMC2209, standalone, **stealthChop**, 1/64 microstepping |
| VREF / motorstroom | 1,0 V → **0,71 A RMS** (= 42 % van nominaal) |
| Motorspanning | 24 V (6S Li-ion, 25,2 V vol → ~18 V leeg) |
| Bereik snelheid | start 0,1 omw/s = 1,91 cm/s · max 1,0 omw/s = 19,1 cm/s |

**Koppelbegroting** — hieruit volgt dat de ramp de juiste oplossing is en méér stroom niet nodig:

| Post | Koppel per wiel |
|---|---|
| Versnellen van 1636 g bij 55 cm/s² | 1,43 N·cm |
| Rotor-inertie (68 g·cm²) | 0,013 N·cm |
| Rolweerstand (ruim geschat) | ~1,2 N·cm |
| **Totaal nodig** | **~2,7 N·cm** |
| **Beschikbaar bij VREF 1 V** | **~22 N·cm** → factor 8 marge |

Bij 22 N·cm en wielradius 3,04 cm is de trekkracht 7,2 N per wiel (14,5 N totaal) tegen een gewicht van 16,1 N: de **wielen slippen eerder dan dat de motor koppel tekortkomt**. Daarom blijft VREF op 1,0 V (1,8 W dissipatie i.p.v. 10,4 W bij vol stroom). stealthChop volstaat: het praktische plafond is ~300 rpm en de kar draait op 60 rpm. De SPREAD-pin hoeft niet verbonden te worden.

#### FIFO-woordformaat

Elk 32-bit woord in de TX-FIFO codeert een heel **segment** in plaats van één stap:

| Bits | Inhoud |
|---|---|
| 15..0 | aantal stappen in dit segment − 1 (max 65536) |
| 31..16 | delay per stap in PIO-cycles (max 65535) |

Daardoor kost een ramp van 2200 stappen maar 256 woorden = **1 kB**. Eén woord per stap zou 36 kB per ramp zijn (147 kB voor twee ramps × twee motoren) en dat past niet betrouwbaar in de MicroPython-heap. Een kruisfase tot **97,8 cm** past in één enkel woord.

De PIO-lus is 7 instructies; vaste overhead **5 cycles per stap** (0,43 % bij topsnelheid).

#### Fasen van een beweging

| Fase | Bron | CPU-kosten |
|---|---|---|
| Ramp op | DMA-tabel, 256 woorden | **0** — segmenten van ~2 ms zijn te snel voor MicroPython; DMA is hier vereist en immuun voor GC-pauzes |
| Kruisfase **zonder** bijsturing | zelfde DMA-transfer, 1 woord | **0** |
| Kruisfase **met** bijsturing | CPU pusht 1 woord per 20 ms per motor | ~0,1 % (≈100 `put()`/s) |
| Ramp af | DMA-tabel, 256 woorden | **0** |

Een complete beweging van 50 cm zonder bijsturing is **513 woorden = 2052 bytes in één DMA-transfer, één IRQ aan het eind** — dus hetzelfde nul-overhead gedrag als het oude concept, mét ramp.

**Bij een lege FIFO stalt `pull(block)` met STEP laag.** Gevolgen:
- de motor houdt zijn positie, er gaat geen stap verloren;
- CPU-latency (ook een GC-pauze van tientallen ms) beïnvloedt de **staptiming niet** — de PIO genereert met hardware-precisie. Te late CPU betekent dat de correctie één slice later komt, geen timingfout;
- loopt de FIFO écht leeg, dan is het gevolg een korte pauze, geen glitch. Dit is netjes degraderend faalgedrag, in tegenstelling tot een CPU-getimede pulsgenerator.

Met 3 slices vooruit in de FIFO is de runway **60 ms** en de regellatentie eveneens ≤60 ms.

**DMA en CPU mogen nooit gelijktijdig in dezelfde FIFO schrijven** — de volgorde zou door elkaar lopen en de twee motoren konden verschillende segmentvolgordes krijgen, waarmee de ramp én de koers onbetrouwbaar worden. Daarom:

- tijdens de opramp pusht de CPU niets; `Move.service()` wacht tot `dma.active()` van **beide** motoren `False` is. De DMA is dan gestopt met *schrijven* terwijl de FIFO nog data bevat — precies de voorsprong die de CPU nodig heeft.
- de ramp-af-DMA wordt pas gestart nadat de CPU is gestopt met pushen.

> Wachten tot de rampstappen ook *uitgevoerd* zijn zou fout zijn: dan loopt de FIFO leeg en pauzeert de motor tussen ramp en kruisfase.

**Brugsegment.** Als de opramp-DMA klaar is met schrijven staan er nog maximaal 4 woorden in de FIFO. Aan het eind van de ramp zitten we op topsnelheid, dus die 4 woorden zijn samen maar ~3 ms — met `service()` elke 10 ms zou de FIFO alsnog leeglopen. Daarom eindigt de opramptabel met één **brugsegment** op kruissnelheid van `BRIDGE_SLICES × SLICE_MS` = 40 ms. Die stappen horen bij de kruisfase maar worden niet bijgestuurd; een correctie 40 ms eerder of later maakt niets uit.

**S-curve:** de snelheid volgt een smoothstep (3p²−2p³) in de afgelegde weg, dus de versnelling is nul aan begin én eind van de ramp — geen koppelschok op de overgangen. Kost niets extra, want de tabel wordt in Python gegenereerd en daarna alleen door DMA afgespeeld. `ACCEL_CM_S2 = 55` bepaalt de ramp-*afstand* (3,28 cm); de piekversnelling is 1,5× = 82 cm/s² = 0,084 g en de ramp *duurt* 0,55 s.

Een beweging korter dan 2 × 3,28 = 6,6 cm haalt de topsnelheid niet en krijgt automatisch een driehoekig profiel.

#### Exacte afstand

Bijsturen verandert **wanneer** stappen komen, niet **hoeveel**. Per motor wordt `committed` bijgehouden: de som van alle weggeschreven `repeat`-waarden. Het afremmpunt wordt op `committed` bepaald, niet op een gemeten positie, dus de eindafstand is **exact** — onafhankelijk van wanneer de regellus toevallig aanroept. Per slice geldt `repeat_A + repeat_B = 2 × base`, waardoor het midden van de kar precies `base` stappen opschuift terwijl het verschil de koers verandert.

Beide motoren krijgen per slice dezelfde slice-**duur** (20 ms) en een verschillend stappenaantal; daardoor blijven ze in de tijd synchroon (gemeten afwijking < 20 µs op 20 ms).

#### Odometrie — signed, want de teller-SM kent de DIR-pin niet

De teller-SM's tellen **STEP-flanken** en weten niets van de richting. Een ruwe pulsteller is daarom géén positie: bij een rotatie op de plaats lopen beide tellers positief op terwijl de wielen tegengesteld draaien. Elke motor houdt daarom naast de monotone pulsteller een **signed positie** bij die bij elke richtingswisseling het teken meeneemt.

| Functie | Betekenis |
|---|---|
| `pio_pos1()` / `pio_pos2()` | monotone pulsteller (ongesigneerd), gebruikt door `busy()` |
| `_Motor.travel()` | signed positie in stappen: vooruit positief, achteruit negatief |
| `distance()` | `(travel_A + travel_B) / 2 × CM_PER_STEP` — een rotatie geeft ~0 cm, achteruit telt negatief |
| `heading()` | `(travel_A − travel_B) / STEPS_PER_DEG` — werkt ook bij een rotatie op de plaats |

> **Let op:** de tellers tellen *commando's*, geen beweging. Bij wielslip (hobbel, gleuf) lopen ze door. Voor werkelijke beweging is de GY9250 nodig.

#### Koerscorrectie — cascade LDR + gyro

Koersverandering komt van een **verschil in stappenaantal** tussen de wielen. Bij fire-and-forget, waar beide motoren aan hetzelfde totaal vastzitten, geeft een snelheidsverschil netto **nul** koersverandering (de kar maakt een boog en komt in dezelfde richting terug). In de kruisfase ligt dat totaal niet vast, dus daar integreert een snelheidsverschil wél tot een blijvende koersverandering.

| Lus | Sensor | Frequentie | Rol |
|---|---|---|---|
| Buiten | LDR A/B-verschil | enkele Hz (1× per 5 slices) | bepaalt **waar** we heen moeten; A ≈ B = recht op de bron. Levert het setpoint voor de draaisnelheid |
| Binnen | GY9250 gyro-Z | elke slice (~50 Hz) | onderdrukt **storingen**: hobbels, gleuven, ongelijke vloer, wielslip. Regelt het verschil tussen gewenste en gemeten draaisnelheid weg |

Dit is **geen dubbele besturing**: de LDR bepaalt de richting, de gyro alleen de storingsonderdrukking. Bijkomend voordeel: bij een kortdurend afgedekte lichtbron houdt de gyro de koers vast.

De **magnetometer/kompas** wordt tijdens het rijden expres níet gebruikt — de stappenmotoren verstoren het veld (zie de GY9250-stappenmotorkalibratie). Het kompas is voor de terugweg, waar een absolute koers nodig is.

De **acceleratiemeter** dient als slipdetectie: stappen die wel uitgestuurd worden maar geen versnelling opleveren betekenen doorslippende wielen. De hardware-teller kan dat per definitie niet zien.

**Stuurgezag:** bij een snelheidsdifferentie van ±20 % is de draaisnelheid ±32 °/s; bij ±5 % is dat ±8 °/s. Resolutie 0,0063° (1 stap verschil).

#### Waarom niet de PIO-klok variëren

Het alternatief — `SMn_CLKDIV` tijdens runtime wijzigen — is bewust **niet** gebruikt, ook al is het technisch mogelijk via `machine.mem32`:
- het zit buiten het datapad, dus niet synchroon met de segmentgrenzen;
- de stappenaantallen zijn dan niet meer exact bekend;
- de delay-waarden staan in PIO-cycles, dus een klokwijziging herschaalt de rampatabel onderweg (en de versnelling schaalt met f²).

Als globale **snelheids-override** (alles langzamer, bv. bij een obstakel) blijft de klokdeler wel bruikbaar.

#### Publieke API

| Functie | Beschrijving |
|---|---|
| `mov(dir, speed, dist)` | Beide motoren, fire-and-forget mét ramp, nul CPU-overhead |
| `s1(dir, speed, dist)` / `s2(...)` | Idem, één motor |
| `rotate(dir, speed, dist)` | Draai op de as, `dist` = booglengte per wiel in cm |
| `rotate_deg(graden, speed)` | Draai op de as over een hoek. Positief = rechts |
| `drive(dist, speed, correction=fn)` | Beweging **met** doorlopende bijsturing; geeft een `Move` terug |
| `adrive(...)` | asyncio-variant van `drive()` |
| `Move.service()` | Vul de FIFO's bij en pas de correctie toe. ≥1× per 10 ms aanroepen. `False` = klaar |
| `Move.finish()` | Breek de kruisfase af en rem netjes af |
| `HeadingController(ldr_diff, gyro_rate)` | Cascade-koersregelaar; `.output` is de callable voor `correction=` |
| `gyro_z_deg_s(sensor)` | **Vereist** rond de GY9250: de driver levert radialen/s, de regelaar rekent in graden/s |
| `hold_heading(gyro_rate)` | Kortste variant: recht rijden, koers vasthouden op alleen de gyro |
| `halt()` (= `stop()`) | Onmiddellijke stop, drivers blijven **aan**: de motoren houden hun positie. Dit is het gedrag van `stepper.stop()` |
| `brake()` | Nette stop: remt af vanaf de **geschatte** huidige snelheid (uit de voortgang door het profiel) |
| `emergency_stop()` | Noodstop: drivers **uit**, DMA stil, FIFO's leeg. Motoren lopen vrij, de kar kan doorrollen |
| `distance()` / `heading()` | Signed odometrie uit de hardware-tellers |
| `busy()` | True zolang niet alle weggeschreven stappen uitgestuurd zijn |
| `STOP_DIST_CM` | Doelafstand tot het voorwerp (13 cm, gemeten met de ultrasoon) |
| `stopping_distance_cm(speed)` | Afstand die de kar ná `finish()` nog aflegt: afremramp + wat al in de FIFO's staat |
| `info()` | Print alle afgeleide ontwerpgetallen (geen hardware nodig) |
| `meet_frequentie()` | Meet de werkelijke STEP-frequentie tegen `_delay_for()`, om `CYCLES_FIXED` te verifiëren |

**Op tijd beginnen met afremmen.** `finish()` stopt niet onmiddellijk — de afremramp en de al weggeschreven slices liggen vast:

| Snelheid | afremramp | in FIFO | **committed** | + ultrasoonlatentie (50 ms) |
|---|---|---|---|---|
| 19,1 cm/s | 3,28 cm | 1,15 cm | **4,43 cm** | 0,96 cm |
| 10 cm/s | 0,88 cm | 0,60 cm | **1,48 cm** | 0,50 cm |
| 5 cm/s | 0,19 cm | 0,30 cm | **0,49 cm** | 0,25 cm |

```python
doel = sr.STOP_DIST_CM + sr.stopping_distance_cm(snelheid)
if ultrasoon.read_cm() <= doel:
    mv.finish()
```

Rem daarom voor de laatste ~25 cm af naar 5 cm/s: dat brengt de stoponzekerheid van 5,4 cm naar 0,74 cm, en dat is robuuster dan proberen 4,43 cm exact te voorspellen.

> Een volledige stap-voor-stap uitleg van de PIO/DMA/FIFO-werking en van het sturen staat in [stepper_ramp.md](stepper_ramp.md).

Ongeldige invoer wordt geweigerd met `ValueError` (onbekende richting, snelheid ≤ 0, acceleratie ≤ 0, niet-eindige getallen). Een nulafstand geeft `False` terug zonder de hardware aan te raken — een DMA-transfer met `count=0` is firmware-afhankelijk gedrag en wordt vermeden. Een nieuw commando **overschrijft** een lopende beweging (er wordt niet geblokkeerd tot die klaar is).

Een gevraagde snelheid onder `V_START_CM_S` wordt gerespecteerd en levert géén ramp op: de startsnelheid is een *bovengrens* voor veilig starten, niet een minimum.

#### Kentallen

| | |
|---|---|
| Topsnelheid 12800 stappen/s | delay 1167 cycles, resolutie 0,085 % |
| Startsnelheid 1280 stappen/s | delay 11714 cycles |
| Ramp | 2200 stappen = 3,28 cm, 256 segmenten, max snelheidssprong 1,78 % per segment |
| Kruis-slice | 20 ms = 256 stappen |
| 360° op de plaats | 28632 stappen per wiel = 2,24 wielomwentelingen = 2,24 s bij 1 omw/s |
| Noodstop | `ENA` hoog + `dma.active(0)` + `sm.init()` om de FIFO te wissen |

#### Odometriekalibratie (nog uit te voeren)

Twee kalibraties, in te passen in de kalibratiesessie. Zonder deze stuurt de kar structureel scheef, hoe goed de ramp ook is:

1. **Afstandsschaal** — rijd een opgemeten 1,00 m, meet de werkelijk afgelegde afstand, corrigeer `WHEEL_CIRC`. Vangt zowel de resterende meetfout als de belaste rolomtrek.
2. **Rotatieschaal** — commandeer exact 360° (28632 stappen per wiel, tegengesteld), meet de resthoek met de GY9250, corrigeer `TRACK_WIDTH`. De effectieve spoorbreedte is door tyre-scrub meestal 1–5 % groter dan de geometrische 136 mm.

#### Slipdetectie

`HeadingController.slipping` wordt gezet als de **gemeten** draaisnelheid aanhoudend (10 ticks) meer dan 8 °/s afwijkt van de **gecommandeerde** draaisnelheid. Dat betekent dat een wiel slipt.

> Wat níet werkt: "motor actief + weinig voorwaartse versnelling". Bij constante snelheid is de voorwaartse versnelling immers nul, dus die test zou de hele kruisfase als slip melden. **Lineaire** slip (beide wielen slippen recht vooruit) is met wielodometrie en een IMU principieel niet te zien; daarvoor is een externe positiereferentie nodig.

#### Tests

[`tests/test_stepper_ramp_math.py`](tests/test_stepper_ramp_math.py) — **158 pure-Python tests, geen hardware nodig** (`machine` en `rp2` worden gestubd, draait ook op de PC met CPython):

```
python3 tests/test_stepper_ramp_math.py
```

Gedekt: exact stappentotaal van `ramp_words()` / `cruise_words()` / `profile_words()` over het hele afstandsbereik (0 t/m 200 000 stappen), monotone snelheid in beide ramprichtingen, delay- en repeat-velden binnen bereik, driehoeksprofiel, nulafstanden, invoervalidatie, de slice-rekenkunde (gelijke duur, exact midden), de signed odometrie bij rotatie en achteruit, `Move.finish()`, en de DMA → CPU overgang (dat de CPU niet pusht zolang de DMA nog schrijft).

#### Nog te verifiëren op hardware

- `TURN_SIGN` (+1 / −1): welke fysieke motor links of rechts zit volgt niet uit de code. Test met **opgeheven wielen**: een handmatige draai naar rechts moet zowel een positieve gemeten (gyro) als een positieve gewenste draaisnelheid geven.
- `CYCLES_FIXED = 5`: de vaste PIO-cyclusoverhead per stap. `meet_frequentie()` vergelijkt de werkelijke STEP-frequentie met `_delay_for()` en print de geïmpliceerde waarde. Verifieer ook met een **logic analyzer**: die ziet ook de pulsbreedte (verwacht 200 ns) en of er stappen wegvallen.
- Maximale startsnelheid en maximale versnelling, met de GY9250 als onafhankelijke referentie (de PIO-teller kan een stall niet zien). De aangenomen 0,1 omw/s is conservatief — 20 volle stappen/s ligt ruim binnen het pull-in gebied van elke NEMA 17.
- Regelversterkingen `kp_ldr` en `kp_gyro`. De standaard `kp_ldr = 25` geeft bij een vol LDR-verschil 25 °/s, net onder het plafond van 32,2 °/s, dus de regelaar verzadigt normaal niet.
- `fifo_join=PIO.JOIN_TX` is **niet** gebruikt (niet geverifieerd in deze MicroPython-versie). Werkt het, dan verdubbelt de FIFO-runway van 60 naar 140 ms.

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

### Rijden naar de lichtbron — positioneren op de bundelas

**Doel: de kar positioneert zich *recht voor* de lichtbron**, op `STOP_DIST_CM = 13 cm` (ultrasoon) van het voorwerp. Dat is meer dan ernaar kijken, en dat verschil bepaalt het hele algoritme.

#### Twee onafhankelijke grootheden, twee signalen

De lichtbron is een **bundelbron** (spleet), geen rondstraler. Daardoor zijn er twee vrijheidsgraden die je apart moet regelen:

| Signaal | Meet | Doel | Regelaar |
|---|---|---|---|
| **A − B** | de *peiling* naar de bron | nul → recht ernaar kijken | `HeadingController`, per slice |
| **Q** (zie onder) | de *positie* t.o.v. de bundelas | maximaal → recht ervóór staan | zijstap tussen de benen |

> **A − B zegt niets over je positie t.o.v. de bundelas.** Je kunt perfect uitgelijnd staan (A = B) en tóch bijna geen licht ontvangen omdat je de bron schuin van de zijkant bekijkt. Nul je alleen de peiling, dan rijdt de kar een achtervolgingskromme: hij kijkt steeds naar de bron maar houdt zijn laterale afwijking. Dat is geen regelprobleem maar een **observeerbaarheidsprobleem** — geen enkele versterking op A − B lost het op, want de informatie zit er niet in.

#### Genormaliseerde helderheid Q

Ontvangen licht is `E = I(φ)/d²`, met `φ = atan(y/d)` de hoek waaronder de bron de kar ziet en `y` de laterale afwijking van de bundelas. Normaliseer de afstand eruit met de ultrasoon:

```
Q = −(1/γ)·ln(R) + 2·ln(d)          ( = ln I(φ) op een constante na )
```

waarbij de lichtsterktes van beide LDR's worden **opgeteld als `R_A^(−1/γ) + R_B^(−1/γ)`** — niet de weerstanden gemiddeld, want die schaal is logaritmisch en niet lineair in licht.

- **Q constant terwijl je nadert** → je zit op de bundelas (φ = 0 op elke afstand).
- **Q daalt terwijl je nadert** → je zit ernaast, want φ groeit. Uit de daling volgt `y`.

Zonder deze normalisatie is dit niet te meten: van 60 naar 12 cm stijgt de ruwe helderheid al met een factor ~19 door 1/d², en dat overschaduwt elke laterale gradiënt.

#### Waarom het licht dichtbij juist afneemt

Met een gemeten bundel van `w ≈ 33°` (1/e-halfhoek) en een laterale afwijking van 20 cm:

| Afstand | φ | `I(φ)` | 1/d² | ontvangen licht |
|---|---|---|---|---|
| 100 cm | 11,3° | 0,89 | 1,0× | 0,33 |
| 60 cm | 18,4° | 0,73 | 2,6× | 0,70 |
| **40 cm** | 26,6° | 0,52 | 5,2× | **1,00 ← piek** |
| 20 cm | 45,0° | 0,16 | 13× | 0,74 |
| 12 cm | 59,0° | 0,041 | 19× | 0,29 |

De bundelafval verslaat 1/d². Vuistregel: **het ontvangen licht piekt rond `d ≈ 2y` en stort daarna in.**

Omgekeerd geldt: **sta je op de bundelas, dan groeit het signaal juist terwijl je nadert.** Het blindheidsprobleem bestaat alleen zolang je scheef staat — dus op de as komen op middellange afstand lost het volledig op.

#### `y` berekenen uit één recht meetbeen

```
Q₁ − Q₂ = (atan(y/d₂)/w)² − (atan(y/d₁)/w)²        →  oplossen naar y (bisectie)
```

Kleine-hoekbenadering voor het gevoel: `y ≈ w·√( ΔQ / (1/d₂² − 1/d₁²) )`.

Eén rechte rit levert dus de **grootte** van de afwijking; alleen het **teken** kost nog één dither (arc links/rechts, kijken welke kant Q verhoogt). Dat is aanzienlijk goedkoper dan iteratief gradiënt-klimmen.

De beenlengte komt uit de **odometer** (exact tot 15 µm), niet uit de ultrasoon; die wordt alleen gebruikt voor de absolute startafstand. Daarmee werkt de afstandsfout één keer door in plaats van twee keer — een factor 1,4 winst.

#### Faseopbouw

| Fase | Afstand | Snelheid | Signaal | Actie |
|---|---|---|---|---|
| 1 Zoeken | — | — | A − B, 370°-scan | peiling naar de bron |
| 2 Naderen + meten | 60 → 45 cm | 19,1 cm/s | ΔQ | bereken `y` |
| 3 Teken | 45 cm | 8 cm/s | dither | links of rechts van de as |
| 4 Zijstap | 45 cm | 8 cm/s | odometrie | berekende correctie `y` |
| 5 Herhalen | 45→30, 30→20 cm | 19,1 cm/s | ΔQ | verfijnen, `Q` moet vlak worden |
| 6 Afremmen | 25 cm | → **5 cm/s** | — | stopafstand van 4,4 → 0,5 cm |
| 7 Naar binnen | 20 → 13 cm | 5 cm/s | A − B nul, Q bewaken | op de as groeit het signaal |
| 8 Stoppen | 13 cm | — | ultrasoon | `mv.finish()` op `STOP_DIST_CM + stopping_distance_cm()` |

Fase 2 en 5 kosten geen extra tijd — die afstand moet je toch rijden.

#### Foutbegroting

Twee heel verschillende grootheden:

- **Koers (waar de kar naar kijkt): ruim beter dan ±1°.** De A − B nuldoorgang is scherp, de gyro onderdrukt storingen, en de stapresolutie is 0,0063°.
- **Laterale positie t.o.v. de bundelas: dit is de beperking.**

Detectiedrempel voor `y` per meetbeen (beenlengte uit de odometer, ultrasoon ±3 mm, Q-drift 0,01):

| Been | huidige bundel (w ≈ 33°) | spleet gehalveerd (w ≈ 16,5°) |
|---|---|---|
| 60 → 45 cm | 4,55 cm | 2,27 cm |
| 45 → 30 cm | 3,01 cm | 1,50 cm |
| 30 → 20 cm | 2,20 cm | 1,09 cm |
| 20 → 14 cm | 1,72 cm | 0,86 cm |
| **eindafwijking, één doorgang** | **≈ 2 cm** | **≈ 1 cm** |

De eindwaarde wordt gezet door het laatste been waarop je nog kúnt bijstellen (~20 cm). **De beperking is de ultrasoon en de lichtdrift, niet de LDR of de ADC** — ADC-ruis draagt ~0,02 procentpunt bij en is verwaarloosbaar.

Hoe die laterale afwijking zich verhoudt tot wat de grijper kan verdragen staat in *Grijpergeometrie en eindpositionering* hieronder. Kort: **tot een objectbreedte van 4 cm volstaat de huidige opstelling; daarboven is de gehalveerde spleet nodig.**

#### Spleet van de lichtbron

| Effect van halveren | Factor | Beoordeling |
|---|---|---|
| `I(φ)` valt 2× sneller af → Q-signaal ×4 | **y-precisie ×2** | hoofdvoordeel |
| lichtstroom ×0,5 → R van 100 → 162 Ω | | bonus: weg van LDR-verzadiging |
| zoekkegel ×0,5 (halfwaarde 27,5° → 13,7°) | | risico voor fase 1 |

**Zet de spleet verticaal** (hoog en smal): dan versmal je de bundel horizontaal, precies de as waarop je precisie nodig hebt, terwijl hij verticaal breed blijft en hoogteverschillen vergeeft. Een horizontale spleet doet het omgekeerde.

Ter vergelijking: de spleet halveren wint een factor 2, de ultrasoonfout halveren maar √2. De spleet is dus de goedkoopste winst.

#### Randvoorwaarden / afhankelijkheden

- **`γ` (LDR-exponent) en `w` (bundelhalfhoek) moeten gemeten worden** — beide zitten in élke formule hierboven. Zie [`tests/test_ldr_beam.py`](tests/test_ldr_beam.py). De huidige `w ≈ 33°` komt uit één meetpunt met een *aangenomen* `γ = 0,7`; bij `γ = 0,9` is de bundel breder, bij `γ = 0,5` smaller.
- **LDR-gain-kalibratie** (gevoeligheidsverschil A/B); zonder die correctie stuurt de kar scheef.
- **Odometriekalibratie** (`WHEEL_CIRC`, `TRACK_WIDTH`).
- **Hardware/code-koppeling:** na het verlagen van R29/R30 naar **1 kΩ** moet `LDR_R_FIXED_OHM` in [`lib/LDR/ldr_scan_isr.py`](lib/LDR/ldr_scan_isr.py) óók 1000 worden. Blijft die op 10000, dan is elke weerstandswaarde een factor 10 fout zonder dat iets faalt. Met 1 kΩ is het signaal in het werkgebied **7,8× groter** dan met 10 kΩ (7,6 % van de ADC-schaal i.p.v. 0,97 %) — en dat is precies wat de `y`-meting nodig heeft: die vraagt ~34 LSB's, en met 10 kΩ zou het 4,4 LSB's zijn en dus in de ruis verdwijnen.
- **`LDR_R_MIN_OHM = 60` klemt** de procentschaal dichtbij de bron vast op 100 %. Zet hem op ~10, of werk in de eindfase direct in `ln R`.
- **Grijsfilter** over de opening als de LDR fysiek verzadigt (100 Ω is erg laag voor CdS). Geen diffusor en geen kleinere opening — die verpesten de richtingsgevoeligheid.
- **Arbitrage met het kompas:** tijdens de nadering is de **LDR leidend** voor de richting; de gyro-Z doet alleen storingsonderdrukking (dat is dus geen dubbele besturing). De **magnetometer** wordt tijdens het rijden níet gebruikt omdat de stappenmotoren het veld verstoren; die is voor de terugweg, waar een absolute koers nodig is.

> **Waarom niet meer segmentgewijs:** de eerdere aanpak (rij kort segment → stop → meet → `rotate()` → herhaal) is vervangen. Bij segmenten van 1–2 cm wordt de topsnelheid nooit gehaald, want de ramp alleen al is 3,28 cm op plus 3,28 cm af. Bovendien kost stop-draai-rij-door ordes meer CPU en wandkloktijd dan bijsturen per slice (~0,1 % CPU), en een stapsgewijze rotatie kan een hobbel of gleuf niet onderdrukken.

---

### Grijpergeometrie en eindpositionering

De kaken sluiten **horizontaal**, maar de vingertoppen bewegen daarbij naar **voren**. Gemeten:

| | Kaakopening | Toppen t.o.v. ultrasoon |
|---|---|---|
| maximaal open | 9 cm | 12 cm |
| bijna dicht | 2 cm | 15 cm |

```
tip_pos_cm(opening) = 12 + (9 − opening) × 3/7        →  0,43 cm vooruit per cm sluiten
```

> Dit is een **rechte door twee meetpunten**. Een vierstangenmechanisme geeft in werkelijkheid een kromme; één extra meting bij ~5 cm opening laat zien hoeveel dat afwijkt.

#### De ultrasoon is alleen bruikbaar met de arm in rust

Tijdens het rijden staan de servo's in rustpositie: de kaken liggen dan **achter** de ultrasoon en vallen buiten de bundel, dus de afstandsmeting is zuiver. Bij 15° openingshoek is die bundel op 12–15 cm ongeveer **8 cm breed**, en de kaken staan 9 cm open — **zodra de arm uitklapt kijkt de sensor dus naar de eigen vingers.** Vanaf dat moment is er geen afstandsterugkoppeling meer.

#### Drie afgeleide grootheden

| Objectbreedte | Stopafstand `stop_dist_cm()` | Grijpvenster `grip_window_cm()` | Laterale tolerantie `lateral_tolerance_cm()` |
|---|---|---|---|
| 3 cm | 14,6 cm | 12,0 – 14,6 (**2,6 cm**) | ± 3,0 cm |
| 4 cm | 14,1 cm | 12,0 – 14,1 (**2,1 cm**) | ± 2,5 cm |
| 5 cm | 13,7 cm | 12,0 – 13,7 (**1,7 cm**) | ± 2,0 cm |
| 6 cm | 13,3 cm | 12,0 – 13,3 (**1,3 cm**) | ± 1,5 cm |
| 7 cm | 12,9 cm | 12,0 – 12,9 (**0,9 cm**) | ± 1,0 cm |
| 8 cm | 12,4 cm | 12,0 – 12,4 (**0,4 cm**) | ± 0,5 cm |

**Stopafstand:** contra-intuïtief maar juist — een *smaller* voorwerp vraagt een *grotere* stopafstand. Smaller betekent verder sluiten, dus meer vooruitgang van de toppen, dus moet de kar verder terug blijven staan. De default `OBJECT_W_CM = 6.0` geeft `STOP_DIST_CM = 13,3 cm`.

**Grijpvenster:** de vooruitgang van 3 cm is een *gratis* venster bovenop de stopnauwkeurigheid. De ondergrens is conservatief 12 cm; de werkelijke ondergrens wordt bepaald door de **kaakdiepte** (palm t.o.v. toppen), die nog niet is opgemeten.

**Laterale tolerantie:** de kaken vegen tijdens het uitklappen door de ruimte waar het voorwerp staat. Bij een grotere afwijking raakt één kaak het voorwerp en **stoot die het om** — een vervelender faalmodus dan alleen misgrijpen.

> **Voorstel: klap de arm uit bóven het voorwerp en laat hem daarna zakken.** Met twee armscharnieren (servo 1 en 2) kan hetzelfde eindpunt via verschillende paden bereikt worden. Van boven naar beneden komen de kaken om het voorwerp heen in plaats van er horizontaal in; dat elimineert de veegbotsing volledig en kost geen hardware.

#### Laterale nauwkeurigheid versus grijpertolerantie

| Objectbreedte | Grijper-tolerantie | Huidige bundel (≈ ± 2 cm) | Spleet gehalveerd (≈ ± 1 cm) |
|---|---|---|---|
| 3 cm | ± 3,0 cm | ✓ ruim | ✓ ruim |
| 4 cm | ± 2,5 cm | ✓ | ✓ |
| 5 cm | ± 2,0 cm | ⚠ op de grens | ✓ |
| 6 cm | ± 1,5 cm | ✗ te krap | ✓ |
| 7 cm | ± 1,0 cm | ✗ te krap | ⚠ op de grens |

**Hiermee is het halveren van de spleet geen optie meer maar een vereiste** voor voorwerpen breder dan 4 cm.

#### Stilstaand nameten — het laatste correctiemoment

Omdat de bundel vrij is zolang de arm in rust staat, kan er ná het stoppen maar vóór het uitklappen een verse, gemiddelde meting worden gedaan. Dan vallen de rijsnelheid en de meetlatentie uit de fout:

| | Onzekerheid in de afstand |
|---|---|
| stoppen bij 5 cm/s (committed + latentie) | 0,74 cm |
| **stilstaand nameten + kruipcorrectie** | **≈ 0,3 cm** (sensornauwkeurigheid) |

`finetune(read_cm, object_w_cm)` doet dit: gemiddelde over 8 **onafhankelijke** metingen (wachttijd > `INTERVAL_MS` = 50 ms, anders lees je dezelfde gebufferde waarde) en daarna `creep()` naar het doel. Bij 2 cm/s is er praktisch geen ramp nodig, dus de kruipbeweging is meteen exact en zacht.

De **laterale** afwijking kan níet worden nagemeten: de `y`-bepaling uit de Q-daling vraagt beweging over twee afstanden. Stilstaand kan alleen worden geverifieerd dat A − B genulled is, en dat is de *peiling*, niet de laterale positie. **De laterale correctie moet dus af zijn op 20–45 cm.**

#### Eindsequentie

| Stap | Actie | Ultrasoon | Terugkoppeling |
|---|---|---|---|
| 1–7 | naderen, bundelas, afremmen naar 5 cm/s | bruikbaar | LDR + gyro + ultrasoon |
| 8 | stoppen rond `STOP_DIST_CM` | bruikbaar | |
| 9 | **`finetune()`** — stilstaand nameten, gemiddeld | bruikbaar | laatste kans op ± 0,3 cm |
| 10 | **`creep()`** naar `stop_dist_cm(breedte)` | bruikbaar | |
| 11 | arm uitklappen **boven** het voorwerp, dan zakken | **blind** | open-loop |
| 12 | grijpen; toppen lopen 12 → 15 cm vooruit | **blind** | stroombegrenzing via PI op ADC2 |

Vanaf stap 11 is alles open-loop; **stap 9–10 is het laatste correctiemoment.** In stap 12 is er nog wél terugkoppeling via de grijperstroomsensor: een greep in de lucht geeft een andere stroomcurve dan een greep om een voorwerp. Dat is het enige "gelukt of niet"-signaal, en het is er al.

---

### `tests/test_ldr_beam.py` — LDR- en bundelkarakterisering

Meetscript voor de constanten waarop de bundelas-positionering rust. Alles vanuit de REPL aan te roepen.

| Functie | Wat | Waar |
|---|---|---|
| `controleer_config()` | of code en gewijzigde hardware bij elkaar passen (draait bij import) | — |
| `gamma()` | LDR-exponent uit 5 afstanden op de as; fit `ln R = 2γ·ln d`, meldt R² | bank |
| `bundel()` | profiel `I(φ)` op een boog met vaste radius, LDR steeds op de bron gericht | bank |
| `nauwkeurigheid()` | haalbare `y`-resolutie per been, met en zonder gehalveerde spleet | geen hardware |
| `meet_y()` | laterale afwijking uit de Q-daling over één rechte rit | op de kar |
| `dither_teken()` | aan welke kant van de bundelas de kar zit | op de kar |

De twee bankmetingen zijn zo opgezet dat ze elk **één factor isoleren**: `gamma()` houdt φ constant (op de as) en varieert alleen de afstand; `bundel()` houdt de afstand én de tunnelhoek constant en varieert alleen φ. `meet_y()` waarschuwt als A − B niet genulled is — dan meet je tunnelvignettering in plaats van de bundel.

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
- [x] Stepper 1/64: `STEPS_REV = 12800` in code gezet. MS-bedrading (MS1→GND, MS2→+5V) nog fysiek verifiëren. `OVERHEAD` hermeten is vervallen: `F_PIO` staat nu op 15 MHz waardoor de afwijking naar ~0,43 % zakt.
- [x] `WHEEL_CIRC` gecorrigeerd naar de gemeten 19,1 cm (was 20,94 → 8,8 % te korte afstanden). `TRACK_WIDTH = 13,6 cm` toegevoegd.
- [ ] `lib/stepper/stepper_ramp.py` op hardware testen: `TURN_SIGN`, maximale startsnelheid en versnelling meten, regelversterkingen `kp_ldr`/`kp_gyro` afstemmen. Daarna beslissen of `stepper.py` vervalt.
- [ ] **Odometriekalibratie** — afstandsschaal (`WHEEL_CIRC`) en rotatieschaal (`TRACK_WIDTH`) opnemen in de kalibratiesessie. Zonder deze stuurt de kar structureel scheef.
- [ ] Rijden-naar-licht: de segmentgewijze aanpak is **vervangen** door de doorlopende kruisfase met bijsturing per slice in `stepper_ramp.py`, plus positioneren op de **bundelas** via de genormaliseerde helderheid `Q`. Zie de sectie *Rijden naar de lichtbron*.
- [ ] **`LDR_R_FIXED_OHM` naar 1000** in `lib/LDR/ldr_scan_isr.py` — R29/R30 zijn naar 1 kΩ gebracht. Blijft de code op 10000, dan is elke weerstandswaarde stil een factor 10 fout. Zet tegelijk `LDR_R_MIN_OHM` naar ~10, want 60 klemt de schaal dichtbij de bron vast op 100 %.
- [ ] **`γ` en `w` meten** met `tests/test_ldr_beam.py` (`gamma()` en `bundel()`). Beide constanten zitten in élke `y`-berekening; de huidige `w ≈ 33°` komt uit één meetpunt met een aangenomen `γ = 0,7`.
- [ ] **Spleet lichtbron halveren — vereist voor voorwerpen breder dan 4 cm.** De grijper verdraagt maar ±(9 − breedte)/2 cm laterale afwijking, en de huidige bundel haalt ≈ ±2 cm. Zet de spleet **verticaal** (smal horizontaal). Eerst `bundel()` draaien, dan aanpassen, dan controleren of de 370°-scan de bron nog vindt vanaf de werkelijke startpositie.
- [x] Laterale tolerantie van de grijper vastgesteld: **±(9 − objectbreedte)/2 cm**. Zie *Grijpergeometrie en eindpositionering*.
- [ ] **Kaakdiepte opmeten** (palm t.o.v. vingertoppen). Die zet de werkelijke ondergrens van het grijpvenster; nu conservatief op 12 cm gehouden, wat elk venster ~2 cm te smal maakt.
- [ ] **Derde meetpunt van de grijper** bij ~5 cm opening. `tip_pos_cm()` is nu een rechte door twee punten; een vierstangenmechanisme geeft een kromme.
- [ ] **Arm uitklappen boven het voorwerp en dan laten zakken**, in plaats van horizontaal naar voren vegen. Voorkomt dat een kaak het voorwerp omstoot bij een laterale afwijking. Vraagt een gecoördineerde beweging van servo 1 en 2.
- [ ] Grijsfilter over de LDR-openingen als de cel fysiek verzadigt (100 Ω is erg laag voor CdS). Geen diffusor en geen kleinere opening — die verpesten de richtingsgevoeligheid.
- [ ] **Naamgeving stappenmotoren inconsistent**: `stepper.py`/`stepper_ramp.py` noemen GPIO16/17/18 "motor A", maar [hardware/gpio_pinout.md](hardware/gpio_pinout.md) wijst GPIO16/17/18 toe aan stepper **B** (U2) en GPIO12/13/14 aan stepper A (U1). Functioneel geen probleem, maar code en schema spreken elkaar tegen. Kiezen welke de waarheid is.
- [ ] **Voeding U5 (DSN-MINI-360, MP2307) buiten spec**: gespecificeerd ingangsbereik 4,75–23 V, accu levert 22,2 V nominaal en 25,2 V vol geladen. Er is een 28 V pin-compatibele versie gevonden; die inbouwen. Bij een doorgeslagen high-side schakelaar komt 25 V op de 5 V-rail te staan, wat Pico, servo's, OLED en ultrasoon in één keer meeneemt.
- [ ] **Servo-rail scheiden van de logica-rail** (optioneel): alle servo's hangen op +5 V uit U5 (1,8 A continu / 3 A piek). Drie MG996R kunnen samen 3–4,5 A piek trekken. De grijper heeft al stroombegrenzing via de PI-regelaar op ADC2, en de servo's worden langzaam naar hun eindpositie gestuurd, dus in de praktijk blijft de stroom laag. Bij een onverklaarbare Pico-reset tijdens het grijpen is dit de eerste plek om te kijken.
- [ ] Bulk-elco bij de VM-pin van U1/U2 verifiëren (≥100 µF, korte sporen). Bij 24 V is een spanningspiek uit motorkabel-inductie de klassieke doodsoorzaak van een TMC2209. Nooit de motorstekker loskoppelen terwijl VM aan staat.
- [ ] `webserver` (microdot-websocket, Pico 2 W): sensoruitlezing, directe rijbesturing met deadman, hoog-niveau commando's; vereist async-refactor van de blokkerende besturing.

---

## Afhankelijkheden

Standaard MicroPython voor RP2350 (Pico 2 W, met CYW43-WiFi). Externe libraries: `mpu9250`, `mpu6500`, `ak8963` (Tuupola, via awesome-micropython), `ssd1306` (SSD1306 OLED driver), `microdot` (asyncio-webserver met websocket-ondersteuning).
