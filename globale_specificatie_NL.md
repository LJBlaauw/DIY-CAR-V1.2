# DIY Robot Car — MicroPython op RP2350 (Pico 2 W)

## Systeembeschrijving

Een autonoom rijdend robotkarretje dat een lichtbron opzoekt, ernaartoe rijdt, met een grijparm een voorwerp oppakt en daarna naar de beginpositie terugkeert.

### Status van waarden

In dit document worden ontwerpwaarden waar nodig als volgt aangeduid:

- **[GEMETEN]** — rechtstreeks op de huidige robot/hardware gemeten.
- **[BEREKEND]** — afgeleid uit gemeten waarden, datasheetwaarden of softwareconstanten.
- **[AANNAME]** — gebruikt in het ontwerpmodel, maar nog niet voldoende gemeten.
- **[TE VERIFIËREN]** — hardware- of softwaregedrag dat nog expliciet getest moet worden.

Een hoge numerieke resolutie is niet automatisch dezelfde grootheid als fysieke nauwkeurigheid. Wielslip, bandvervorming, sensorbias, mechanische speling en kalibratie blijven afzonderlijke foutbronnen.

### Werkvolgorde

1. **LDR-scan** — Draai in één richting een volledige omwenteling + overlap (**370°**) en meet lichtsterkte met twee gepaarde LDR's. Een volledige omwenteling bevat altijd het maximum; de 10° overlap voorkomt dat een piek rond 0°/360° op de rand valt. Er is dus géén pre-roll (terugdraaien vóór de scan) meer nodig.
2. **Uitlijnen** — Draai naar de gevonden richting via de **kortste weg** (door- of terugdraaien, afhankelijk van welke hoek kleiner is). De laatste graden worden altijd in dezelfde richting benaderd zodat de speling/backlash constant blijft. De eindpositie wordt closed-loop bijgeregeld via de LDR's (zie hieronder). Daarmee worden hoek-/uitlijnfouten door gemiste stappen of mechanische afwijking gecorrigeerd; dit is geen detectie of correctie van algemene lineaire wielslip.
3. **Rijden met LDR-correctie** — Rij naar de lichtbron en corrigeer onderweg continu op koersafwijking: zolang er een verschil tussen de twee LDR-waarden (na gain-correctie) optreedt, stuurt de kar bij richting de helderste kant tot A ≈ B. Stop op ingestelde afstand via de ultrasoonsensor (draait onafhankelijk in de achtergrond, SM4).
4. **Grijpen** — Bestuur twee scharnier-servo's zodat de arm het voorwerp bereikt.
5. **Sluiten grijper** — Servo 4 met stroombeperking (PI-limiter) zodat de servo niet overbelast raakt.
6. **Terugkeren** — Omdraaien en via opgeslagen wielodometrie/trajectinformatie terug naar de startpositie; voorwerp neerzetten. De nauwkeurigheid hiervan wordt begrensd door cumulatieve odometriefout en wielslip.
7. **IMU-ondersteuning (ICM-20948)** — gyro en magnetometer ondersteunen de oriëntatie/koers; ze leveren geen absolute X/Y-positie. Het oorspronkelijke "GY-9250"-bordje bleek bij hardwarevalidatie (2026-09-19) een kale MPU-6500 zonder magnetometer (zie *Bekende issues/TODO*) en is vervangen door een **ICM-20948** (opvolger van de MPU-9250, AK09916-magnetometer in dezelfde behuizing). Driver en scripts staan in `lib/ICM20948/` (`icm20948.py`, `ICM20948_basic.py`, `ICM20948_fusion.py`, `fusion.py`, kalibratiescripts). Gemonteerd op een 150mm mast achterin (buiten de zwaai van de servo-arm), gekalibreerd en in gebruik; het kompas is binnenshuis bruikbaar tot op ca. ±5°. De oude MPU-6500-driver (`lib/GY9250/`) is verwijderd — koerscorrectie (`stepper_ramp.py`, cascade LDR+gyro) gebruikt nu overal de ICM-20948.

---

## Hardware

| Component | Aantal | Type | Aansluiting | extra info |
|---|---|---|---|
| Stappenmotor | 2 | — | Driver: TMC2209 |motor type:17HS8401 |
| Servomotor (arm) | 2 | MG996R | GPIO2, GPIO3 |
| Servomotor (grijper) | 1 | MG996R | GPIO22 — met stroomsensor |
| Servomotor (optioneel/reserve) | 1 | MG996R | GPIO4 — bedraad op PCB, nog niet aangesloten/geïmplementeerd |
| Ultrasoonsensor | 1 | RCWL-1601 | GPIO19 (Echo), GPIO20 (Trig) |
| LDR | 2 | 47 kΩ spanningsdeler (pull-up naar 3V3) | GPIO26, GPIO27 |
| Laser (kruishaar) | 1 | — | GPIO15 via MOSFET |intesity: PWM |
| OLED | 1 | SSD1306 | I2C0: SDA=GPIO0, SCL=GPIO1 |Oled 126x64 pixels 0.96 inch |
| Gyroscoop/accelerometer/magnetometer | 1 | **ICM-20948** (besteld; vervangt het MPU-6500-bordje) | I2C1: SDA=GPIO10, SCL=GPIO11. Adres 0x69 (AD0 hoog) of 0x68 (AD0 laag), driver zoekt zelf; magnetometer AK09916 op 0x0C via I2C-bypass. Driver `lib/ICM20948/icm20948.py`; controle bij levering met `tests/test_icm20948_i2c_scan.py` (verwacht WHO_AM_I `0xEA`, AK09916 `0x48`/`0x09`) |
| Stroomsensor grijper | — | 0,1 Ω shunt + opamp (gain 13×; nominaal 13,05×) | ADC2 = GPIO28 |
| LED | 1 | WS2812B | GPIO6 |type:multicoler 1 wire |
| WiFi/Bluetooth | 1 | Infineon CYW43439 (onboard Pico 2 W) | interne boardverbinding |

## Voedingen

| Onderdeel | spanning |maximale stroom |
|---|---|---|
|batterij|24V |multifuse 2,5A |
|buck convertor|24 naar 5V|5A |
|SEPA convertor|5 naar 3,3V|1A intern van de RP2350 |

## Afmetingen

**Arm (schouder/elleboog) — bijgewerkt naar CAD-exacte waarden (2026-09-21, zie *Armkinematica* verderop).** De oorspronkelijke rolmaat-schattingen bleken 15-50% af te wijken; de kolom "richting" uit de eerste versie van deze tabel is voor deze rijen **geschrapt** — bovenarm en onderarm zijn geen vaste "horizontaal"/"verticaal"-onderdelen, ze draaien mee met J1/J2. Voor de hoek-afhankelijke positie: zie de formule in *Armkinematica*.

| Component | vanaf | tot | maat |
|---|---|---|---|
| Schouderas (J1) | grond | servo-as | **103mm** (was ruw geschat op 95mm) |
| Schouderas (J1) | ultrasoon (voorkant) | servo-as | **37,2mm, áchter de ultrasoon** (was ruw geschat op 36mm — toevallig dichtbij, maar zonder richting) |
| Bovenarm | schouderservo | elleboogservo (J2) | **74,0mm** (was ruw geschat op 50mm) |
| Onderarm | elleboogservo | tandwielcentrum grijpermechanisme | **34,25mm** (was ruw geschat op 35mm — kwam al dicht in de buurt) |

**Grijper/vingers — nog ongewijzigd, met een onopgeloste inconsistentie:**

| Component | vanaf | tot | maat | richting |
|---|---|---|---|---|
| hand | handservo | 1e vingerkootje | 22mm | horizontaalvlak |
| lengte 1e vingerkootje | handservo | 2e kootje | 41mm | horizontaalvlak |
| lengte 2e vingerkootje | 1e kootje | 2e kootje | 32mm | horizontaalvlak |
| lengte vingers | 2e kootje | uiteinde vingers | 30mm | horizontaalvlak |

> **[TE VERIFIËREN]** Rij 2 ("1e vingerkootje: handservo → 2e kootje = 41mm") en rij 3 ("2e vingerkootje: 1e kootje → 2e kootje = 32mm") overlappen: beide eindigen bij "2e kootje" maar beginnen ergens anders. Was rij 2 bedoeld als "1e kootje → 2e kootje", of zit er een tussenliggend punt tussen? Dit bepaalt de exacte `ARM_L3_CM` (pols→vingertop) in `lib/gripper/geometry.py`, nu op 125mm (22+41+32+30) gezet als voorlopige som.

**Chassis — ongewijzigd:**

| Component | vanaf | tot | maat | richting |
|---|---|---|---|---|
| totale lengte CAR | ultrasoon | achterkant batterij | 230mm | horizontaal |
| zwenkwiel | wielas | zwenkwiel | **150mm** (2026-10-22 opgegeven; stond eerder als 155mm "wiel → zwenkwiel") | horizontaal |
| zwenkwiel | grond | montagevlak | 50mm | verticaal |

**Gewichtsverdeling en massa — [GEMETEN] 2026-10-22, arm in rust, zonder last:**

| Steunpunt | Belasting | Aandeel |
|---|---|---|
| Aandrijfwiel links | 0,55 kg | 34 % |
| Aandrijfwiel rechts | 0,55 kg | 34 % |
| Zwenkwiel (150 mm achter de wielas) | 0,50 kg | 31 % |
| **Totaal** | **1,60 kg** | (gewogen totaalmassa: 1636 g, zie *Ontwerpbasis*) |

- **[BEREKEND]** Zwaartepunt: 0,50 / 1,60 × 150 mm ≈ **47 mm achter de wielas** (momentenevenwicht om de wielas).
- De **accu zit bewust helemaal achterin** als contragewicht voor de arm: bij tillen met de grijper schuift het zwaartepunt naar voren, gaat er meer last op de aandrijfwielen en minder op het zwenkwiel. Het zwenkwiel daarom **niet** naar voren verplaatsen; het is ook het achterste steunpunt tegen achterover kantelen.
- **Gevolg voor draaien op de plaats:** het zwenkwiel beschrijft een cirkel met r = 150 mm. Staat het dwars (na een richtingswissel), dan schuift het in plaats van te rollen. Remkoppel ≈ μ × 0,50 kg × 15 cm = **μ × 7,5 kg·cm**, tegen een maximaal aandrijfkoppel van 2 × μ × 0,55 kg × halve spoorbreedte (5,6–6,8 cm) = **μ × 6,2–7,5 kg·cm**. De aandrijfwielen slippen dus eerder dan dat het zwenkwiel schuift, tot het omklapt. Gemeten: `rotate_deg(+360)` op gladde vloer direct na een linksom-beweging gaf 311° (gyro); op een ruwe mat 335° in beide richtingen. Oplossing in software: `rotate_deg_gyro()`; mechanisch: een kogelzwenkwiel (1″ kogel, bv. Pololu #2692 + ~21 mm afstandsstuk).
- **Voor later (massatraagheid):** met de puntmassa's hierboven ligt het draaipunt bij rotatie op de plaats op de wielas, het zwaartepunt 47 mm daarachter. Een eerste schatting van het traagheidsmoment om de verticale as door het wielasmidden vraagt nog de massaverdeling van accu (positie en massa), motoren en arm; die zijn nog niet apart gewogen.

De robotcontroller is een **Raspberry Pi Pico 2 W** met een **RP2350** microcontroller en een onboard **Infineon CYW43439** wireless controller. In MicroPython wordt deze via de CYW43-driver aangestuurd. De webserver/websocket-besturing gebruikt WiFi; de geplande softwarearchitectuur houdt netwerk- en hoofdcontroltaken op core 0 en reserveert core 1 voor ICM-20948/displaytaken. Deze coreverdeling moet nog als geheel op hardware worden gevalideerd.

Zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md) voor de volledige GPIO-tabel en verificatiestatus.

### Ontwikkelomgeving

| Onderdeel | Configuratie |
|---|---|
| Robotboard | Raspberry Pi Pico 2 W |
| Microcontroller | RP2350 |
| Robotfirmware | MicroPython voor Pico 2 W — **exacte versie/build nog vastleggen** |
| Ontwikkel-PC | Fedora Linux 44 |
| Desktop | KDE Plasma |
| PC-tests | CPython; pure-Python tests uit `tests/` |
| Hardwarevalidatie | logic analyzer voor PIO/STEP-timing; multimeter/meetopstelling voor voeding en sensorkalibratie |

> **[TE VERIFIËREN] Reproduceerbaarheid:** leg vóór de definitieve hardwaretest de exacte MicroPython-versie/build en de gebruikte CPython-versie vast. De software gebruikt RP2350-specifieke voorzieningen zoals PIO2 en `rp2.DMA()`.

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
- **[GEMETEN]** `WHEEL_CIRC = 19,1 cm` — wielomtrek. Was 20,94 cm; die waarde gaf **8,8 % te korte afstanden** (50 cm gecommandeerd → 45,6 cm gereden). De belaste rolomtrek van een rubberwiel is iets kleiner dan de vrije omtrek, dus deze waarde nog kalibreren (zie *Odometriekalibratie* hieronder).
- **[GEMETEN]** `TRACK_WIDTH = 13,6 cm` — geometrische spoorbreedte hart-op-hart (alleen documentatie).
- **[GEMETEN]** `TRACK_WIDTH_EFF = 11,26 cm` — effectieve spoorbreedte, gebruikt voor elke rotatie- en koersberekening (`STEPS_PER_DEG`). Gemeten 2026-10-21 met de ICM-20948-gyro: een gecommandeerde 360°-rotatie gaf 434,9° (kompas ~420°). Het ASA-frame zakt onder accu en motoren door, waardoor de banden op de binnenrand van het loopvlak lopen.
- `STEPS_REV = 12800` — stappen per omwenteling (**1/64 microstepping**, TMC2209: 200 volle stappen × 64). MS-pinnen hardwired: **MS1 → GND, MS2 → VCC_IO** *(in de huidige documentatie als +5 V aangegeven; fysiek verifiëren)* (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md)).
- **[BEREKEND]** `CM_PER_STEP ≈ 14,9 µm/stap`
- **[BEREKEND]** `STEPS_PER_DEG ≈ 159` — stappenverschil tussen de wielen per graad koersverandering. Resolutie dus **0,0063°**.
- **[BEREKEND/CONFIG]** `F_PIO = 15 MHz` — 150 MHz sysclk / 10, dus een **integer klokdeler** (geen jitter van de fractionele deler). `speed_to_delay()` geeft bij de topsnelheid van 12800 stappen/s de waarde **576**. De delaylus van déze PIO is twee instructies (`nop().side(0)` + `jmp(y_dec)`), dus elke eenheid is 2 cycles en de **snelheidsresolutie is 0,17 %**. De STEP-puls blijft 11 cycles = **733 ns**, ruim boven het TMC2209-minimum van ~100 ns.
  > De vaak genoemde 1167 cycles en 0,085 % horen **niet** hier maar bij `_delay_for()` in `stepper_ramp.py`, waar de delaylus één cycle per eenheid is. Verwissel die twee niet.

> **`OVERHEAD` in `speed_to_delay()`:** blijft op de op hardware gemeten 9. De PIO-lus kost werkelijk 13 cycles vaste overhead terwijl de formule er 18 aanneemt; bij 15 MHz zakt die **systematische** afwijking bij topsnelheid automatisch van ~2,3 % naar ~0,43 %, dus hermeten is niet meer nodig. (Eerder stond hier ~11 %; dat was te hoog ingeschat.) Daar komt nog de afkapping van `int()` bij, maximaal 2 cycles ≈ 0,17 %.

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

Dual stappenmotor controller **met ramp** (PIO + DMA) en doorlopende koerscorrectie. Vervangt `stepper.py`.

Het is een **API-grotendeels compatibele opvolger, geen drop-in replacement.** De namen `mov`, `s1`, `s2`, `rotate`, `stop`, `enable`, `disable`, `status`, `distance`, `pio_pos1/2` en `reset_PIO_distance` bestaan nog met dezelfde betekenis, maar bij migratie zijn dit de breekpunten:

| Verschil | Gevolg |
|---|---|
| Geen globale `sm0`..`sm3` meer | code die daar direct aan komt, breekt |
| De generator-SM's blijven **permanent actief** en stallen op een lege FIFO | wachten op `sm0.active()` werkt niet meer; gebruik `busy()` |
| `distance()` print niets en geeft **één signed middenafstand** i.p.v. twee motorafstanden | aanroepers die twee waarden uitpakken, breken |
| De bewegingsfuncties geven `True`/`False` i.p.v. impliciet `None` | alleen relevant als de returnwaarde gebruikt werd |
| Andere stop- en voltooiingssemantiek | `halt()` / `brake()` / `emergency_stop()` zijn nu drie verschillende dingen |
| `reset_PIO_distance()` weigert tijdens een beweging | roep eerst `halt()` of `brake()` aan |
| Ongeldige invoer geeft `ValueError` i.p.v. stil door te rekenen | een negatieve `dist` werd eerder stil positief gemaakt |

#### Ontwerpbasis

| Grootheid | Waarde |
|---|---|
| Massa kar | **[GEMETEN]** 1636 g |
| Wielomtrek / -radius | **[GEMETEN/BEREKEND]** 19,1 cm / 3,04 cm |
| Spoorbreedte (hart-op-hart) | **[GEMETEN]** 13,6 cm geometrisch, **11,26 cm effectief** (gyro) |
| Motor | 17HS8401 — NEMA 17, 1,7 A, 52 N·cm, 1,8 Ω, 3,2 mH, rotor 68 g·cm² |
| Driver | TMC2209, standalone, **stealthChop**, 1/64 microstepping |
| VREF / motorstroom | **[BEREKEND, TE VERIFIËREN]** Was 1,0 V (~0,71 A RMS); **verhoogd naar 1,5 V** (2026-09-19) → ca. **1,07 A RMS** volgens huidige driveraannames; controleer sense-resistor/driveruitvoering |
| Motorspanning | 24 V (6S Li-ion, 25,2 V vol → ~18 V leeg). Draait sinds 2026-09-19 ook daadwerkelijk op 24 V i.p.v. eerdere 12V-bankproeven |
| Bereik snelheid | **[ONTWERPWAARDE, TE VERIFIËREN]** start 0,1 omw/s = 1,91 cm/s · max 1,0 omw/s = 19,1 cm/s (huidige `V_MAX_CM_S` in de code, ongewijzigd). Met de hogere voedingsspanning (24 V i.p.v. 12 V, meer headroom om de wikkelinductie snel op te laden) is een **verdubbeling naar 2,0 omw/s = 38,2 cm/s elektrisch haalbaar** — nog niet in de code doorgevoerd, wacht op hardwarevalidatie (max. startsnelheid/versnelling, zie *Nog te verifiëren op hardware*) |

**Koppelbegroting [BEREKEND]** — deze begroting ondersteunt de keuze voor een ramp en laat op papier ruime koppelmarge zien; de werkelijke marge moet met de huidige motorstroom en belasting op hardware worden bevestigd:

| Post | Koppel per wiel |
|---|---|
| Versnellen van 1636 g bij 55 cm/s² | 1,37 N·cm |
| Rotor-inertie (68 g·cm²) | 0,013 N·cm |
| Rolweerstand (ruim geschat) | ~1,2 N·cm |
| **Totaal nodig** | **~2,6 N·cm** |
| **Beschikbaar bij VREF 1,5 V** | **~32,6 N·cm** (was ~22 N·cm bij 1,0 V) → factor ~12,5 marge (was factor 8) |

De post "versnellen" is `m·a/2 × r` = 1,636 kg × 0,55 m/s² ÷ 2 wielen × 0,0304 m; de eigen traagheid van de wielen zit er nog niet in, maar die is bij deze marge niet interessant. Beschikbaar koppel schaalt hier lineair mee met de motorstroom (`52 N·cm × I/1,7 A`, dezelfde aanname als bij de vorige VREF-waarde).

Bij 32,6 N·cm en wielradius 3,04 cm is de trekkracht ~10,7 N per wiel (~21,4 N totaal) tegen een gewicht van 16,1 N — dat overschrijdt het gewicht van de kar, dus de **wielen slippen nog eerder** dan bij VREF 1,0 V, en zeker eerder dan dat de motor koppel tekortkomt. VREF = 1,5 V geeft ~4,1 W dissipatie i.p.v. 10,4 W bij vol stroom (was 1,8 W bij 1,0 V). stealthChop volstaat nog steeds: het praktische plafond is ~300 rpm en de kar draait op 60–120 rpm (bij een eventuele snelheidsverdubbeling). De SPREAD-pin hoeft niet verbonden te worden.

#### FIFO-woordformaat

Elk 32-bit woord in de TX-FIFO codeert een heel **segment** in plaats van één stap:

| Bits | Inhoud |
|---|---|
| 15..0 | aantal stappen in dit segment − 1 (max 65536) |
| 31..16 | delay per stap in PIO-cycles (max 65535) |

Daardoor kost een ramp van 2200 stappen maar 256 woorden = **1 kB**. Eén 32-bit woord per stap zou 2200 × 4 = **8800 bytes ≈ 8,6 KiB per ramp** kosten. Vier losse tabellen (op/af voor twee motoren) zouden samen ongeveer **34,4 KiB** vragen, exclusief Python-objectoverhead. Segmentcodering reduceert dit sterk. Een kruisfase tot **97,8 cm** past in één enkel woord.

De PIO-lus is 7 instructies; vaste overhead **5 cycles per stap** (0,43 % bij topsnelheid).

#### Fasen van een beweging

| Fase | Bron | CPU-kosten |
|---|---|---|
| Ramp op | DMA-tabel, 256 woorden | **0** — segmenten van ~2 ms zijn te snel voor MicroPython; DMA is hier vereist en immuun voor GC-pauzes |
| Kruisfase **zonder** bijsturing | zelfde DMA-transfer, 1 woord | **0** |
| Kruisfase **met** bijsturing | CPU pusht 1 woord per 20 ms per motor | ~0,1 % (≈100 `put()`/s) |
| Ramp af | DMA-tabel, 256 woorden | **0** |

Een complete beweging van 50 cm zonder bijsturing is **513 woorden = 2052 bytes in één DMA-transfer** — hetzelfde nul-overhead gedrag als het oude concept, mét ramp. Er is daarbij **geen interrupt**: `stepper_ramp.py` configureert nergens een DMA- of PIO-IRQ. Tijdens de uitvoering doet de CPU niets, en voltooiing wordt gepolld met `busy()`. (De oude `stepper.py` hierboven had wél `sm2.irq`/`sm3.irq`.)

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

Bijsturen verandert **wanneer** stappen komen, niet **hoeveel**. Per motor wordt `committed` bijgehouden: de som van alle weggeschreven `repeat`-waarden. Het afremmpunt wordt op `committed` bepaald, niet op een gemeten positie, dus het **gecommandeerde gemiddelde stappentotaal is exact** — onafhankelijk van wanneer de regellus toevallig aanroept. De *fysieke* afstand is dat niet: microstepping-nauwkeurigheid, wielslip, bandvervorming en de kalibratie van `WHEEL_CIRC` komen daar nog overheen. Per slice geldt `repeat_A + repeat_B = 2 × base`, waardoor het midden van de kar precies `base` stappen opschuift terwijl het verschil de koers verandert.

Beide motoren krijgen per slice dezelfde slice-**duur** (20 ms) en een verschillend stappenaantal; daardoor blijven ze in de tijd synchroon (**berekende** afrondingsfout < 20 µs op 20 ms; nog niet met een logic analyzer nagemeten).

#### Odometrie — signed, want de teller-SM kent de DIR-pin niet

De teller-SM's tellen **STEP-flanken** en weten niets van de richting. Een ruwe pulsteller is daarom géén positie: bij een rotatie op de plaats lopen beide tellers positief op terwijl de wielen tegengesteld draaien. Elke motor houdt daarom naast de monotone pulsteller een **signed positie** bij die bij elke richtingswisseling het teken meeneemt.

| Functie | Betekenis |
|---|---|
| `pio_pos1()` / `pio_pos2()` | monotone pulsteller (ongesigneerd), gebruikt door `busy()` |
| `_Motor.travel()` | signed positie in stappen: vooruit positief, achteruit negatief |
| `distance()` | `(travel_A + travel_B) / 2 × CM_PER_STEP` — een rotatie geeft ~0 cm, achteruit telt negatief |
| `heading()` | `(travel_A − travel_B) / STEPS_PER_DEG` — werkt ook bij een rotatie op de plaats |

> **Let op:** de tellers tellen STEP-pulsen/commando's, geen gegarandeerde fysieke wielverplaatsing. Bij slip of een gemiste motorstap kan de odometrie dus afwijken. De ICM-20948 levert een onafhankelijke meting van draaisnelheid en oriëntatie, maar geen absolute lineaire X/Y-positie; lineaire slip van beide wielen vraagt een externe positiereferentie om rechtstreeks te detecteren.

#### Koerscorrectie — cascade LDR + gyro

Koersverandering komt van een **verschil in stappenaantal** tussen de wielen. Bij fire-and-forget, waar beide motoren aan hetzelfde totaal vastzitten, geeft een snelheidsverschil netto **nul** koersverandering (de kar maakt een boog en komt in dezelfde richting terug). In de kruisfase ligt dat totaal niet vast, dus daar integreert een snelheidsverschil wél tot een blijvende koersverandering.

| Lus | Sensor | Frequentie | Rol |
|---|---|---|---|
| Buiten | LDR A/B-verschil | enkele Hz (1× per 5 slices) | bepaalt **waar** we heen moeten; A ≈ B = recht op de bron. Levert het setpoint voor de draaisnelheid |
| Binnen | ICM-20948 gyro-Z | elke slice (~50 Hz) | onderdrukt **storingen**: hobbels, gleuven, ongelijke vloer, wielslip. Regelt het verschil tussen gewenste en gemeten draaisnelheid weg |

Dit is **geen dubbele besturing**: de LDR bepaalt de richting, de gyro alleen de storingsonderdrukking.

Let op wat de binnenlus wél en niet doet. Hij regelt het verschil tussen gewenste en gemeten **draaisnelheid** weg, dus hij dempt een verstoring *zolang die plaatsvindt*. Hij integreert geen hoekfout, dus de verdraaiing die er onderaan de streep uit komt wordt daarna niet teruggedraaid; de LDR-buitenlus doet dat. Zonder LDR (`damp_yaw_rate()`) is er dus **geen koershouden** — alleen demping, met bovendien blijvende drift door gyrobias. Bij een kortdurend afgedekte lichtbron blijft de kar daarom ongeveer rechtdoor gaan, maar hij komt niet op de oude koers terug.

De **magnetometer/kompas** wordt tijdens het rijden expres níet gebruikt — de stappenmotoren verstoren het veld (zie de ICM-20948-stappenmotorkalibratie). Het kompas is voor de terugweg, waar een absolute koers nodig is.

De **acceleratiemeter** wordt níet voor slipdetectie gebruikt. Dat idee — "stappen zonder versnelling betekent slip" — werkt niet: bij constante snelheid is de voorwaartse versnelling per definitie nul, dus normaal rijden en volledig doorslippen zien er identiek uit.

Wat er wél in zit is een **koersvolgfout** op gyro-Z: wijkt de gemeten draaisnelheid aanhoudend af van wat er werkelijk is weggeschreven, dan staat `HeadingController.yaw_tracking_error` op `True`. Dat is een *aanwijzing*, geen slipbewijs — regeldynamiek, gyrovertraging, verzadiging en een verkeerde gain geven hetzelfde beeld. Lineaire slip (beide wielen recht vooruit) is er sowieso niet mee te zien; dat vraagt een externe positiereferentie.

**Stuurgezag:** bij een snelheidsdifferentie van ±20 % is de draaisnelheid ±32 °/s; bij ±5 % is dat ±8 °/s. Resolutie 0,0063° (1 stap verschil). Die 32 °/s geldt **bij topsnelheid**: het gezag is een fractie van de rijsnelheid en schaalt dus evenredig mee (bij 5 cm/s nog 8,4 °/s). `turn_authority_deg_s(rate)` rekent dat uit en `Move()` zet er het plafond van de `HeadingController` mee.

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
| `rotate_deg_gyro(graden, gyro_rate, speed=5)` | Draai op de as, gestuurd op de **gyro** (`gyro_rate = gyro_z_deg_s(sensor)`). Remt op de gyrohoek met een onderweg geleerde ondergrondfactor `k` en corrigeert daarna langzaam tot ±0,5°. Nodig omdat de stappenhoek afhangt van ondergrond en draairichting: `rotate_deg(360)` gaf op gladde vloer 311–370°, op een ruwe mat 335°. Getest 2026-10-21 op gladde vloer: ±360° met fout 0,33/0,38° (laser 3 mm / 6 mm op 50 cm). Blokkeert ~8,5 s per 360° bij 5 cm/s |
| `drive(dist, speed, correction=fn)` | Beweging **met** doorlopende bijsturing; geeft een `Move` terug |
| `adrive(...)` | asyncio-variant van `drive()` |
| `Move.service()` | Vul de FIFO's bij en pas de correctie toe. ≥1× per 10 ms aanroepen. `False` = klaar |
| `Move.finish()` | Breek de beweging af en rem netjes af — ook tijdens de opramp, waar eerst de lopende DMA transactioneel wordt gewist en daarna vanaf de wérkelijke snelheid wordt afgeremd |
| `HeadingController(ldr_diff, gyro_rate)` | Cascade-koersregelaar. Geef de **controller zelf** mee aan `correction=`: dan zet `Move()` zijn stuurgezag op de werkelijke rijsnelheid en krijgt hij terugkoppeling over wat er is weggeschreven |
| `gyro_z_deg_s(sensor, sign=None)` | **Vereist** rond de ICM-20948: de driver levert radialen/s, de regelaar rekent in graden/s. Past `GYRO_Z_SIGN` toe |
| `damp_yaw_rate(gyro_rate)` | Demping van de draaisnelheid op alleen de gyro. **Houdt geen koers vast** — heette daarom eerder ten onrechte `hold_heading()`; zie de cascade-uitleg hierboven |
| `turn_authority_deg_s(rate)` | Maximale draaisnelheid bij een gegeven rijsnelheid |
| `halt()` (= `stop()`) | Onmiddellijke stop, drivers blijven **aan**: de motoren houden hun positie. Dit is het gedrag van `stepper.stop()` |
| `brake()` | Nette stop: remt af vanaf de **geschatte** huidige snelheid (uit de voortgang door het profiel) |
| `emergency_stop()` | Noodstop: drivers **uit**, DMA stil, FIFO's leeg. Motoren lopen vrij, de kar kan doorrollen |
| `distance()` / `heading()` | Signed odometrie uit de hardware-tellers. `heading()` is positief naar rechts, hetzelfde teken als `rotate_deg()` en als de bijsturing |
| `creep(dist, speed)` | Kleine correctiebeweging; de enige functie met een **signed** afstand |
| `busy()` | True zolang niet alle weggeschreven stappen uitgestuurd zijn |
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
doel = approach.brake_target_cm(snelheid)   # lib/gripper/approach.py
cm, kind = ultrasoon.read_cm()
if kind == 'ok' and cm <= doel:
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
2. **Rotatieschaal** — commandeer exact 360° (28632 stappen per wiel, tegengesteld), meet de resthoek met de ICM-20948, corrigeer `TRACK_WIDTH`. De effectieve spoorbreedte is door tyre-scrub meestal 1–5 % groter dan de geometrische 136 mm.

#### Koersvolgfout (voorheen "slipdetectie")

`HeadingController.yaw_tracking_error` wordt gezet als de **gemeten** draaisnelheid aanhoudend (10 ticks) meer dan 8 °/s afwijkt van de draaisnelheid die er **werkelijk is weggeschreven**.

Twee dingen daarbij, waar dit eerder misging:

- **Vergelijk met wat er echt gebeurd is, niet met het setpoint.** Het stuurgezag is een fractie van de rijsnelheid, dus bij lage snelheid is er veel minder gezag dan de ±32 °/s bij topsnelheid. Werd er tegen het geklemde setpoint vergeleken, dan kon de regelaar 25 °/s "commanderen" terwijl de wielen er fysiek maar ~8 °/s konden leveren — en dat verschil las als slip. `Move()` zet daarom het plafond op `turn_authority_deg_s(r1)` en meldt via `note_applied()` terug wat er na afronden en aftoppen echt is weggeschreven.
- **Een afwijking is geen slipbewijs.** Regeldynamiek, gyrovertraging, verzadiging en een verkeerd afgestelde gain geven exact hetzelfde beeld. De vlag heet daarom `yaw_tracking_error` en niet `slipping`.

> Wat níet werkt: "motor actief + weinig voorwaartse versnelling". Bij constante snelheid is de voorwaartse versnelling immers nul, dus die test zou de hele kruisfase als slip melden. **Lineaire** slip (beide wielen slippen recht vooruit) is met wielodometrie en een IMU principieel niet te zien; daarvoor is een externe positiereferentie nodig.

#### Tests

[`tests/test_stepper_ramp_math.py`](tests/test_stepper_ramp_math.py) — **226 pure-Python tests, geen hardware nodig** (`machine` en `rp2` worden gestubd, draait ook op de PC met CPython):

```
python3 tests/test_stepper_ramp_math.py
```

Gedekt: exact stappentotaal van `ramp_words()` / `cruise_words()` / `profile_words()` over het hele afstandsbereik (0 t/m 200 000 stappen), monotone snelheid in beide ramprichtingen, delay- en repeat-velden binnen bereik, driehoeksprofiel, nulafstanden, invoervalidatie, de slice-rekenkunde (gelijke duur, exact midden), de signed odometrie bij rotatie en achteruit, `Move.finish()`, en de DMA → CPU overgang (dat de CPU niet pusht zolang de DMA nog schrijft).

Als regressie ook de drie toestandsfouten die hierin zaten: `finish()` tijdens de opramp-DMA (waarbij `busy()` eeuwig `True` bleef), een nieuw commando tijdens een lopende beweging (DIR en tellers gingen om zonder eerst te stoppen), en de tekenconventie (`rotate_deg(+90)` gaf `heading() = −90`).

[`tests/test_gripper_geometry.py`](tests/test_gripper_geometry.py) — 32 tests voor de grijpergeometrie, eveneens zonder hardware.

#### Nog te verifiëren op hardware

- **Drie losse tekenconventies.** Ze corrigeren verschillende fouten en horen dus niet in één constante — een omgekeerd gemonteerde ICM-20948 vraagt een ander gyroteken zonder dat er iets mankeert aan de motorbekabeling. Publiek geldt overal: **positief = naar rechts**.
  - `MOTOR_TURN_SIGN` (+1 / −1): welke fysieke motor links of rechts zit. Test: `rotate_deg(+90)` moet naar **rechts** draaien en `heading()` moet daarna ≈ +90 geven.
  - `GYRO_Z_SIGN` (+1 / −1): opgeheven wielen, draai de kar met de hand naar rechts — `gyro_z_deg_s(sensor)()` moet **positief** zijn.
  - `LDR_DIFF_SIGN` (+1 / −1): houd een lamp rechts van de kar — `ldr_diff()` moet **positief** zijn.
- `CYCLES_FIXED = 13`: de vaste PIO-cyclusoverhead **per stap binnen een segment** (STEP hoog `mov[7]` + `nop[2]` = 11, delaylus = delay + 1, `jmp(y_dec)` = 1). De STEP-puls was eerst 3 cycles (200 ns); daarmee miste de TMC2209 ~40 % van de pulsen (`rotate_deg(+90)` draaide 54°). Met 733 ns, zoals `stepper.py`: 90,8° (gemeten 2026-10-21). Daar komt per **segmentovergang** nog 4 cycles bij voor `pull`/`out`/`out`/`mov`, die niet in de formule zitten; de effectieve overhead is dus `13 + 4/repeat`. Bij de 256 rampsegmenten is dat 0,02 %, bij segmenten van 1 stap 0,4 %. De waarde is **berekend, nog niet gemeten**. `meet_frequentie()` vergelijkt de werkelijke STEP-frequentie met `_delay_for()`; meet zowel lange segmenten als segmenten van 1, 2, 8 en 256 stappen, en controleer met een **logic analyzer** ook de pulsbreedte (verwacht 733 ns) en of er stappen wegvallen.
- Maximale startsnelheid en maximale versnelling, met de ICM-20948 als onafhankelijke referentie (de PIO-teller kan een stall niet zien). De aangenomen 0,1 omw/s is conservatief — 20 volle stappen/s ligt ruim binnen het pull-in gebied van elke NEMA 17.
- **Snelheidsverdubbeling (2026-09-19).** Met VREF 1,0→1,5 V en de motor nu ook echt op 24 V i.p.v. eerdere 12V-bankproeven is `V_MAX_CM_S` elektrisch ruim onderbouwd te verdubbelen (19,1 → 38,2 cm/s, zie *Ontwerpbasis*/koppelbegroting hierboven) — bewust nog niet doorgevoerd in `stepper_ramp.py`, want dat verandert ook ramptijd, remweg (`stopping_distance_cm()`) en alle kentallen die op 19,1 cm/s gebaseerd zijn. Eerst hier valideren, dan pas `V_MAX_CM_S` aanpassen én de kentallen-tabel herzien.
- Regelversterkingen `kp_ldr` en `kp_gyro`. De standaard `kp_ldr = 25` geeft bij een vol LDR-verschil 25 °/s, net onder het plafond van 32,2 °/s, dus de regelaar verzadigt normaal niet.
- `fifo_join=PIO.JOIN_TX` is **niet** gebruikt (niet geverifieerd in deze MicroPython-versie). Werkt het, dan verdubbelt de FIFO-runway van 60 naar 140 ms.

---

### `lib/servo/servo_crl.py`

`ServoController` klasse voor 3× MG996R servo (servo 1, 2, 4) + kruishaarlaser.

**Init:** `sc = ServoController()` — alle servo's gaan direct naar rustpositie.

**Kalibratie (duty %), per servo — handmatig gekalibreerd 2026-09-19:**

Bron: [`config.json`](config.json) (sectie `"servo"`). `lib/servo/servo_crl.py` leest dit bestand bij het opstarten; ontbreekt het (of de servo-sectie erin), dan gelden dezelfde waarden als hardgecodeerde fallback (`DEFAULT_REST_PCT`/`DEFAULT_MAX_PCT` in dat bestand). **Elke servo heeft nu zijn eigen duty-bereik** (`pct_min[nr]`/`pct_max[nr]`) — dit vervangt de oude, gedeelde 2,5–12,5%-mapping die niet bij de werkelijke hardware paste.

| Servo | GPIO | Functie | Rust = 0° (duty%) | Max = 180° (duty%) | Overig |
|---|---|---|---|---|---|
| 1 | GPIO2 | Onderste scharnier arm (schouder) | 3,08% (ingetrokken) | 13,0% (gemeten uiterste) | Horizontaal gestrekt: 12,5% (`horizontal_pct`, losse referentiepositie — zit niet in de graden-mapping) |
| 2 | GPIO3 | Bovenste scharnier arm (elleboog) | 2,6% (ingetrokken) | 13,0% (gemeten uiterste) | — |
| 4 | GPIO22 | Grijper | 3,0% (maximaal open) | 8,6% (maximaal dicht) | Rust = open, zodat de grijper bij `servo_rest()` teruggaat naar de open stand |

`rest_pct[nr]` is een **losse, vrij aanpasbare** waarde (initieel gelijk aan `pct_min[nr]`, dus rust ligt bij 0°) — `set_rest_pct()` verandert alleen waar "rust" naartoe wijst, niet de onderliggende graden↔duty-mapping (`pct_min`/`pct_max` zelf), zodat de huidige fysieke positie van een servo niet ongewild springt zodra de rustwaarde wijzigt.

> **"Graden" is een genormaliseerde schaal, geen letterlijke hoekmeting.** De werkelijke mechanische slag van de gebruikte servo's is iets **groter dan 180°**. `servo_pos()`/`_deg_to_pct()` verdelen 0–180° lineair tussen `pct_min[nr]` (rust) en `pct_max[nr]` (max) — dat zijn de betrouwbare, gemeten ankerpunten; "180°" zelf is puur een normalisatie-eenheid, geen exacte fysieke hoek.

> **Correctie:** de eerder in `te_doen_NL.md` genoemde "grijper nooit meer dan 90° open"-veiligheidsgrens klopt niet (bevestigd door de gebruiker) en is losgelaten. De harde `pct_min`/`pct_max`-klem per servo (hierboven) is de eigenlijke veiligheidsgrens en vervangt een aparte hoek-gebaseerde regel.

**PWM-tick:** GPIO5 (INPUT) moet fysiek verbonden zijn met GPIO2 (servo1 PWM). De **rising edge** triggert de servo-update ISR @ 50 Hz — bewust niet de falling edge, die verschuift met de gecommandeerde servohoek (zie `lib/servo/servo_crl.py`). Via `attach_tick_callback()` kan een externe, allocatievrije functie op dezelfde harde IRQ meeliften; dat wordt gebruikt voor fase-gesynchroniseerde LDR-sampling, zie *Rijden naar de lichtbron met LDR-correctie* → **Fase-gesynchroniseerde achtergrond-sampling** hieronder.

**Publieke methoden:**

| Methode | Beschrijving |
|---|---|
| `servo_pos(nr, graden, graden_per_sec)` | Relatief t.o.v. rust (nooit onder rust). Voorbeeld: `servo_pos(1, 30)` → rust+30° |
| `servo_rest(nr=None)` | Terug naar rust. Zonder argument: volgorde 4→1→2 @ 20°/s |
| `set_rest_pct(nr, pct)` | Stel rustpositie in als duty-%, geklemd tussen `pct_min[nr]`/`pct_max[nr]` van díe servo |
| `servo_cur(mA, graden_per_sec)` | **SETPOINT-modus**: PI-regelaar houdt servo 4 op ingestelde stroom |
| `servo_cur_limit(mA)` | **LIMIT-modus**: positie is leidend, stroom wordt begrensd door PI-limiter |
| `update_cur_limit(mA, reset_integrator)` | Pas stroomlimiet aan tijdens runtime |
| `clear_cur_limit()` | Zet LIMIT-modus uit |
| `stop_cur()` | Zet SETPOINT-modus uit |
| `laser_power(percent)` | Laser duty-cycle 0–100% via MOSFET |
| `laser_off()` | Laser uit |
| `close()` | Deinit alle PWM en verwijder IRQ |

**Stroomregeling servo 4 (grijper):**
- Shunt: 0,1 Ω, niet-inverterende opamp gain `1 + 47 kΩ / 3,9 kΩ = 13,05×` (in software 13,0×), gemeten op ADC2 (GPIO28).
- Terugrekening: `I = V_ADC / (R_shunt × gain)`; bij 1 A is `V_ADC ≈ 1,3 V`.
- EMA-filter (α=0,2) op de gemeten stroom.
- SETPOINT-modus: PI stuurt positie bij zodat stroom op setpoint blijft.
- LIMIT-modus: positie is leidend; PI-limiter trekt terug als stroom de limiet overschrijdt.

**`config.json` (projectroot):** eerste stap van het geplande centrale configuratiebestand (zie *Bekende issues/TODO* → calibraties). Bevat nu alleen een `"servo"`-sectie, met per servo-nummer (als string-key: `"1"`, `"2"`, `"4"`) `rest_pct`, `max_pct` en optioneel `horizontal_pct`, plus vrije `label`/`note`-velden voor documentatie. Wordt uitsluitend gelezen door `servo_crl.py` bij het instantiëren van `ServoController`; ontbreekt het bestand of de sectie, dan gelden de hardgecodeerde `DEFAULT_REST_PCT`/`DEFAULT_MAX_PCT` in dat bestand. ICM-20948-, LDR- en netwerk-secties zijn nog niet geïmplementeerd (zie de betreffende to-do-items).

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

> **API-conventie:** `read_us()` en `read_cm()` retourneren altijd een tuple `(waarde, kind)`. Controleer `kind == 'ok'` voordat de waarde numeriek wordt gebruikt; bij `timeout` of `overflow` mag de afstand niet als geldig getal in de regelaar terechtkomen.

De ldr-scan routine gebruikt inmiddels een eigen PIO-blok (SM8, zie tabel hierboven) in plaats van hetzelfde blok als de ultrasoonsensor (SM4) — de RP2350 heeft 3 onafhankelijke PIO-blokken. Hiermee is het eerder gemelde PIO-conflict tussen ultrasoon en ldr-scan opgelost; SM4 hoeft niet meer gestopt te worden tijdens de LDR-scan.

---

### `lib/LDR/ldr_scan_isr.py`

LDR-scan module. Draait het karretje via `stepper.rotate()`, samples LDR A en B synchroon met een PIO-klok (SM8 / PIO2 SM0) via ISR.

**Status: herzien algoritme geïmplementeerd; nog op hardware verifiëren.**

**Scan-algoritme (herzien):**

1. **Volledige-omwentelingsscan (370°).** Draai in één richting 370° en sample beide LDR's apart. Geen pre-roll/terugdraaien vooraf: een volle omwenteling bevat altijd het maximum en de 10° overlap houdt een piek rond 0°/360° van de rand af. De 0–10° regio komt twee keer in de buffer voor; de helderste geldige A−B-nuldoorgang wordt gekozen.
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
- `WHEEL_BASE_CM = 11,26 cm` — effectieve spoorbreedte, voor graden→cm omrekening. **Stond op 18,5 cm** (370°-scan draaide 503°) en daarna op de geometrische 13,6 cm (nog ~21 % te veel, gyro-meting 2026-10-21). Moet gelijk blijven aan `TRACK_WIDTH_EFF` in `stepper_ramp.py`; een restcorrectie die alleen voor de scan geldt, hoort in `ROT_SCALE`.
- `ROT_SCALE_L = 360 / 369,95`, `ROT_SCALE_R = 1,0` — correctie per draairichting. Gemeten 2026-10-21 met `stepper.rotate()` 360° bij 5 cm/s: rechtsom 360,0°, linksom 370,1° en 369,8° op de gyro (laserstip 8 cm verder op 50 cm ≈ 9,1°). Linksom draait de kar ~2,7 % verder, waarschijnlijk door het zwenkwiel en de gewichtsverdeling.
- `RETURN_FORWARD_ONLY = True` — na de scan altijd doordraaien in scanrichting naar de lamp in plaats van de kortste weg. Bij een richtingswissel klapt het zwenkwiel om (ruk en slip).
- **[CONFIG]** `LDR_R_FIXED_OHM = 47 000` — pull-up naar 3V3 (R29/R30), ook in `config.json` → `"ldr"`. **Geschiedenis: 10 000 → 1000 → 10 000 → 47 000.** Met de huidige lamp loopt de LDR van ~4,7 kΩ (15 cm) tot ~450 kΩ (4 m); 47 kΩ ligt op het geometrisch midden en geeft ~350–1000 echte 12-bit codes per e-voud over dat hele bereik (rond 0° ≈ 16–46 codes per graad koersfout). 200 kΩ wint boven ~2 m, maar zakt bij 13 cm (grijpafstand) van ~16 naar ~4 codes per graad. Oude toelichting: De stap naar 1 kΩ was voor meer resolutie op de bundelas (1 kΩ geeft ~9,1–16,7 % full-scale i.p.v. ~0,99–1,96 % bij 10 kΩ voor een LDR van 100–200 Ω, ca. 372–683 tegen 41–80 echte 12-bit counts), maar is met de nieuwe, fellere lichtbron weer teruggedraaid. **[TE VERIFIËREN]** of de eerdere zorg — dat de bundelas-Q-daling van ~34 LSB met 10 kΩ nog maar ~4,4 LSB overhoudt en in de ruis verdwijnt — met de fellere bron nog geldt. MicroPython `read_u16()` schaalt deze ruwe ADC-waarde naar 16 bit, dus 12-bit ADC-counts en `read_u16()`-counts mogen niet door elkaar worden gebruikt.
- **[CONFIG]** `LDR_R_MIN_OHM = 100` — ondergrens procentschaal (100 %). Was 60 en daarna 20 Ω; met 47 kΩ is het gemeten minimum ~4,7 kΩ (15 cm), 100 Ω laat ruim marge voor een fellere bundelas.
- **[CONFIG]** `LDR_R_MAX_OHM = 5 000 000` — **was 20 000**, waardoor de procentschaal al vlakbij vastklemde (4 m ≈ 300–450 kΩ). 5 MΩ ligt ruim boven 4 m (extrapolatie ~15 m) en onder de donkerwaarde (~8–15 MΩ).
- **[GEKALIBREERD 2026-10-19]** `LDR_B_GAMMA = 1,0772`, `LDR_B_LN_C = −0,6005` (ook in `config.json` → `"ldr"`) — vervangen de vaste `LDR_GAIN_B = 1,136`. Cel B wordt naar cel A omgerekend met een machtsverband: `ln R_B' = (ln R_B − LN_C) / GAMMA`. De cellen verschillen in helling, niet in een vaste factor: de beste vaste gain liet op 0,15–4 m tot 0,20 restfout in ln R over (~5° koersfout), de machtscorrectie 0,036 (~1°). Meetdata en analyse: [`Meting_LDR_afstand_hoek.md`](Meting_LDR_afstand_hoek.md).
- **Richtingssignaal** `ldr_diff = ln R_A − ln R_B'` (`diff_from_adc()`, `measure_diff()`, `read_synced_diff()`): positief = bron rechts (B is de rechter cel), dus `LDR_DIFF_SIGN = +1`. Op 0,5 m: ±0,02 bij 1°, ±0,2 bij 5°, ±0,75 bij 10°, ±1,4 bij 15°; monotoon tot ±15°, teken klopt tot ±25°. Bewust geen (A−B)/(A+B), die explodeert als beide cellen donker zijn.
- `TARGET_SAMPLES_PER_DEG = 3` — bij 370° ≈ 1110 opgeslagen scanpunten. De drie resultaatarrays gebruiken samen ~13 KB; de drie 10-elements-ringbuffers voegen circa 80 bytes toe.

**Fysieke montage (gemeten, 2026-09-19):**
- LDR's zitten **55 mm boven de grond**, **40 mm uit elkaar** in het horizontale vlak (zie ook punt 2 hierboven, "LDR's staan horizontaal uit elkaar"). De **laser (kruishaar) zit precies in het midden** tussen de twee LDR's.
- Elke LDR zit in een **koker van 5 mm diameter**, op **15 mm diepte** vanaf de opening. Dat geeft de LDR een eigen richtinggevoeligheid ("tunnelhoek", zie `tests/test_ldr_beam.py`), los van de bundelbreedte van de lichtbron zelf (zie *Rijden naar de lichtbron* → Spleet van de lichtbron):
  ```
  tunnelhoek (halfhoek) = atan(straal / diepte) = atan(2,5 / 15) ≈ 9,5°   (volledig ≈ 19°)
  ```
  Dit is de **ontvangsthoek van de sensor**; de "spleet"-discussie elders gaat over de **uitzendhoek van de bron**. Beide beperken de bruikbare hoek, maar zijn onafhankelijke, niet-optelbare grootheden (de een is bij de ontvanger, de ander bij de zender) — verwar ze niet met elkaar bij een foutbegroting.

**Delertopologie:** `R_FIXED` is de **pull-up** naar 3V3, de LDR de pull-down naar GND. Fel licht → lage LDR-weerstand → **lage** ADC-waarde. `_adc_to_res_ohm()` rekent daarop. Controle: schijn licht op LDR A en lees de ruwe ADC; gaat die naar **nul**, dan is de aanname juist.

**100Hz-netrimpel:** een 230V-ledlamp geeft voornamelijk 100Hz-rimpel (periode circa 10 ms). `measure_now()` middelt daarom 10 samples met 1 ms tussenruimte. De bewegende `scan()` gebruikt nu hetzelfde tijdvenster zonder te blokkeren: SM8 triggert iedere 1 ms, waarna een vooraf gealloceerde ringbuffer van tien A/B/stepperposities met lopende sommen wordt bijgewerkt. Ieder opgeslagen scanpunt bevat het gemiddelde van de laatste tien LDR-samples én de gemiddelde stepperpositie uit exact hetzelfde venster. De filtergroepsvertraging van circa 4,5 ms wordt daardoor niet als hoekverschuiving geïnterpreteerd. De 1kHz-data worden met een fase-accumulator gedecimeerd naar circa drie opgeslagen punten per graad, zodat het RAM-gebruik rond 1110 scanpunten blijft.

**Fase-gesynchroniseerde achtergrond-sampling (alternatief voor `measure_now()`):**

`measure_now()` middelt de netrimpel weg door **over** een volle 10ms-periode te samplen — correct, maar blokkerend. Een tweede aanpak, gekoppeld aan de servo-PWM-tick (zie *Servo-aansturing* → PWM-tick hierboven): sample bewust altijd op **dezelfde fase** t.o.v. de netrimpel, getriggerd door de rising edge van servo1's 50Hz-PWM (GPIO2→GPIO5). Dat middelt de rimpel niet weg, maar **bevriest** hem op één punt van de golf — geen ruis meer (de fase ligt vast), wel een **statische bias** ter grootte van de rimpelwaarde op dat punt. Voor een differentieel signaal (A−B, of een Q-verschil tussen twee tijdstippen) valt die bias grotendeels weg, want beide kanalen zien 'm in dezelfde IRQ-call vrijwel gelijktijdig; voor een absolute meting verdwijnt hij in de gefitte constanten (`GAMMA`, `LDR_GAIN_B`).

> **Servo-PWM-klok en 230V-net zijn onafhankelijke oscillators** — de fase driftlangzaam weg, orde seconden-tot-minuten voor een volledige cyclus. "Statisch" geldt dus per meting of per rijsegment, niet over een sessie van meerdere minuten (bv. de `input()`-gestuurde bank-metingen in `gamma()`/`bundel()`).

Implementatie: `ServoController.attach_tick_callback(fn)` (`lib/servo/servo_crl.py`) roept `fn` aan **in de harde PWM-IRQ zelf**, vóór het schedulen van de servo-update — dus zo dicht mogelijk op de fysieke flank. `lib/LDR/ldr_scan_isr.py`'s `_sync_tick()` gebruikt dat om `SYNC_BURST_N = 8` samples per tick (back-to-back, geen 1ms-spacing — dat middelt alleen ADC-/thermische ruis, niet de netrimpel) op te tellen in een ping-pong-buffer; `read_synced()` leest niet-blokkerend de laatst-klare buffer (max ~20 ms oud, één servo-PWM-frame). Slaat over tijdens een actieve `scan()`, omdat twee IRQ's die tegelijk hetzelfde ADC-kanaal omschakelen elkaars mux-instelling kunnen verstoren.

**Publieke functies:**

| Functie | Beschrijving |
|---|---|
| `scan(dir, speed_cm_s, graden=370, start_graden=0, go_max=True, excel=True, out_csv=...)` | 370°-scan met 1kHz-sampling, 10×1ms-ringgemiddelde en uitlijning op A−B=0 |
| `measure_now(n=10, interval_ms=1)` | Direct LDR-waarde lezen (%, tuple A/B), 1 ms-gemiddeld tegen 100Hz-netrimpel (blokkerend, ~10 ms) |
| `enable_irq_timing(True/False)` / `irq_timing_stats()` | Meet optioneel de ISR-uitvoeringstijd (gemiddelde tijd in µs gedeeld door 10 is het CPU-percentage bij 1 kHz), het grootste gat tussen twee ISR-starts (`gap_max_us`) en het aantal gemiste 1ms-ticks (`missed_ticks`) |
| `start_synced_sampling(servo_controller)` / `stop_synced_sampling(...)` | Koppel/ontkoppel LDR-sampling aan de 50Hz-servo-PWM-tick |
| `read_synced()` | Niet-blokkerend: laatste fase-gesynchroniseerde LDR-waarde (%, tuple A/B), max ~20 ms oud |
| `attach_stepper_reader(fn)` | Koppel stepper.pio_pos1 als positiebron voor de scan |

**`scan()` retourneert:**
```python
{
  'samples': int,                 # aantal gemeten samples
  'sample_ms': 1,                  # vaste ADC-sampleperiode
  'average_samples': 10,           # ringvenster van 10 ms
  'output_interval_ms': float,     # interval tussen opgeslagen scanpunten
  'irq_timing': dict,              # optionele ISR-duur- en gatmeting
  'est_time_s': float,            # geschatte scanduur
  'total_deg': 370.0,
  'dist_cm': float,
  'alignment': dict,              # bron, nuldoorgang en beide LDR-piekindices
  'alignment_angle_deg': float,   # grove richting binnen 0..360 graden
  'peak_a_index': int,
  'peak_b_index': int,
  'backtrack': dict,              # kortste route + backlash-benadering
  'null_seek': dict,              # resultaat begrensde closed-loop fijnregeling
  'csv_path': str,
  'csv_error': str,
}
```

---

### Rijden naar de lichtbron — positioneren op de bundelas

**Doel: de kar positioneert zich *recht voor* de lichtbron**, op `STOP_DIST_CM = 13,3 cm` (ultrasoon) van het voorwerp — bij de default `OBJECT_W_CM = 6,0 cm`. Rond dit niet af naar 13 cm: bij een **ontwerpdoel** voor de eindnauwkeurigheid van ~0,3 cm eet die afronding de hele foutbegroting op. Dat is meer dan ernaar kijken, en dat verschil bepaalt het hele algoritme.

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

Zonder deze normalisatie is dit niet te meten: van 100 naar 12 cm stijgt de 1/d²-term al met een factor ~19 (zie de tabel hieronder), en dat overschaduwt elke laterale gradiënt.

#### Waarom het licht dichtbij juist afneemt

Met de huidige **[AANNAME/VOORLOPIGE SCHATTING]** `w ≈ 33°` (1/e-halfhoek) en een laterale afwijking van 20 cm:

| Afstand | φ | `I(φ)` | 1/d² | ontvangen licht |
|---|---|---|---|---|
| 100 cm | 11,3° | 0,89 | 1,0× | 0,33 |
| 60 cm | 18,4° | 0,73 | 2,6× | 0,70 |
| **40 cm** | 26,6° | 0,52 | 5,2× | **1,00 ← piek** |
| 20 cm | 45,0° | 0,16 | 13× | 0,74 |
| 12 cm | 59,0° | 0,041 | 19× | 0,29 |

De kolom `1/d²` is genormeerd op de rij van 100 cm en rekent met de **schuine** afstand √(d² + y²), niet met `d` alleen; de laatste kolom is `I(φ)` × die factor, genormeerd op de piek.

De bundelafval verslaat 1/d². Vuistregel: **het ontvangen licht piekt rond `d ≈ 2y` en stort daarna in.**

Omgekeerd geldt: **sta je op de bundelas, dan groeit het signaal juist terwijl je nadert.** Het blindheidsprobleem bestaat alleen zolang je scheef staat — dus op de as komen op middellange afstand lost het volledig op.

#### `y` berekenen uit één recht meetbeen

```
Q₁ − Q₂ = (atan(y/d₂)/w)² − (atan(y/d₁)/w)²        →  oplossen naar y (bisectie)
```

Kleine-hoekbenadering voor het gevoel: `y ≈ w·√( ΔQ / (1/d₂² − 1/d₁²) )`.

Eén rechte rit levert dus de **grootte** van de afwijking; alleen het **teken** kost nog één dither (arc links/rechts, kijken welke kant Q verhoogt). Dat is aanzienlijk goedkoper dan iteratief gradiënt-klimmen.

De beenlengte komt uit de **odometer** (nominale stapresolutie 14,9 µm — dat is de resolutie, niet de nauwkeurigheid: microstepping, slip en de kalibratie van `WHEEL_CIRC` zijn een orde grover), niet uit de ultrasoon; die wordt alleen gebruikt voor de absolute startafstand. Daarmee werkt de afstandsfout één keer door in plaats van twee keer — een factor 1,4 winst.

#### Faseopbouw

| Fase | Afstand | Snelheid | Signaal | Actie |
|---|---|---|---|---|
| 1 Zoeken | — | — | A − B, 370°-scan | peiling naar de bron |
| 2 Naderen + meten | 60 → 45 cm | 19,1 cm/s | ΔQ | bereken `y` |
| 3 Teken | 45 cm | 8 cm/s | dither | links of rechts van de as |
| 4 Zijstap | 45 cm | 8 cm/s | odometrie | berekende correctie `y` |
| 5 Herhalen | 45→30, 30→20 cm | 19,1 cm/s | ΔQ | verfijnen, `Q` moet vlak worden |
| 6 Afremmen | 25 cm | → **5 cm/s** | — | stopafstand van 4,4 → 0,5 cm |
| 7 Naar binnen | 20 → 13,3 cm | 5 cm/s | A − B nul, Q bewaken | op de as groeit het signaal |
| 8 Stoppen | 13,3 cm | — | ultrasoon | `mv.finish()` op `approach.brake_target_cm()` |

Fase 2 en 5 kosten geen extra tijd — die afstand moet je toch rijden.

#### Foutbegroting

Twee heel verschillende grootheden:

- **Koers (waar de kar naar kijkt): ruim beter dan ±1°.** De A − B nuldoorgang is scherp, de gyro onderdrukt storingen, en de stapresolutie is 0,0063°.
- **Laterale positie t.o.v. de bundelas: dit is de beperking.**

Detectiedrempel voor `y` per meetbeen (beenlengte uit de odometer, ultrasoon ±3 mm, Q-drift 0,01):

| Been | voorlopig model (w ≈ 33°) | model met gehalveerde spleet (w ≈ 16,5°) |
|---|---|---|
| 60 → 45 cm | 4,55 cm | 2,27 cm |
| 45 → 30 cm | 3,01 cm | 1,50 cm |
| 30 → 20 cm | 2,20 cm | 1,09 cm |
| 20 → 14 cm | 1,72 cm | 0,86 cm |
| **eindafwijking, één doorgang** | **≈ 2 cm** | **≈ 1 cm** |

Deze getallen komen uit `nauwkeurigheid()` in [`tests/test_ldr_beam.py`](tests/test_ldr_beam.py).

De eindwaarde wordt gezet door het laatste been waarop je nog kúnt **bijstellen**, en dat is het been 30 → 20 cm (2,20 cm, afgerond ≈ 2 cm). Het been 20 → 14 cm staat er alleen ter informatie in: daar kan wel gemeten maar niet meer gecorrigeerd worden, want de zijstap moet vóór 20 cm klaar zijn (zie *Stilstaand nameten*). **De beperking is de ultrasoon en de lichtdrift, niet de LDR of de ADC** — ADC-ruis draagt ~0,02 procentpunt bij en is verwaarloosbaar.

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

- **`γ` (LDR-exponent) en `w` (bundelhalfhoek) moeten gemeten worden** — beide zitten in élke formule hierboven. Zie [`tests/test_ldr_beam.py`](tests/test_ldr_beam.py). De huidige `w ≈ 33°` komt uit één meetpunt met een *aangenomen* `γ = 0,7`; bij `γ = 0,9` is de bundel breder, bij `γ = 0,5` smaller. **Onafhankelijke schatting (2026-09-19):** de lichtbron heeft volgens opgave een **openingshoek van ca. 90°, en daarna vrijwel donker** (geen geleidelijke uitdoving). Datasheet-"openingshoek" is typisch een halve-intensiteit-specificatie, dezelfde definitie als `halfwaarde_halfhoek = w × 0,8326` in `bundel()` — terugrekenen geeft `w ≈ 45 / 0,8326 ≈ 54°`, merkbaar breder dan de huidige `w ≈ 33°`-aanname. Dit is een ruwe fabrieksopgave, geen vervanging voor een echte `bundel()`-meting, maar wel een aanwijzing dat de huidige `w` een onderschatting kan zijn.
  **Constructie (gemeten):** de lichtspleet is **verticaal, 8 mm breed**, met de LED's **~8 mm achter het front**. Een zuivere geometrische aperture-inschatting voor een puntbron op die diepte (`atan(halve breedte / diepte) = atan(4/8) ≈ 26,6°` halfhoek) ligt merkbaar smaller dan de gemeten ~45°, dus de LED's stralen zelf al breder uit dan de spleet minimaal zou toelaten — de spleet vignetteert gedeeltelijk, maar is niet de enige beperkende factor. **[TE VERIFIËREN]** Een scherpe "90° en dan donker"-overgang past eigenlijk beter bij een (deels) aperture-gedomineerd, hard afgekapt profiel dan bij de nu aangenomen **Gaussische** bundelvorm (`exp(-(φ/w)²)`), die per definitie geleidelijk uitdooft en nooit echt "donker" wordt. Dit is een aanwijzing dat het Gaussische model bij grote hoeken kan afwijken van de werkelijkheid, ook al is het dichtbij de as (waar de kar meestal werkt) waarschijnlijk een goede benadering — pas te beoordelen met een echte `bundel()`-meting.
- **LDR-gain-kalibratie** (gevoeligheidsverschil A/B) — **gedaan 2026-10-19** met een machtscorrectie, zie `LDR_B_GAMMA`.
- **Odometriekalibratie** (`WHEEL_CIRC`, `TRACK_WIDTH`).
- **Hardware/code-koppeling:** R29/R30 zijn nu **47 kΩ** (eerder 10 kΩ → 1 kΩ → 10 kΩ), en `LDR_R_FIXED_OHM` in [`lib/LDR/ldr_scan_isr.py`](lib/LDR/ldr_scan_isr.py) en `config.json` staan op 47 000. Oude toelichting: Blijft die op 1000 staan terwijl de hardware op 10 kΩ zit, dan is elke weerstandswaarde een factor 10 fout zonder dat iets faalt — `tests/test_ldr_beam.py` z'n `controleer_config()` vangt dat verschil af. **[TE VERIFIËREN]** de gevoeligheidszorg die destijds tot de stap naar 1 kΩ leidde: de `y`-meting moet een ΔQ van ~0,01 zien, wat met 1 kΩ ongeveer **38 u16-LSB's** was tegen ~4,5 LSB's met 10 kΩ — een factor 8,4. Of de nieuwe, fellere lichtbron dat verschil compenseert (hoger signaalniveau, dus effectief meer LSB's per ΔQ) moet nog op hardware bevestigd worden. De 10×1ms-middeling in `measure_now()` en `scan()` (zie boven) onderdrukt alleen de 100Hz-netrimpel, niet deze ADC-quantisatiegrens.
- `LDR_R_MIN_OHM` staat nu op **100 Ω**; dit laat ruime marge onder het gemeten minimum van ~4,7 kΩ zonder de procentschaal vroeg af te kappen.
- **Grijsfilter** over de opening als de LDR fysiek verzadigt (100 Ω is erg laag voor CdS). Geen diffusor en geen kleinere opening — die verpesten de richtingsgevoeligheid.
- **Arbitrage met het kompas:** tijdens de nadering is de **LDR leidend** voor de richting; de gyro-Z doet alleen storingsonderdrukking (dat is dus geen dubbele besturing). De **magnetometer** wordt tijdens het rijden níet gebruikt omdat de stappenmotoren het veld verstoren; die is voor de terugweg, waar een absolute koers nodig is.

> **Waarom niet meer segmentgewijs:** de eerdere aanpak (rij kort segment → stop → meet → `rotate()` → herhaal) is vervangen. Bij segmenten van 1–2 cm wordt de topsnelheid nooit gehaald, want de ramp alleen al is 3,28 cm op plus 3,28 cm af. Bovendien kost stop-draai-rij-door ordes meer CPU en wandkloktijd dan bijsturen per slice (~0,1 % CPU), en een stapsgewijze rotatie kan een hobbel of gleuf niet onderdrukken.

---

### Armkinematica — hoogte en reikwijdte van de grijper (J1 + J2)

Waar de grijper zich bevindt (hoogte boven de grond, horizontale afstand tot de ultrasoon) als functie van de schouder- (J1) en elleboog-hoek (J2). CAD-exact afgeleid uit twee FreeCAD-zijaanzicht-projecties (2026-09-21, `hardware/FreeCAD/Assembly_Grijper_zijaanzicht*.dxf`) bij twee bekende standen — rust (J1=J2=0°) en volledig gestrekt (J1=J2=180°) — en geïmplementeerd in [`lib/gripper/geometry.py`](lib/gripper/geometry.py) (`arm_positie_cm()`, `vingertop_cm()`).

**Assen en oorsprong:** x = horizontaal, positief naar de lichtbron/het voorwerp toe, oorsprong bij de voorkant van de ultrasoon; z = hoogte boven de grond (de bovenkant van het controlbord ligt 75mm boven de grond en diende als tussenstap voor de CAD-meting).

**Vaste maten (CAD-exact):**

| Grootheid | Waarde | Betekenis |
|---|---|---|
| Schouderas (J1) | x = −3,72 cm, z = 10,30 cm | vast punt, beweegt nooit |
| Bovenarm (schouder → elleboog) | 7,40 cm | **was ruw geschat op 5 cm** |
| Onderarm (elleboog → polsmechanisme/tandwielcentrum) | 3,425 cm | was geschat op 3,5 cm — kwam al dicht in de buurt |
| Pols → vingertop | 12,5 cm | **[TE VERIFIËREN]** — komt uit de oudere handmatige vingerafmetingen (22+41+32+30mm), die onderling een niet-opgehelderde inconsistentie hadden (zie *Bekende issues/TODO*) |

**Hoekconventie** (`label_j1`/`label_j2` zijn de 0–180 servo-graden zoals `servo_pos()` ze aanneemt: 0° = rust, 180° = max):

```
alpha1  = label_j1                    0° = horizontaal-achter, 90° = recht omhoog, 180° = horizontaal-voor
alpha2  = label_j1 + 90 − label_j2    onderarmhoek (absoluut), zelfde conventie
alpha_h = alpha2 + 90                 handhoek — vaste 90° t.o.v. de onderarm, GEEN eigen servo
```

Een schouderhoek die een volle 180° draait, gaat vanzelf door **recht omhoog** (bij 90°) — dat verklaart waarom zowel "rust" als "volledig gestrekt" met een horizontale bovenarm gemeten zijn, aan weerszijden van diezelfde verticale doorgang.

**De twee validatiepunten:**

| Stand | J1 | J2 | Elleboog (x,z) | Pols (x,z) | Vingertop (x,z) |
|---|---|---|---|---|---|
| Rust | 0° | 0° | (−11,12 ; 10,30) | (−11,12 ; 13,73) | (+1,38 ; 13,73) |
| Volledig gestrekt | 180° | 180° | (+3,68 ; 10,30) | (+3,68 ; 13,73) | (+16,18 ; 13,73) |

(cm, x t.o.v. ultrasoon, z boven de grond)

Bij rust reikt de vingertop dus al **1,4 cm vóór** de ultrasoon — dit is de eerder gevonden verklaring voor waarom de grijper naar rust gebracht moet worden vóór je de ultrasoon-metingen doet (anders botst/overlapt hij ermee). Bij volledige uitstrekking reikt de grijper tot **~16,2 cm vóór** de ultrasoon, op **13,7 cm** hoogte.

> **Gevalideerd met een derde, onafhankelijk meetpunt (2026-09-21).** De formule is gekalibreerd op twee uitersten (0° en 180°); bij een tussenliggende, niet voor de kalibratie gebruikte stand (J1=12,5% / J2=12,2% duty, ≈171°/166° label) voorspelt de formule een vingertop op **163,3 mm** vóór de ultrasoon, tegen een gemeten **160 mm** — verschil ~3,3 mm (~2%), waarschijnlijk grotendeels toe te schrijven aan de niet-CAD-exacte `ARM_L3_CM` (pols→vingertop). Dat bevestigt dat de servo-graden binnen deze nauwkeurigheid lineair met de fysieke hoek meebewegen. Zie ook de mechanische botsingsrandvoorwaarde bij `lib/servo/servo_crl.py` (schouder in rust + elleboog >30° van zijn rust af = botsing met het controlbord) — die combinatie treedt bij dit model op zodra `label_j1` klein is en `label_j2` ver van 0 afligt.

---

### Grijpergeometrie en eindpositionering

De kaken sluiten **horizontaal**, maar de vingertoppen bewegen daarbij naar **voren**. Gemeten:

| | Kaakopening | Toppen t.o.v. ultrasoon |
|---|---|---|
| maximaal open (J10=3% PWM) | **8,7 cm** (CAD/exact gemeten 2026-09-21, was 9 cm) | 12 cm |
| bijna dicht (J10=8,6% PWM) | 2 cm | 15 cm |

```
tip_pos_cm(opening) = 12 + (8,7 − opening) × 3/6,7      →  ~0,45 cm vooruit per cm sluiten
```

> Dit is een **rechte door twee meetpunten**, geen exacte formule. Het mechanisme is inmiddels wel geïdentificeerd uit het bovenaanzicht (2026-09-21): een **parallellogram-vierstangenmechanisme** — vanaf de tandwielas loopt een balk naar de achterkant van de vinger, en vanaf een vast scharnier op de handplaat een tweede, **exact even lange** balk (beide 42,0 mm) naar het midden van de vinger. Omdat de balken gelijk zijn en het vaste-scharnieren-frame (30,84 mm) exact gelijk is aan de afstand tussen de twee vingerbevestigingspunten, blijft de **oriëntatie van de vinger constant**; hij verschuift alleen langs een cirkelboog. Dat verklaart waarom een rechte lijn door twee meetpunten hier een redelijke, maar geen exacte, benadering is. Een volledige boogformule is niet afgemaakt (zou nog de exacte vingertop-positie en de PWM→rotatiehoek-relatie vergen) — bij de huidige nauwkeurigheidsbehoefte is de rechte-lijn-benadering voldoende.

#### De ultrasoon is alleen bruikbaar met de arm in rust

Tijdens het rijden staan de servo's in rustpositie: de kaken liggen dan **achter** de ultrasoon en vallen buiten de bundel, dus de afstandsmeting is zuiver. Bij een halve openingshoek van 15° is die bundel 6,4 cm breed op 12 cm en **8 cm breed op 15 cm**, en de kaken staan 8,7 cm open — **zodra de arm uitklapt kijkt de sensor dus naar de eigen vingers.** Vanaf dat moment is er geen afstandsterugkoppeling meer.

#### Drie afgeleide grootheden

Deze geometrie zat eerst in `stepper_ramp.py`. Dat maakte een generieke motordriver afhankelijk van de afmetingen van één grijper en de stopstrategie van één missie; het staat nu in [`lib/gripper/geometry.py`](lib/gripper/geometry.py) (puur rekenwerk, geen imports) en [`lib/gripper/approach.py`](lib/gripper/approach.py) (`finetune()`, `mean_dist_cm()`, `brake_target_cm()`).

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

| Objectbreedte | Grijper-tolerantie | Voorlopig model huidige bundel (≈ ± 2 cm) | Model gehalveerde spleet (≈ ± 1 cm) |
|---|---|---|---|
| 3 cm | ± 3,0 cm | ✓ ruim | ✓ ruim |
| 4 cm | ± 2,5 cm | ✓ | ✓ |
| 5 cm | ± 2,0 cm | ⚠ op de grens | ✓ |
| 6 cm | ± 1,5 cm | ✗ te krap | ✓ |
| 7 cm | ± 1,0 cm | ✗ te krap | ⚠ op de grens |

**Ontwerpverwachting:** met de huidige modelparameters is een gehalveerde spleet nodig voor voorwerpen breder dan 4 cm. Dit wordt pas een definitieve systeemeis nadat `γ` en `w` met `tests/test_ldr_beam.py` zijn gemeten en de laterale fout op de robot is gevalideerd.

#### Stilstaand nameten — het laatste correctiemoment

Omdat de bundel vrij is zolang de arm in rust staat, kan er ná het stoppen maar vóór het uitklappen een verse, gemiddelde meting worden gedaan. Dan vallen de rijsnelheid en de meetlatentie uit de fout:

| | Onzekerheid in de afstand |
|---|---|
| stoppen bij 5 cm/s (committed + latentie) | 0,74 cm |
| **stilstaand nameten + kruipcorrectie** | **ontwerpdoel ≈ 0,3 cm**; hardwarevalidatie nodig |

`approach.finetune(read_cm, object_w_cm)` doet dit (in [`lib/gripper/approach.py`](lib/gripper/approach.py)): gemiddelde over 8 **onafhankelijke** metingen (wachttijd > `INTERVAL_MS` = 50 ms, anders lees je dezelfde gebufferde waarde) en daarna `creep()` naar het doel. Bij 2 cm/s is er praktisch geen ramp nodig, dus de kruipbeweging is meteen exact en zacht. Na een correctie wordt **altijd** opnieuw gemeten, ook na de laatste poging — anders zou het oordeel op de meting van vóór die correctie gebaseerd zijn.

De **laterale** afwijking kan níet worden nagemeten: de `y`-bepaling uit de Q-daling vraagt beweging over twee afstanden. Stilstaand kan alleen worden geverifieerd dat A − B genulled is, en dat is de *peiling*, niet de laterale positie. **De laterale correctie moet dus af zijn op 20–45 cm.**

#### Eindsequentie

| Stap | Actie | Ultrasoon | Terugkoppeling |
|---|---|---|---|
| 1–7 | naderen, bundelas, afremmen naar 5 cm/s | bruikbaar | LDR + gyro + ultrasoon |
| 8 | stoppen rond `STOP_DIST_CM` | bruikbaar | |
| 9 | **`finetune()`** — stilstaand nameten, gemiddeld | bruikbaar | laatste correctiemoment; ontwerpdoel ± 0,3 cm |
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

Webserver op basis van **microdot** (asyncio) met een websocket, zodat het karretje vanuit een standaard browser bediend en uitgelezen kan worden. Is ontworpen om op **core 0** naast de hoofdcontroltaken te draaien; core 1 is gereserveerd voor ICM-20948 + display. De interactie tussen MicroPython, CYW43/lwIP, `_thread` en de control-loop moet nog onder belasting worden gevalideerd.

**Functionaliteit:**
- **Sensoren uitlezen** (browser, ~5–10 Hz): LDR A/B in %, ultrasoon-afstand in cm (of time-out), kompasrichting in graden, servoposities in %, stepper-snelheid/richting/afgelegde weg.
- **Directe rijbesturing:** vooruit/achteruit/links/rechts + snelheid, met een **noodstop** en een **deadman/heartbeat**: bij een verbroken websocket of uitblijvende heartbeat stopt de kar automatisch.
- **Hoog-niveau commando's:** LDR-scan, rijden-naar-licht, grijpen, terugkeren, kalibratiemodus starten/stoppen.

**Netwerk:** AP-modus (kar als eigen accesspoint) heeft de voorkeur voor mobiel gebruik; station-modus optioneel. SSID/wachtwoord in de centrale `config.json`.

**Client is altijd een apart extern apparaat.** Besturing en logging (websocket én `/log.csv`) gebeuren per ontwerp altijd vanaf een **losse pc, laptop of tablet** die met het AP van de kar verbindt — nooit vanaf de pc die de Pico via USB programmeert/opvolgt (die kan gewoon op het normale netwerk blijven, los van de kar). Testscripts en documentatie (zie `tests/test_data_logging.py`) gaan hier consequent van uit.

**Aandachtspunten / beperkingen (nog te implementeren):**
- **Async-refactor vereist.** De besturing is nu volledig blokkerend: busy-wait `while sm.active(): pass` in `_wait_stepper_done`, blokkerende `scan()`/ADC-lussen, en `ServoController.servo_rest()` zonder argument, die per servo met `time.sleep_ms(10)` wacht tot de positie bereikt is met een timeout van 5 s — dus tot ~15 s achter elkaar. Die verhongeren alle de asyncio-event-loop. Nodig: coöperatieve taken met `await asyncio.sleep`, of een commandowachtrij + gedeelde state tussen webserver en een control-task.
- **Geheugen:** microdot + asyncio + lwIP + bestaande modules op ~520 KB RAM is haalbaar maar krap — bewaken.
- **Veiligheid:** commandovalidatie en een verplichte deadman-stop (zie boven), zodat een rijdende kar bij connectieverlies niet doorrijdt.

---

## Bekende issues / TODO

- [x] `lib/LDR/ldr_scan_isr.py` — herzien scan-algoritme geïmplementeerd: 370°-scan (geen pre-roll), helderste A≈B-kruispunt i.p.v. som-maximum, kortste weg terug met vaste backlash-benadering, en begrensde closed-loop null-seek op A−B. Nog op hardware verifiëren.
- [x] **Gyroscoop/kompas (GY9250) hardwarevalidatie (2026-09-19) — negatief resultaat.** I2C-scan (`tests/test_gy9250_i2c_scan.py`) laat zien dat de AK8963-magnetometer nooit op de bus verschijnt; whoami van de accel/gyro-chip is `0x70` ("standalone MPU6500"), een echte MPU-9250 geeft `0x71`. **Huidig bordje heeft dus geen magnetometer.** Gyro/accelerometer werken wél (worden al gebruikt in `stepper_ramp.py`). Besluit: vervangend bordje bestellen.
- [x] **Vervanger: ICM-20948 besteld (2026-10-19)** in plaats van een MPU-9250 (die is end-of-life en wordt veel nagemaakt). Code omgezet: nieuwe driver `lib/ICM20948/icm20948.py` met dezelfde API als de Tuupola-driver (`acceleration`, `gyro` in rad/s, `magnetic` in µT, `temperature`, `whoami`); basic/fusion/kalibratiescripts en tests hernoemd naar `ICM20948`/`test_icm20948_*`. Kalibratiewaarden gaan nu als `mag_offset=`/`mag_scale=` naar de `ICM20948`-constructor. De magnetometerassen worden in de driver uitgelijnd op die van accel/gyro (X gelijk, Y en Z omgekeerd volgens datasheet), wat de kantelcompensatie in `fusion.py` nodig heeft.
- [ ] Na levering ICM-20948: `tests/test_icm20948_i2c_scan.py` draaien (WHO_AM_I `0xEA`, `0x0c` na bypass, AK09916 `0x48`/`0x09`). Daarna: asuitlijning van de magnetometer controleren (kop naar het noorden en 90° draaien → koers verandert ~90° in dezelfde richting als de geïntegreerde gyro-Z), `GYRO_Z_SIGN` opnieuw verifiëren (andere chip/montage), bias, magnetometerkalibratie en hardwaregedrag met draaiende stappenmotoren.
- [ ] OLED (SSD1306) — integratie nog niet geïmplementeerd; voorstel staat in `te doen.md`.
- [x] GPIO-tabel gecontroleerd en geverifieerd tegen KiCad netlist V1.2 (zie [hardware/gpio_pinout.md](hardware/gpio_pinout.md))
- [ ] `test_all.py` converteren naar afzonderlijke testfuncties per module; `test_all.py` behouden als snelle controletest.
- [x] Stepper 1/64: `STEPS_REV = 12800` in code gezet. `F_PIO` staat op 15 MHz; de berekende resterende snelheidsafwijking is ~0,43 %.
- [ ] TMC2209 MS-bedrading fysiek verifiëren: `MS1→GND`, `MS2→VCC_IO` (in huidige documentatie +5 V).
- [x] `WHEEL_CIRC` gecorrigeerd naar de gemeten 19,1 cm (was 20,94 → 8,8 % te korte afstanden). `TRACK_WIDTH = 13,6 cm` toegevoegd.
- [x] `MOTOR_TURN_SIGN = +1` en `GYRO_Z_SIGN = -1` bevestigd op hardware (2026-10-21): `rotate_deg(+90)` draait naar rechts, `heading()` +90, gyro +90,8°.
- [ ] `lib/stepper/stepper_ramp.py` op hardware testen: `LDR_DIFF_SIGN`, `CYCLES_FIXED` met een logic analyzer, maximale startsnelheid en versnelling meten, regelversterkingen `kp_ldr`/`kp_gyro` afstemmen. Daarna beslissen of `stepper.py` vervalt.
- [ ] **Odometriekalibratie** — afstandsschaal (`WHEEL_CIRC`) en rotatieschaal (`TRACK_WIDTH`) opnemen in de kalibratiesessie. Zonder deze stuurt de kar structureel scheef.
- [ ] Rijden-naar-licht: de segmentgewijze aanpak is **vervangen** door de doorlopende kruisfase met bijsturing per slice in `stepper_ramp.py`, plus positioneren op de **bundelas** via de genormaliseerde helderheid `Q`. Zie de sectie *Rijden naar de lichtbron*.
- [x] `LDR_R_MIN_OHM` uiteindelijk naar 100 in `lib/LDR/ldr_scan_isr.py`. Delertopologie gedocumenteerd in de code.
- [x] `LDR_R_FIXED_OHM`: 10 000 → 1000 (voor resolutie op de bundelas) → **weer 10 000**, na het terugzetten van R29/R30 naar 10 kΩ met de nieuwe, fellere lichtbron. Later vervangen door **47 000** na de definitieve meting. `tests/test_ldr_beam.py` (`controleer_config()`) waarschuwt als code en hardware uit elkaar lopen.
- [x] `measure_now()` en de bewegende `scan()` middelen nu **10 samples, 1 ms uit elkaar** tegen de 100Hz-netrimpel van de LED-lichtbron; `scan()` doet dit niet-blokkerend met een ringbuffer en koppelt elk gemiddelde aan de gemiddelde stepperpositie van hetzelfde venster.
- [x] **Fase-gesynchroniseerde achtergrond-sampling toegevoegd:** `ServoController.attach_tick_callback()` (`lib/servo/servo_crl.py`, GPIO5-IRQ omgezet van falling naar **rising** edge) plus `start_synced_sampling()`/`read_synced()` in `lib/LDR/ldr_scan_isr.py`. Zie *Rijden naar de lichtbron* → Fase-gesynchroniseerde achtergrond-sampling.
- [ ] **Op hardware verifiëren:** dat `attach_tick_callback()` de servo-timing niet meetbaar verstoort (de callback loopt vóór `micropython.schedule()` in dezelfde harde IRQ); dat er geen ADC-mux-conflict optreedt als `read_synced()`-sampling en een actieve `scan()` elkaar toch overlappen; en of de aanname "statische bias, verwaarloosbare drift binnen één rijsegment" in de praktijk klopt (zie de kanttekening bij servo-PWM-klok vs. 230V-net-fase in de LDR-sectie).
- [x] `WHEEL_BASE_CM` in `ldr_scan_isr.py` gecorrigeerd van 18,5 naar 13,6 cm — een 370°-scan draaide in werkelijkheid 503°.
- [x] **LDR-kalibratie (2026-10-19):** `LDR_GAIN_B` vervangen door de machtscorrectie `LDR_B_GAMMA`/`LDR_B_LN_C`, `LDR_R_FIXED_OHM` → 47 000 (werkelijke R29/R30), `LDR_R_MIN_OHM` 20 Ω → 100 Ω, `LDR_R_MAX_OHM` 20 kΩ → 5 MΩ, nieuw richtingssignaal `ldr_diff = ln R_A − ln R_B'`. Gebaseerd op [`Meting_LDR_afstand_hoek.md`](Meting_LDR_afstand_hoek.md).
- [ ] HeadingController op hardware afstemmen met `read_synced_diff()` als `ldr_diff` (`kp_ldr`, deadband 0,03 ≈ 1°).
- [ ] **[TE VERIFIËREN]** of de fellere lichtbron de eerdere resolutiezorg bij 10 kΩ (~4,5 LSB ΔQ op de bundelas, tegen ~38 LSB met 1 kΩ) compenseert — zie *Randvoorwaarden / afhankelijkheden* hierboven.
- [ ] **Delertopologie eenmalig verifiëren op hardware:** schijn licht op LDR A en lees de ruwe ADC. Gaat die naar nul, dan is `_adc_to_res_ohm()` juist; gaat die naar 65535, dan loopt de hele schaal omgekeerd en moet de formule `R_FIXED × (65535 − adc)/adc` worden.
- [ ] **370°-scan opnieuw controleren** met de gecorrigeerde `WHEEL_BASE_CM`. Waren eerdere scans empirisch afgestemd op 18,5, dan wijken piekposities nu af.
- [ ] **`γ` en `w` meten** met `tests/test_ldr_beam.py` (`gamma()` en `bundel()`). Beide constanten zitten in élke `y`-berekening; de huidige `w ≈ 33°` komt uit één meetpunt met een aangenomen `γ = 0,7`.
- [ ] **Spleet lichtbron halveren — voorlopig ontwerpvoorstel voor voorwerpen breder dan 4 cm.** De grijper verdraagt maar ±(8,7 − breedte)/2 cm laterale afwijking, en de huidige bundel haalt ≈ ±2 cm. Zet de spleet **verticaal** (smal horizontaal). Eerst `bundel()` draaien, dan aanpassen, dan controleren of de 370°-scan de bron nog vindt vanaf de werkelijke startpositie.
- [x] Laterale tolerantie van de grijper vastgesteld: **±(8,7 − objectbreedte)/2 cm**. Zie *Grijpergeometrie en eindpositionering*.
- [ ] **Kaakdiepte opmeten** (palm t.o.v. vingertoppen). Die zet de werkelijke ondergrens van het grijpvenster; nu conservatief op 12 cm gehouden, wat elk venster ~2 cm te smal maakt.
- [ ] **Derde meetpunt van de grijper** bij ~5 cm opening. `tip_pos_cm()` is nu een rechte door twee punten; een vierstangenmechanisme geeft een kromme.
- [ ] **Arm uitklappen boven het voorwerp en dan laten zakken**, in plaats van horizontaal naar voren vegen. Voorkomt dat een kaak het voorwerp omstoot bij een laterale afwijking. Vraagt een gecoördineerde beweging van servo 1 en 2. **Mechanische randvoorwaarde (2026-09-19, gebruiker):** staat servo 1 (schouder) in rust, dan maakt servo 2 (elleboog) al bij **meer dan ca. 30°** vanuit zíjn rust contact met het controlbord (PCB) — een harde fysieke botsing, geen timingkwestie. Veilige volgorde: J1 eerst weg van rust, pas dan J2 bewegen; bij het terugkeren J2 eerst terug naar rust (J1 nog weg van rust = veilig), J1 pas daarna. Geldt voor de **toestand** (welke servo waar staat), niet alleen voor de volgorde van bewegen — J1 mag dus nooit in rust staan terwijl J2 >30° van zijn rust af staat, in geen enkele fase van een gecoördineerde beweging. Geïmplementeerd in `tests/test_servo.py`. Servo 4 (grijper) heeft eigen, andere beperkingen (open/dicht-bereik, stroomlimiet) en zit niet in deze botsingsketen.
- [x] **Armkinematica (J1+J2) CAD-gevalideerd (2026-09-21).** Schouderpositie en schakel-lengtes exact bepaald uit FreeCAD-zijaanzicht-projecties bij rust en volledig gestrekt; hoogte/afstand-formule geïmplementeerd in `lib/gripper/geometry.py` (`arm_positie_cm()`). Zie *Armkinematica*.
- [x] **Derde tussenliggende meting voor de armformule (2026-09-21).** Gevalideerd op J1=12,5%/J2=12,2% duty (≈171°/166° label): voorspeld 163,3mm, gemeten 160mm (~2% verschil). Bevestigt de lineaire label→fysieke-hoek-aanname.
- [ ] **`GRIP_OPEN_CM` bijgewerkt naar 8,7cm** (was 9,0cm) op basis van een exacte meting; `TIP_NEAR_CM`/`TIP_FAR_CM` (12/15cm) zijn nog niet herbevestigd met dezelfde precisie.
- [ ] **Parallellogram-vingermechanisme geïdentificeerd** (balken 42,0mm, frame 30,84mm) maar niet omgezet in een exacte boogformule voor `tip_pos_cm()` — zou de exacte vingertop-positie en de PWM→rotatiehoek-relatie van J10 vergen. De rechte-lijn-benadering blijft voorlopig in gebruik.
- [ ] **Vingerkootjes-tabel inconsistent** (zie *Afmetingen*): "1e vingerkootje: handservo→2e kootje=41mm" en "2e vingerkootje: 1e kootje→2e kootje=32mm" overlappen. Bepaalt `ARM_L3_CM` in `lib/gripper/geometry.py` (nu 125mm als voorlopige som van alle vier de segmenten).
- [ ] Grijsfilter over de LDR-openingen als de cel fysiek verzadigt (100 Ω is erg laag voor CdS). Geen diffusor en geen kleinere opening — die verpesten de richtingsgevoeligheid.
- [ ] **Naamgeving stappenmotoren inconsistent**: `stepper.py`/`stepper_ramp.py` noemen GPIO16/17/18 "motor A", maar [hardware/gpio_pinout.md](hardware/gpio_pinout.md) wijst GPIO16/17/18 toe aan stepper **B** (U2) en GPIO12/13/14 aan stepper A (U1). Functioneel geen probleem, maar code en schema spreken elkaar tegen. Kiezen welke de waarheid is.
- [ ] **BLOCKER vóór vol 6S-accubedrijf — voeding U5 (DSN-MINI-360, MP2307) buiten spec**: gespecificeerd ingangsbereik 4,75–23 V, accu levert 22,2 V nominaal en 25,2 V vol geladen. Er is een 28 V pin-compatibele versie gevonden; die inbouwen. Bij een doorgeslagen high-side schakelaar komt 25 V op de 5 V-rail te staan, wat Pico, servo's, OLED en ultrasoon in één keer meeneemt.
- [ ] **Servo-rail scheiden van de logica-rail** (optioneel): alle servo's hangen op +5 V uit U5 (1,8 A continu / 3 A piek). Drie MG996R kunnen samen 3–4,5 A piek trekken. De grijper heeft al stroombegrenzing via de PI-regelaar op ADC2, en de servo's worden langzaam naar hun eindpositie gestuurd, dus in de praktijk blijft de stroom laag. Bij een onverklaarbare Pico-reset tijdens het grijpen is dit de eerste plek om te kijken.
- [ ] **BLOCKER vóór langdurige motortest:** bulk-elco bij de VM-pin van U1/U2 verifiëren (≥100 µF, korte sporen). Bij 24 V is een spanningspiek uit motorkabel-inductie de klassieke doodsoorzaak van een TMC2209. Nooit de motorstekker loskoppelen terwijl VM aan staat.
- [ ] `webserver` (microdot-websocket, Pico 2 W): sensoruitlezing, directe rijbesturing met deadman, hoog-niveau commando's; vereist async-refactor van de blokkerende besturing.
- [ ] Exacte MicroPython-build en CPython-versie vastleggen in deze specificatie.


---

## Afhankelijkheden

MicroPython voor **Raspberry Pi Pico 2 W (RP2350 + CYW43439)**. **Exacte firmwareversie/build nog vastleggen** voor reproduceerbaarheid. Externe libraries: `mpu9250`, `mpu6500`, `ak8963` (Tuupola, via awesome-micropython; alleen nog voor het oude MPU-6500-bordje), `ssd1306` (SSD1306 OLED driver), `microdot` (asyncio-webserver met websocket-ondersteuning).
