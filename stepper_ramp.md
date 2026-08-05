# Werking van `stepper_ramp.py` — ramp via PIO/DMA en sturen per slice

Stap-voor-stap uitleg van [`lib/stepper/stepper_ramp.py`](lib/stepper/stepper_ramp.py). Voor de constanten, de API-tabel en de plaats in het systeem: zie [globale_specificatie.md](globale_specificatie.md).

> `stepper_ramp.py` is een **vervanger** van `stepper.py`, geen aanvulling. Beide claimen PIO0 SM0..SM3 en dezelfde GPIO's. Importeer er altijd maar één.

---

## 1. Het probleem dat de ramp oplost

De oude module commandeert een snelheid in één keer. Vanuit stilstand betekent dat: de rotor moet binnen één microstap van 78 µs naar de eindsnelheid springen. Dat is een oneindige versnelling — de rotor kan het roterende veld niet volgen, verliest synchronisme, en de motor blijft zoemend staan.

Het is dus **geen koppelprobleem**. De koppelbegroting laat een factor 8 marge zien bij VREF = 1,0 V:

| Post | Koppel per wiel |
|---|---|
| Versnellen van 1636 g bij 55 cm/s² | 1,43 N·cm |
| Rotor-inertie (68 g·cm²) | 0,013 N·cm |
| Rolweerstand (ruim geschat) | ~1,2 N·cm |
| **Totaal nodig** | **~2,7 N·cm** |
| **Beschikbaar** | **~22 N·cm** |

Bij 22 N·cm en wielradius 3,04 cm is de trekkracht 14,5 N tegen een gewicht van 16,1 N: **de wielen slippen eerder dan dat de motor koppel tekortkomt.** Meer stroom of spreadCycle helpt dus niet — een ramp wel.

De eis erbij: het oude concept liep 100 % in de PIO zonder CPU-overhead. Dat willen we behouden.

---

## 2. De kerntruc: één FIFO-woord is een heel segment

De naïeve aanpak — één FIFO-woord per stap — loopt vast op RAM. Een ramp van 2200 stappen zou 8,8 kB zijn, en bij een fijnere ramp loopt dat op naar tientallen kB per ramp × 2 ramps × 2 motoren. Dat past niet betrouwbaar in de MicroPython-heap.

Daarom codeert elk 32-bit woord een **segment**:

```
 31                          16 15                           0
+------------------------------+------------------------------+
|   delay in PIO-cycles        |   aantal stappen - 1         |
|   (max 65535)                |   (max 65536)                |
+------------------------------+------------------------------+
```

Een ramp van 2200 stappen in 256 segmenten is daarmee **1 kB**. Een kruisfase tot 65536 stappen (97,8 cm) past in **één woord**.

Verpakken gebeurt in `_word(repeat, delay)`; beide velden worden geklemd, niet gemaskeerd, zodat een te grote waarde niet stil omklapt.

---

## 3. Het PIO-programma, instructie voor instructie

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

| # | Instructie | Wat er gebeurt | Cycles |
|---|---|---|---|
| 0 | `pull(block)` | haal het volgende segment uit de FIFO. **Is de FIFO leeg, dan stalt de SM hier** — met STEP laag, want side-set wordt óók tijdens een stall toegepast. | 1 |
| 1 | `out(y, 16)` | laagste 16 bits → Y = `repeat − 1` (shift-richting is RIGHT) | 1 |
| 2 | `out(x, 16)` | hoogste 16 bits → X = delay | 1 |
| 3 | `mov(isr, x)` | delay opzij zetten in ISR. **ISR is hier kladregister**, geen invoer-FIFO — autopush staat uit. Nodig omdat X straks wordt leeggeteld en er maar twee scratchregisters zijn. | 1 |
| 4 | `mov(x, isr) [2]` | STEP **hoog**, delay uit ISR terughalen. `[2]` maakt de puls 3 cycles = **200 ns** bij 15 MHz, ruim boven het TMC2209-minimum van ~100 ns en breed genoeg voor de teller-SM. | 3 |
| 5 | `jmp(x_dec, "wait")` | STEP **laag**, X aftellen. Springt naar zichzelf, dus dit is de delaylus. | delay + 1 |
| 6 | `jmp(y_dec, "pulse")` | volgende stap in dit segment; is Y op, dan valt hij door en **wrapt automatisch naar 0** voor het volgende segment. | 1 |

**Vaste overhead per stap = 3 + 1 + 1 = 5 cycles.** Dat is `CYCLES_FIXED`, en daarom geldt:

```
delay = round(F_PIO / stapfrequentie) − 5
```

Bij `F_PIO = 15 MHz` en de topsnelheid van 12800 stappen/s is dat delay = 1167 cycles. De overhead is dan 5 van 1172 = **0,43 %**, tegen ~2,3 % bij de oude 3 MHz. `F_PIO = 15 MHz` is bovendien 150 MHz sysclk / 10 en dus een **integer klokdeler**, zonder jitter van de fractionele deler.

Het programma is 7 instructies. De teller-SM is er 3, dus PIO0 gebruikt 10 van de 32 instructieplaatsen.

### De stapgenerator staat altijd aan

Er is geen start/stop-toestand. De SM's worden bij init geactiveerd en blijven actief. Geen data in de FIFO betekent stallen op `pull(block)` met STEP laag — en dat is de rusttoestand waarin de motor zijn positie houdt. Daarmee vervalt een hele klasse toestandsfouten.

---

## 4. De teller-SM: hardware-odometer

```python
@asm_pio()
def step_counter():
    label("loop")
    wait(0, pin, 0)          # wacht tot STEP laag
    wait(1, pin, 0)          # wacht op de stijgende flank
    jmp(y_dec, "loop")
```

Y loopt af vanaf `0xFFFFFFFF`; het aantal pulsen is `0xFFFFFFFF − Y`. Uitlezen gebeurt met `exec()`:

```python
self.cnt.exec("mov(isr, y)")
self.cnt.exec("push()")
return 0xFFFFFFFF - self.cnt.get()
```

Twee dingen om te weten:

- **De teller kent de DIR-pin niet.** Hij telt flanken. Bij een rotatie op de plaats lopen beide tellers positief op terwijl de wielen tegengesteld draaien. Zie §8.
- **De teller telt commando's, geen beweging.** Bij wielslip loopt hij door. Voor werkelijke beweging is de GY9250 nodig.

---

## 5. DMA naar de TX-FIFO

Per motor één DMA-kanaal (van de 16 die de RP2350 heeft):

```python
self.dma = rp2.DMA()
self._ctrl = self.dma.pack_ctrl(size=2,            # 32-bit transfers
                                inc_read=True,     # door de tabel heen lopen
                                inc_write=False,   # altijd naar dezelfde FIFO
                                treq_sel=(0 << 3) + sm_id)
```

Drie details:

- **`treq_sel = (pio_num << 3) + sm_num`** koppelt de DMA aan de DREQ van die TX-FIFO, zodat er alleen geschreven wordt als er ruimte is. Deze formule klopt óók op de RP2350: `DREQ_PIO2_TX0 = 16 = 2 << 3`.
- **`write=self.sm`** mag direct: een `StateMachine` ondersteunt het buffer-protocol, dus geen `mem32`-adresgehannes.
- **De buffer moet blijven leven** zolang de transfer loopt; daarom `self._buf = array('I', words)`.

`start_table()` weigert een lege lijst, want een transfer met `count = 0` is firmware-afhankelijk gedrag. Loopt er nog een transfer, dan wordt die **afgekapt** in plaats van erop te wachten — wachten zou seconden kunnen blokkeren, en de aanroeper heeft expliciet besloten iets nieuws te starten.

---

## 6. De vier fasen van een beweging

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

| Fase | Bron | CPU-kosten | Waarom zo |
|---|---|---|---|
| Ramp op | DMA, 256 woorden | **0** | segmenten van ~2 ms zijn te snel voor MicroPython; DMA is immuun voor GC-pauzes |
| Brug | onderdeel van dezelfde DMA-transfer | **0** | zie §7 |
| Kruisfase **zonder** bijsturing | zelfde DMA-transfer, 1 woord | **0** | hele beweging in één transfer, één IRQ aan het eind |
| Kruisfase **met** bijsturing | CPU, 1 woord per 20 ms per motor | ~0,1 % | hier zit de regelkring |
| Ramp af | DMA, 256 woorden | **0** | |

Een beweging van 50 cm zonder bijsturing is **513 woorden = 2052 bytes in één DMA-transfer**: precies het nul-overhead gedrag van het oude concept, mét ramp.

### De S-curve

`ramp_words()` verdeelt de rampstappen over 256 segmenten van gelijk stappenaantal, en zet de snelheid per segment volgens een **smoothstep in de afgelegde weg**:

```python
p = (i + 0.5) / n_seg
s = p * p * (3.0 - 2.0 * p)
rate = rate0 + (rate1 - rate0) * s
```

De afgeleide van `3p² − 2p³` is nul aan beide uiteinden, dus de **versnelling is nul aan begin én eind** van de ramp. Geen koppelschok op de overgangen. Dat kost niets extra, want de tabel wordt in Python berekend en daarna alleen door DMA afgespeeld.

`ACCEL_CM_S2 = 55` bepaalt de ramp-**afstand** via `(v₁² − v₀²)/(2a)` = 3,28 cm. De piekversnelling is 1,5× = 82 cm/s² = 0,084 g. Let op: omdat de S-curve aan begin en eind traag is, **duurt** de ramp 0,55 s en niet de 0,31 s van een lineaire ramp over dezelfde afstand.

Is de beweging korter dan 2 × 3,28 = 6,6 cm, dan verlaagt `plan()` de topsnelheid tot op + af precies passen (driehoeksprofiel).

---

## 7. De overgang DMA → CPU, en waarom de brug er is

**Regel: DMA en CPU mogen nooit gelijktijdig in dezelfde FIFO schrijven.** Doen ze dat wel, dan lopen de segmenten door elkaar en kunnen de twee motoren zelfs een verschillende volgorde krijgen — dan is zowel de ramp als de koers onbetrouwbaar.

Daarom wacht `service()` in de `_RAMP_UP`-toestand:

```python
if MA.dma.active() or MB.dma.active():
    return True                  # DMA schrijft nog; CPU blijft eraf
self._state = self._CRUISE
```

Merk op wat hier **niet** staat: er wordt niet gewacht tot de rampstappen zijn *uitgevoerd*. Dat zou fout zijn — dan loopt de FIFO leeg en pauzeert de motor tussen ramp en kruisfase. `dma.active() == False` betekent alleen dat de DMA is gestopt met **schrijven**, terwijl de FIFO nog data bevat. Precies de voorsprong die de CPU nodig heeft, en de FIFO bewaart de volgorde.

### Waarom dat nog niet genoeg is

Als de DMA klaar is met schrijven, staat er nog maximaal 4 woorden in de FIFO. Maar aan het eind van de ramp zitten we op topsnelheid, dus die laatste segmenten zijn kort: samen maar **~3 ms**. Met `service()` elke 10 ms zou de FIFO alsnog leeglopen en krijg je een hapering.

Daarom eindigt de opramptabel met één **brugsegment** op kruissnelheid van `BRIDGE_SLICES × SLICE_MS = 40 ms`:

```python
up = ramp_words(self.n_ramp, self.r0, self.r1)
up.extend(cruise_words(bridge, self.r1))
```

Die stappen horen bij de kruisfase maar worden niet bijgestuurd — een correctie 40 ms eerder of later maakt niets uit.

De andere kant op is het simpeler: de ramp-af-DMA wordt pas gestart nadat de CPU is **gestopt** met pushen, dus daar kan de volgorde niet door elkaar lopen.

### Waarom een lege FIFO geen ramp is

Loopt de FIFO écht leeg, dan stalt de PIO op `pull(block)` met STEP laag. Gevolgen:

- de motor **houdt zijn positie**, er gaat geen stap verloren;
- CPU-latency — ook een GC-pauze van tientallen ms — beïnvloedt de **staptiming niet**, want de PIO genereert met hardware-precisie. Te laat komen betekent dat de correctie één slice later komt, geen timingfout;
- het gevolg is een korte **pauze**, geen glitch.

Dat is netjes degraderend faalgedrag. Bij een CPU-getimede pulsgenerator geeft een GC-pauze van 40 ms direct verloren stappen. Met 3 slices vooruit in de FIFO is de runway 60 ms.

---

## 8. Sturen: hoe een slice een koersverandering wordt

### Waarom snelheid alleen niet stuurt

Koersverandering komt van een **verschil in stappenaantal** tussen de wielen. Zitten beide motoren vast aan hetzelfde totaal — zoals bij fire-and-forget — dan geeft een snelheidsverschil netto **nul** koersverandering: de kar maakt een boog en komt in dezelfde richting terug.

In de kruisfase ligt dat totaal níet vast. Daar integreert een snelheidsverschil dus wél tot een blijvende koersverandering. Dat is precies waarom het sturen in de kruisfase hoort.

### Van graden/s naar twee FIFO-woorden

Per slice met duur `t`:

```python
omega = TURN_SIGN * self.correction()                 # graden/s, + = rechts
delta = int(omega * STEPS_PER_DEG * t / 2.0 + 0.5)    # stappen verschil per wiel
delta = klem(delta, ±base * MAX_DIFF_FRAC)            # max ±20 %

ra = base + delta
rb = base - delta

cyc = F_PIO * base / self.r1                          # PIO-cycles voor deze slice
MA.push(ra, _clamp_delay(int(cyc / ra + 0.5) - CYCLES_FIXED))
MB.push(rb, _clamp_delay(int(cyc / rb + 0.5) - CYCLES_FIXED))
```

Twee dingen zijn hier bewust zo:

1. **De slice heeft een vaste DUUR, niet een vast stappenaantal.** Beide motoren krijgen dezelfde `cyc`, maar een verschillend aantal stappen en dus een verschillende delay. Zo blijven ze in de tijd synchroon; gemeten afwijking is < 20 µs op 20 ms. Bij een vast stappenaantal zouden ze uit de pas lopen.
2. **`ra + rb = 2 × base` exact.** Het midden van de kar schuift dus precies `base` stappen op, hoe groot het stuurverschil ook is.

Met `STEPS_PER_DEG ≈ 159` en `base = 256` stappen per 20 ms:

| Differentie | Δ stappen | per slice | draaisnelheid |
|---|---|---|---|
| ±5 % | 25 | 0,16° | 8 °/s |
| ±20 % (maximum) | 102 | 0,64° | 32 °/s |

Resolutie: 1 stap verschil = **0,0063°**.

### Waarom niet de PIO-klok variëren

`SMn_CLKDIV` is tijdens runtime schrijfbaar via `machine.mem32`, en dat zou ook een snelheidsverschil geven. Bewust niet gedaan:

- het zit **buiten het datapad**, dus niet synchroon met de segmentgrenzen;
- de stappenaantallen zijn dan niet meer exact bekend;
- de delays staan in PIO-cycles, dus een klokwijziging **herschaalt de rampatabel onderweg** (en de versnelling schaalt met f²).

Als globale snelheids-override (alles langzamer, bijvoorbeeld bij een obstakel) blijft de klokdeler wel bruikbaar.

---

## 9. Exacte afstand ondanks het sturen

Bijsturen verandert **wanneer** stappen komen, niet **hoeveel**. Per motor wordt `committed` bijgehouden: de som van alle weggeschreven `repeat`-waarden.

Het afremmpunt wordt op `committed` bepaald, **niet** op een gemeten positie:

```python
remaining = self.n_total - self.n_ramp - self._centre
base = min(self._slice_base, remaining)
if base < 1:
    self._state = self._RAMP_DOWN
```

Daarmee is de eindafstand **exact**, onafhankelijk van wanneer de regellus toevallig aanroept. Geen poll-onzekerheid. De laatste slice wordt precies op maat gemaakt.

### Signed odometrie

Omdat de teller-SM de DIR-pin niet kent, houdt elke motor naast de monotone pulsteller een **signed positie** bij:

```python
def travel(self):
    p = self.pulses()
    d = p - self._seen
    if d:
        self._pos += self._sign * d
        self._seen = p
    return self._pos
```

`set_dir()` roept eerst `travel()` aan met het **oude** teken en wisselt daarna `_sign`. Zo blijft de positie kloppen over een richtingswisseling heen.

| Functie | Betekenis |
|---|---|
| `pio_pos1()` / `pio_pos2()` | monotone pulsteller, gebruikt door `busy()` |
| `distance()` | `(travel_A + travel_B)/2 × CM_PER_STEP` — rotatie geeft ~0 cm, achteruit negatief |
| `heading()` | `(travel_A − travel_B)/STEPS_PER_DEG` — werkt ook bij rotatie op de plaats |

---

## 10. Stoppen — drie soorten

| Functie | DMA | FIFO | Driver | Motor |
|---|---|---|---|---|
| `halt()` (= `stop()`) | stil | gewist | **aan** | stopt direct, **houdt positie** |
| `brake()` | stil | gewist, dan afremramp | aan | remt af vanaf de geschatte snelheid |
| `emergency_stop()` | stil | gewist | **uit** | loopt vrij, kar kan doorrollen |

De FIFO wissen gebeurt met `sm.init(...)`: MicroPython heeft daar geen aparte API voor, en `init()` doet intern `clear_fifos + restart`. Dat is veiliger dan zelf in `SHIFTCTRL` schrijven. Daarna klopt `committed` niet meer met wat er nog gaat gebeuren, dus die wordt op de werkelijke pulsstand gezet.

`brake()` schat de huidige snelheid uit de voortgang door het profiel (`current_rate()`), want afremmen vanaf een te hoog veronderstelde snelheid zou eerst een snelheidssprong omhoog geven — en dus synchronismeverlies.

### Op tijd beginnen met afremmen

`finish()` stopt niet onmiddellijk. De afremramp en de al weggeschreven slices liggen vast:

| Snelheid | afremramp | in FIFO | **committed** | + ultrasoonlatentie (50 ms) |
|---|---|---|---|---|
| 19,1 cm/s | 3,28 cm | 1,15 cm | **4,43 cm** | 0,96 cm |
| 10 cm/s | 0,88 cm | 0,60 cm | **1,48 cm** | 0,50 cm |
| 5 cm/s | 0,19 cm | 0,30 cm | **0,49 cm** | 0,25 cm |

```python
doel = sr.stop_dist_cm(6.0) + sr.stopping_distance_cm(snelheid)
if ultrasoon.read_cm() <= doel:
    mv.finish()
```

**Rem voor de laatste ~25 cm af naar 5 cm/s.** Dat brengt de stoponzekerheid van 5,4 cm naar 0,74 cm, en dat is robuuster dan proberen 4,43 cm exact te voorspellen.

### En daarna: stilstaand nameten

Tijdens het rijden staan de servo's in rustpositie, dus de kaken liggen achter de ultrasoon en de bundel is vrij. Ná het stoppen maar **vóór** het uitklappen kun je daarom een verse gemiddelde meting doen — dan vallen de rijsnelheid en de meetlatentie uit de fout:

| | Onzekerheid |
|---|---|
| stoppen bij 5 cm/s | 0,74 cm |
| **`finetune()`: stilstaand nameten + `creep()`** | **≈ 0,3 cm** |

```python
sr.finetune(ultrasoon.read_cm, object_w_cm=5.0)
```

Zodra de arm uitklapt staan de kaken (9 cm open) in een bundel die op 12–15 cm ongeveer 8 cm breed is — de sensor kijkt dan naar de eigen vingers. **`finetune()` is dus het laatste correctiemoment**; alles daarna is open-loop, op de grijperstroomsensor na.

`_mean_dist()` wacht 60 ms tussen metingen, want `ultrasoon.INTERVAL_MS` is 50 ms: lees je sneller, dan krijg je dezelfde gebufferde waarde terug en doet het gemiddelde niets.

### Grijpergeometrie

De kaken sluiten horizontaal, maar de toppen bewegen naar voren: `tip_pos_cm(o) = 12 + (9 − o) × 3/7`. Daaruit volgen drie dingen:

| Objectbreedte | `stop_dist_cm()` | `grip_window_cm()` | `lateral_tolerance_cm()` |
|---|---|---|---|
| 3 cm | 14,6 cm | 2,6 cm breed | ± 3,0 cm |
| 5 cm | 13,7 cm | 1,7 cm breed | ± 2,0 cm |
| 7 cm | 12,9 cm | 0,9 cm breed | ± 1,0 cm |

Een *smaller* voorwerp vraagt een *grotere* stopafstand: smaller betekent verder sluiten, dus meer vooruitgang van de toppen. De volledige tabel en de onderbouwing staan in [globale_specificatie.md](globale_specificatie.md) onder *Grijpergeometrie en eindpositionering*.

---

## 11. Doorgerekend voorbeeld: 50 cm bij 19,1 cm/s

| Grootheid | Waarde |
|---|---|
| Totaal | 33 508 stappen |
| Ramp op / af | 2200 stappen elk = 3,28 cm, 256 segmenten = 1024 byte |
| Brugsegment | 512 stappen = 40 ms, 1 woord |
| Kruisfase | 28 596 stappen ≈ 112 slices van 20 ms |
| Delay bij topsnelheid | 1167 cycles |
| Delay bij startsnelheid | 11 714 cycles |
| Duur | ramp 0,55 s + brug 0,04 s + kruis 2,23 s + ramp 0,55 s ≈ **3,37 s** |
| Gemiddelde snelheid | 14,8 cm/s (de ramps zijn traag) |
| **Zonder** bijsturing | 513 woorden = 2052 byte, **één** DMA-transfer, nul CPU |
| **Met** bijsturing | 257 + 256 woorden DMA, plus ~224 `put()`-aanroepen (~0,1 % CPU) |

---

## 12. Gebruik

**Fire-and-forget, nul CPU-overhead:**

```python
import stepper_ramp as sr
sr.mov('f', 19.1, 50)         # 50 cm vooruit, met ramp
sr.rotate_deg(90)             # 90 graden naar rechts
while sr.busy():
    pass
```

**Met doorlopende koerscorrectie:**

```python
import stepper_ramp as sr, ultrasoon, time
from stepper_ramp import HeadingController, gyro_z_deg_s

hc = HeadingController(ldr_diff=mijn_ldr_verschil,
                       gyro_rate=gyro_z_deg_s(sensor))

mv = sr.drive(200, 19.1, correction=hc.output)
doel = sr.STOP_DIST_CM + sr.stopping_distance_cm(19.1)
while mv.service():
    if ultrasoon.read_cm() <= doel:
        mv.finish()
    time.sleep_ms(10)
```

**Alleen koers vasthouden op de gyro:**

```python
mv = sr.drive(50, 19.1, correction=sr.hold_heading(gyro_z_deg_s(sensor)))
```

`gyro_z_deg_s()` is **verplicht** rond de GY9250: [`mpu6500.py`](lib/GY9250/mpu6500.py) heeft `gyro_sf=SF_RAD_S` als default en levert dus **radialen/s**, terwijl de regelaar in graden/s rekent. Rechtstreeks `sensor.gyro[2]` doorgeven maakt de correctie 57,3× te klein.

---

## 13. Tests en verificatie

[`tests/test_stepper_ramp_math.py`](tests/test_stepper_ramp_math.py) — **158 pure-Python tests, geen hardware nodig** (`machine` en `rp2` worden gestubd, draait op de PC met CPython):

```
python3 tests/test_stepper_ramp_math.py
```

Gedekt: exact stappentotaal over het hele bereik (0 t/m 200 000 stappen), monotone snelheid in beide ramprichtingen, veldgrenzen, driehoeksprofiel, nulafstanden, invoervalidatie, slice-rekenkunde, signed odometrie bij rotatie en achteruit, `Move.finish()`, en de DMA → CPU overgang.

### Nog te verifiëren op hardware

| Wat | Hoe |
|---|---|
| `TURN_SIGN` (+1 / −1) | opgeheven wielen: een handmatige draai naar rechts moet zowel een positieve gemeten (gyro) als een positieve gewenste draaisnelheid geven |
| `CYCLES_FIXED = 5` | `meet_frequentie()` vergelijkt de werkelijke STEP-frequentie met `_delay_for()`. Controleer ook met een logic analyzer: die ziet de pulsbreedte (verwacht 200 ns) en of er stappen wegvallen |
| Startsnelheid en max. versnelling | met de GY9250 als onafhankelijke referentie — de PIO-teller kan een stall niet zien |
| `kp_ldr`, `kp_gyro` | de standaard `kp_ldr = 25` geeft bij een vol LDR-verschil 25 °/s, net onder het plafond van 32,2 °/s, dus de regelaar verzadigt normaal niet |
| Odometriekalibratie | 1,00 m rijden (`WHEEL_CIRC`) en 360° draaien (`TRACK_WIDTH`) |
| `fifo_join=PIO.JOIN_TX` | niet gebruikt, want niet geverifieerd in deze MicroPython-versie. Werkt het, dan verdubbelt de runway van 60 naar 140 ms |
