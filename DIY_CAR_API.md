# Stepper Controller API (stepper1.py)

Deze API beschrijft alle publieke functies om de twee stappenmotoren aan te sturen
met de `stepper1.py` library.

De library ondersteunt:
- Twee onafhankelijke motoren (Motor A en Motor B)
- Vooruit/achteruit bewegingen
- Gelijktijdige bewegingen
- Rotaties (differentiële besturing)
- Automatisch stoppen bij bereiken van target
- Hardware‑teller voor afgelegde afstand

---

# 1. Functieoverzicht

Publieke functies:

- `s1(dir, speed, distance)`
- `s2(dir, speed, distance)`
- `mov(dir, speed, distance)`
- `rotate(dir, speed, distance)`
- `stop()`
- `status()`
- `distance()`
- `reset_PIO_distance()`

Interne functies (niet bedoeld voor direct gebruik):

- `_start_motor(...)`
- `speed_to_delay(...)`
- `pio_pos1()`, `pio_pos2()`
- IRQ‑handlers `counter_irq0`, `counter_irq1`

---

# 2. Functiedocumentatie

---

## 2.1 `s1(dir, speed, distance)`

Beweeg **Motor A**.

### Signatuur
```python
s1(dir: str, speed: float, distance: float) -> None
Parameters
Parameter	Waarde	Betekenis
dir	'f'	vooruit
'b'	achteruit
speed	cm/s	snelheid
distance	cm	gewenste afstand
Gedrag
Zet de richting van Motor A via DIR1

Berekent het aantal stappen op basis van distance en CM_PER_STEP

Bepaalt de delta in stappen (target_pos1)

Start:

stepper‑state machine sm0

counter‑state machine sm2

Motor stopt automatisch wanneer het target bereikt is (via PIO‑IRQ)

Voorbeeld
python
# Motor A 25 cm vooruit met 10 cm/s
s1('f', 10, 25)
2.2 s2(dir, speed, distance)
Beweeg Motor B.

Signatuur
python
s2(dir: str, speed: float, distance: float) -> None
Parameters
Parameter	Waarde
dir	'f' of 'b'
speed	cm/s
distance	cm
Gedrag
Zet de richting van Motor B via DIR2

Berekent stappen en delta (target_pos2)

Start:

stepper‑state machine sm1

counter‑state machine sm3

Voorbeeld
python
# Motor B 10 cm achteruit met 5 cm/s
s2('b', 5, 10)
2.3 mov(dir, speed, distance)
Beweeg beide motoren dezelfde kant op.

Signatuur
python
mov(dir: str, speed: float, distance: float) -> None
Parameters
Parameter	Waarde
dir	'f' of 'b'
speed	cm/s
distance	cm
Gedrag
Roept s1(dir, speed, distance) en s2(dir, speed, distance) aan

Beide motoren leggen dezelfde afstand af met dezelfde snelheid

Voorbeeld
python
# Robot rijdt 50 cm vooruit met 8 cm/s
mov('f', 8, 50)
2.4 rotate(dir, speed, distance)
Laat de robot draaien door de motoren tegengesteld te laten lopen.

Signatuur
python
rotate(dir: str, speed: float, distance: float) -> None
Parameters
Parameter	Waarde	Betekenis
dir	'l'	linksom
'r'	rechtsom
speed	cm/s	draaisnelheid
distance	cm	lineaire afstand per wiel
Gedrag
Rechtsom:

Motor A: achteruit → s1('b', ...)

Motor B: vooruit → s2('f', ...)

Linksom:

Motor A: vooruit → s1('f', ...)

Motor B: achteruit → s2('b', ...)

Voorbeeld
python
# Draai linksom met 6 cm/s, wielafstand 20 cm
rotate('l', 6, 20)
2.5 stop()
Stop beide motoren onmiddellijk.

Signatuur
python
stop() -> None
Gedrag
Zet sm0.active(0) en sm1.active(0)

Counter‑state machines blijven actief (voor afstandsteller)

Voorbeeld
python
stop()
2.6 status()
Print de status van beide motoren.

Signatuur
python
status() -> None
Output
Voor elke motor:

running of stopped

Start_Pos: softwarepositie (posX * CM_PER_STEP)

Target_Pos: target‑delta (target_posX * CM_PER_STEP)

Distance: hardware‑afstand (pio_posX() * CM_PER_STEP)

Voorbeeld
python
status()
# === STATUS ===
# Motor A: running/stopped, Start_Pos: ..., Target_Pos: ..., Distance: ...
# Motor B: ...
2.7 distance()
Geef de afgelegde afstand van beide motoren in cm.

Signatuur
python
distance() -> tuple[float, float]
Gedrag
Leest Y‑registers van sm2 en sm3 via pio_pos1() en pio_pos2()

Converteert naar cm met CM_PER_STEP

Voorbeeld
python
d1, d2 = distance()
print("Motor A:", d1, "cm")
print("Motor B:", d2, "cm")
2.8 reset_PIO_distance()
Reset de hardware‑afstandsteller.

Signatuur
python
reset_PIO_distance() -> None
Gedrag
Zet Y‑registers van sm2 en sm3 op 0xFFFFFFFF

Daarna telt Y weer omlaag per step‑puls

Voorbeeld
python
reset_PIO_distance()
3. Samenvatting
Deze API abstraheert de onderliggende PIO‑logica naar eenvoudige functies in termen van:

richting ('f', 'b', 'l', 'r')

snelheid in cm/s

afstand in cm

De PIO‑state machines zorgen voor:

nauwkeurige step‑generatie

hardware‑target‑detectie

hardware‑afstandstelling

De Python‑laag zorgt voor:

cumulatieve positie

eenvoudige besturing via de bovenstaande API‑functies