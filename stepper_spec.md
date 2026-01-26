
---

### 2) `SPEC.md`

```markdown
# Specificatie Stepper Controller Library (`stepper1.py`)

Dit document beschrijft de functionele en technische eisen van de stepper‑controller.
Op basis hiervan kan een volledige implementatie worden opgebouwd.

---

## Doel

De library moet:

- Twee stappenmotoren (A en B) aansturen
- Bewegingen in cm en cm/s abstraheren
- Automatisch stoppen bij bereiken van een targetafstand
- Een hardware‑afstandsteller bijhouden via PIO
- Softwarematig de cumulatieve positie bijhouden

---

## Hardware‑mapping

- Motor A:
  - `STEP1` op pin 17
  - `DIR1` op pin 16
  - `ENA1` op pin 22
- Motor B:
  - `STEP2` op pin 12
  - `DIR2` op pin 11
  - `ENA2` op pin 15

- PIO:
  - `sm0`: stepper‑generator Motor A
  - `sm1`: stepper‑generator Motor B
  - `sm2`: target‑counter Motor A
  - `sm3`: target‑counter Motor B

---

## Constantes

- `F_PIO = 3_000_000` (Hz)
- `WHEEL_CIRC = 20.94` (cm)
- `STEPS_REV = 1600` (1/8 microstepping)
- `CM_PER_STEP = WHEEL_CIRC / STEPS_REV`

---

## PIO‑programma’s

### Stepper‑generator (`stepper`)

**Eisen**

- Sideset stuurt STEP‑pin
- STEP‑puls:
  - korte high‑tijd (via `nop [7]`)
  - daarna low‑tijd bepaald door delay‑lus
- Delay‑waarde wordt via `pull()` in OSR geladen
- Delay wordt in X/Y gezet en afgeteld in een lus

### Target‑counter (`stepper_counter`)

**Eisen**

- `in_base` zo gekozen dat:
  - `pin 0` = DIR (niet gebruikt in PIO)
  - `pin 1` = STEP
- Wacht op rising edge van STEP:
  - `wait(0, pin, 1)`
  - `wait(1, pin, 1)`
- Y‑register:
  - start op `0xFFFFFFFF`
  - `y_dec` per puls → totale pulsenteller
- X‑register:
  - wordt vanuit Python geladen met aantal target‑stappen
  - `x_dec` per puls
  - bij X==0 → `irq(0)`
- Na `irq(0)`:
  - PIO springt terug naar `loop`
  - wacht op nieuwe pulsen
  - Python moet X opnieuw laden bij een nieuwe beweging

---

## Software‑state

Globale variabelen:

- `pos1`, `pos2`  
  - cumulatieve positie in stappen
- `target_pos1`, `target_pos2`  
  - delta in stappen voor de huidige beweging

---

## IRQ‑afhandeling

### `counter_irq0(sm)`

- Wordt aangeroepen bij `irq(0)` van `sm2`
- Doel:
  - `sm0.active(0)` → Motor A stoppen
  - `pos1 += target_pos1` → cumulatieve positie bijwerken

### `counter_irq1(sm)`

- Wordt aangeroepen bij `irq(0)` van `sm3`
- Doel:
  - `sm1.active(0)` → Motor B stoppen
  - `pos2 += target_pos2`

---

## API‑gedrag

### `s1(dir, speed, distance)`

- Bepaal `forward = (dir == 'f')`
- Zet `DIR1`:
  - forward → `DIR1 = 0`
  - backward → `DIR1 = 1`
- Bereken `steps = int(distance / CM_PER_STEP)`
- Zet `target_pos1 = steps` of `-steps`
- Roep `_start_motor(sm0, delay, sm2, steps)` aan

### `s2(dir, speed, distance)`

- Idem voor Motor B met `DIR2`, `sm1`, `sm3`, `target_pos2`

### `_start_motor(sm, delay, sm_counter, steps)`

- Laad `steps` in X van `sm_counter`:
  - `sm_counter.put(steps)`
  - `sm_counter.exec("pull()")`
  - `sm_counter.exec("mov(x, osr)")`
  - `sm_counter.active(1)`
- Start stepper‑SM:
  - `sm.put(delay)`
  - `sm.active(1)`
- Zorg dat `enable()` wordt aangeroepen

---

## Afstandsteller

### `reset_PIO_distance()`

- Voor `sm2` en `sm3`:
  - `put(0xFFFFFFFF)`
  - `pull()`
  - `mov(y, osr)`

### `pio_pos1()`, `pio_pos2()`

- Lezen Y‑register:
  - `mov(isr, y)`
  - `push()`
  - `get()`
- Aantal pulsen:
  - `pulses = 0xFFFFFFFF - y_val`

### `distance()`

- `d1 = pio_pos1() * CM_PER_STEP`
- `d2 = pio_pos2() * CM_PER_STEP`

---

## Randvoorwaarden

- Als `stop()` wordt aangeroepen vóór target:
  - IRQ komt niet
  - `pos1`/`pos2` worden niet aangepast
- Nieuwe beweging:
  - moet X opnieuw laden in `sm2`/`sm3`
- `sm2`/`sm3` blijven actief om Y‑teller te behouden

