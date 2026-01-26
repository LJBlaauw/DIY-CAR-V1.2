# Uitleg van de PIO‑code in stepper1.py

Dit document legt de werking van beide PIO‑programma’s uit:
- De stepper‑generator (`stepper`)
- De target‑counter (`stepper_counter`)

---

# 1. Stepper‑generator (sm0, sm1)

```python
@asm_pio(sideset_init=PIO.OUT_LOW)
def stepper():
    pull(noblock) .side(1)
    nop() [7]
    mov(x, osr)
    mov(y, x)
    label("delay")
    nop().side(0)
    jmp(y_dec, "delay")
Doel
Deze PIO‑SM genereert een step‑puls met een instelbare delay tussen pulsen.

Uitleg per instructie
pull(noblock) .side(1)
Haalt delay‑waarde uit de TX‑FIFO (OSR)

.side(1) zet STEP‑pin hoog → begin van de puls

nop() [7]
Houdt STEP‑pin 8 cycles hoog

Dit is de pulsbreedte

mov(x, osr)
Zet delay‑waarde in X‑register

mov(y, x)
Kopieert X naar Y

Y wordt gebruikt als countdown‑timer

Delay‑lus
python
label("delay")
nop().side(0)
jmp(y_dec, "delay")
STEP‑pin wordt laag gezet

Y wordt afgeteld

Bij Y==0 → volgende puls

2. Target‑counter (sm2, sm3)
python
@asm_pio()
def stepper_counter():
    label("loop")
    wait(0, pin, 1)
    wait(1, pin, 1)
    jmp(y_dec, "dec_x")
    label("dec_x")
    jmp(x_dec, "loop")
    irq(0)
    jump("loop")
Doel
Deze PIO‑SM detecteert:

Elke step‑puls (hardware‑afstandsteller)

Wanneer het target bereikt is (via X‑register)

Uitleg per instructie
wait(0, pin, 1)
Wacht tot STEP‑pin laag is.

wait(1, pin, 1)
Wacht tot STEP‑pin hoog is → rising edge → één stap

jmp(y_dec, "dec_x")
Y‑register telt totale pulsen (afstandsteller)

Start op 0xFFFFFFFF

Na N pulsen: Y = 0xFFFFFFFF - N

jmp(x_dec, "loop")
X bevat target‑stappen

X-- bij elke puls

Zolang X != 0 → terug naar loop

irq(0)
X==0 → target bereikt

Eén IRQ naar Python

jump("loop")
Counter‑SM wacht op nieuwe pulsen

Python laadt X opnieuw bij een nieuwe beweging

3. Samenwerking tussen stepper‑SM en counter‑SM
Python start beweging:

Zet richting

Laadt delay in sm0/sm1

Laadt target in sm2/sm3 (X‑register)

Stepper‑SM genereert pulsen

Counter‑SM telt:

Y: totale afstand

X: resterende stappen

Bij X==0:

IRQ naar Python

Python stopt stepper‑SM

Positie wordt bijgewerkt

4. Waarom deze architectuur efficiënt is
Geen CPU‑interrupts per stap

PIO doet al het timing‑kritische werk

Python wordt alleen wakker bij target‑bereiken

Y‑register geeft hardware‑nauwkeurige afstand

X‑register geeft hardware‑nauwkeurige target‑detectie