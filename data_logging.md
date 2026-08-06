# Data logging

## Doel

Tijdens een meetrit worden sensorwaarden, regelgegevens en voertuigstatus op een
pc opgeslagen zonder dat de real-time motorbesturing wordt opgehouden.

De belangrijkste ontwerpregel is:

> Het verliezen van logregels is acceptabel; het missen van deadlines in de
> motor- of regellus niet.

Daarom schrijft de regellus nooit rechtstreeks naar wifi of flash. De logger
plaatst complete CSV-regels in een begrensde RAM-buffer. Een afzonderlijke
HTTP-stream verstuurt de inhoud wanneer de netwerkverbinding daar ruimte voor
heeft.

## Gekozen aanpak

De voorkeursoplossing bestaat uit:

1. een gedeelde snapshot met de laatst bekende sensor- en regelwaarden;
2. een logtaak die deze snapshot standaard met 10 Hz omzet naar CSV;
3. een begrensde ringbuffer in RAM;
4. een afzonderlijke Microdot-route `/log.csv`;
5. `curl` op de pc, dat de stream rechtstreeks naar een bestand schrijft;
6. een klein eventlog op flash als vangnet voor toestandswisselingen en fouten.

De bestaande websocket blijft bestemd voor bediening en langzame
UI-telemetrie. Meetdata en UI-data worden niet in dezelfde stream
samengevoegd.

```text
sensor- en regeltaken
        |
        v
 gedeelde snapshot
        |
   logger op 10 Hz
        |
        v
 RAM-ringbuffer ----> /log.csv ----> curl ----> CSV-bestand op pc
        |
        +------------> dropped-teller

toestandswisselingen en fouten ----> klein eventlog op flash
```

## Waarom HTTP-streaming

HTTP-streaming past het beste bij dit project omdat Microdot al voor de
webinterface wordt gebruikt en de pc geen speciale clientcode nodig heeft.
Verder bepaalt de pc wanneer de opname begint en eindigt, en schrijft `curl` de
data direct naar schijf.

Voordelen:

- geen websocket-framing of aparte Python-client;
- een aparte verbinding voor meetdata;
- CSV is tijdens en na de rit direct bruikbaar;
- opnieuw verbinden is eenvoudig;
- een verbroken verbinding mag nooit de regellus blokkeren.

## Frequenties

Gebruik verschillende frequenties voor regeling, logging en presentatie:

| Functie | Richtwaarde | Opmerking |
|---|---:|---|
| `Move.service()` en snelle regeling | 50-100 Hz | Volg de eisen van de motorbesturing |
| Gyro en binnenste regellus | circa 50 Hz | Niet verlagen om logging te vereenvoudigen |
| CSV-logging | 10 Hz | Verhoog alleen als metingen aantonen dat dit nodig is |
| UI-telemetrie | 2-5 Hz | Alleen waarden voor menselijke presentatie |
| Events | direct | Alleen toestandswisselingen, resultaten en fouten |

De logtaak leest uitsluitend bestaande snapshots. Zij start zelf geen
ultrasoonmeting, ADC-conversiereeks, kompasfusie of regelberekening.

## CSV-formaat

Gebruik puntkomma's als scheidingsteken, zodat decimale punten geen probleem
vormen. Iedere stream begint met één header.

Voorbeeld:

```text
run_id;boot_id;seq;t_ms;state;ldr_a_adc;ldr_b_adc;us_cm;us_sid;puls_a;puls_b;heading_deg;turn_setpoint_dps;gyro_z_dps;turn_applied_dps;dropped
```

Betekenis van de identificatievelden:

- `run_id`: identificatie van de huidige meetrit;
- `boot_id`: verandert bij iedere herstart van de Pico;
- `seq`: loopt op voor iedere geproduceerde sample, ook als de buffer vol is;
- `t_ms`: tijd sinds het begin van de meetrit;
- `dropped`: cumulatief aantal regels dat niet in de ringbuffer paste.

Een sprong in `seq` toont exact waar samples ontbreken. Een stijging van
`dropped` verklaart dat het verlies op de Pico plaatsvond. Na opnieuw verbinden
begint de nieuwe CSV opnieuw met de header, maar lopen `run_id`, `boot_id` en
`seq` door.

## RAM-ringbuffer

De buffer is begrensd en `put()` wacht nooit. Als een complete CSV-regel niet
past, wordt die regel geheel verworpen. Er komt dus nooit bewust een halve regel
in de buffer.

```python
# lib/log/ring.py
class Ring:
    """Begrensde byte-ring voor complete logrecords.

    put() blokkeert niet. Als een record niet past, wordt het volledig
    verworpen en wordt dropped verhoogd.

    Let op: de aanroeper en sommige slices kunnen nog steeds tijdelijke
    objecten alloceren. De logger als geheel is dus niet allocatievrij.
    """

    def __init__(self, size=8192):
        self.buf = bytearray(size)
        self.mv = memoryview(self.buf)
        self.size = size
        self.r = 0
        self.w = 0
        self.used = 0
        self.dropped = 0

    def put(self, data):
        n = len(data)
        if n == 0:
            return True
        if n > self.size - self.used:
            self.dropped += 1
            return False

        end = self.w + n
        if end <= self.size:
            self.mv[self.w:end] = data
        else:
            first = self.size - self.w
            self.mv[self.w:] = data[:first]
            self.mv[:n - first] = data[first:]

        self.w = end % self.size
        self.used += n
        return True

    def take(self, max_bytes=1024):
        """Neem maximaal één aaneengesloten blok uit de ring."""
        if self.used == 0:
            return None

        n = min(self.used, self.size - self.r, max_bytes)
        data = bytes(self.mv[self.r:self.r + n])
        self.r = (self.r + n) % self.size
        self.used -= n
        return data
```

Bij ongeveer 1,5 kB/s overbrugt een buffer van 8 kB circa vijf seconden. De
ring is alleen een korte verzekering tegen netwerkvertraging, niet een opslag
voor een volledige rit. Duurt een storing langer, dan worden nieuwe regels
verworpen en loopt `dropped` op.

## Gedeelde snapshot

Sensor- en regeltaken werken hun eigen toestand bij. De logger kopieert die
waarden in één korte, niet-blokkerende bewerking. Pas de namen aan de uiteindelijke
applicatiestructuur aan.

```python
LOG_STATE = {
    "state": "idle",
    "ldr_a": 0,
    "ldr_b": 0,
    "us_cm": None,
    "us_sid": 0,
    "puls_a": 0,
    "puls_b": 0,
    "heading": 0.0,
    "turn_setpoint": 0.0,
    "gyro_z": 0.0,
    "turn_applied": 0.0,
}

def log_snapshot():
    # Lees ieder veranderlijk veld precies één keer. Als core 1 deze toestand
    # bijwerkt, moet de uiteindelijke core-overdracht expliciet worden ontworpen.
    s = LOG_STATE
    return (
        s["state"],
        s["ldr_a"], s["ldr_b"],
        s["us_cm"], s["us_sid"],
        s["puls_a"], s["puls_b"],
        s["heading"], s["turn_setpoint"],
        s["gyro_z"], s["turn_applied"],
    )
```

Roep vanuit de logger dus niet opnieuw `ultrasoon.read_cm()` of een functie als
`HeadingController.output()` aan. Zulke functies kunnen een meting of
regelberekening beïnvloeden. De logger observeert alleen.

## Periodieke logtaak

Een absolute deadline voorkomt dat de periode na iedere iteratie verder
wegdrijft. De precieze veldnamen zijn voorbeelden en moeten bij integratie aan
de definitieve applicatie worden gekoppeld.

```python
import asyncio
import time

LOG = Ring(8192)
LOG_PERIOD_MS = 100

async def log_task(run_id, boot_id):
    t0 = time.ticks_ms()
    next_ms = t0
    seq = 0

    while True:
        next_ms = time.ticks_add(next_ms, LOG_PERIOD_MS)
        seq += 1

        (state, ldr_a, ldr_b, us_cm, us_sid, puls_a, puls_b,
         heading, turn_setpoint, gyro_z, turn_applied) = log_snapshot()

        us_text = "" if us_cm is None else "%.1f" % us_cm
        row = ("%s;%s;%d;%d;%s;%d;%d;%s;%d;%d;%d;%.1f;%.1f;%.1f;%.1f;%d\n" % (
            run_id,
            boot_id,
            seq,
            time.ticks_diff(time.ticks_ms(), t0),
            state,
            ldr_a,
            ldr_b,
            us_text,
            us_sid,
            puls_a,
            puls_b,
            heading,
            turn_setpoint,
            gyro_z,
            turn_applied,
            LOG.dropped,
        )).encode()
        LOG.put(row)

        delay = time.ticks_diff(next_ms, time.ticks_ms())
        if delay > 0:
            await asyncio.sleep_ms(delay)
        else:
            # Deadline gemist: geef andere taken minstens één schedulerkans.
            await asyncio.sleep_ms(0)
```

CSV-formattering en `encode()` alloceren geheugen. Bij 10 Hz is dat doorgaans
goed beheersbaar, maar valideer dit op de Pico terwijl wifi, gyro en motoren
tegelijk actief zijn. Plan garbage collection alleen op een gemeten veilig
moment; voer het niet willekeurig vanuit de regellus uit.

## Microdot-stream

### MicroPython-beperking

MicroPython ondersteunt geen functies die tegelijk `async def` en generator
zijn. Deze vorm mag daarom niet worden gebruikt:

```python
# Niet bruikbaar op MicroPython
async def stream():
    yield b"data"
```

Microdot ondersteunt op MicroPython wel een class-based async iterator. Daarmee
kan de stream wachten wanneer de ring leeg is zonder de eventloop bezig te
houden.

### Implementatie

```python
import asyncio
from microdot import Response

CSV_HEADER = (
    b"run_id;boot_id;seq;t_ms;state;ldr_a_adc;ldr_b_adc;us_cm;us_sid;"
    b"puls_a;puls_b;heading_deg;turn_setpoint_dps;gyro_z_dps;"
    b"turn_applied_dps;dropped\n"
)

class LogStream:
    def __init__(self, ring):
        self.ring = ring
        self.header_pending = True

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.header_pending:
            self.header_pending = False
            return CSV_HEADER

        while True:
            chunk = self.ring.take(1024)
            if chunk:
                return chunk
            await asyncio.sleep_ms(20)


@app.route("/log.csv")
async def log_csv(request):
    return Response(
        body=LogStream(LOG),
        headers={
            "Content-Type": "text/csv; charset=utf-8",
            "Content-Disposition": 'attachment; filename="run.csv"',
            "Cache-Control": "no-store",
        },
)
```

Deze eenvoudige implementatie heeft bewust één consumer. Start daarom maar één
`curl`-proces tegelijk. Twee clients zouden afwisselend bytes uit dezelfde ring
halen en dus allebei een onvolledig bestand opslaan.

Als de webinterface later zelf een opname kan starten, voeg dan centraal
clientbeheer toe dat een tweede verbinding met status `409` weigert. Test het
vrijgeven van die vergrendeling bij disconnect tegen de daadwerkelijk op de
Pico geïnstalleerde Microdot-versie; de precieze disconnect-lifecycle hoort niet
in de real-time logger zelf thuis.

## Opname op de pc

Start een opname met:

```bash
curl --fail --no-buffer \
  http://192.168.4.1/log.csv \
  -o "run_$(date +%F_%H%M%S).csv"
```

Live meekijken en tegelijk opslaan:

```bash
curl --fail --no-buffer http://192.168.4.1/log.csv \
  | tee "run_$(date +%F_%H%M%S).csv"
```

Stop met `Ctrl-C`. Bij een verbroken verbinding wordt `curl` opnieuw gestart en
ontstaat een nieuw bestand. `run_id`, `boot_id` en `seq` maken zichtbaar welke
bestanden bij elkaar horen en hoeveel samples ertussen ontbreken.

Automatisch opnieuw verbinden kan bijvoorbeeld met een pc-script, maar laat
iedere verbinding naar een nieuw bestand schrijven. Zo blijft ieder bestand
zelfstandig leesbaar met een eigen header.

## Eventlog op flash

Bewaar naast de meetstream een klein eventlog met alleen belangrijke
gebeurtenissen:

- begin en einde van een run;
- toestandswisselingen;
- resultaat van de LDR-scan;
- grijperresultaat;
- noodstop en fouten;
- totaal aantal verloren logregels.

Schrijf dit log alleen wanneer de motorbesturing stilstaat of op een ander
aantoonbaar veilig moment. Beperk de omvang tot enkele tientallen regels per
rit. Het eventlog is een vangnet bij netwerkuitval of plotseling verlies van de
voeding, geen vervanging voor de meetstream.

## Alternatief 1: rauwe TCP-stream

Een TCP-stream naar `ncat` is alleen aantrekkelijk voor een kale meetrit zonder
Microdot of UI.

Op de pc:

```bash
ncat -lk 9000 > run.csv
```

Nadelen:

- de Pico moet het IP-adres van de pc kennen;
- reconnect- en headerbeheer moeten zelf worden gebouwd;
- gedeeltelijke `send()`-operaties vereisen een aparte pending-buffer;
- reeds uit de ring genomen bytes mogen bij `EAGAIN` niet achter nieuwe data
  worden teruggeplaatst, want dan verandert de volgorde van de CSV-stream.

Een correcte zender bewaart daarom een gedeeltelijk verzonden chunk apart:

```python
class NetLog:
    def __init__(self, host, port=9000):
        self.addr = (host, port)
        self.sock = None
        self.pending = None
        self.offset = 0

    def service(self):
        if self.sock is None:
            return

        if self.pending is None:
            self.pending = LOG.take(1024)
            self.offset = 0
            if self.pending is None:
                return

        try:
            n = self.sock.send(self.pending[self.offset:])
            self.offset += n
            if self.offset >= len(self.pending):
                self.pending = None
                self.offset = 0
        except OSError as exc:
            if exc.args[0] not in (11,):  # EAGAIN is geen verbroken verbinding
                self.sock.close()
                self.sock = None
```

Ook hierbij moet de socket niet-blokkerend zijn. Een blokkerende wifi-send kan
lang genoeg duren om de 60 ms FIFO-runway van de motorbesturing op te maken.
Houd er rekening mee dat `self.pending[self.offset:]` nog een tijdelijk object
kan alloceren.

## Alternatief 2: websocket

Een websocket is geschikt voor interactieve besturing en UI-telemetrie, maar
niet de voorkeursroute voor meetlogging.

Nadelen voor logging:

- er is clientcode nodig;
- logging en bediening raken sneller gekoppeld;
- een browser bewaart data vaak in RAM tot op een downloadknop wordt gedrukt;
- bij een crash of gesloten tabblad kan de volledige rit verloren gaan.

Gebruik daarom de bestaande websocket voor commando's en circa 2-5 Hz
telemetrie, en de HTTP-route voor de CSV-meetstream.

## Validatie op echte hardware

Voer vóór ingebruikname minimaal de volgende tests uit:

1. Laat motorbesturing, gyro, ultrasoon, websocket en logging tegelijk draaien.
2. Controleer met een logic analyzer of logging geen zichtbare motorpauzes geeft.
3. Stop `curl` tijdens het rijden en controleer dat de regeling doorloopt.
4. Laat de buffer vollopen en controleer dat `dropped` stijgt zonder blokkeren.
5. Verbind opnieuw en controleer header, `run_id`, `boot_id` en `seq`.
6. Test de afhandeling van een tweede gelijktijdige logclient.
7. Meet heapgebruik en garbage-collection-pauzes tijdens een lange rit.
8. Controleer de disconnect-opruiming tegen de geïnstalleerde Microdot-versie.
9. Trek bij stilstand de voeding weg en controleer het flash-eventlog.

## Samenvatting

| Gebruik | Aanpak |
|---|---|
| Normale meetrit | HTTP/CSV-stream naar `curl` |
| Bediening en schermtelemetrie | Bestaande websocket |
| Netwerk- en voedingsvangnet | Klein eventlog op flash |
| Kale prestatietest zonder webserver | Eventueel rauwe TCP-stream |

De HTTP-stream is de standaardoplossing. De ringbuffer beschermt de deadlines,
`seq` en `dropped` maken dataverlies zichtbaar, en het eventlog bewaart de
belangrijkste gebeurtenissen wanneer de netwerkstream niet beschikbaar is.
