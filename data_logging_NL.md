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
3. een begrensde, **record-aware** ringbuffer in RAM;
4. een expliciete run-status (`start`, `active`, `stop`);
5. een afzonderlijke Microdot-route `/log.csv`;
6. `curl` op de pc, dat de stream rechtstreeks naar een bestand schrijft;
7. een klein eventlog op flash als vangnet voor reeds vastgelegde
   toestandswisselingen en fouten.

De bestaande websocket blijft bestemd voor bediening en langzame
UI-telemetrie. Meetdata en UI-data worden niet in dezelfde stream
samengevoegd.

Een **run** en een **HTTP-verbinding** zijn bewust twee verschillende dingen:

- het starten van een run maakt de RAM-buffer leeg, zet de run-tellers terug en
  kiest een nieuwe `run_id`;
- `/log.csv` leest uitsluitend data uit de actieve run;
- een verbroken HTTP-verbinding stopt de run niet;
- opnieuw verbinden met `/log.csv` geeft opnieuw een header en gaat verder bij
  de eerstvolgende volledige logregel;
- het stoppen van een run stopt de productie van nieuwe CSV-regels. Eventuele
  resterende regels mogen daarna nog worden uitgelezen.

Daardoor bepaalt de pc of webinterface expliciet wanneer een meetrit begint en
ophoudt, zonder dat een korte wifi-storing automatisch een nieuwe run maakt.

```text
sensor- en regeltaken
        |
        v
 gedeelde snapshot
        |
   logger op 10 Hz
        |
        v
 record-aware RAM-ring ----> /log.csv ----> curl ----> CSV-bestand op pc
        |                         |
        +--> ring_dropped         +--> disconnects
        +--> log_missed

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
| `Move.service()` en snelle regeling | >= 100 Hz | Minstens 1x per 10 ms; zie `globale_specificatie.md` |
| Gyro en binnenste regellus | circa 50 Hz | Niet verlagen om logging te vereenvoudigen |
| CSV-logging normaal | 10 Hz | Standaard voor complete meetritten |
| CSV-logging regeltuning | 50 Hz | Alleen voor korte tuningtests; CPU/heap eerst valideren |
| UI-telemetrie | 2-5 Hz | Alleen waarden voor menselijke presentatie |
| Events | direct | Alleen toestandswisselingen, resultaten en fouten |

De 100 Hz voor `Move.service()` is geen richtwaarde maar een harde eis: de
FIFO-runway is 60 ms en het brugsegment aan het eind van de opramp bestaat juist
omdát 4 FIFO-woorden op topsnelheid samen maar circa 3 ms zijn. Logging mag die
aanroepfrequentie dus nooit verlagen.

De logtaak leest uitsluitend bestaande snapshots. Zij start zelf geen
ultrasoonmeting, ADC-conversiereeks, kompasfusie of regelberekening.

10 Hz is voldoende voor traject-, sensor- en toestandsanalyse. Voor het tunen
van de circa 50 Hz gyro-binnenlus is 10 Hz te laag om snelle oscillaties en
korte koersverstoringen goed terug te zien. Gebruik daarvoor tijdelijk het
50 Hz-profiel en controleer op de echte Pico dat wifi, heap en garbage
collection de motorbesturing niet beïnvloeden.

## CSV-formaat

Gebruik puntkomma's als scheidingsteken en decimale punten voor floats. Iedere
HTTP-stream begint met één header.

Aanbevolen schema:

```text
run_id;boot_id;seq;t_ms;state;ldr_a_u16;ldr_b_u16;ldr_sid;us_cm;us_sid;travel_a_steps;travel_b_steps;odom_heading_deg;turn_setpoint_dps;gyro_z_dps;gyro_sid;gyro_t_ms;turn_applied_dps;ring_dropped;log_missed;disconnects
```

Betekenis van de belangrijkste velden:

- `run_id`: identificatie van de huidige meetrit;
- `boot_id`: verandert bij iedere herstart van de Pico;
- `seq`: nummer van het **nominale sample-slot**; loopt ook door over gemiste
  logdeadlines en ring-overflow;
- `t_ms`: tijd sinds het begin van de meetrit;
- `ldr_a_u16`, `ldr_b_u16`: MicroPython `ADC.read_u16()`-waarden in het bereik
  0...65535; dit zijn dus niet rechtstreeks de 12-bit hardware-counts;
- `ldr_sid`: sequence-ID van de laatst beschikbare LDR-meting;
- `us_cm`: laatst bekende ultrasoonafstand in cm met één decimaal. Het veld is
  **leeg** wanneer er geen geldige meting beschikbaar is (`None` in de snapshot).
  Een leeg veld is dus geen 0 cm en moet bij het inlezen als ontbrekende waarde
  worden behandeld;
- `us_sid`: sequence-ID van de laatst beschikbare ultrasoonmeting;
- `travel_a_steps`, `travel_b_steps`: **signed** wielposities; niet de monotone
  PIO-pulstellers die de DIR-pin niet kennen;
- `odom_heading_deg`: koers uit de signed wielodometrie; de naam maakt expliciet
  dat dit geen magnetometerkoers of geïntegreerde gyrohoek is;
- `gyro_sid`, `gyro_t_ms`: identiteit en meettijd van de gyrowaarde, zodat
  zichtbaar blijft hoe oud die waarde was toen de 10 Hz logger de snapshot nam;
- `ring_dropped`: cumulatief aantal complete logrecords dat wegens een volle
  RAM-ring niet kon worden toegevoegd;
- `log_missed`: cumulatief aantal nominale logslots dat wegens een te late
  loggerdeadline bewust is overgeslagen;
- `disconnects`: aantal verbroken logstreamverbindingen tijdens de run.

Een sprong in `seq` laat zien dat nominale samples ontbreken. `ring_dropped`
verklaart verlies vóór de netwerkstream; `log_missed` verklaart overgeslagen
loggerdeadlines. Een netwerkdisconnect kan daarnaast een reeds uit de ring
gehaald record kosten. Dat verlies is niet betrouwbaar als `ring_dropped` te
meten; `disconnects` en een `seq`-sprong in het volgende bestand maken het wel
zichtbaar.

Na opnieuw verbinden begint het nieuwe CSV-bestand opnieuw met de header, maar
blijven `run_id`, `boot_id` en `seq` doorlopen zolang dezelfde run actief is.

### Tekstvelden

Om een algemene CSV-quote/escape-routine in de real-time logger te vermijden,
worden tekstvelden beperkt gehouden:

- `run_id` en `boot_id`: alleen letters, cijfers, `_` en `-`;
- `state`: vaste enum uit de applicatie, eveneens zonder `;`, CR of LF.

Hiermee kan iedere logregel met eenvoudige `%`-formattering worden gemaakt
zonder ambigu CSV-formaat.

## RAM-ringbuffer

De buffer is begrensd en `put()` wacht nooit. Als een complete CSV-regel niet
past, wordt die regel geheel verworpen en stijgt `ring_dropped`.

De ring is **record-aware**: de recordlengtes worden apart bijgehouden. De
consumer verwijdert altijd precies één volledige CSV-regel. Dat is belangrijk
bij reconnects. Een byte-ring die willekeurig 1024 bytes verwijdert kan midden
in een CSV-regel stoppen; na een disconnect zou een nieuw bestand dan met een
header plus het tweede halve stuk van die oude regel kunnen beginnen.

```python
# lib/log/ring.py
class RecordRing:
    """Begrensde byte-ring voor complete logrecords.

    put() blokkeert niet. Als bytes of een recordslot ontbreken, wordt het
    volledige record verworpen en stijgt dropped.

    take_record() retourneert altijd precies één compleet record. De omzetting
    naar bytes alloceert een object; bij 10 Hz is dat bewust geaccepteerd en
    moet het op hardware worden gevalideerd.

    Ook de wrap-around-tak van put() alloceert twee tijdelijke slices. Dat is
    met memoryview(data) te vermijden als heapmetingen daar aanleiding voor
    geven.
    """

    def __init__(self, size=8192, max_records=96):
        self.buf = bytearray(size)
        self.mv = memoryview(self.buf)
        self.size = size

        # Vooraf gealloceerde ring met recordlengtes. Een Python-list gebruikt
        # meer RAM dan uint16, maar veroorzaakt tijdens normaal gebruik geen
        # groei van de lijst. Optimaliseer pas als heapmetingen dat nodig maken.
        self.lengths = [0] * max_records
        self.max_records = max_records

        self.r = 0
        self.w = 0
        self.used = 0

        self.lr = 0
        self.lw = 0
        self.records = 0
        self.dropped = 0

    def clear(self, reset_dropped=False):
        self.r = 0
        self.w = 0
        self.used = 0
        self.lr = 0
        self.lw = 0
        self.records = 0
        if reset_dropped:
            self.dropped = 0

    def put(self, data):
        n = len(data)
        if n == 0:
            return True

        if n > self.size:
            self.dropped += 1
            return False

        if n > self.size - self.used or self.records >= self.max_records:
            self.dropped += 1
            return False

        end = self.w + n
        if end <= self.size:
            self.mv[self.w:end] = data
        else:
            first = self.size - self.w
            self.mv[self.w:] = data[:first]
            self.mv[:n - first] = data[first:]

        self.lengths[self.lw] = n
        self.lw = (self.lw + 1) % self.max_records
        self.records += 1
        self.used += n
        self.w = end % self.size
        return True

    def take_record(self):
        if self.records == 0:
            return None

        n = self.lengths[self.lr]
        end = self.r + n

        if end <= self.size:
            data = bytes(self.mv[self.r:end])
        else:
            first = self.size - self.r
            out = bytearray(n)
            out[:first] = self.mv[self.r:]
            out[first:] = self.mv[:n - first]
            data = bytes(out)

        self.lr = (self.lr + 1) % self.max_records
        self.records -= 1
        self.used -= n
        self.r = end % self.size
        return data
```

Een logregel is met realistische waarden ongeveer 115-140 bytes. Bij 10 Hz is dat
circa 1,2-1,5 kB/s en overbrugt een bytebuffer van 8 kB dus ruim vijf seconden.
`max_records=96` geeft bij 10 Hz 9,6 seconden, dus de bytelimiet bindt het eerst.
Controleer de werkelijke gemiddelde regellengte en heap op hardware.

Bij het 50 Hz tuningprofiel geldt die runway **niet**: 8 kB is dan nog maar
ongeveer 1,2-1,4 seconde en `max_records=96` slechts 1,9 seconde. Elke
netwerkhapering van meer dan circa een seconde levert bij 50 Hz dus direct
`ring_dropped` op. Vergroot voor dat profiel bewust `size` en `max_records`, of
accepteer het verlies expliciet.

De ring is alleen een korte verzekering tegen netwerkvertraging, niet een
opslag voor een volledige rit. Duurt een storing langer, dan worden nieuwe
records verworpen en loopt `ring_dropped` op.

### Netwerkverlies na `take_record()`

Zodra Microdot een record uit `take_record()` heeft ontvangen, is dat record uit
de ring verwijderd. Als de verbinding exact daarna wegvalt, kan niet worden
bewezen dat de pc alle bytes van dat record heeft ontvangen. Een reconnect
begint dankzij de record-aware ring wel altijd op een **nieuwe volledige
CSV-regel**, maar één of meer complete regels kunnen ontbreken. Daarom blijven
`seq` en `disconnects` noodzakelijk.

## Gedeelde snapshot

Sensor- en regeltaken werken hun eigen toestand bij. De logger leest alleen de
laatst gepubliceerde waarden en start zelf geen nieuwe metingen of
regelberekeningen.

Voorbeeld zolang alle schrijvers op dezelfde MicroPython-core draaien:

```python
LOG_STATE = {
    "state": "idle",
    "ldr_a_u16": 0,
    "ldr_b_u16": 0,
    "ldr_sid": 0,
    "us_cm": None,
    "us_sid": 0,
    "travel_a_steps": 0,
    "travel_b_steps": 0,
    "odom_heading_deg": 0.0,
    "turn_setpoint_dps": 0.0,
    "gyro_z_dps": 0.0,
    "gyro_sid": 0,
    "gyro_t_ms": 0,
    "turn_applied_dps": 0.0,
}


def log_snapshot():
    # Geen await en geen sensorfunctie-aanroepen in deze functie.
    s = LOG_STATE
    return (
        s["state"],
        s["ldr_a_u16"], s["ldr_b_u16"], s["ldr_sid"],
        s["us_cm"], s["us_sid"],
        s["travel_a_steps"], s["travel_b_steps"],
        s["odom_heading_deg"], s["turn_setpoint_dps"],
        s["gyro_z_dps"], s["gyro_sid"], s["gyro_t_ms"],
        s["turn_applied_dps"],
    )
```

Roep vanuit de logger dus niet opnieuw `ultrasoon.read_cm()`, een ADC-meetlus of
`HeadingController.output()` aan. Zulke functies kunnen een meting of
regelberekening beïnvloeden. De logger observeert alleen.

### Multicore-randvoorwaarde

Bovenstaande dictionary is **geen gegarandeerd coherente snapshot** wanneer
core 1 gelijktijdig meerdere velden bijwerkt. Eén CSV-regel kan dan bijvoorbeeld
een nieuwe gyrowaarde en een oude `turn_applied_dps` combineren.

Voordat core 1 rechtstreeks logvelden mag publiceren moet de core-overdracht
expliciet worden ontworpen, bijvoorbeeld met een double-buffer/mailbox of een
version/seqlock-achtig protocol. De real-time regellus mag daarbij nooit op een
loggerlock hoeven wachten. Tot die overdracht is gevalideerd geldt: publiceer de
snapshot vanuit één core.

## Periodieke logtaak

Een absolute deadline voorkomt normale periodedrift, maar gemiste periodes mogen
**niet worden ingehaald met een burst**. Als de logger meer dan één periode te
laat is, worden de tussenliggende nominale sample-slots bewust overgeslagen.
Dat is conform de hoofdregel: verlies meetdata voordat de regellus extra
belasting krijgt.

De precieze veldnamen zijn voorbeelden en moeten bij integratie aan de
definitieve applicatie worden gekoppeld.

```python
import asyncio
import time

LOG = RecordRing(8192, max_records=96)
LOG_PERIOD_MS = 100

LOG_STATS = {
    "log_missed": 0,
    "disconnects": 0,
}

RUN = {
    "active": False,
    "run_id": "",
    "boot_id": "",
    "t0": 0,
    "seq": 0,
}


def start_run(run_id, boot_id):
    # Aanroepen vanuit de applicatie-/commandotaak, niet vanuit een motor-IRQ.
    global LOG_CLIENT_ACTIVE

    LOG.clear(reset_dropped=True)
    LOG_STATS["log_missed"] = 0
    LOG_STATS["disconnects"] = 0

    # Vangnet: als een vorige stream zijn aclose() nooit heeft gehaald, mag een
    # nieuwe run niet met een blijvend vergrendelde logclient beginnen.
    # Zie de Microdot-sectie voor LOG_CLIENT_ACTIVE.
    LOG_CLIENT_ACTIVE = False

    RUN["run_id"] = run_id
    RUN["boot_id"] = boot_id
    RUN["t0"] = time.ticks_ms()
    RUN["seq"] = 0
    RUN["active"] = True


def stop_run():
    RUN["active"] = False


async def log_task():
    next_ms = time.ticks_ms()

    while True:
        if not RUN["active"]:
            # Bij een nieuwe run opnieuw vanaf diens t0 faseren.
            await asyncio.sleep_ms(20)
            next_ms = RUN["t0"]
            continue

        now = time.ticks_ms()
        delay = time.ticks_diff(next_ms, now)
        if delay > 0:
            await asyncio.sleep_ms(delay)
            continue

        # Zijn we >= één volledige periode te laat, sla die nominale slots over.
        late_ms = time.ticks_diff(now, next_ms)
        if late_ms >= LOG_PERIOD_MS:
            missed = late_ms // LOG_PERIOD_MS
            RUN["seq"] += missed
            LOG_STATS["log_missed"] += missed
            next_ms = time.ticks_add(next_ms, missed * LOG_PERIOD_MS)

        RUN["seq"] += 1
        seq = RUN["seq"]

        (state, ldr_a, ldr_b, ldr_sid, us_cm, us_sid,
         travel_a, travel_b, odom_heading, turn_setpoint,
         gyro_z, gyro_sid, gyro_t_ms, turn_applied) = log_snapshot()

        us_text = "" if us_cm is None else "%.1f" % us_cm
        t_ms = time.ticks_diff(time.ticks_ms(), RUN["t0"])

        row = (
            "%s;%s;%d;%d;%s;%d;%d;%d;%s;%d;%d;%d;%.2f;%.2f;%.2f;%d;%d;%.2f;%d;%d;%d\n"
            % (
                RUN["run_id"],
                RUN["boot_id"],
                seq,
                t_ms,
                state,
                ldr_a,
                ldr_b,
                ldr_sid,
                us_text,
                us_sid,
                travel_a,
                travel_b,
                odom_heading,
                turn_setpoint,
                gyro_z,
                gyro_sid,
                gyro_t_ms,
                turn_applied,
                LOG.dropped,
                LOG_STATS["log_missed"],
                LOG_STATS["disconnects"],
            )
        ).encode()

        LOG.put(row)
        next_ms = time.ticks_add(next_ms, LOG_PERIOD_MS)

        # Altijd minstens één schedulerkans na formatteren/kopiëren.
        await asyncio.sleep_ms(0)
```

### Waarom `seq` vóór de buffer wordt verhoogd

`seq` hoort bij het nominale meetslot, niet bij een succesvol opgeslagen record.
Als `LOG.put()` faalt, ontbreekt dat nummer dus in het bestand. Hetzelfde geldt
voor bewust overgeslagen deadlines: `seq` springt vooruit met het aantal gemiste
slots. Daarmee blijft achteraf zichtbaar **wanneer** er data ontbreekt.

CSV-formattering en `encode()` alloceren geheugen. Bij 10 Hz is dat doorgaans
goed beheersbaar, maar valideer dit op de Pico terwijl wifi, gyro en motoren
tegelijk actief zijn. Plan garbage collection alleen op een gemeten veilig
moment; voer het niet willekeurig vanuit de regellus uit.

## Microdot-stream

### MicroPython-beperking

Microdot accepteert elk object met een `__anext__`-methode als response-body en
gebruikt dat object direct als async iterator (zie `body_iter()` in
`lib/microdot/microdot.py`). Een class-based async iterator is daarmee de
veiligste vorm op MicroPython: de stream kan wachten wanneer de ring leeg is
zonder de eventloop bezig te houden.

Belangrijk voor de opruiming: `Response.write()` roept `await iter.aclose()` aan
als de body die methode heeft, zowel na een normaal einde van de iterator als
wanneer een `awrite()` faalt met een socketfout uit `MUTED_SOCKET_ERRORS`
(32, 54, 104, 128). Dat is dus de hook voor disconnect-opruiming. Bij een
`OSError` buiten die lijst wordt `aclose()` niet aangeroepen; daarvoor is een
extra vangnet nodig (zie hieronder).

Valideer deze lifecycle opnieuw wanneer de Microdot-versie in `lib/microdot/`
wordt bijgewerkt.

### Implementatie

```python
import asyncio
from microdot import Response

CSV_HEADER = (
    b"run_id;boot_id;seq;t_ms;state;ldr_a_u16;ldr_b_u16;ldr_sid;"
    b"us_cm;us_sid;travel_a_steps;travel_b_steps;odom_heading_deg;"
    b"turn_setpoint_dps;gyro_z_dps;gyro_sid;gyro_t_ms;turn_applied_dps;"
    b"ring_dropped;log_missed;disconnects\n"
)


LOG_CLIENT_ACTIVE = False


class LogStream:
    def __init__(self, ring):
        self.ring = ring
        self.header_pending = True
        self.closed = False

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.header_pending:
            self.header_pending = False
            return CSV_HEADER

        while True:
            row = self.ring.take_record()
            if row is not None:
                return row

            # Als de run is gestopt en de ring leeg is, mag de response eindigen.
            if not RUN["active"]:
                raise StopAsyncIteration

            await asyncio.sleep_ms(20)

    async def aclose(self):
        # Microdot roept dit aan bij een normaal einde van de iterator én bij een
        # verbroken verbinding met een socketfout uit MUTED_SOCKET_ERRORS.
        # Idempotent, want een dubbele aanroep mag disconnects niet dubbel tellen.
        global LOG_CLIENT_ACTIVE

        if self.closed:
            return
        self.closed = True

        LOG_CLIENT_ACTIVE = False
        if RUN["active"]:
            # De run loopt nog door; de client is dus vroegtijdig weggevallen.
            LOG_STATS["disconnects"] += 1


@app.route("/log.csv")
async def log_csv(request):
    global LOG_CLIENT_ACTIVE

    # Geen await tussen de controle en het zetten van de vlag, dus onder asyncio
    # op één core is dit atomair.
    if LOG_CLIENT_ACTIVE:
        return Response("log client already connected\n", status_code=409)

    if not RUN["active"] and LOG.records == 0:
        return Response("no active run\n", status_code=409)

    LOG_CLIENT_ACTIVE = True

    return Response(
        body=LogStream(LOG),
        headers={
            "Content-Type": "text/csv; charset=utf-8",
            "Content-Disposition": 'attachment; filename="run.csv"',
            "Cache-Control": "no-store",
        },
    )
```

Deze implementatie heeft bewust één consumer. Een tweede client krijgt status
`409`; twee consumers mogen nooit om-en-om records uit dezelfde ring halen.

Omdat `aclose()` niet wordt aangeroepen bij een `OSError` buiten
`MUTED_SOCKET_ERRORS`, moet `start_run()` `LOG_CLIENT_ACTIVE` altijd op `False`
zetten. Anders kan één ongebruikelijke socketfout iedere volgende logclient
permanent met een `409` buitensluiten.

### Disconnect-semantiek

Een disconnect:

1. stopt **niet** de meetrun;
2. verhoogt `disconnects`;
3. moet de single-client-vergrendeling weer vrijgeven;
4. mag de real-time logtaak of motorbesturing nooit laten wachten.

Punt 2 en 3 gebeuren in `LogStream.aclose()`. Microdot roept die methode aan na
een normaal einde van de iterator en bij een verbroken verbinding met een
socketfout uit `MUTED_SOCKET_ERRORS`. Omdat beide paden dezelfde methode
gebruiken, is `aclose()` idempotent gemaakt en telt hij `disconnects` alleen mee
zolang de run nog actief is: eindigt de stream doordat de run is gestopt en de
ring leeg is, dan is dat geen disconnect.

Merk op dat Microdot een weggevallen client pas merkt bij de volgende `awrite()`.
Wacht `__anext__()` op een lege ring, dan wordt de disconnect dus pas bij het
eerstvolgende record zichtbaar. Dat is voor de administratie geen probleem, maar
het betekent wel dat `disconnects` iets kan naijlen.

Omdat `take_record()` uitsluitend complete regels verwijdert, begint een
reconnect nooit met een halve CSV-regel. Een record dat al aan de HTTP-stack was
overgedragen op het moment van disconnect kan wel volledig verloren gaan; de
volgende `seq` maakt dat zichtbaar.

## Opname op de pc

### Run starten en stoppen

Een nieuwe run moet expliciet door de applicatie worden gestart. De exacte
Microdot-controlroute kan samen met de hoofdwebserver worden gekozen, maar de
semantiek ligt vast:

```text
start run  -> nieuwe run_id, ring leeg, tellers nul, logger actief
GET log.csv -> alleen consumer verbinden; verandert run_id niet
disconnect -> run blijft actief
stop run   -> geen nieuwe records meer; resterende ring mag worden uitgelezen
```

Voor een eenvoudige eerste implementatie kunnen start/stop-knoppen in de
bestaande webinterface `start_run()` en `stop_run()` aanroepen. Een
commandline-endpoint kan later worden toegevoegd als de exacte Microdot-API
daarvan is vastgelegd.

### CSV opnemen

Wanneer de run actief is:

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

Stop alleen de **HTTP-client** met `Ctrl-C`. Dat is niet hetzelfde als
`stop_run()`: de Pico blijft de actieve run loggen totdat de run expliciet wordt
gestopt of de buffer volloopt.

Bij een verbroken verbinding wordt `curl` opnieuw gestart en ontstaat een nieuw
bestand. `run_id`, `boot_id` en `seq` maken zichtbaar welke bestanden bij elkaar
horen en waar records ontbreken. Iedere verbinding begint met een eigen header
en dankzij de record-aware ring altijd met een volledige volgende CSV-regel.

Automatisch opnieuw verbinden kan met een pc-script. Laat iedere verbinding
naar een nieuw bestand schrijven; zo blijft ieder fragment zelfstandig
leesbaar en kunnen bestanden achteraf op `run_id` + `seq` worden samengevoegd.

## Eventlog op flash

Bewaar naast de meetstream een klein eventlog met alleen belangrijke
gebeurtenissen:

- begin en einde van een run;
- toestandswisselingen;
- resultaat van de LDR-scan;
- grijperresultaat;
- noodstop en fouten;
- eindwaarden van `ring_dropped`, `log_missed` en `disconnects`.

Schrijf dit log alleen wanneer de motorbesturing stilstaat of op een ander
aantoonbaar veilig moment. Beperk de omvang tot enkele tientallen regels per
rit.

Het eventlog bewaart alleen gebeurtenissen die **al veilig naar flash zijn
gecommit**. Bij plotseling voedingsverlies tijdens het rijden is er geen garantie
dat de allerlaatste RAM-toestand nog kan worden opgeslagen. Voor een gegarandeerd
"laatste event bij power loss" is extra energie-buffering of andere hardware
nodig. Het eventlog is daarom een netwerk-/diagnostiekvangnet, geen
transactielog en geen vervanging voor de meetstream.

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
import errno

_WOULD_BLOCK = (errno.EAGAIN, errno.EWOULDBLOCK)


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
            self.pending = LOG.take_record()
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
            if exc.args[0] in _WOULD_BLOCK:
                return                       # geen verbroken verbinding

            self.sock.close()
            self.sock = None

            # Het half verzonden record MOET worden weggegooid. Zou een
            # reconnect vanaf self.offset verder gaan, dan begint de nieuwe
            # verbinding met de staart van een oude regel: precies de halve
            # CSV-regel die de record-aware ring juist voorkomt.
            self.pending = None
            self.offset = 0
```

Deze klasse verbindt zelf niet opnieuw; `service()` doet niets zolang `self.sock`
`None` is. Reconnectlogica, het opnieuw sturen van een header en het bijhouden
van een `disconnects`-teller moeten er nog omheen worden gebouwd.

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
3. Stop `curl` midden tijdens een rit en controleer dat de regeling én de run
   blijven doorlopen.
4. Verbind opnieuw en controleer dat het nieuwe bestand begint met een header en
   daarna een **volledige** CSV-regel, nooit een regelfragment.
5. Laat de ring vollopen en controleer dat `ring_dropped` stijgt zonder blokkeren.
6. Forceer een loggervertraging van meerdere periodes en controleer dat
   `log_missed` en `seq` vooruit springen zonder catch-up burst.
7. Controleer start/stop-semantiek: nieuwe run = lege ring + nieuwe `run_id`;
   reconnect = dezelfde `run_id`.
8. Test de afhandeling van een tweede gelijktijdige logclient (`409`).
9. Meet heapgebruik en garbage-collection-pauzes tijdens een lange rit op 10 Hz.
10. Herhaal een korte prestatietest op 50 Hz voor het regeltuning-profiel en meet
    daarbij de werkelijke regellengte; controleer of de ringruimte nog bij de
    gewenste runway past.
11. Controleer de coherentie van `gyro_sid`, `gyro_t_ms`, `us_sid` en `ldr_sid`.
12. Controleer vóór multicore-gebruik dat de snapshot-overdracht geen gemengde
    oude/nieuwe veldcombinaties kan opleveren.
13. Controleer dat `LogStream.aclose()` daadwerkelijk wordt aangeroepen bij een
    normaal streameinde én bij een harde disconnect, dat `LOG_CLIENT_ACTIVE`
    daarna weer `False` is en dat een volgende client dus geen `409` krijgt.
14. Meet de aanroepfrequentie van `Move.service()` terwijl logging actief is en
    controleer dat die boven 100 Hz blijft.
15. Controleer dat een ultrasoonmeting zonder geldige waarde een leeg `us_cm`-veld
    geeft en dat de pc-analyse dat als ontbrekend en niet als 0 cm inleest.
16. Trek bij stilstand de voeding weg en controleer welke reeds gecommitteerde
    flash-events behouden blijven; claim niet dat het laatste RAM-event daarmee
    gegarandeerd is.

## Samenvatting

| Gebruik | Aanpak |
|---|---|
| Normale meetrit | HTTP/CSV-stream naar `curl`, standaard 10 Hz |
| Regeltuning | Tijdelijk 50 Hz logprofiel na hardwarevalidatie; ring vergroten |
| Bediening en schermtelemetrie | Bestaande websocket |
| Korte netwerkstoring | Record-aware RAM-ring (circa 5 s bij 10 Hz), run blijft actief |
| Dataverliesdiagnose | `seq`, `ring_dropped`, `log_missed`, `disconnects` |
| Netwerk-/diagnostiekvangnet | Klein eventlog op flash voor reeds gecommitteerde events |
| Kale prestatietest zonder webserver | Eventueel rauwe TCP-stream |

De HTTP-stream blijft de standaardoplossing. De belangrijkste ontwerpregel blijft
dat logging nooit de motor- of regellus mag ophouden. De record-aware ring
voorkomt halve CSV-regels na reconnects, gemiste logdeadlines worden overgeslagen
in plaats van ingehaald, en een run is bewust losgekoppeld van de levensduur van
een HTTP-verbinding.

`seq`, `ring_dropped`, `log_missed` en `disconnects` maken verschillende vormen
van dataverlies afzonderlijk zichtbaar. Signed wielposities en expliciete
sensor-ID's/timestamps maken de logs bovendien bruikbaarder voor latere
regel- en trajectanalyse.
