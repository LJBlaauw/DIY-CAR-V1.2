# Updaten van de code base
**Alle stappen worden na elkaar geïmplementeerd en eerst gecontroleerd op correctheid, probleemanalyse en oplossingsvoorstellen**
 1. Behoud de PIO-asm code voor zover dit mogelijk is.
 2. Volgorde van implementatie (i.v.m. onderlinge afhankelijkheden): eerst ICM-20948-kompas, dan display (toont o.a. de kompasrichting), dan WS2812B-status, dan de laser-kruisbesturing, dan de ldr-scan fixes, dan servo-kalibratie, dan LDR-kalibratie, dan de stepper-ramp (voorstel). "Update globale_specificatie.md" gebeurt na elke stap, niet pas aan het eind.
 3. Kalibratiewaarden (ICM-20948, servo's, LDR's) niet in losse ad-hoc bestanden opslaan, maar in één centraal configuratiebestand op de RP2350 (bv. `config.json`) met een aparte sectie per module.

# IMU: ICM-20948 (gyro + accelerometer + kompas)

De oorspronkelijke code voor de ICM-20948 kwam van awesome-micropython (auteur Tuupola). Voor de ICM-20948 is een eigen driver geschreven met dezelfde API: [`lib/ICM20948/icm20948.py`](lib/ICM20948/icm20948.py).

> **[BLOKKEREND — HARDWARE] Huidig bordje is géén GY-9250/MPU-9250, maar een kale MPU-6500** (2026-09-19). I2C-scan (zie `tests/test_gy9250_i2c_scan.py`, inmiddels vervangen door `test_icm20948_i2c_scan.py`) laat zien dat de AK8963-magnetometer nooit op de bus verschijnt, ook niet na het correct zetten van de bypass-bit of bij 100kHz. De whoami van de accel/gyro-chip is **0x70** ("standalone MPU6500" — zie het commentaar in `lib/GY9250/mpu6500.py`); een echte MPU-9250 geeft **0x71**. Alle code die `sensor.magnetic`/AK8963 gebruikt (`GY9250_basic.py`, `GY9250_fusion.py`, `test_gy9250_calibrate.py`, `test_gy9250_auto_calibrate.py`, `automatische_calibratie_magnetische_verstoring.py`) kan hierdoor niet werken op dit exemplaar. **Besluit: nieuw, echt MPU-9250-bordje bestellen** — de bestaande software blijft ongewijzigd en zou moeten werken zodra whoami 0x71 teruggeeft en `0x0c` in de I2C-scan verschijnt (test opnieuw met hetzelfde scanscript).
>
> **Update 2026-10-19:** als vervanger is een **ICM-20948** besteld (opvolger van de MPU-9250, AK09916-magnetometer in dezelfde behuizing). Code en tests zijn omgezet naar `lib/ICM20948/` en `tests/test_icm20948_*.py`. Bij levering eerst [`tests/test_icm20948_i2c_scan.py`](tests/test_icm20948_i2c_scan.py) draaien: verwacht WHO_AM_I **0xEA** en na bypass **0x0c** met AK09916-ID **0x48/0x09**. Het scanscript herkent ook een verkeerd geleverd 6-assig bordje (ICM-20648/20649, MPU-6500).

 1. Deze micropython code voor de ICM-20948 gaat op core 1 van de RP2350 draaien, zodat de overige modules er geen last van hebben (rekenintensief).
 2. In het main programma kan gekozen worden of we `ICM-20948_basic.py` library functie importeren (alleen kompas functie) of de `ICM-20948_fusion.py` library functie (kompas plus accelerator correctie gaan gebruiken). Beide staan in `lib/ICM20948/` (nog voorbeeldcode met een eigen hoofdlus; moet nog omgebouwd worden tot importeerbare module).
 3. Zorg voor veilige data-uitwisseling tussen core 0 en core 1 (bv. een lock of atomische lees/schrijf) voor de gedeelde kompaswaarde — dit is niet automatisch veilig met `_thread` op de RP2350.
 4. De code moet nog gereviewd en getest worden.
 5. De bedoeling van het kompas is om het karretje in de juiste richting te laten rijden. De ondergrond kan oneffen zijn en de wielomtrek kan een klein beetje afwijken. Het kompas kan dan gebruikt worden om de koers te corrigeren.
 6. De kalibratie van de ICM-20948 (hard-iron offset + soft-iron scale) wordt uitgevoerd via `tests/test_icm20948_auto_calibrate.py`. Dit script laat het karretje ronddraaien en bepaalt de magnetometer-offsets. De gevonden kalibratie wordt opgeslagen in de centrale configuratie (`config.json`) en tijdens normaal bedrijf door `ICM-20948_basic.py`/`ICM-20948_fusion.py` toegepast om de kompaswaarde te compenseren.
 7. De kalibratiewaarden (het correctiebestand uit punt 6) opslaan op de RP2350 (zie centrale configuratie, sectie hierboven).
 8. De kompasroutine wordt maar eens per 100/200 ms aangeroepen dit afhankelijk van de tijd nodig om de berekeningen af te ronden; zodra bekend is hoelang de berekening duurt leggen we vast wat de cyclustijd wordt. De resterende tijd binnen die cyclus op core 1 is beschikbaar voor de displayafhandeling (zie volgende sectie).

# toevoegen display routine voor de 128x64 0,96 inch oled display met ssd1306 driver chip
**Het display wordt gebruikt om realtime sensor- en bewegingsinformatie te tonen**
 1. Het display loopt net als de ICM-20948 op core 1. De routines voor de ICM-20948 en het display lopen steeds na elkaar, in het tempo van de ICM-20948-routine: de resterende tijd van elke kompascyclus (zie punt 8 hierboven) wordt gebruikt voor de displayafhandeling.
 2. Leg vast wat de minimaal acceptabele ververssnelheid is: als de ICM-20948-fusion traag is, kan "realtime" stepper-snelheid/afgelegde-weg-informatie merkbaar achterlopen.
 3. Voor zover mogelijk de informatie direct uit de PIO lezen.

**te displayen informatie**
 1. lichtwaarden van beide LDR's in %
 2. afstandsmeting van de ultrasoonsensor in cm, of time-out
 3. kompasrichting in graden t.o.v. het noorden (max 180 kan + of min zijn t.o.v. het noorden)
 4. servoposities in % (rust = 0%, max = 100%)
 5. stepper motors snelheid, richting en afgelegde weg

# toevoegen besturing WS2812B meerkleuren LED
**de LED wordt gebruikt voor algemene status van het systeem**
 1. groen: alles ok (rust)
 2. rood (vast): catastrofale fout
 3. wit: karretje op weg naar target
 4. rood knipperend: niet-fatale waarschuwing (bv. servo-stroomlimiet bereikt) — apart van vast rood, dat gereserveerd blijft voor een echte stop
 5. blauw knipperend: LDR-scan bezig
 6. geel/oranje: karretje rijdt terug naar startpositie
 7. paars: kalibratiemodus (servo/LDR/kompas)
 8. uit: systeem uit / slaapstand
 9. maximale stroomverbruik meerkleuren led 25%

# toevoegen besturing laser kruis
 1. Het laser kruis kan vanuit het hoofdprogramma aangestuurd worden met pwm regeling. pwm 0 is uit en pwm max is 100%
 2. ~~Laser fysiek uitlijnen.~~ **Gedaan (2026-09-19, gebruiker).** Relevant voor de LDR-kalibratieprocedure hieronder, die de laserpointer gebruikt om de kar handmatig op de lichtbron te richten voordat een meting genomen wordt.

# fouten in de ldr-scan routine oplossen
 1. ~~Nu moet voordat de ldr_scan start de ultrasoonroutine gestopt worden.~~ **Opgelost**: ldr_scan gebruikt nu SM8 (PIO2 SM0), ultrasoon gebruikt SM4 (PIO1 SM0) — aparte PIO-blokken, geen conflict meer.
 2. De ldr-scan moet tijdens de scan de twee LDR-waarden apart opnemen en daarna de richting van de lichtbron bepalen (nu wordt gepiekt op het gemiddelde van beide LDR's, dat is de bug). De LDR's zijn in het horizontale vlak uit elkaar geplaatst, er ontstaan tijdens het scannen dus twee maxima's. De gewenste richting ligt dus tussen de twee maxima's. Het kruispunt waar LDR A ≈ LDR B is het midden, dit na correctie van het gain-verschil tussen beide LDR's.
 3. De gemeten waarden in een csv-bestand opslaan op de RP2350.
 4. Na het terugdraaien van de kar naar de lichtbron controleren of dit overeenkomt met de berekende waarden.
 5. **Volledige-omwentelingsscan (370°) i.p.v. pre-roll.** Niet eerst terugdraaien, maar direct 370° in één richting draaien. Een volle omwenteling bevat altijd het maximum; de 10° overlap houdt een piek rond 0°/360° van de rand af. De pre-roll (`start_graden`/terugdraaien vóór de scan) vervalt. Randvoorwaarde: de kar moet vrij op zijn as kunnen draaien zonder kabels die bij 360°+ mee-twisten.
 6. **Kortste weg naar de gevonden richting.** Na 370° op eindhoek 370° is terugdraaien naar piekhoek θ gelijk aan `370 − θ` en doordraaien aan `(θ − 10) mod 360`; kies de kleinste. Laat de laatste graden altijd in dezelfde richting eindigen (bij terugdraaien iets voorbij en dan vooruit terug), zodat de backlash constant blijft.
 7. **Closed-loop uitlijnen via null-seek op A−B.** Grof naar de verwachte piekpositie op stappen, daarna fijn bijregelen met `measure_now()`. Zoek de **nuldoorgang van A−B** (na gain-correctie), niet de opgeslagen maximum-magnitude: het verschilsignaal is scherper en ongevoelig voor helderheidsdrift, terwijl een magnitude-drempel te vroeg stopt doordat de bewegings-scan de piek uitsmeert. Begrens de fijnregeling tot een venster (bv. ±15°) rond de verwachte positie en val bij geen duidelijke nuldoorgang terug op de stappen-target. Dit corrigeert stepper-slip (vervangt/verfijnt de huidige stappen-gebaseerde `backtrack` en punt 4 hierboven).

# routine voor de kalibratie servomotoren toevoegen
**De servomotoren hebben nu vast ingestelde rustwaarden, dit moet via een kalibratieroutine**
 1. De kalibratie verloopt via de REPL.
 2. Automatisch: de kalibratie verloopt door de arm-servo's na elkaar op de grijper-servo-plug te plaatsen (dit is de enige connector met stroommeting). Door het meten van de stroom kan bepaald worden wanneer de servo in de uiterste stand komt (stroom overschrijdt de grenswaarde). Voer dan een kleine correctie door (pwm verminderen zodat er marge is) om te voorkomen dat de servo bij vastlopen veel stroom blijft verbruiken. Controleer daarna of de servo 180 graden gedraaid kan worden. Dit alles geldt alleen voor de arm-servo's. **Nog te doen** — de handmatige kalibratie (punt 5) is inmiddels gedaan, deze automatische stroom-gebaseerde routine niet.
 3. ~~De grijper (servo 4, GPIO22) mag nooit meer dan 90 graden geopend worden (maximale stand).~~ **Achterhaald/onjuist** (2026-09-19, gebruiker): de 90°-grens klopt niet. Met de handmatige kalibratie van punt 5 hieronder is dit sowieso opgelost op een robuustere manier: de grijper heeft nu zijn eigen gemeten, harde duty-grenzen (open 3,0% / dicht 8,6%) die `servo_crl.py` altijd klemt (`pct_min`/`pct_max`, zie `_clamp_pct()`) — een aparte hoek-gebaseerde veiligheidsgrens is dus niet meer nodig. Zie ook de kanttekening bij punt 5: de werkelijke mechanische slag van de gebruikte servo's is iets **groter dan 180°**, dus "graden" in `servo_pos()` is sowieso een genormaliseerde schaal tussen rust en max, geen letterlijk gemeten fysieke hoek.
 4. Servo 3 (GPIO4, connector J3) is een optionele, reserve servo-aansluiting — géén onderdeel van de grijper. Deze is op de PCB bedraad (KiCad-netlist V1.2 bevestigt de aansluiting), maar wordt momenteel niet aangestuurd en is nog niet geïmplementeerd in `ServoController` (`lib/servo/servo_crl.py`). Verduidelijk of deze aansluiting alsnog meegenomen moet worden in de kalibratieroutine, of dat servo 1, 2 en 4 voorlopig volstaan.
 5. ~~Handmatig: door het opgeven van pwm-waarden.~~ **Gedaan (2026-09-19)**, per servo een eigen duty-bereik i.p.v. één gedeeld bereik: servo1 rust 3,08% / max 13,0% / horizontaal-gestrekt 12,5%; servo2 rust 2,6% / max 13,0%; servo4 (grijper) open 3,0% / dicht 8,6%. Zie [globale_specificatie_NL.md](globale_specificatie_NL.md) → `lib/servo/servo_crl.py`. **Kanttekening (gebruiker):** de werkelijke hoekverdraaiing van de huidige servo's is iets groter dan 180°; "rust" en "max" zijn dus de betrouwbare, gemeten ankerpunten, en "180°" in `servo_pos()` is een normalisatie daartussen, geen exacte fysieke hoekmeting.
 6. ~~Alle grenswaarden vastleggen in de centrale configuratie op de RP2350 (zie boven).~~ **Gedaan**: [`config.json`](config.json), sectie `"servo"` — eerste sectie van het geplande centrale configuratiebestand. ICM-20948/LDR/netwerk-secties volgen later.
 7. Als de configuratie ontbreekt: rustpositie zetten op de huidige vaste standaardwaarden. **Bijgewerkt**: dit waren de generieke midden-van-de-range-placeholders (7,5% voor alle drie); `lib/servo/servo_crl.py`'s `DEFAULT_REST_PCT`/`DEFAULT_MAX_PCT` bevatten nu de echte, hierboven gemeten kalibratiewaarden als fallback. Servo 3 (optioneel, nog niet geïmplementeerd) valt hier nog steeds buiten.

# routine voor de kalibratie van de LDR's toevoegen
**de LDR-weerstanden hebben onderling behoorlijk grote afwijkingen en moeten dus gecompenseerd worden. Voorheen hebben we ze twee aan twee gepaard. Nu gaan we een semi-automatische route maken**
 1. De meetopdrachten worden via de REPL gegeven, beginnend op 5 meter afstand tussen kar en lichtbron (draai de kar zodat de laserpointer naar de lichtbron wijst). De kar wordt met de hand op de te meten afstanden gelegd en met de laser pointer uitgelijnd.
 2. Geef de volgende meetafstand op en meet beide LDR's.
 3. Herhaal dit tot een afstand van 5 cm. Meten bij afstanden 5m, 4m, 3m, 2m, 1m, 50cm, 15cm en 5cm.
 4. Bereken een correctie en sla die op op de RP2350 (zie centrale configuratie). Een correctietabel die de gevoeligheid tussen beide LDR's corrigeert.

**Stand 2026-10-19:** handmatig gemeten (zie [`Meting_LDR_afstand_hoek.md`](Meting_LDR_afstand_hoek.md)) en verwerkt als machtscorrectie in `config.json` → `"ldr"` en `lib/LDR/ldr_scan_isr.py`. Nog te doen: de semi-automatische REPL-routine hierboven, zodat dit na een LDR-wissel herhaald kan worden.

# voorstel maken voor het toevoegen van een ramp voor de stepper motors
**Nu wordt direct een snelheid opgegeven voor de stappenmotor; als de massatraagheid te groot is zal de kar niet in beweging komen. Nu loopt het aansturen en meten van de afgelegde weg 100% in de PIO zonder CPU-overhead (behalve een eenmalige interrupt als de targetpositie gehaald is)**
 1. ~~Is er een lineaire versnellingsregeling mogelijk die geen of minimale CPU-overhead nodig heeft?~~ **Uitgewerkt en geïmplementeerd** in [`lib/stepper/stepper_ramp.py`](lib/stepper/stepper_ramp.py); werkingsprincipe volledig beschreven in [globale_specificatie.md](globale_specificatie.md). Antwoord: ja. Elk FIFO-woord codeert een heel segment `(aantal stappen, delay)` in plaats van één stap, waardoor een S-curve-ramp van 2200 stappen in 1 kB past en via DMA wordt afgespeeld. Zonder bijsturing loopt een complete beweging als **één DMA-transfer met nul CPU-overhead** — dus precies het oude gedrag, mét ramp. (Er is géén IRQ: voltooiing wordt gepolld met `busy()`.) Met bijsturing kost het ~0,1 % CPU.
    - Waarom eerdere pogingen faalden: het was geen koppelprobleem (er is factor 8 marge bij VREF 1 V) maar een **synchronisme**-fout. Een snelheid direct commanderen betekent dat de rotor binnen één microstap van 78 µs naar de eindsnelheid moet springen; dat kan geen enkele motor volgen.
    - Nog te doen op hardware: `TURN_SIGN` verifiëren, maximale startsnelheid en versnelling meten met de ICM-20948 als referentie, en `kp_ldr`/`kp_gyro` afstemmen.

# calibraties
**Alle calibraties worden in een aparte code sectie uitgevoerd, los van het besturingsprogramma. Bedoeling is dat alle losse kalibraties (ICM-20948-stappenmotorcompensatie, servo's, LDR's) na elkaar in één sessie doorlopen kunnen worden; kalibraties die niet nodig zijn worden overgeslagen.**
 1. Na opstart wordt via de REPL gevraagd of er gekalibreerd moet worden. Als binnen 2 seconden geen bevestiging via het keyboard gegeven wordt, start het besturingsprogramma op.
 2. Bij bevestiging worden de kalibraties stap voor stap (na elkaar) aangeboden, elk met de vraag of deze stap uitgevoerd moet worden:
    - **n**: deze stap wordt overgeslagen, door naar de volgende kalibratie.
    - **y**: de kalibratie wordt uitgevoerd; het resultaat wordt getoond met de vraag of dit opgeslagen moet worden (y/n).
    - **x**: de gehele kalibratiesessie wordt direct beëindigd, met de vraag of de tot dan toe uitgevoerde kalibraties opgeslagen moeten worden.
 3. De configuratie wordt bij start van de sessie ingelezen. Zodra tijdens een kalibratiestap daadwerkelijk een wijziging optreedt, wordt eerst de nog ongewijzigde (sessie-start-)configuratie weggeschreven naar `config_backup.json` (eenmalige backup per sessie), en pas daarna de nieuwe waarde in het configuratiebestand (`config.json`) opgeslagen.
 4. Tijdens de kalibratiesessie toont de WS2812B-LED paars (zie WS2812B-sectie hierboven) en toont het display een tekst die aangeeft dat het systeem in kalibratiemodus staat.
 5. Deze sectie is de overkoepelende opstartflow die de ICM-20948-stappenmotorkalibratie (punt 6 hierboven), de servo-kalibratieroutine en de LDR-kalibratieroutine na elkaar aanroept.

# stepper omzetten naar 1/64 microstepping (12800 stappen/omw)
**De microstepping is van 1/8 (1600 stappen/omw) naar 1/64 (12800 stappen/omw) gebracht, o.a. om de stepper-ramp gladder en eenvoudiger te kunnen implementeren (kleinere snelheidssprong per puls).**
 1. MS-bedrading fysiek verifiëren: TMC2209 MS1 → GND, MS2 → +5V (10 kΩ pull-ups verwijderd). Waarheidstabel: MS2=H/VIO, MS1=L/GND → 1/64.
 2. ~~In `lib/stepper/stepper.py` `STEPS_REV = 12800` zetten.~~ **Gedaan.** `CM_PER_STEP ≈ 14,9 µm/stap` (na correctie van `WHEEL_CIRC` naar de gemeten 19,1 cm).
 3. `OVERHEAD` in `speed_to_delay()` hermeten: bij 8× meer pulsen wordt de vaste overhead een groter aandeel van de delay (~11% bij 30 cm/s) → snelheidsafwijking. Alternatief/aanvullend: `F_PIO` verhogen (fijnere resolutie, verwaarloosbare overhead).
 4. Randvoorwaarde bij het verhogen van `F_PIO`: de STEP-puls moet ≥ ± 100 ns hoog/laag blijven (TMC2209-minimum); eventueel een extra `nop` in de PIO-puls houden.

# rijden naar de lichtbron met LDR-correctie
**Tijdens het rijden naar de lichtbron wordt continu gecorrigeerd op koersafwijking: het verschil tussen de twee LDR-waarden (na gain-correctie) naar nul brengen (A ≈ B = recht op de bron).**
 1. Gekozen aanpak: segmentgewijs (past op de bestaande fire-and-forget + IRQ-stop stepper-architectuur). Rij een kort segment → meet beide LDR's → buiten een deadband een kleine, in hoek begrensde `rotate()`-correctie richting de helderste kant → herhaal.
 2. Stop op ingestelde afstand via de ultrasoonsensor (draait onafhankelijk in de achtergrond, SM4).
 3. Afhankelijk van de LDR-kalibratie (correctie gevoeligheidsverschil A/B) — zonder goede kalibratie stuurt de kar scheef.
 4. Arbitrage met het kompas vastleggen: LDR is leidend tijdens de nadering; de ICM-20948-koerscorrectie is voor de terugweg. Beide sturen niet tegelijk.

# microdot-websocket voor besturing via de browser (Pico 2 W)
**De kar (Pico 2 W, onboard CYW43-WiFi) wordt via een standaard browser bediend en uitgelezen met een microdot-webserver + websocket. Draait op core 0 naast de besturing; core 1 blijft voor ICM-20948 + display.**
 1. Sensoren uitlezen (~5–10 Hz): LDR A/B in %, ultrasoon-afstand in cm (of time-out), kompasrichting in graden, servoposities in %, stepper-snelheid/richting/afgelegde weg.
 2. Directe rijbesturing: vooruit/achteruit/links/rechts + snelheid, met noodstop en een deadman/heartbeat (bij verbroken websocket of uitblijvende heartbeat stopt de kar automatisch).
 3. Hoog-niveau commando's: LDR-scan, rijden-naar-licht, grijpen, terugkeren, kalibratiemodus starten/stoppen.
 4. Async-refactor vereist: de besturing is nu blokkerend (busy-wait `while sm.active(): pass`, blokkerende `scan()`/ADC-lussen) en verhongert de asyncio-event-loop. Nodig: coöperatieve taken (`await asyncio.sleep`) of een commandowachtrij + gedeelde state tussen webserver en control-task.
 5. Netwerk: AP-modus (kar als eigen accesspoint) heeft de voorkeur; station-modus optioneel. SSID/wachtwoord in de centrale `config.json`. **Vastgelegd (2026-09-19, gebruiker):** besturing en logging gebeuren altijd vanaf een aparte, externe pc/laptop/tablet die met het AP verbindt — nooit vanaf de pc die de Pico programmeert. Zie `globale_specificatie_NL.md` → webserver-sectie en `tests/test_data_logging.py`.
 6. Geheugen bewaken (microdot + asyncio + lwIP + bestaande modules op ~520 KB RAM is krap) en commandovalidatie/veiligheid borgen.

# updaten globale_specificatie.md
**Deze te-doen-lijst betekent dat de globale_specificatie.md na elke succesvol afgeronde stap in deze lijst aangepast moet worden.**
