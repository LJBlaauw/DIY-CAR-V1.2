# DIY Robot CAR

**Dit project is begonnen met idee van mijn zwager om een robot karretje te maken**
Ik heb zelf een achtergrond in ontwikkeling van elektronica (microcontrollers, vermogens-elektronica en audio).
Voor mijn zwager was dit onbekent gebied dus zoals wel vaker gebeurd is heb ik meijn hulp aangeboden.
Voor de ontwikkeling is er gebruik gemaakt van opensourse software. Als OS heb ik Linux fedora 44 desktop KDE plasma gebruikt.
De uitdaging is om het karretje autonoom deze handelingen te laten uitvoeren, vooral het real-time karakter hierbi is een uitdaging.
Eerst hebben we alles op een breadbordje gebouwd. Dit was niet erg succesvol omdat de bedrading vaak geen betrouwbare verbindingen gaf.
Ook was de keuze van het type stappenmotor (unipolair) en servo's niet toerijkend (te licht, te veel mechanische speling)
Dus versie nul is geupdate naar versie 1 met bipolaire stappen motoren en drivers, MG996 servo's met metalen tandwielen.
Om dit te kunnen monteren zijn er diverse onderdelen 3D-geprint. 
In versie 1 was het grootste deel van de mechanica opgebouwd uit plaatmateriaal en aluminium profielen.
In versie 2 is er een electronisch kompas en versnelling opnemer toegevoegd. Ook is het grijper principeel gewijzigd en is zovell als mogelijk 3D-geprint.


De eerste vragen die beantwoord moesten worden waren:
 1. Wat is de scoop van het project (wat moet het robot karretje doen)
 2. Wat is goed online verkrijgbaar 
 3. Welke materialen gaan we gebruiken (motoren en sensoren)
 4. Wat voor software gaan we gebruiken om de besturing mogelijk te maken.
 5. presentatie van status en meet gegevens
 6. Ontwikkel gereedschappen en omgeving
 
1. **Scoop: Het Robot karretje moet autonoom zoeken naar een lichtbron en daar een voorwerp pakken en naar het vertrekpunt brengen**

2. **verkrijgbaarheid:**
 1. Bij diverse hobby winkels, amazon en aliexpress konden we alles wel vinden.
 
3. **Materialen**
 1. *Motoren:*
    1. Twee onafhankelijk te besturen stappenmotoren voor de voortbeweging.
    2. Drie onafhankelijk te besturen servomotoren voor de robotarm en grijper.
    *Sensoren:*
    1. Ultrasoonsensor om de afstand van de kar tot het opject te meten.
    2. Twee licht gevoelige weerstanden om de richting van de lichtbron te vinden.
    3. We hebben als Microcontroller gekozen voor de RP2040 van Raspberry Pi, in versie 2 de RP2350W.
    4. Als optie een kompas/versnellings opnemer.
    5. Als optie een wifi koppeling met een websocket om sensor data te lezen en opdragten te geven.
    
 4. **Programmeertaal**
    Voor de programmeertaal hebben we gekozen voor Micropython dit is eenvoudiger te leren dan C/C++.
    Het voordeel is dat er niet vertaald hoeft te worden en er veel kant en klare bibliotheek componenten ingebouwd zijn.
    Om een goed real-time werkend systeem te krijgen zijn er wel een aantal uitdagingen.
    Aangezien Micropython een interpreter is loopt dit een faktor 100 langzamer als C.
    Er zijn een aantal mogelijkheden om de realtime eigenschappen te optimaliseren.
    1. Gebruik maken van de in de microcontroller geintegreerde PIO statemachine's.
    2. De Software modules (werkbelasting) te verdelen over de twee CPU cores
    3. eventueel gebruik maken van de coorparate multitasking (asyncio).
    
5. **Presentatie**
    Als presentatie hebben we als status melding een meerkleuren LED en een oled display van 128x64 dots.
    Met versie twee hebben we gekozen voor de RP2350W. Deze heeft 2 M33 i.p.v de M0 kernen.
    Met deze processor update is het rekenen aan de kompas/versnellings veel sneller,
    ook is met een websocket remote besturing/uitlezing mogelijk. 
    Verder zijn er twee ledjes die de 5V en 3.3V signaleren.
    Hierbij is ook de REPL van de IDE gereedschappen aanwezig om met print opdrachten informatie te tonen.
    
6. **Ontwikkelgereedschappen**
    Voor het ontwerpen en produceren van de printplaat voor de elektronica maak ik gebruik van:
    1. Schema en layout: KiCAD (nu versie 10.4), FlatCAM geber naar Gcode, CNC machine een geupgrade 3018 met GRBLHAL controller.
    2. Mechanisch ontwerp: FreeCAD voor 3D parametrische modellen en een prusa MK4 3D-printer.
    3. Software ontwikkel omgeving: IDE VSCODE met micropico extentie en AI claude 4.6 chat.
    4. Thonny IDE simpelere ontwikkelomgeving dan VSCODE.
    

