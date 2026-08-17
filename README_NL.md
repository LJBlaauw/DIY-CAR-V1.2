# DIY Robot CAR

**Dit project is begonnen met het idee van mijn zwager om een robot karretje te maken**
Ik heb zelf een achtergrond in ontwikkeling van elektronica (microcontrollers, vermogens-elektronica en audio).
Voor mijn zwager was dit onbekend gebied dus zoals wel vaker gebeurt heb ik mijn hulp aangeboden.
Voor de ontwikkeling is er gebruik gemaakt van opensource software. Als OS heb ik Linux Fedora 44 desktop KDE plasma gebruikt.
De uitdaging is om het karretje autonoom deze handelingen te laten uitvoeren, vooral het real-time karakter hierbij is een uitdaging.
Eerst hebben we alles op een breadboardje gebouwd. Dit was niet erg succesvol omdat de bedrading vaak geen betrouwbare verbindingen gaf.
Ook was de keuze van het type stappenmotor (unipolair) en servo's niet toereikend (te licht, te veel mechanische speling)
Dus versie nul is geüpdatet naar versie 1 met bipolaire stappenmotoren en drivers, MG996 servo's met metalen tandwielen.
Om dit te kunnen monteren zijn er diverse onderdelen 3D-geprint. 
In versie 1 was het grootste deel van de mechanica opgebouwd uit plaatmateriaal en aluminium profielen.
In versie 2 is er een elektronisch kompas en versnellingsopnemer toegevoegd. Ook is de grijper principieel gewijzigd en is zoveel als mogelijk 3D-geprint.


De eerste vragen die beantwoord moesten worden waren:
 1. Wat is de scope van het project (wat moet het robot karretje doen)
 2. Wat is goed online verkrijgbaar 
 3. Welke materialen gaan we gebruiken (motoren en sensoren)
 4. Wat voor software gaan we gebruiken om de besturing mogelijk te maken.
 5. presentatie van status en meet gegevens
 6. Ontwikkel gereedschappen en omgeving
 
1. **Scope: Het robot karretje moet autonoom zoeken naar een lichtbron en daar een voorwerp pakken en naar het vertrekpunt brengen**

2. **verkrijgbaarheid:**
 1. Bij diverse hobby winkels, amazon en aliexpress konden we alles wel vinden.
 
3. **Materialen**
 1. *Motoren:*
    1. Twee onafhankelijk te besturen stappenmotoren voor de voortbeweging.
    2. Drie onafhankelijk te besturen servomotoren voor de robotarm en grijper.
    *Sensoren:*
    1. Ultrasoonsensor om de afstand van de kar tot het object te meten.
    2. Twee licht gevoelige weerstanden om de richting van de lichtbron te vinden.
    3. We hebben als Microcontroller gekozen voor de RP2040 van Raspberry Pi, in versie 2 de RP2350W.
    4. Als optie een kompas/versnellings opnemer.
    5. Als optie een wifi koppeling met een websocket om sensor data te lezen en opdragten te geven.
    
 4. **Programmeertaal**
    Voor de programmeertaal hebben we gekozen voor Micropython dit is eenvoudiger te leren dan C/C++.
    Het voordeel is dat er niet vertaald hoeft te worden en er veel kant en klare bibliotheek componenten ingebouwd zijn.
    Om een goed real-time werkend systeem te krijgen zijn er wel een aantal uitdagingen.
    Aangezien Micropython een interpreter is loopt dit een factor 100 langzamer dan C.
    Er zijn een aantal mogelijkheden om de realtime eigenschappen te optimaliseren.
    1. Gebruik maken van de in de microcontroller geïntegreerde PIO-statemachines.
    2. De Software modules (werkbelasting) te verdelen over de twee CPU cores
    3. eventueel gebruik maken van de coöperatieve multitasking (asyncio).
    
5. **Presentatie**
    Als presentatie hebben we als status melding een meerkleuren LED en een oled display van 128x64 dots.
    Met versie twee hebben we gekozen voor de RP2350W. Deze heeft 2 M33 i.p.v de M0 kernen.
    Met deze processor update is het rekenen aan de kompas/versnellings veel sneller,
    ook is met een websocket remote besturing/uitlezing mogelijk. 
    Verder zijn er twee ledjes die de 5V en 3.3V signaleren.
    Hierbij is ook de REPL van de IDE gereedschappen aanwezig om met print opdrachten informatie te tonen.
    
6. **Ontwikkelgereedschappen**
    Voor het ontwerpen en produceren van de printplaat voor de elektronica maak ik gebruik van:
    1. Schema en layout: KiCAD (nu versie 10.4), FlatCAM Gerber naar Gcode, CNC machine een geüpgrade 3018 met GRBLHAL controller.
    2. Mechanisch ontwerp: FreeCAD voor 3D parametrische modellen en een Prusa MK4 3D-printer.
    3. Software ontwikkel omgeving: IDE VSCODE met micropico-extensie en AI claude 4.6 chat.
    4. Thonny IDE simpelere ontwikkelomgeving dan VSCODE.
    

