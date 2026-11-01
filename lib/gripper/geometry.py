# ================================================================
# Grijpergeometrie — waar moet de kar stoppen om te kunnen pakken?
# ================================================================
#
# Stond eerder in lib/stepper/stepper_ramp.py. Daar hoorde het niet: een
# generieke motordriver werd zo afhankelijk van de afmetingen van één grijper
# en van de stopstrategie van één missie. Deze module is puur rekenwerk —
# geen hardware, geen imports — zodat hij overal en ook op de PC te testen is.
#
# De kaken sluiten HORIZONTAAL, maar de vingertoppen bewegen daarbij naar VOREN.
# Gemeten (2026-10-31, arm volledig gestrekt J1=J2=180°): rust/maximaal open
# (J10=3% PWM) -> 8,5 cm, toppen 11 cm vóór de ultrasoon.
#
# Rust draait het vingerscharnier een stukje VOORBIJ de 90°-referentiestand:
# bij het scharnier exact op 90° meet je wél 8,7 cm/12 cm (de oudere waarden).
# Dat is geen meetfout maar het vierstangenmechanisme zelf: de horizontale
# projectie van de vinger heeft een top bij 90° scharnierhoek en neemt
# daarna weer af (cosinus-vormig), dus "verder open draaien" voorbij 90°
# klapt de opening juist een beetje terug. Omdat GRIP_OPEN_CM hier bedoeld is
# als "wat er gebeurt bij rust" (niet "bij 90°"), zijn de constanten op de
# rust-meting gezet; rest_pct is bewust NIET aangepast (laat de volle
# mechanische slag van de servo intact).
#
# LET OP: tip_pos_cm() is een RECHTE door twee meetpunten. Een
# vierstangenmechanisme geeft in werkelijkheid een kromme.
#
# Derde meetpunt gedaan (2026-10-31, J4 label 78 graden, opening 7,5cm):
# gemeten top-tot-ultrasoon 13,5cm tegen 11,56cm voorspeld door de rechte
# lijn -- 1,94cm (17%) afwijking, veel meer dan de ~2% die bij de
# armkinematica-validatie werd gevonden. Verklaring: de vinger VERSCHUIFT
# (geen rotatie) langs een cirkelboog, dus x (top-tot-ultrasoon) en y (halve
# opening) zijn beide een cosinus/sinus van HETZELFDE scharnierhoek, met 90
# graden faseverschil. Rust zit al dicht bij de top van de sinus (vandaar
# 8,5 i.p.v. 8,7cm, zie hierboven) -- en precies daar is de cosinus (x) het
# steilst. Een rechte lijn tussen twee punten in opening-ruimte is dus
# fundamenteel de verkeerde vorm, niet alleen een ruwe benadering, en de
# fout is het grootst juist in het praktisch relevante midden-gebied
# (objectbreedtes 3-8cm liggen allemaal in dat midden).
#
# Cirkelboog-fit op de 3 meetpunten (x=tip_cm, y=opening_cm/2): middelpunt
# (11,51; 0,29), straal 3,99cm -- opvallend dicht bij de ONTWORPEN balklengte
# van het parallellogram (4,2cm, zie hierboven), wat de cirkelboog-hypothese
# sterk onderbouwt. GEEN vervangformule ingebouwd: het dichte-eind-meetpunt
# (0,5cm) ligt dicht bij het x-extreem van deze cirkel, en met maar 3
# ruisgevoelige handmetingen is niet zeker hoe de boog zich daar precies
# gedraagt. Een betrouwbare vervanger vraagt meer meetpunten (vooral rond de
# dichte kant) of een herleiding uit de bekende mechanismematen in plaats van
# een fit op drie losse metingen -- bewust niet vandaag gedaan.
#
# Tijdens het rijden staan de servo's in rustpositie: de kaken liggen dan ACHTER
# de ultrasoon en vallen buiten de bundel, dus de afstandsmeting is dan zuiver.
# Zodra de arm uitklapt staan de kaken (8,7 cm open) in een bundel die op 12-15 cm
# ongeveer 8 cm breed is -> vanaf dat moment kijkt de sensor naar de eigen
# vingers en is er GEEN terugkoppeling meer.
# ================================================================

GRIP_OPEN_CM  = 8.5      # kaakopening bij rust (J10=3% PWM). Herijkt 2026-10-31
                         # (was 8,7cm, 2026-09-21): bij het scharnier exact op
                         # 90° is het wel 8,7cm, maar rust draait er net voorbij
                         # (zie toelichting hierboven). 8,5cm is wat er bij
                         # rust werkelijk gebeurt.
GRIP_MIN_CM   = 0.5      # kaakopening dicht (J10=8,6% PWM), gemeten tussen de
                         # VINGERTOPPEN. Herijkt 2026-10-31 (was 2,0cm; de
                         # werkelijke tip-tip-afstand is maar 5mm, niet 2cm).
TIP_NEAR_CM   = 11.0     # toppen t.o.v. ultrasoon bij rust. Herijkt 2026-10-31
                         # (was 12,0cm), zelfde reden als GRIP_OPEN_CM hierboven.
TIP_FAR_CM    = 15.5     # toppen t.o.v. ultrasoon bij dicht. Herijkt 2026-10-31
                         # (was 15,0cm).

# Elke vinger heeft een symmetrische CONCAVE uitsparing in het midden (niet aan
# de top): bij dicht geeft dat halverwege de vinger een opening van 8mm, tegen
# 5mm (GRIP_MIN_CM) aan de toppen zelf. Bedoeld om kleine/ronde voorwerpen
# tussen de uitsparingen vast te houden i.p.v. alleen op de toppen te pinchen.
# Nog niet in een apart kengetal verwerkt -- puur ter documentatie.

# Vingerlengte (scharnier -> top): 2,7 cm (gemeten 2026-10-31). Dit is de
# eigenlijke "kaakdiepte": twee vingers die vanaf de zijkant tot in het hart
# van een voorwerp moeten reiken, kunnen dus samen maximaal 2 x 2,7 = 5,4 cm
# overbruggen. Voorwerpen BREDER dan 5,4 cm kunnen niet met de vingertoppen
# voorbij het midden grijpen -- let op: de huidige default OBJECT_W_CM (6,0cm,
# zie hieronder) ligt daar AL OVER. Voor zo'n voorwerp pinchen de toppen alleen
# het oppervlak, zonder om het midden heen te reiken -- minder stabiele greep,
# geen harde onmogelijkheid.
OBJECT_W_CM   = 6.0      # aangenomen objectbreedte; per missie te overschrijven


def tip_pos_cm(opening_cm):
    """Afstand van de vingertoppen tot de ultrasoon bij een gegeven kaakopening."""
    f = (GRIP_OPEN_CM - opening_cm) / (GRIP_OPEN_CM - GRIP_MIN_CM)
    return TIP_NEAR_CM + f * (TIP_FAR_CM - TIP_NEAR_CM)


def stop_dist_cm(object_w_cm=None):
    """Doelafstand (ultrasoon) voor een voorwerp van deze breedte.

    De kaken raken het voorwerp op het moment dat de opening gelijk is aan de
    objectbreedte; dan staan de toppen op tip_pos_cm(breedte).

    Contra-intuïtief maar juist: een SMALLER voorwerp vraagt een GROTERE
    stopafstand. Smaller betekent verder sluiten, dus meer vooruitgang van de
    toppen, dus moet de kar verder terug blijven staan.

    Herijkt 2026-10-31 (GRIP_OPEN_CM/GRIP_MIN_CM/TIP_NEAR_CM/TIP_FAR_CM, zie
    hierboven):

        3 cm -> 14,1 cm      6 cm -> 12,4 cm
        5 cm -> 13,0 cm      8 cm -> 11,3 cm
    """
    return tip_pos_cm(OBJECT_W_CM if object_w_cm is None else object_w_cm)


def grip_window_cm(object_w_cm=None):
    """(min, max) ultrasoonafstand waarbij het voorwerp nog gegrepen wordt.

    Het voorwerp wordt gegrepen zolang de toppen er langs vegen terwijl de
    opening nog ruimer is dan het voorwerp. De vooruitgang van 4,5 cm (TIP_FAR_CM
    - TIP_NEAR_CM) is dus een gratis venster bovenop de stopnauwkeurigheid.
    Herijkt 2026-10-31:

        3 cm breed -> 3,1 cm venster      7 cm breed -> 0,8 cm venster
        5 cm breed -> 2,0 cm venster      8 cm breed -> 0,3 cm venster

    De ondergrens is conservatief TIP_NEAR_CM. De werkelijke ondergrens kan
    lager, bepaald door de KAAKDIEPTE (vingerlengte scharnier->top, gemeten
    2026-10-31: 2,7 cm) -- in principe tot TIP_NEAR_CM - 2,7 = 8,3 cm, zodra de
    vingers over de objectrand heen mogen reiken i.p.v. er alleen tegenaan te
    komen. Nog NIET verwerkt in de return hieronder: vraagt een bewuste
    ontwerpkeuze (grijpt de vinger dan om het voorwerp, of raakt de palm het
    eerst?), dus voorlopig bij de conservatieve TIP_NEAR_CM gelaten.
    """
    return TIP_NEAR_CM, stop_dist_cm(object_w_cm)


def lateral_tolerance_cm(object_w_cm=None):
    """Maximale laterale afwijking waarbij het voorwerp nog tussen de kaken past.

    De kaken vegen tijdens het uitklappen door de ruimte waar het voorwerp
    staat. Bij een grotere afwijking raakt één kaak het voorwerp en STOOT die
    het om -- een vervelender faalmodus dan alleen misgrijpen. Klap de arm
    daarom uit BOVEN het voorwerp en laat hem zakken; dan komen de kaken er
    van boven om heen in plaats van er horizontaal in.

    Herijkt 2026-10-31 (GRIP_OPEN_CM 8,7->8,5cm):

        3 cm -> +/- 2,75 cm      6 cm -> +/- 1,25 cm
        4 cm -> +/- 2,25 cm      7 cm -> +/- 0,75 cm
        5 cm -> +/- 1,75 cm      8 cm -> +/- 0,25 cm
    """
    w = OBJECT_W_CM if object_w_cm is None else object_w_cm
    return 0.5 * (GRIP_OPEN_CM - w)


# 12,41 cm bij een voorwerp van 6 cm (herijkt 2026-10-31; was 13,29cm). Rond
# dit NIET af: bij een gemikte eindnauwkeurigheid van ~0,3 cm is 0,3 cm
# afronding de hele foutbegroting.
STOP_DIST_CM = tip_pos_cm(OBJECT_W_CM)


# ================================================================
# Armkinematica (schouder J1 + elleboog J2) — hoogte/afstand van de grijper
# ================================================================
#
# CAD-exact afgeleid uit FreeCAD-zijaanzicht-projecties (2026-09-21), oorsprong
# = voorkant ultrasoon (x) / bovenkant controlbord (z, dat ligt 75mm boven de
# grond). Gevalideerd tegen twee gemeten standen: rust (J1=J2=0°) en volledig
# gestrekt (J1=J2=180°) — zie globale_specificatie_NL.md, sectie
# "Armkinematica", voor de volledige herleiding.
#
# Assen: x = horizontaal, positief naar de lichtbron/het voorwerp toe
# (oorsprong = voorkant ultrasoon, dus NEGATIEF = achter de ultrasoon);
# z = hoogte boven de grond, in cm (net als de rest van deze module).
#
# Hoekconventie — label_j1/label_j2 zijn de 0-180 servo-graden zoals
# servo_pos() ze aanneemt (0 = rust, 180 = max):
#   alpha1  = label_j1                  0°=horizontaal-achter, 90°=recht
#                                        omhoog, 180°=horizontaal-voor
#   alpha2  = label_j1 + 90 - label_j2  onderarmhoek, zelfde conventie
#   alpha_h = alpha2 + 90               handhoek -- vaste 90° t.o.v. de
#                                        onderarm, de hand heeft geen eigen
#                                        servo/vrijheidsgraad
#
# ARM_L3_CM (pols -> vingertop) komt NIET uit de CAD-projectie maar uit de
# eerdere handmatige vingerafmetingen (22+41+32+30mm); die tabel had een
# interne inconsistentie die nog niet is opgehelderd — [TE VERIFIËREN].

import math

SHOULDER_X_CM = -3.72     # schouderas t.o.v. voorkant ultrasoon (negatief = erachter)
SHOULDER_Z_CM = 10.30     # schouderas, hoogte boven de grond
ARM_L1_CM     = 7.40      # schouder -> elleboog (CAD-exact)
ARM_L2_CM     = 3.425     # elleboog -> polsmechanisme/tandwielcentrum (CAD-exact)
ARM_L3_CM     = 12.5      # pols -> vingertop [TE VERIFIËREN, zie hierboven]


def _joint_xz(x0, z0, length_cm, angle_deg):
    """Eindpunt van één schakel: x0,z0 + length_cm in richting angle_deg,
    met de armconventie hierboven (0°=-x, 90°=+z, 180°=+x)."""
    rad = math.radians(angle_deg)
    return x0 - length_cm * math.cos(rad), z0 + length_cm * math.sin(rad)


def arm_positie_cm(label_j1, label_j2):
    """Positie (x, z) van elleboog, pols en vingertop bij gegeven J1/J2-hoeken
    (0-180 graden, zoals servo_pos() ze aanneemt).

    Retourneert ((x_elleboog, z_elleboog), (x_pols, z_pols), (x_vingertop, z_vingertop)),
    in cm t.o.v. de ultrasoon (x) en de grond (z).
    """
    alpha1 = label_j1
    alpha2 = label_j1 + 90.0 - label_j2
    alpha_h = alpha2 + 90.0

    x1, z1 = _joint_xz(SHOULDER_X_CM, SHOULDER_Z_CM, ARM_L1_CM, alpha1)
    x2, z2 = _joint_xz(x1, z1, ARM_L2_CM, alpha2)
    x3, z3 = _joint_xz(x2, z2, ARM_L3_CM, alpha_h)
    return (x1, z1), (x2, z2), (x3, z3)


def vingertop_cm(label_j1, label_j2):
    """Kortweg: (x, z) van de vingertop bij gegeven J1/J2-hoeken (graden)."""
    return arm_positie_cm(label_j1, label_j2)[2]
