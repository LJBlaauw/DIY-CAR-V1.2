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
# Gemeten: maximaal open (9 cm) -> toppen 12 cm vóór de ultrasoon;
#          bijna dicht  (2 cm) -> toppen 15 cm vóór de ultrasoon.
#
# LET OP: tip_pos_cm() is een RECHTE door die twee meetpunten. Een
# vierstangenmechanisme geeft in werkelijkheid een kromme; één extra meting bij
# ~5 cm opening laat zien hoeveel dat afwijkt.
#
# Tijdens het rijden staan de servo's in rustpositie: de kaken liggen dan ACHTER
# de ultrasoon en vallen buiten de bundel, dus de afstandsmeting is dan zuiver.
# Zodra de arm uitklapt staan de kaken (9 cm open) in een bundel die op 12-15 cm
# ongeveer 8 cm breed is -> vanaf dat moment kijkt de sensor naar de eigen
# vingers en is er GEEN terugkoppeling meer.
# ================================================================

GRIP_OPEN_CM  = 9.0      # kaakopening maximaal open
GRIP_MIN_CM   = 2.0      # kaakopening bijna dicht
TIP_NEAR_CM   = 12.0     # toppen t.o.v. ultrasoon bij maximaal open
TIP_FAR_CM    = 15.0     # toppen t.o.v. ultrasoon bij bijna dicht
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

        3 cm -> 14,6 cm      6 cm -> 13,3 cm
        5 cm -> 13,7 cm      8 cm -> 12,4 cm
    """
    return tip_pos_cm(OBJECT_W_CM if object_w_cm is None else object_w_cm)


def grip_window_cm(object_w_cm=None):
    """(min, max) ultrasoonafstand waarbij het voorwerp nog gegrepen wordt.

    Het voorwerp wordt gegrepen zolang de toppen er langs vegen terwijl de
    opening nog ruimer is dan het voorwerp. De vooruitgang van 3 cm is dus een
    gratis venster bovenop de stopnauwkeurigheid:

        3 cm breed -> 2,6 cm venster      7 cm breed -> 0,9 cm venster
        5 cm breed -> 1,7 cm venster      8 cm breed -> 0,4 cm venster

    De ondergrens is conservatief TIP_NEAR_CM. De werkelijke ondergrens ligt
    lager en wordt bepaald door de KAAKDIEPTE (palm t.o.v. toppen), die nog niet
    is opgemeten.
    """
    return TIP_NEAR_CM, stop_dist_cm(object_w_cm)


def lateral_tolerance_cm(object_w_cm=None):
    """Maximale laterale afwijking waarbij het voorwerp nog tussen de kaken past.

    De kaken vegen tijdens het uitklappen door de ruimte waar het voorwerp
    staat. Bij een grotere afwijking raakt één kaak het voorwerp en STOOT die
    het om -- een vervelender faalmodus dan alleen misgrijpen. Klap de arm
    daarom uit BOVEN het voorwerp en laat hem zakken; dan komen de kaken er
    van boven om heen in plaats van er horizontaal in.

        3 cm -> +/- 3,0 cm      6 cm -> +/- 1,5 cm
        5 cm -> +/- 2,0 cm      7 cm -> +/- 1,0 cm
    """
    w = OBJECT_W_CM if object_w_cm is None else object_w_cm
    return 0.5 * (GRIP_OPEN_CM - w)


# 13,29 cm bij een voorwerp van 6 cm. Rond dit NIET af naar 13 cm: bij een
# gemikte eindnauwkeurigheid van ~0,3 cm is 0,3 cm afronding de hele foutbegroting.
STOP_DIST_CM = tip_pos_cm(OBJECT_W_CM)
