# ================================================================
# Eindnadering — het laatste correctiemoment voor de grijper
# ================================================================
#
# Stond eerder als finetune() in lib/stepper/stepper_ramp.py. Het combineert
# drie dingen die niet in een motordriver horen: grijpergeometrie,
# ultrasoonmiddeling en een stopstrategie. De stepper levert alleen nog
# creep/drive/brake/busy; hier wordt bepaald WAARHEEN.
# ================================================================

import sys

for _p in ("/lib/gripper", "/lib/stepper"):
    if _p not in sys.path:
        sys.path.append(_p)

import geometry                                     # noqa: E402
import stepper_ramp as stepper                      # noqa: E402


def mean_dist_cm(read_cm, n=8, wacht_ms=60):
    """Gemiddelde afstand uit n ONAFHANKELIJKE ultrasoonmetingen.

    wacht_ms moet groter zijn dan ultrasoon.INTERVAL_MS (50 ms), anders lees je
    dezelfde gebufferde meting meerdere keren en doet het gemiddelde niets.

    Geeft None als geen enkele meting bruikbaar was.
    """
    import time
    som, tel = 0.0, 0
    for i in range(n):
        if i:
            time.sleep_ms(wacht_ms)
        d = read_cm()
        if d and d > 0:
            som += d
            tel += 1
    return (som / tel) if tel else None


def finetune(read_cm, object_w_cm=None, tol_cm=0.1, pogingen=3,
             speed_cm_s=2.0, n_meet=8):
    """Meet stilstaand na en kruip naar de exacte stopafstand.

    Dit is het LAATSTE correctiemoment. Zodra de arm uitklapt kijkt de ultrasoon
    naar de eigen vingers en is er geen terugkoppeling meer -- alles daarna is
    open-loop. Stilstaand meten haalt de rijsnelheid en de meetlatentie uit de
    fout, waardoor de onzekerheid van ~0,74 cm naar ~0,3 cm zakt.

    `read_cm` is een callable die de ultrasoonafstand in cm geeft (bv.
    ultrasoon.read_cm), zodat deze module sensor-agnostisch blijft.

    Na een correctie wordt ALTIJD opnieuw gemeten, ook na de laatste poging.
    Anders zou het oordeel op de meting van vóór die correctie gebaseerd zijn en
    kon de kar goed staan terwijl er "BUITEN VENSTER" gerapporteerd werd.

    Geeft (gemeten_afstand, doel, gelukt) terug.
    """
    doel = geometry.stop_dist_cm(object_w_cm)
    lo, hi = geometry.grip_window_cm(object_w_cm)

    for _ in range(max(1, int(pogingen))):
        d = mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel, False
        fout = d - doel
        if abs(fout) <= tol_cm:
            break
        stepper.creep(fout, speed_cm_s)
        while stepper.busy():
            pass
    else:
        # De lus liep af zonder break: er is na de laatste correctie nog niet
        # gemeten. Doe dat alsnog, anders beoordelen we een oude stand.
        d = mean_dist_cm(read_cm, n_meet)
        if d is None:
            return None, doel, False

    gelukt = lo <= d <= hi + tol_cm
    print("finetune: gemeten %.2f cm, doel %.2f cm, venster %.2f-%.2f -> %s"
          % (d, doel, lo, hi, "OK" if gelukt else "BUITEN VENSTER"))
    return d, doel, gelukt


def brake_target_cm(speed_cm_s, object_w_cm=None):
    """Ultrasoonafstand waarop je Move.finish() moet aanroepen.

    De kar legt na finish() nog stopping_distance_cm() af (afremramp plus de
    slices die al in de FIFO's staan), dus het remmoment ligt die afstand vóór
    de doelafstand:

        doel = brake_target_cm(snelheid)
        if ultrasoon.read_cm() <= doel:
            mv.finish()

    NIET meegerekend: de meetlatentie van de ultrasoon zelf (INTERVAL_MS = 50 ms,
    dus tot 0,96 cm bij volle snelheid en 0,25 cm bij 5 cm/s).
    """
    return (geometry.stop_dist_cm(object_w_cm)
            + stepper.stopping_distance_cm(speed_cm_s))
