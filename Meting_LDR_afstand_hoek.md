Meting LDR spannings waarden met 10K naar +3.29V in een verduisterde ruimte.
De meting is uitgevoerd met behulp van test_ldr.py en de laser op 100% 
om de kar exact op het midden van de bron te richten.
We hebben de spanning over de LDR's gemeten bij verschillende afstanden en als laatste met verschillende hoeken t.o.v de lichtbron.

afstand (m)     LDRA(V) LDRB(V)
4               3,28    3,29    (LICHTBRON UIT)
4               2,86    2,99
3               2,67    2,82
2               2,28    2,47
1               1,52    1,70
0,7             1,03    1,14
0,5             0,76    0,84
0,3             0,45    0,5
0,2             0,34    0,36
0,15            0,3     0,31
0,1             0,34    0,32
0,05            0,7     0,91

De volgende metingen zijn uitgevoerd op een afstand van steeds 0,5 meter van af de bron met steeds een 
andere hoek ten opzichte van de lichtbron. De kar is steeds 10 graden verder naar rechts (LDRB) verschoven.
De laser wijst steeds naar het middelpunt van de licht bron.

Hoek    LDRA(V) LDRB(V) hoek-   LDRA(V) LDRB(V)
1       0,75    0,85
2       0,75    0,85
3       0,75    0,85
4       0,76    0,86
5       0,76    0,86    -5      0,78    0,85
10      0,77    0,88    -10     0,79    0,87
15      0,79    0,92    -15     0,81    0,88
20      0,82    0,97    -20     0,83    0,91
25      0,87    1,02    -25     0,90    0,94
30      0,91    1,07    -30     0,97    1,02
35      0,94    1,13    -35     1,07    1,11
40      1,01    1,23    -40     1,18    1,21
45      1,10    1,38    -45     1,37    1,35
50      1,23    1,61    -50     1,63    1,58
55      1,45    1,92    -55     1,99    1,85
60      1,78    2,36    -60     2,39    2,25
65      2,17    2,83    -65     2,87    2,55

De volgende metingen zijn o
uitgevoerd met de kar 0,5 meter recht voor de lichtbron maar de laser wijst steeds iets verder van de lichtbron af

Hoek    LDRA(V) LDRB(V) hoek -  LDRA(V) LDRB(V)
1       0,78    0,88    -1      0,78    0,85
2       0,78    0,89    -2      0,80    0,86
3       0,77    0,92    -3      0,83    0,85
4       0,77    0,95    -4      0,87    0,85
5       0,78    1,00    -5      0,90    0,85
10      0,92    1,64    -10     1,51    1,02
15      1,60    2,80    -15     2,58    1,74
20      2,55    2,91    -20     2,71    2,73
25      2,85    3,02    -25     2,84    2,84

---

## Analyse (2026-10-19)

**Let op:** de pull-up (R29/R30) is **4,7 kΩ**, niet 10 kΩ zoals bovenaan staat. De spanningen hierboven zijn gemeten met `test_ldr.py` (V = ADC/65535 × 3,3), dus de weerstand is R = 4700 × V / (3,3 − V).

### LDR-weerstand per afstand

| afstand (m) | R_A (Ω) | R_B (Ω) | R_B/R_A |
|---|---|---|---|
| 4 | 30 550 | 45 332 | 1,48 |
| 3 | 19 919 | 27 612 | 1,39 |
| 2 | 10 506 | 13 987 | 1,33 |
| 1 | 4 013 | 4 994 | 1,24 |
| 0,7 | 2 133 | 2 481 | 1,16 |
| 0,5 | 1 406 | 1 605 | 1,14 |
| 0,3 | 742 | 839 | 1,13 |
| 0,2 | 540 | 576 | 1,07 |
| 0,15 | 470 | 487 | 1,04 |
| 0,1 | 540 | 505 | (bron buiten blikveld) |
| 0,05 | 1 265 | 1 790 | (bron buiten blikveld) |

- R ∝ d^1,32 (A) en d^1,42 (B). Onder ~15 cm stijgt de weerstand weer: de bron valt buiten het blikveld. Daar de ultrasoon gebruiken.
- De verhouding B/A is niet constant, dus een vaste gain past niet. Een machtsverband wel:
  **ln R_B = 1,0772 · ln R_A − 0,4228** (fit 0,15–4 m, restfout ≤ 0,036 in ln R).
  Beste vaste gain (1,21) laat tot 0,20 restfout over.
- Verwerkt in `config.json` → `"ldr"` (`b_gamma`, `b_ln_c`) en `lib/LDR/ldr_scan_isr.py`.

### Richtingsgevoeligheid (kar gedraaid op 0,5 m)

Richtingssignaal `ldr_diff = ln R_A − ln R_B'` (B gecorrigeerd naar A). Positief = bron rechts van de kar (B is de rechter cel). In de tabel betekent "+" dat de laser rechts van de bron wijst, de bron ligt dan links.

| hoek | laser links van bron (bron rechts) | laser rechts van bron (bron links) |
|---|---|---|
| 1° | +0,023 | −0,020 |
| 2° | +0,042 | −0,035 |
| 3° | +0,106 | −0,094 |
| 5° | +0,215 | −0,186 |
| 10° | +0,790 | −0,726 |
| 15° | +1,388 | −1,447 |
| 20° | +0,284 | −0,429 |
| 25° | +0,344 | −0,149 |

- Symmetrisch, nulpunt binnen ~1°. Rond 0° ongeveer 0,04–0,05 per graad.
- Monotoon tot ±15°; daarna kijken beide cellen langs de bron. Het teken blijft tot ±25° goed.
- Met de vaste gain zou de koersfout op 4 m ~5° zijn, met de machtscorrectie ~1°.

### Lampbundel (kar langs de boog, laser op de bron)

Deze reeks meet de lichtverdeling van de lamp, niet de LDR's: relatieve helderheid ~90 % bij ±20°, ~55 % bij ±40°, 5–15 % bij ±65°. Buiten de bundelas ontstaat ook bij recht richten een A/B-verschil (≈ 2–3° koersfout bij 30° naast de as, ≈ 7° bij 50°); de kar buigt dan iets af naar de bundelas.
