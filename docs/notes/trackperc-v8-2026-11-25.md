# trackperc-v8 (2026-11-25): arc-length labels; internal screen failed, not scored on H

Labeller corner and arc-length fix (coordinator decisions under owner delegation), then v8 on
the w2-v6 split. Prestatement: `evaluation/prereg/w2-trackperc-v8-2026-11-25.yaml` (8ca8e64,
committed before any v8 training). Candidate bundle:
`car-data/pnp/models-candidate/trackperc-v8-2026-11-25`, "(candidate)" in the Net bundle picker.
No default changes; the prototype keeps v2.

**Held-out scoring count: H-layout 3 (v3, v4, v5), H-session 3 (v3, v4, v5). v8 did not add one.**

## Labeller: `selflabel --arc-length-stations --corner-smooth-m 0.1` (default off)

`arc_label.py` takes the tracer's raw marched polyline (before its resampling and the 16-station
cut). It densifies it, Gaussian-smooths the centre over 0.10 m of arc (the half widths move with
it, so the walls stay where they were measured), and places stations exactly 0.20 m apart in a
straight line from the point nearest the car. The labeller's gates are unchanged. With the flag
off it reproduces selflabels-v4-fast (same kept counts on the dev bags). New labels:
`car-data/pnp/labels/selflabels-v5-arc` (v4-fast untouched).

Train-role labels, three dev bags, label-free numbers (plain -> arc 0.1):

| bag | kept | chord arc (m) | intrusion | deficit p90 (m) | containment |
|---|---|---|---|---|---|
| session 2 | 758 -> 1116 | 2.932 -> 3.000 | 0.0007 -> 0.0013 | 0.0222 -> 0.0182 | 0.978 -> 0.983 |
| 2026-11-16T22-38-47 | 879 -> 1049 | 2.944 -> 3.000 | 0.0004 -> 0.0007 | 0.0183 -> 0.0087 | 0.982 -> 0.989 |
| 2026-11-13T22-20-18 | 2575 -> 2712 | 2.975 -> 3.000 | 0.0001 -> 0.0001 | 0.0097 -> 0.0069 | 0.9852 -> 0.9843 |

Smoothing 0.0 / 0.1 / 0.2 m was compared. 0.2 smooths more but drops full-length labels (92
percent on session 2) and containment on 2026-11-13. All bags: session 2 1505 -> 2138 labels,
2026-11-16 bags 3431 -> 4135. Station 15 of the arc labels sits 0.12 to 0.22 m from the plain
label on session 2: the arc compression plus the corner rounding.

The corner fix moves E_c for every net. On the arc labels' internal part, v2's session 2 E_c is
0.1454 (0.1681 on v4-fast labels) and v5-dev's is 0.1367 (0.1596). The old labels' corners were
inflating E_c for all nets.

## Recipe space (stated first) and result

Stage 1 is the v4 recipe on the arc labels, spacing weight 0 or 0.1. Stage 2 is the v5 sigma
refit, floor 0.10 or 0.15 m. Seed 0 of each, arc-label internal part, refined:

| config | V s2 | V 11-16 | V old | E_c s2 | E_c 11-16 | E_c old |
|---|---|---|---|---|---|---|
| sp 0, floor 0.10 | 14.92 | 15.04 | 15.62 | 0.1354 | 0.1153 | 0.0662 |
| sp 0, floor 0.15 | 15.12 | 15.06 | 15.62 | 0.1357 | 0.1156 | 0.0662 |
| sp 0.1, floor 0.10 | 15.32 | 15.35 | 15.77 | 0.1415 | 0.1133 | 0.0672 |
| sp 0.1, floor 0.15 (v8, seed 0 of 3 by raw E_c) | 15.56 | 15.38 | 15.77 | 0.1417 | 0.1135 | 0.0673 |
| v2 | 15.25 | 15.17 | 15.40 | 0.1454 | 0.1308 | 0.0653 |
| v5-dev | 14.75 | 14.84 | 15.37 | 0.1367 | 0.1123 | 0.0663 |

Screen limits on the arc labels:
- item 1, V: session 2 >= 14.95, 11-16 >= 15.04
- item 2, V: session 2 >= 15.15, 11-16 >= 15.07, old >= 15.15
- item 3, E_c: session 2 <= 0.1454, 11-16 <= 0.1308, old <= 0.0663
- item 4, E_c: session 2 <= 0.1408, 11-16 <= 0.1157, old <= 0.0683

Every config fails:
- spacing 0: item 2 on V (session 2 15.12, 11-16 15.06)
- spacing 0.1: item 4 on session 2 (0.1417) and item 3 on the older bags (0.0673)

On the v4-fast labels, v8 also fails item 3 on the older bags (0.0819 vs v5-dev 0.0751). A net
trained on the rounded, arc-length corners disagrees with the old labels' corners, so the
prestatement's "pass on both label sets" cannot be met by any arc-trained net.

Report only, never a pass item: E_c on the stations all three nets keep (arc labels).

| group | v8 | v5-dev | v2 |
|---|---|---|---|
| session 2 | 0.1339 | 0.1354 | 0.1396 |
| 2026-11-16 | 0.1079 | 0.1110 | 0.1244 |
| older | 0.0648 | 0.0658 | 0.0648 |

## Reading

The corner fix works as a label fix: the labels are cleaner by every label-free number on the new
bags, and every net's E_c drops about 13 percent. v8 has the best valid range so far (+0.8 over
v5-dev on session 2) and the best E_c on common stations. On the registered E_c, though, it is
still 0.6 percent over item 4 on session 2 and 1.5 percent over item 3 on the older bags,
because it keeps more far stations. The both-label-sets requirement in the v8 prestatement is
structurally unreachable for nets trained on the new labels. Changing that is a decision for the
coordinator, not something to loosen after seeing the result.
