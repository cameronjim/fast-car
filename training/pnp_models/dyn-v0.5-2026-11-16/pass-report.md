# v0.5 dynamics: the held-out pass (GP2f-S, GP2f-T, GP2f-L, GP2f-V, one pass)

Freeze commit: `24da648c9ac58b0afbf0f0f9eb4fc0290490ea29`.

- Constants SHA-256 2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82; session split file SHA-256 4824ec40750db64d2de98b16259149290238a681d5d51f86e80313333af4cbcd.
- Windows: 1006 on identical windows under T2 and T2s (0 only under T2, 0 only under T2s, dropped); direction of a window from the measured T2 heading of its 5 s block.
- Pp and Yp are a re-read of G1b and T2b (the only pose data), labelled as such.
- Frozen v0.4 pickles (SHA-256): n4 905e3d154da8e8c9, r4 9fe163c2ca4ad8ca, t4 5d4484ed6956818f, tl4 39ca964ad08a07aa.
- TL5 shipping: wd1e-03 seed 0 (chosen by selection P1).

## Comparator, identical windows, T2 unless the column says T2s

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Pp (m) | Yp (rad) |
|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.1216 | 0.0927 | 0.0778 | 0.1115 | 0.0973 | 0.1219 | 0.0982 | 0.0257 |
| T5 | 0.0904 | 0.0614 | 0.0657 | 0.0926 | 0.0973 | 0.0897 | 0.0668 | 0.0199 |
| TL5 seed 0 | 0.0793 | 0.0537 | 0.0576 | 0.0771 | 0.0973 | 0.0788 | 0.0636 | 0.0196 |
| TL5 seed 1 | 0.0791 | 0.0542 | 0.0570 | 0.0767 | 0.0973 | 0.0786 | 0.0649 | 0.0204 |
| TL5 seed 2 | 0.0789 | 0.0537 | 0.0570 | 0.0758 | 0.0973 | 0.0785 | 0.0640 | 0.0199 |
| N4 | 0.1108 | 0.0809 | 0.0745 | 0.0942 | 0.0776 | 0.1110 | 0.0880 | 0.0238 |
| R4 | 0.1199 | 0.0927 | 0.0749 | 0.0844 | 0.0973 | 0.1203 | 0.0783 | 0.0182 |
| T4 | 0.0935 | 0.0637 | 0.0680 | 0.1115 | 0.0973 | 0.0928 | 0.0795 | 0.0257 |
| TL4 | 0.0802 | 0.0535 | 0.0592 | 0.0835 | 0.0973 | 0.0798 | 0.0662 | 0.0208 |

## Strata, position RMSE at 1.0 s (m)

| stratum | windows | dyn-v0.1 | T5 | TL5 seed 0 | TL5 seed 1 | TL5 seed 2 | N4 | R4 | T4 | TL4 |
|---|---|---|---|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 131 | 0.0647 | 0.0502 | 0.0434 | 0.0439 | 0.0431 | 0.0688 | 0.0609 | 0.0545 | 0.0471 |
| ccw:0.6-0.85 | 89 | 0.0983 | 0.0797 | 0.0675 | 0.0668 | 0.0667 | 0.0864 | 0.0965 | 0.0792 | 0.0672 |
| ccw:0.85+ | 468 | 0.0993 | 0.0687 | 0.0632 | 0.0633 | 0.0638 | 0.0853 | 0.0933 | 0.0741 | 0.0642 |
| cw:0.25-0.6 | 101 | 0.0739 | 0.0533 | 0.0473 | 0.0466 | 0.0466 | 0.0767 | 0.0726 | 0.0604 | 0.0495 |
| cw:0.6-0.85 | 49 | 0.1123 | 0.1084 | 0.0850 | 0.0853 | 0.0837 | 0.1155 | 0.1105 | 0.1132 | 0.0847 |
| cw:0.85-1.25 | 95 | 0.1763 | 0.1310 | 0.1088 | 0.1096 | 0.1091 | 0.1410 | 0.1760 | 0.1324 | 0.1101 |
| cw:1.25+ | 73 | 0.2267 | 0.1413 | 0.1397 | 0.1384 | 0.1392 | 0.2018 | 0.2296 | 0.1404 | 0.1390 |

## Everything printed on the same windows

### Per direction (macro over the direction's strata) and yaw rate bias

| model | P1 ccw | Y1 ccw | bias ccw (rad/s) | P1 cw | Y1 cw | bias cw (rad/s) |
|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.0874 | 0.0989 | -0.0485 | 0.1473 | 0.1209 | -0.0279 |
| T5 | 0.0662 | 0.0810 | -0.0206 | 0.1085 | 0.1013 | +0.0090 |
| TL5 seed 0 | 0.0580 | 0.0690 | -0.0263 | 0.0952 | 0.0831 | +0.0054 |
| TL5 seed 1 | 0.0580 | 0.0687 | -0.0245 | 0.0949 | 0.0826 | +0.0068 |
| TL5 seed 2 | 0.0579 | 0.0669 | -0.0226 | 0.0947 | 0.0824 | +0.0052 |
| N4 | 0.0802 | 0.0816 | -0.0185 | 0.1338 | 0.1037 | +0.0109 |
| R4 | 0.0836 | 0.0627 | -0.0039 | 0.1472 | 0.1007 | +0.0026 |
| T4 | 0.0693 | 0.0989 | -0.0485 | 0.1116 | 0.1209 | -0.0279 |
| TL4 | 0.0595 | 0.0741 | -0.0301 | 0.0958 | 0.0905 | +0.0064 |

### Near-limit windows (mean |steering share| at least 0.85), position RMSE at 1.0 s (m)

| model | left (n=0) | right (n=164) |
|---|---|---|
| dyn-v0.1 | nan | 0.1493 |
| T5 | nan | 0.1030 |
| TL5 seed 0 | nan | 0.0949 |
| TL5 seed 1 | nan | 0.0943 |
| TL5 seed 2 | nan | 0.0944 |
| N4 | nan | 0.1337 |
| R4 | nan | 0.1512 |
| T4 | nan | 0.1074 |
| TL4 | nan | 0.0960 |

### Transition windows (n=226), position RMSE at 1.0 s (m)

| model | RMSE |
|---|---|
| dyn-v0.1 | 0.1838 |
| T5 | 0.1321 |
| TL5 seed 0 | 0.1128 |
| TL5 seed 1 | 0.1125 |
| TL5 seed 2 | 0.1129 |
| N4 | 0.1491 |
| R4 | 0.1819 |
| T4 | 0.1340 |
| TL4 | 0.1135 |

### Fast windows per direction, position RMSE at 1.0 s (m)

| model | ccw >= 1.15 (n=29) | cw >= 1.15 (n=96) | ccw >= 1.3 (n=7) | cw >= 1.3 (n=57) |
|---|---|---|---|---|
| dyn-v0.1 | 0.1816 | 0.2221 | 0.1770 | 0.2307 |
| T5 | 0.1340 | 0.1422 | 0.0989 | 0.1421 |
| TL5 seed 0 | 0.1116 | 0.1353 | 0.0831 | 0.1413 |
| TL5 seed 1 | 0.1117 | 0.1345 | 0.0810 | 0.1395 |
| TL5 seed 2 | 0.1130 | 0.1350 | 0.0849 | 0.1406 |
| N4 | 0.1384 | 0.1919 | 0.1237 | 0.2077 |
| R4 | 0.1759 | 0.2240 | 0.1662 | 0.2342 |
| T4 | 0.1391 | 0.1422 | 0.1254 | 0.1412 |
| TL4 | 0.1115 | 0.1349 | 0.0739 | 0.1407 |

Bootstrap of the P1 ratio (blocks of 5 s, 2000 resamples, seed 0): T5 over dyn-v0.1 0.7428 [0.6816, 0.8276]; TL5 over T5 0.8773 [0.8453, 0.9087].

### GP2f-S the tables may be trusted: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1 supported nodes within 0.005 of a bound (alpha and gamma) | 0 | <= 0 | PASS | none |
| 2a split half: largest alpha difference at a supported node | 0.0316512 | <= 0.05 | PASS |  |
| 2b split half: largest gamma difference at a supported node | 0.0662605 | <= 0.08 | PASS |  |
| 3 share of held-out rollout steps in the cell of a supported node | 0.908482 | >= 0.95 | FAIL |  |

### GP2f-T T5 is a candidate: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0 GP2f-S passed | 0 | >= 1 | FAIL |  |
| 1a P1 ratio T5 / dyn-v0.1 | 0.742838 | <= 0.9 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.827618 | < 0.95 | PASS |  |
| 2a A1 ratio | 0.662265 | <= 1 | PASS |  |
| 2b C1 ratio | 0.844292 | <= 1 | PASS |  |
| 3a Y1 ratio | 0.830531 | <= 0.9 | PASS |  |
| 3b S1(T5) minus S1(dyn-v0.1) | 0 | <= 1e-06 | PASS | absolute m/s |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.77571 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.810501 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.691541 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.721243 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.965244 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85-1.25 position RMSE ratio | 0.743053 | <= 1.05 | PASS |  |
| 4 stratum cw:1.25+ position RMSE ratio | 0.623262 | <= 1.05 | PASS |  |
| 5 ccw P1 ratio | 0.756885 | <= 0.95 | PASS |  |
| 5 ccw Y1 ratio | 0.818689 | <= 1 | PASS |  |
| 5 cw P1 ratio | 0.736585 | <= 0.95 | PASS |  |
| 5 cw Y1 ratio | 0.837798 | <= 1 | PASS |  |
| 6 ccw absolute yaw rate bias (rad/s) | 0.0206483 | <= 0.03 | PASS | at most 0.03 rad/s or 0.5 of the dyn-v0.1 bias -0.0485 rad/s |
| 6 cw absolute yaw rate bias (rad/s) | 0.008956 | <= 0.03 | PASS | at most 0.03 rad/s or 0.5 of the dyn-v0.1 bias -0.0279 rad/s |
| 7 near-limit left position RMSE ratio | 0 | at least 30 | CANNOT BE APPLIED | cannot be applied: 0 windows, at least 30 needed |
| 7 near-limit right position RMSE ratio (164 windows) | 0.689369 | <= 1 | PASS |  |
| 8 transition position RMSE ratio (226 windows) | 0.719066 | <= 1 | PASS |  |
| 9 P1 ratio under T2s | 0.735399 | <= 1 | PASS |  |
| 10a Pp ratio on G1b and T2b | 0.679817 | <= 1 | PASS |  |
| 10b Yp ratio on G1b and T2b | 0.777142 | <= 1 | PASS |  |

### GP2f-L TL5 replaces T5 (not evaluated: GP2f-T failed): not evaluated

GP2f outcome: **the one-speed contract stays**.

## Validity above 1.3 m/s (GP2f-V)

Top supported speed: 1.3.

T5: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at 1.3.

- 1 the bundle's model passed its gate: 0 >= 1 FAIL
- 2 ccw: windows with start speed >= 1.15 m/s: 29 >= 30 FAIL the fast item cannot be applied
- 2 cw start speed >= 1.15 m/s: P1 ratio to dyn-v0.1 (96 windows): 0.640322 <= 1 PASS
- 3 ccw: windows with start speed >= 1.3 m/s: 7 >= 30 FAIL the fast item cannot be applied
- 3 cw start speed >= 1.3 m/s: P1 ratio to dyn-v0.1 (57 windows): 0.615998 <= 1 PASS
- 4 a top supported speed exists: 1.3 >= 0 PASS highest speed node with a supported turning node on each side in both tables

TL5: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at 1.3.

- 1 the bundle's model passed its gate: 0 >= 1 FAIL
- 2 ccw: windows with start speed >= 1.15 m/s: 29 >= 30 FAIL the fast item cannot be applied
- 2 cw start speed >= 1.15 m/s: P1 ratio to dyn-v0.1 (96 windows): 0.609152 <= 1 PASS
- 3 ccw: windows with start speed >= 1.3 m/s: 7 >= 30 FAIL the fast item cannot be applied
- 3 cw start speed >= 1.3 m/s: P1 ratio to dyn-v0.1 (57 windows): 0.612519 <= 1 PASS
- 4 a top supported speed exists: 1.3 >= 0 PASS highest speed node with a supported turning node on each side in both tables

## Reported only: the v0.2 split held-out segments (read before, decide nothing)

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Pp (m) | Yp (rad) |
|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.0887 | 0.0690 | 0.0540 | 0.0739 | 0.0387 | nan | nan | nan |
| T5 | 0.0529 | 0.0371 | 0.0360 | 0.0450 | 0.0387 | nan | nan | nan |
| TL5 seed 0 | 0.0492 | 0.0335 | 0.0343 | 0.0420 | 0.0387 | nan | nan | nan |
| TL5 seed 1 | 0.0496 | 0.0336 | 0.0346 | 0.0424 | 0.0387 | nan | nan | nan |
| TL5 seed 2 | 0.0491 | 0.0330 | 0.0347 | 0.0424 | 0.0387 | nan | nan | nan |
| N4 | 0.0869 | 0.0718 | 0.0478 | 0.0506 | 0.0387 | nan | nan | nan |
| R4 | 0.0858 | 0.0706 | 0.0469 | 0.0421 | 0.0387 | nan | nan | nan |
| T4 | 0.0557 | 0.0377 | 0.0397 | 0.0739 | 0.0387 | nan | nan | nan |
| TL4 | 0.0495 | 0.0328 | 0.0353 | 0.0434 | 0.0387 | nan | nan | nan |

T2s, Pp and Yp are not computed for these segments.

