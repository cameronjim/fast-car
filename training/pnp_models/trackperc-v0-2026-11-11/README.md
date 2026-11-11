# trackperc-v0-2026-11-11

Track perception net (`circ_conv1d_v0`), trained in simulation only: labels come from the
track geometry and the real bags were never trained on. The weights are not committed;
the bundle lives in `/Users/cameronjim/code/car-data/pnp/models/` on the dev machine and
is copied to the car.

Real-bag gate verdict: **net fails, use backend:=geometric tonight**.

Real-bag gate (both 2026-11-11T03-54-11 bags together, moving scans). The net may drive
only if the median boundary residual is at most 0.10 m, the mean intrusion fraction at
most 0.02 and at most 5 percent of scans have valid_count under 6. The thresholds were
not tuned.

| backend | median residual (m) | p90 residual (m) | mean intrusion | share valid < 6 |
|---|---|---|---|---|
| geometric | 0.057 | 0.160 | 0.092 | 0.006 |
| net | 0.107 | 0.167 | 0.205 | 0.000 |

Simulation, held-out family tracks (net, median error over stations): centreline
0 to 7 / 8 to 15 and half width, in metres.

| track | centreline 0-7 | centreline 8-15 | half width |
|---|---|---|---|
| stadium_l1.0_s2.0_r0.4 | 0.010 | 0.039 | 0.010 |
| rounded_rect_l1.0_3.0x2.0_r0.4 | 0.009 | 0.033 | 0.009 |
| rounded_rect_l1.2_2.0x1.6_r0.8 | 0.008 | 0.018 | 0.009 |

Everything, as stored in the bundle manifest (`metrics.json` is the same):

