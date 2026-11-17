# B4 reading, 2026-11: D3-S0 remains FAIL

This note re-derives, from B4's stored result files, the evidence of brief 48. It edits and re-scores nothing: B4's decisions (D3-S0 FAIL, D3-S1 PASS, D3-S2(d) FAIL, D3-S2(e) PASS) stand, and B4's cases and seeds are development data from now on. Produced by `tests/l5_pnp_laps/b4_reading.py`.

## Lap 1 of the four runs 2.3 s off (S0b(d) and S0b(e))

| run | system lap 1 s | tool's truth s | truth by the registered origin s | first truth odometry after DRIVING s | lap-1 wheel length m (median of case and direction) |
|---|---|---|---|---|---|
| d_N3_cw_23 | 32.00 | 29.65 | 31.94 | +2.36 | 12.43 (11.92) |
| e_N1_cw_23 | 32.75 | 30.38 | 32.67 | +2.30 | 14.73 (13.80) |
| e_N2_cw_23 | 35.88 | 33.52 | 35.81 | +2.36 | 13.37 (12.94) |
| e_N3_ccw_22 | 32.30 | 30.07 | 32.26 | +2.29 | 12.60 (12.11) |

The registered origin is the stamp of the first /scan after the first DRIVING report: the system's own lap 1 starts there (first LapEvent crossing minus lap time minus half the 0.1 s sweep). In these four runs the scorer's truth recording began 2.3 s after it (the startup reset finished late), so the tool's lap 1 was 2.3 s short; by the registered origin truth and system agree to within 0.08 s.

## Lap 3 of the three condition c runs (S0b(c))

System crossing stamp minus ground-truth crossing stamp, crossings 1 to 6 (s), and the system's wheel length of laps 1 to 6 (m):

| run | offsets s | lap lengths m | crossing 3 | median of the others |
|---|---|---|---|---|
| c_N1_ccw_21 | +0.087 +0.069 -0.418 +0.073 +0.073 +0.075 | 13.88 13.98 13.83 14.41 14.08 14.07 | -0.418 | +0.073 |
| c_N1_ccw_22 | +0.129 +0.097 -0.325 +0.073 +0.100 +0.063 | 13.89 13.98 13.89 14.34 14.09 14.04 | -0.325 | +0.097 |
| c_N2_cw_22 | +0.110 +0.093 -0.458 +0.104 +0.087 +0.092 | 12.93 12.93 12.64 13.24 12.94 12.96 | -0.458 | +0.093 |
