# Lap 3 of three B4 condition c runs (brief 48 item 5): not reproduced, cause outside h1 to h3

Development runs, labelled so (B4's cases and seeds are development data from brief 48 on). Brief 48 item 5 stop rule: the cause is none of h1 to h3, so the fix of the brief is not made, nothing in `gate.py` changed for this item, and B7 did not run (the launcher decides).

## What was run

The B4 command of condition c (D v0) for `c_N1_ccw_21`, `c_N1_ccw_22`, `c_N2_cw_22` and the good control `c_N1_ccw_23`, on the fixed harness, `--stop-after-laps 5`, with the lap node's input dump (`RUNNER_LAP_DUMP_DIR`), 2 to 3 sims at a time in the ros-dev container, wall time stepping as in B4. Then the three runs again three more times (`__r2` to `__r4`): 13 development runs. Each dump was replayed offline through the engine and the gate (`tests/l5_pnp_laps/lap3_diagnosis.py table`), with the gate of this PR and with the gate of main before it (both close every lap at the same place on these inputs).

## Result: no early crossing in 13 runs

System lap time minus registered truth lap time per lap, and system crossing stamp minus truth crossing stamp per crossing. B4's three events were crossing 3 at -0.418, -0.325 and -0.458 s with every other crossing at +0.06 to +0.13 s.

| run | system - truth lap s, laps 1 to 5 | crossing offsets s, 1 to 5 | max /drive_raw gap s |
|---|---|---|---|
| c_N1_ccw_21 | +0.04 -0.01 -0.01 -0.01 +0.02 | +0.093 +0.085 +0.072 +0.067 +0.086 | 0.0249 |
| c_N1_ccw_21__r2 | -0.24 -0.42 -0.28 -0.19 | +2.505 +2.088 +1.805 +1.613 | 0.027 |
| c_N1_ccw_21__r3 | +0.05 +0.00 -0.03 -0.00 -0.01 | +0.103 +0.106 +0.081 +0.076 +0.064 | 0.0697 |
| c_N1_ccw_21__r4 | +0.04 -0.00 -0.03 +0.03 -0.01 | +0.090 +0.086 +0.058 +0.085 +0.073 | 0.0245 |
| c_N1_ccw_22 | +0.07 -0.03 +0.01 -0.03 +0.01 | +0.115 +0.084 +0.091 +0.062 +0.073 | 0.0251 |
| c_N1_ccw_22__r2 | +0.06 -0.01 -0.00 -0.02 -0.01 | +0.108 +0.098 +0.097 +0.074 +0.069 | 0.0242 |
| c_N1_ccw_22__r3 | +0.07 -0.02 -0.01 -0.02 +0.02 | +0.117 +0.099 +0.089 +0.065 +0.080 | 0.0827 |
| c_N1_ccw_22__r4 | +0.05 -0.01 +0.01 -0.01 -0.01 | +0.105 +0.089 +0.095 +0.084 +0.076 | 0.0283 |
| c_N1_ccw_23 | +0.04 -0.01 +0.01 -0.03 -0.00 | +0.087 +0.079 +0.090 +0.063 +0.059 | 0.0234 |
| c_N2_cw_22 | +0.04 -0.00 -0.01 -0.01 +0.02 | +0.089 +0.087 +0.080 +0.066 +0.086 | 0.0245 |
| c_N2_cw_22__r2 | +0.04 +0.03 -0.02 -0.01 -0.01 | +0.095 +0.124 +0.103 +0.094 +0.088 | 0.0234 |
| c_N2_cw_22__r3 | +0.07 -0.03 +0.01 -0.03 -0.00 | +0.124 +0.098 +0.110 +0.080 +0.079 | 0.0806 |
| c_N2_cw_22__r4 | +0.07 -0.03 +0.00 -0.00 +0.01 | +0.119 +0.089 +0.093 +0.091 +0.099 | 0.0316 |

`c_N1_ccw_21__r2` is not a gate fault: its lap node started 2.7 s after the planner reported DRIVING (the first runner of this PR started the planner and the lap node together, and a host under load imported the node's modules slowly), so the system's start line was 2.7 s of driving past the truth's and every lap differs by the speed profile. The runner now starts the planner only once the lap node answers; the later runs (`__r3`, `__r4`) used it.

## h1 to h3 on these inputs

- h1 (the `past` rule closes on far candidates before near ones arrive): the structure is there but with the other sign. A candidate's line estimate is late by about its distance from the line (+0.55 to +0.60 m at x = -0.55 to -0.60, +0.04 to +0.06 m from x = 0), because the match along a straight lane lags until the lane end is in view. The median of the surviving candidates (the 1.5 s TTL drops the far ones) puts the crossing +0.04 to +0.07 m late, the +0.06 to +0.13 s every normal lap shows. It cannot place a crossing 0.3 m early.
- h2 (a slid match carried by the seed chain): 184 of the 189 candidates of laps 2 to 4 won with the previous result advanced by odometry as the seed (first of two, always fresh), 5 with the identity seed; the spread of the candidates per lap is 0.02 to 0.08 m and no lap shows a persistent offset.
- h3 (the first lap with a trusted length takes another path): lap 3 of these runs is identical in structure to laps 2 and 4 (12 to 16 candidates, spread 0.02 to 0.08 m, crossing +0.04 to +0.07 m).

## What reproduces B4's signature: the wheel odometry arriving late

`StartScanGate._at` extrapolates a scan's pose at most `MAX_EXTRAPOLATION_S` = 0.2 s past the last wheel odometry sample. If the node serves a scan before the odometry queued behind it (a starved host: B4 ran 5 sims at once and 44 of 90 first attempts were starved), the wheel total at the scan is under-read by the speed times the lag beyond 0.2 s, every candidate line is early by that much, and so is the crossing. Replaying the same dump of `c_N1_ccw_21` with the odometry delivered later than it was stamped:

| odometry lag s | crossing - truth m, laps 1 to 5 | crossing - truth s, laps 1 to 5 |
|---|---|---|
| 0.0 | +0.043 +0.046 +0.046 +0.048 +0.069 | +0.093 +0.085 +0.072 +0.067 +0.088 |
| 0.25 | +0.013 +0.018 +0.008 +0.008 +0.017 | +0.029 +0.033 +0.012 +0.011 +0.022 |
| 0.45 | -0.077 -0.094 -0.116 -0.134 -0.133 | -0.169 -0.171 -0.182 -0.186 -0.170 |
| 0.7 | -0.134 -0.195 -0.182 -0.161 -0.184 | -0.293 -0.356 -0.350 -0.383 -0.372 |

A lag of 0.7 s moves every crossing by -0.36 to -0.38 s against +0.07 s without it: the -0.33 to -0.46 s of B4's crossing 3 (and a short lap 3 followed by a long lap 4 by the same amount). A lag that lasted only while the closure of lap 3 was decided makes the one-lap event. Nothing in B4 recorded the node's odometry latency, so this is a reproduction of the signature, not a measurement of the cause: B4's three runs do not stand out by their `/drive_raw` gaps (0.032 to 0.037 s, ranks 2, 4 and 8 of its 18 condition c runs). The 13 development runs here, at 3 sims at a time, have `/drive_raw` gaps of 0.023 to 0.083 s (three runs, all `__r3`, over the 0.040 s starvation limit, and none of them with an early crossing) and no early crossing.

This is a cause outside h1 to h3 and it is a robustness fault of the gate on a loaded host, not of the lap logic. The stop rule applies: the allowed fix (near-line placement, `consensus_near_m`) answers h1 and is not made. Two ways forward for the launcher: a defensive rule in `gate.py` that refuses to place a candidate on a scan whose odometry is staler than `MAX_EXTRAPOLATION_S` (about five lines, an unregistered rule that does not move a threshold), or the keyframe ring of 15.2. Neither was built.

## Tables (gate of this PR, laps 2 to 4, run by run)

## c_N1_ccw_21

| lap | t s | x_m | line - truth m | inlier | rms | violation | seed | station m |
|---|---|---|---|---|---|---|---|---|
| 2 | 55.50 | -0.57 | +0.571 | 0.90 | 0.023 | 0.000 | 1/2 fresh | 13.41 |
| 2 | 55.60 | -0.52 | +0.525 | 0.93 | 0.022 | 0.000 | 1/2 fresh | 13.46 |
| 2 | 55.70 | -0.45 | +0.460 | 0.90 | 0.022 | 0.000 | 1/2 fresh | 13.52 |
| 2 | 55.80 | -0.39 | +0.398 | 0.92 | 0.022 | 0.000 | 1/2 fresh | 13.57 |
| 2 | 55.90 | -0.36 | +0.360 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.63 |
| 2 | 56.00 | -0.28 | +0.284 | 0.94 | 0.022 | 0.000 | 1/2 fresh | 13.68 |
| 2 | 56.10 | -0.24 | +0.249 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.74 |
| 2 | 56.20 | -0.19 | +0.193 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.79 |
| 2 | 56.30 | -0.12 | +0.130 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 13.85 |
| 2 | 56.40 | -0.07 | +0.080 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.90 |
| 2 | 56.50 | -0.01 | +0.041 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 13.96 |
| 2 | 56.60 | +0.02 | +0.064 | 0.96 | 0.020 | 0.000 | 2/2 fresh | 14.01 |
| 2 | 56.70 | +0.10 | +0.041 | 0.94 | 0.020 | 0.000 | 1/2 fresh | 14.07 |
| 2 | 56.80 | +0.16 | +0.031 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.12 |
| 2 | 56.90 | +0.21 | +0.037 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.18 |
| 2 | 57.00 | +0.25 | +0.054 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 14.23 |
| 2 | 57.10 | +0.30 | +0.056 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.28 |
| 3 | 77.90 | -0.59 | +0.594 | 0.90 | 0.022 | 0.000 | 1/2 fresh | 13.44 |
| 3 | 78.00 | -0.54 | +0.543 | 0.91 | 0.022 | 0.000 | 1/2 fresh | 13.51 |
| 3 | 78.10 | -0.48 | +0.488 | 0.92 | 0.023 | 0.000 | 1/2 fresh | 13.57 |
| 3 | 78.20 | -0.39 | +0.401 | 0.93 | 0.020 | 0.000 | 1/2 fresh | 13.63 |
| 3 | 78.30 | -0.33 | +0.336 | 0.91 | 0.020 | 0.000 | 1/2 fresh | 13.70 |
| 3 | 78.40 | -0.27 | +0.274 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.76 |
| 3 | 78.50 | -0.20 | +0.208 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.82 |
| 3 | 78.60 | -0.15 | +0.155 | 0.94 | 0.020 | 0.000 | 1/2 fresh | 13.89 |
| 3 | 78.70 | -0.08 | +0.083 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.95 |
| 3 | 78.80 | -0.00 | +0.034 | 0.96 | 0.021 | 0.000 | 2/2 fresh | 14.02 |
| 3 | 78.90 | +0.05 | +0.042 | 0.97 | 0.019 | 0.000 | 1/2 fresh | 14.08 |
| 3 | 79.00 | +0.10 | +0.058 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 14.14 |
| 3 | 79.10 | +0.16 | +0.060 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.21 |
| 3 | 79.20 | +0.22 | +0.061 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 14.27 |
| 3 | 79.30 | +0.30 | +0.051 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.33 |
| 4 | 97.80 | -0.59 | +0.596 | 0.88 | 0.024 | 0.000 | 1/2 fresh | 13.47 |
| 4 | 97.90 | -0.52 | +0.522 | 0.92 | 0.022 | 0.000 | 1/2 fresh | 13.54 |
| 4 | 98.00 | -0.44 | +0.445 | 0.93 | 0.022 | 0.000 | 1/2 fresh | 13.61 |
| 4 | 98.10 | -0.36 | +0.366 | 0.91 | 0.022 | 0.000 | 1/2 fresh | 13.68 |
| 4 | 98.20 | -0.29 | +0.299 | 0.92 | 0.022 | 0.000 | 1/2 fresh | 13.75 |
| 4 | 98.30 | -0.23 | +0.237 | 0.92 | 0.021 | 0.000 | 1/2 fresh | 13.83 |
| 4 | 98.40 | -0.16 | +0.168 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.90 |
| 4 | 98.50 | -0.09 | +0.101 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.97 |
| 4 | 98.60 | -0.01 | +0.051 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 14.04 |
| 4 | 98.70 | +0.07 | +0.040 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.11 |
| 4 | 98.80 | +0.13 | +0.046 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.18 |
| 4 | 98.90 | +0.21 | +0.038 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 14.26 |
| 4 | 99.00 | +0.28 | +0.042 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.33 |
| 4 | 99.10 | +0.33 | +0.065 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.40 |

| lap | closed at station m | length m | crossing - truth m | crossing - truth s | system - truth lap s | candidates | spread m | missed |
|---|---|---|---|---|---|---|---|---|
| 1 | 14.19 | 13.89 | +0.043 | +0.093 | +0.043 | 15 | 0.04 | False |
| 2 | 14.28 | 13.98 | +0.046 | +0.085 | -0.008 | 15 | 0.03 | False |
| 3 | 14.33 | 14.04 | +0.046 | +0.072 | -0.013 | 15 | 0.03 | False |
| 4 | 14.40 | 14.06 | +0.048 | +0.067 | -0.005 | 14 | 0.03 | False |
| 5 | 14.39 | 14.09 | +0.069 | +0.088 | +0.021 | 12 | 0.04 | False |

lap 2: 17 candidates, line - truth median +0.130 m, min +0.031, max +0.571
lap 3: 15 candidates, line - truth median +0.155 m, min +0.034, max +0.594
lap 4: 14 candidates, line - truth median +0.135 m, min +0.038, max +0.596

## c_N1_ccw_22

| lap | t s | x_m | line - truth m | inlier | rms | violation | seed | station m |
|---|---|---|---|---|---|---|---|---|
| 2 | 55.50 | -0.55 | +0.552 | 0.92 | 0.024 | 0.000 | 1/2 fresh | 13.39 |
| 2 | 55.60 | -0.50 | +0.505 | 0.95 | 0.023 | 0.000 | 1/2 fresh | 13.45 |
| 2 | 55.70 | -0.44 | +0.450 | 0.93 | 0.023 | 0.000 | 1/2 fresh | 13.50 |
| 2 | 55.80 | -0.40 | +0.402 | 0.91 | 0.022 | 0.000 | 1/2 fresh | 13.56 |
| 2 | 55.90 | -0.34 | +0.348 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.61 |
| 2 | 56.00 | -0.26 | +0.268 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.66 |
| 2 | 56.10 | -0.20 | +0.209 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.72 |
| 2 | 56.20 | -0.17 | +0.178 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.77 |
| 2 | 56.30 | -0.10 | +0.110 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.83 |
| 2 | 56.40 | -0.06 | +0.069 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.88 |
| 2 | 56.50 | -0.01 | +0.044 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 13.94 |
| 2 | 56.60 | +0.03 | +0.066 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.99 |
| 2 | 56.70 | +0.09 | +0.054 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.05 |
| 2 | 56.80 | +0.16 | +0.044 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.10 |
| 2 | 56.90 | +0.20 | +0.059 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.16 |
| 2 | 57.00 | +0.27 | +0.045 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 14.21 |
| 2 | 57.10 | +0.32 | +0.048 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.27 |
| 3 | 78.00 | -0.53 | +0.541 | 0.92 | 0.023 | 0.000 | 1/2 fresh | 13.51 |
| 3 | 78.10 | -0.47 | +0.477 | 0.93 | 0.023 | 0.000 | 1/2 fresh | 13.57 |
| 3 | 78.20 | -0.41 | +0.415 | 0.93 | 0.023 | 0.000 | 1/2 fresh | 13.63 |
| 3 | 78.30 | -0.36 | +0.362 | 0.94 | 0.022 | 0.000 | 1/2 fresh | 13.70 |
| 3 | 78.40 | -0.29 | +0.297 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 13.76 |
| 3 | 78.50 | -0.24 | +0.247 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.83 |
| 3 | 78.60 | -0.17 | +0.172 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.89 |
| 3 | 78.70 | -0.10 | +0.103 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.95 |
| 3 | 78.80 | -0.01 | +0.037 | 0.96 | 0.021 | 0.000 | 2/2 fresh | 14.02 |
| 3 | 78.90 | +0.04 | +0.046 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.08 |
| 3 | 79.00 | +0.10 | +0.057 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 14.14 |
| 3 | 79.10 | +0.16 | +0.056 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.21 |
| 3 | 79.20 | +0.21 | +0.068 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 14.27 |
| 3 | 79.30 | +0.27 | +0.072 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 14.33 |
| 3 | 79.40 | +0.34 | +0.066 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 14.40 |
| 4 | 97.90 | -0.53 | +0.540 | 0.93 | 0.023 | 0.000 | 1/2 fresh | 13.51 |
| 4 | 98.00 | -0.46 | +0.468 | 0.94 | 0.023 | 0.000 | 1/2 fresh | 13.58 |
| 4 | 98.10 | -0.39 | +0.402 | 0.93 | 0.022 | 0.000 | 1/2 fresh | 13.65 |
| 4 | 98.20 | -0.33 | +0.338 | 0.95 | 0.023 | 0.000 | 1/2 fresh | 13.73 |
| 4 | 98.30 | -0.25 | +0.257 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.80 |
| 4 | 98.40 | -0.17 | +0.180 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.87 |
| 4 | 98.50 | -0.11 | +0.115 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.94 |
| 4 | 98.60 | -0.02 | +0.034 | 0.97 | 0.022 | 0.000 | 2/2 fresh | 14.01 |
| 4 | 98.70 | +0.04 | +0.040 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.08 |
| 4 | 98.80 | +0.12 | +0.033 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.16 |
| 4 | 98.90 | +0.19 | +0.039 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.23 |
| 4 | 99.00 | +0.24 | +0.060 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.30 |
| 4 | 99.10 | +0.31 | +0.057 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.37 |

| lap | closed at station m | length m | crossing - truth m | crossing - truth s | system - truth lap s | candidates | spread m | missed |
|---|---|---|---|---|---|---|---|---|
| 1 | 14.22 | 13.89 | +0.053 | +0.115 | +0.066 | 15 | 0.06 | False |
| 2 | 14.27 | 13.95 | +0.046 | +0.084 | -0.031 | 15 | 0.04 | False |
| 3 | 14.40 | 14.06 | +0.058 | +0.091 | +0.007 | 15 | 0.04 | False |
| 4 | 14.37 | 14.05 | +0.045 | +0.062 | -0.029 | 13 | 0.03 | False |
| 5 | 14.37 | 14.08 | +0.057 | +0.073 | +0.011 | 12 | 0.04 | False |

lap 2: 17 candidates, line - truth median +0.110 m, min +0.044, max +0.552
lap 3: 15 candidates, line - truth median +0.103 m, min +0.037, max +0.541
lap 4: 13 candidates, line - truth median +0.115 m, min +0.033, max +0.540

## c_N2_cw_22

| lap | t s | x_m | line - truth m | inlier | rms | violation | seed | station m |
|---|---|---|---|---|---|---|---|---|
| 2 | 55.30 | -0.55 | +0.555 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 12.36 |
| 2 | 55.40 | -0.52 | +0.522 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 12.41 |
| 2 | 55.50 | -0.47 | +0.474 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 12.46 |
| 2 | 55.60 | -0.42 | +0.424 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 12.51 |
| 2 | 55.70 | -0.37 | +0.373 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 12.56 |
| 2 | 55.80 | -0.32 | +0.323 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 12.61 |
| 2 | 55.90 | -0.27 | +0.270 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 12.66 |
| 2 | 56.00 | -0.22 | +0.221 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 12.71 |
| 2 | 56.10 | -0.16 | +0.167 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 12.76 |
| 2 | 56.20 | -0.11 | +0.118 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 12.81 |
| 2 | 56.30 | -0.07 | +0.072 | 0.98 | 0.020 | 0.000 | 1/2 fresh | 12.86 |
| 2 | 56.40 | -0.02 | +0.046 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 12.91 |
| 2 | 56.50 | +0.04 | +0.045 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 12.97 |
| 2 | 56.60 | +0.09 | +0.042 | 0.96 | 0.019 | 0.000 | 1/2 fresh | 13.02 |
| 2 | 56.70 | +0.13 | +0.053 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 13.07 |
| 2 | 56.80 | +0.17 | +0.060 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 13.12 |
| 2 | 56.90 | +0.24 | +0.044 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 13.17 |
| 2 | 57.00 | +0.29 | +0.044 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.22 |
| 2 | 57.10 | +0.35 | +0.035 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.27 |
| 3 | 79.70 | -0.58 | +0.590 | 0.94 | 0.023 | 0.000 | 1/2 fresh | 12.34 |
| 3 | 79.80 | -0.53 | +0.539 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 12.40 |
| 3 | 79.90 | -0.48 | +0.483 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 12.46 |
| 3 | 80.00 | -0.41 | +0.420 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 12.51 |
| 3 | 80.10 | -0.36 | +0.365 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 12.57 |
| 3 | 80.20 | -0.31 | +0.312 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 12.62 |
| 3 | 80.30 | -0.25 | +0.260 | 0.98 | 0.021 | 0.000 | 1/2 fresh | 12.68 |
| 3 | 80.40 | -0.20 | +0.207 | 0.97 | 0.019 | 0.000 | 1/2 fresh | 12.74 |
| 3 | 80.50 | -0.15 | +0.160 | 0.98 | 0.019 | 0.000 | 1/2 fresh | 12.79 |
| 3 | 80.60 | -0.10 | +0.105 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 12.85 |
| 3 | 80.70 | -0.04 | +0.060 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 12.91 |
| 3 | 80.80 | +0.03 | +0.047 | 0.98 | 0.020 | 0.000 | 1/2 fresh | 12.96 |
| 3 | 80.90 | +0.10 | +0.025 | 0.96 | 0.019 | 0.000 | 1/2 fresh | 13.02 |
| 3 | 81.00 | +0.16 | +0.027 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 13.07 |
| 3 | 81.10 | +0.20 | +0.043 | 0.96 | 0.019 | 0.000 | 1/2 fresh | 13.13 |
| 3 | 81.20 | +0.24 | +0.057 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 13.19 |
| 3 | 81.30 | +0.31 | +0.046 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.24 |
| 4 | 104.00 | -0.58 | +0.585 | 0.92 | 0.022 | 0.000 | 1/2 fresh | 12.35 |
| 4 | 104.10 | -0.52 | +0.525 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 12.41 |
| 4 | 104.20 | -0.46 | +0.468 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 12.47 |
| 4 | 104.30 | -0.39 | +0.394 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 12.52 |
| 4 | 104.40 | -0.32 | +0.323 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 12.58 |
| 4 | 104.50 | -0.26 | +0.265 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 12.64 |
| 4 | 104.60 | -0.21 | +0.211 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 12.70 |
| 4 | 104.70 | -0.16 | +0.163 | 0.96 | 0.018 | 0.000 | 1/2 fresh | 12.76 |
| 4 | 104.80 | -0.11 | +0.112 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 12.81 |
| 4 | 104.90 | -0.05 | +0.059 | 0.98 | 0.020 | 0.000 | 1/2 fresh | 12.87 |
| 4 | 105.00 | -0.00 | +0.042 | 0.97 | 0.020 | 0.000 | 1/2 fresh | 12.93 |
| 4 | 105.10 | +0.06 | +0.038 | 0.96 | 0.020 | 0.000 | 1/2 fresh | 12.99 |
| 4 | 105.20 | +0.12 | +0.040 | 0.97 | 0.019 | 0.000 | 1/2 fresh | 13.04 |
| 4 | 105.30 | +0.15 | +0.064 | 0.98 | 0.020 | 0.000 | 1/2 fresh | 13.10 |
| 4 | 105.40 | +0.21 | +0.061 | 0.96 | 0.019 | 0.000 | 1/2 fresh | 13.16 |
| 4 | 105.50 | +0.27 | +0.062 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.21 |
| 4 | 105.60 | +0.32 | +0.061 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.27 |

| lap | closed at station m | length m | crossing - truth m | crossing - truth s | system - truth lap s | candidates | spread m | missed |
|---|---|---|---|---|---|---|---|---|
| 1 | 13.22 | 12.93 | +0.038 | +0.086 | +0.037 | 16 | 0.03 | False |
| 2 | 13.27 | 12.93 | +0.044 | +0.086 | +0.000 | 16 | 0.03 | False |
| 3 | 13.24 | 12.94 | +0.045 | +0.080 | -0.007 | 15 | 0.03 | False |
| 4 | 13.27 | 12.93 | +0.038 | +0.066 | -0.014 | 15 | 0.05 | False |
| 5 | 13.27 | 12.95 | +0.048 | +0.087 | +0.022 | 15 | 0.02 | False |

lap 2: 19 candidates, line - truth median +0.118 m, min +0.035, max +0.555
lap 3: 17 candidates, line - truth median +0.160 m, min +0.025, max +0.590
lap 4: 17 candidates, line - truth median +0.112 m, min +0.038, max +0.585

## c_N1_ccw_23

| lap | t s | x_m | line - truth m | inlier | rms | violation | seed | station m |
|---|---|---|---|---|---|---|---|---|
| 2 | 55.50 | -0.57 | +0.571 | 0.93 | 0.023 | 0.000 | 1/2 fresh | 13.41 |
| 2 | 55.60 | -0.51 | +0.519 | 0.92 | 0.023 | 0.000 | 1/2 fresh | 13.47 |
| 2 | 55.70 | -0.46 | +0.468 | 0.94 | 0.023 | 0.000 | 1/2 fresh | 13.52 |
| 2 | 55.80 | -0.40 | +0.405 | 0.92 | 0.022 | 0.000 | 1/2 fresh | 13.58 |
| 2 | 55.90 | -0.35 | +0.351 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.63 |
| 2 | 56.00 | -0.30 | +0.305 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.69 |
| 2 | 56.10 | -0.23 | +0.235 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.74 |
| 2 | 56.20 | -0.17 | +0.174 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 13.80 |
| 2 | 56.30 | -0.12 | +0.128 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.85 |
| 2 | 56.40 | -0.08 | +0.084 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.91 |
| 2 | 56.50 | +0.00 | +0.031 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.96 |
| 2 | 56.60 | +0.06 | +0.031 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.02 |
| 2 | 56.70 | +0.10 | +0.043 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 14.07 |
| 2 | 56.80 | +0.15 | +0.045 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.13 |
| 2 | 56.90 | +0.21 | +0.039 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 14.18 |
| 2 | 57.00 | +0.27 | +0.038 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.24 |
| 2 | 57.10 | +0.33 | +0.037 | 0.96 | 0.023 | 0.000 | 1/2 fresh | 14.29 |
| 3 | 78.00 | -0.57 | +0.573 | 0.92 | 0.021 | 0.000 | 1/2 fresh | 13.51 |
| 3 | 78.10 | -0.49 | +0.494 | 0.93 | 0.022 | 0.000 | 1/2 fresh | 13.57 |
| 3 | 78.20 | -0.42 | +0.424 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.64 |
| 3 | 78.30 | -0.35 | +0.355 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 13.70 |
| 3 | 78.40 | -0.28 | +0.287 | 0.95 | 0.020 | 0.000 | 1/2 fresh | 13.76 |
| 3 | 78.50 | -0.22 | +0.223 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.83 |
| 3 | 78.60 | -0.15 | +0.154 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.89 |
| 3 | 78.70 | -0.09 | +0.091 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.95 |
| 3 | 78.80 | -0.00 | +0.029 | 0.97 | 0.021 | 0.000 | 2/2 fresh | 14.02 |
| 3 | 78.90 | +0.05 | +0.037 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 14.08 |
| 3 | 79.00 | +0.10 | +0.056 | 0.94 | 0.021 | 0.000 | 1/2 fresh | 14.14 |
| 3 | 79.10 | +0.15 | +0.062 | 0.97 | 0.022 | 0.000 | 1/2 fresh | 14.21 |
| 3 | 79.20 | +0.22 | +0.060 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 14.27 |
| 3 | 79.30 | +0.28 | +0.061 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.33 |
| 3 | 79.40 | +0.34 | +0.071 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.40 |
| 4 | 97.90 | -0.53 | +0.543 | 0.92 | 0.023 | 0.000 | 1/2 fresh | 13.51 |
| 4 | 98.00 | -0.46 | +0.462 | 0.94 | 0.023 | 0.000 | 1/2 fresh | 13.59 |
| 4 | 98.10 | -0.41 | +0.418 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.66 |
| 4 | 98.20 | -0.33 | +0.333 | 0.94 | 0.022 | 0.000 | 1/2 fresh | 13.73 |
| 4 | 98.30 | -0.26 | +0.263 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 13.80 |
| 4 | 98.40 | -0.18 | +0.184 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 13.87 |
| 4 | 98.50 | -0.10 | +0.109 | 0.96 | 0.021 | 0.000 | 1/2 fresh | 13.94 |
| 4 | 98.60 | -0.04 | +0.044 | 0.95 | 0.022 | 0.000 | 1/2 fresh | 14.01 |
| 4 | 98.70 | +0.03 | +0.044 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 14.08 |
| 4 | 98.80 | +0.12 | +0.029 | 0.97 | 0.021 | 0.000 | 1/2 fresh | 14.15 |
| 4 | 98.90 | +0.16 | +0.057 | 0.96 | 0.022 | 0.000 | 1/2 fresh | 14.23 |
| 4 | 99.00 | +0.24 | +0.049 | 0.95 | 0.021 | 0.000 | 1/2 fresh | 14.30 |
| 4 | 99.10 | +0.31 | +0.057 | 0.95 | 0.023 | 0.000 | 1/2 fresh | 14.37 |

| lap | closed at station m | length m | crossing - truth m | crossing - truth s | system - truth lap s | candidates | spread m | missed |
|---|---|---|---|---|---|---|---|---|
| 1 | 14.18 | 13.88 | +0.040 | +0.087 | +0.038 | 16 | 0.07 | False |
| 2 | 14.29 | 13.98 | +0.043 | +0.080 | -0.008 | 15 | 0.03 | False |
| 3 | 14.40 | 14.05 | +0.057 | +0.090 | +0.011 | 15 | 0.06 | False |
| 4 | 14.37 | 14.06 | +0.045 | +0.063 | -0.027 | 13 | 0.03 | False |
| 5 | 14.37 | 14.06 | +0.047 | +0.059 | -0.004 | 12 | 0.08 | False |

lap 2: 17 candidates, line - truth median +0.128 m, min +0.031, max +0.571
lap 3: 15 candidates, line - truth median +0.091 m, min +0.029, max +0.573
lap 4: 13 candidates, line - truth median +0.109 m, min +0.029, max +0.543

