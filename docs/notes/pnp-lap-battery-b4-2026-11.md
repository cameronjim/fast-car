# Lap battery B4, 2026-11 (v0.3 pre-registration section 1)

What ran: battery B4 once, conditions a to e, cases N1 to N3 (N4 dropped by the registered check:
condition a, seed 21, both directions, the map loaded, 0 laps, 3 wall contacts, stalled), seeds 21
to 23, ccw and cw, 12 laps, 90 runs, in the ros-dev container with 5 workers.

Code: commit 1bdaaf8cc0a0e205301dd04cffae515090c4d502 (PR 185: lap gate v2 as merged in f5db7f1,
plus the tooling, the runner's parser split and its guidance floor counters; nothing under
ros_ws/src/racer_laps or ros_ws/src/racer_planner differs from main). Constants file SHA-256
4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1 (the registered value, checked,
and written into every run JSON). Raw results and report.md:
/Users/cameronjim/code/car-data/pnp/battery/v0.3-b4/.

Decisions: D3-S0 FAIL, D3-S1 PASS, D3-S2(d) FAIL, D3-S2(e) PASS, v0.2 D-S1 rule (record only) FAIL.

Host: 5 workers. 44 of 90 first attempts were starved (a /drive_raw gap over 40 ms) and were rerun alone as registered (PR 166 rule), 139 attempts in all, every one kept under `attempts/`. Two runs were still starved after three attempts and were accepted as they are, flagged: `c_N2_ccw_22` and `e_N2_ccw_21`. No run was dropped. Wall time stepping, as in v0.2 (`--sim-time` was not used).

Observations that are not gates:
- S0b(c) fails on the share: 6 laps (laps 3 and 4 of N1/ccw/21, N1/ccw/22, N2/cw/22) are 0.4 to 0.56 s off, 210 of 216 within 0.3 s.
- S0b(d) and S0b(e) fail only the 1.0 s rule, on lap 1 (2.35 s in d, N3/cw/23; 2.37 s in e): the system's lap 1 is longer than the ground truth lap 1.
- Condition e: 2 of 18 runs (N2/cw/23, N3/ccw/22) stalled after 2 laps with no contact and did not complete; with the floor at 0 the speed bound can reach zero. D3-S2(e) still passes because 16 of 18 is the registered minimum.
- d and e are close: paired T_late(d)/T_late(e) median 1.002 (min 0.997, max 1.901).
- The v0.2 D-S1 rule, for the record, fails: 12 of 18 runs with r <= 0.80 (the six runs above 0.80 are all N2, r 0.805 to 0.822, and in each of them c reaches b within 0.7 percent, so on N2 the ceiling control itself only improves to about 0.8).

## Decisions

D3-S0: FAIL
  ok: S0a: system lap count equals ground truth in every condition (a) run: 18 of 18 runs agree []
  ok: S0a: every system lap time within 0.3 s of ground truth: 0 of 216 laps outside []
  ok: S0a: no condition (a) run failed to execute: 0 errors []
  ok: S0b(c): lap count equal in every run: 18 of 18 runs agree []
  FAIL: S0b(c): at least 98% of laps within 0.3 s: 210 of 216 laps within (0.9722); outside: ['N1/ccw/21 lap 3: system 21.99 vs truth 22.47', 'N1/ccw/21 lap 4: system 20.38 vs truth 19.89', 'N1/ccw/22 lap 3: system 22.06 vs truth 22.48', 'N1/ccw/22 lap 4: system 20.29 vs truth 19.89', 'N2/cw/22 lap 3: system 23.85 vs truth 24.40', 'N2/cw/22 lap 4: system 24.72 vs truth 24.16']
  ok: S0b(c): no lap off by more than 1.0 s: largest error 0.56 s
  ok: S0b(c): no run failed to execute: 0 errors []
  ok: S0b(d): lap count equal in every run: 18 of 18 runs agree []
  ok: S0b(d): at least 98% of laps within 0.3 s: 215 of 216 laps within (0.9954); outside: ['N3/cw/23 lap 1: system 32.00 vs truth 29.65']
  FAIL: S0b(d): no lap off by more than 1.0 s: largest error 2.35 s
  ok: S0b(d): no run failed to execute: 0 errors []
  ok: S0b(e): lap count equal in every run: 18 of 18 runs agree []
  ok: S0b(e): at least 98% of laps within 0.3 s: 193 of 196 laps within (0.9847); outside: ['N1/cw/23 lap 1: system 32.75 vs truth 30.38', 'N2/cw/23 lap 1: system 35.88 vs truth 33.52', 'N3/ccw/22 lap 1: system 32.30 vs truth 30.07']
  FAIL: S0b(e): no lap off by more than 1.0 s: largest error 2.37 s
  ok: S0b(e): no run failed to execute: 0 errors []

D3-S1: PASS
  ok: 1. 0 wall contacts in the 18 runs of condition c: 0 contacts, runs with a contact: []
  ok: 2. runs of condition c complete 10 laps: 18 of 18 complete, not complete: []
  ok: 3. reach: T_late(c) <= 1.05 x T_late(b) in at least 16 of 18 runs: 18 of 18 reach; ratios [0.993, 0.995, 0.996, 0.998, 1.0, 1.0, 1.002, 1.002, 1.003, 1.003, 1.004, 1.004, 1.004, 1.005, 1.005, 1.006, 1.007, 1.007]
  ok: 4a. T_late below T1 in every completing run: 0 runs not below []
  ok: 4b. median r <= 0.8: median r 0.502 over 18 completing runs

D3-S2(d): FAIL (no better than the schedule in sim)
  ok: 0 wall contacts in the 18 runs of condition d: 0 contacts, runs with a contact: []
  ok: runs of condition d complete 10 laps: 18 of 18 complete, not complete: []
  FAIL: median T_late(d) / T_late(c) <= 0.95: median 0.952 over 18 of 18 pairs
  ok: mean TTC or obstacle brakes per lap, laps 8 to 10, <= 0.2: 0.000 over 18 runs

D3-S2(e): PASS
  ok: 0 wall contacts in the 18 runs of condition e: 0 contacts, runs with a contact: []
  ok: runs of condition e complete 10 laps: 16 of 18 complete, not complete: ['N2/cw/23', 'N3/ccw/22']
  ok: median T_late(e) / T_late(c) <= 0.95: median 0.949 over 16 of 18 pairs
  ok: mean TTC or obstacle brakes per lap, laps 8 to 10, <= 0.2: 0.000 over 16 runs

v0.2 D-S1 rule (for the record, decides nothing): FAIL
  ok: 0 wall contacts in 18 runs: 0 contacts, runs with a contact: []
  FAIL: at least 16 of 18 runs complete 10 laps with r <= 0.8: 18 complete, 12 with r <= 0.8; r: [0.41, 0.41, 0.41, 0.41, 0.41, 0.42, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.8, 0.81, 0.81, 0.82, 0.82, 0.82]

## Reported without a threshold

- T_late(d) / T_late(e), paired: median 1.002 over 16 pairs (min 0.997, max 1.901)
- condition d: 0 contacts, 1 obstacle brakes over all laps, 0.000 per lap over laps 8 to 10 (18 runs)
- condition e: 0 contacts, 1 obstacle brakes over all laps, 0.000 per lap over laps 8 to 10 (16 runs)
- d: floor was the active bound in 986 of 34019 guidance messages with stations (0.0290)
- d: zones created 19, zones standing at the end 2, merged zones (created minus standing) 17

## What goes on the car, by owner answer on the floor and merge

- owner confirms: no D on the car (D3-S0 or D3-S1 failed); table row would give e
- owner rejects: no D on the car (D3-S0 or D3-S1 failed); table row would give e
- owner undecided: no D on the car (D3-S0 or D3-S1 failed); table row would give v0


## Runs


| cond | case | dir | seed | laps | T1 s | T_late s | r | contacts | obstacle brakes per lap (late) | complete |
|---|---|---|---|---|---|---|---|---|---|---|
| a | N1 | ccw | 21 | 12 | 30.63 | 30.46 | 0.994 | 0 | 0.00 | yes |
| a | N1 | ccw | 22 | 12 | 30.62 | 30.45 | 0.994 | 0 | 0.00 | yes |
| a | N1 | ccw | 23 | 12 | 30.64 | 30.41 | 0.992 | 0 | 0.00 | yes |
| a | N1 | cw | 21 | 12 | 30.49 | 30.26 | 0.992 | 0 | 0.00 | yes |
| a | N1 | cw | 22 | 12 | 30.45 | 30.29 | 0.995 | 0 | 0.00 | yes |
| a | N1 | cw | 23 | 12 | 30.48 | 30.30 | 0.994 | 0 | 0.00 | yes |
| a | N2 | ccw | 21 | 12 | 30.33 | 30.15 | 0.994 | 0 | 0.00 | yes |
| a | N2 | ccw | 22 | 12 | 30.36 | 30.17 | 0.994 | 0 | 0.00 | yes |
| a | N2 | ccw | 23 | 12 | 30.41 | 30.16 | 0.992 | 0 | 0.00 | yes |
| a | N2 | cw | 21 | 12 | 30.21 | 30.00 | 0.993 | 0 | 0.00 | yes |
| a | N2 | cw | 22 | 12 | 30.21 | 30.01 | 0.993 | 0 | 0.00 | yes |
| a | N2 | cw | 23 | 12 | 30.19 | 29.99 | 0.993 | 0 | 0.00 | yes |
| a | N3 | ccw | 21 | 12 | 26.64 | 26.54 | 0.996 | 0 | 0.00 | yes |
| a | N3 | ccw | 22 | 12 | 26.62 | 26.46 | 0.994 | 0 | 0.00 | yes |
| a | N3 | ccw | 23 | 12 | 26.62 | 26.45 | 0.994 | 0 | 0.00 | yes |
| a | N3 | cw | 21 | 12 | 26.28 | 25.97 | 0.988 | 0 | 0.00 | yes |
| a | N3 | cw | 22 | 12 | 26.23 | 25.97 | 0.990 | 0 | 0.00 | yes |
| a | N3 | cw | 23 | 12 | 26.24 | 26.00 | 0.991 | 0 | 0.00 | yes |
| b | N1 | ccw | 21 | 12 | 15.49 | 15.29 | 0.987 | 0 | 0.00 | yes |
| b | N1 | ccw | 22 | 12 | 15.53 | 15.30 | 0.985 | 0 | 0.00 | yes |
| b | N1 | ccw | 23 | 12 | 15.55 | 15.32 | 0.985 | 0 | 0.00 | yes |
| b | N1 | cw | 21 | 12 | 15.63 | 15.40 | 0.985 | 0 | 0.00 | yes |
| b | N1 | cw | 22 | 12 | 15.62 | 15.40 | 0.986 | 0 | 0.00 | yes |
| b | N1 | cw | 23 | 12 | 15.62 | 15.43 | 0.988 | 0 | 0.00 | yes |
| b | N2 | ccw | 21 | 12 | 24.80 | 24.85 | 1.002 | 0 | 0.00 | yes |
| b | N2 | ccw | 22 | 12 | 24.94 | 24.77 | 0.993 | 0 | 0.00 | yes |
| b | N2 | ccw | 23 | 12 | 25.08 | 24.79 | 0.989 | 0 | 0.00 | yes |
| b | N2 | cw | 21 | 12 | 24.27 | 24.33 | 1.003 | 0 | 0.00 | yes |
| b | N2 | cw | 22 | 12 | 24.52 | 24.36 | 0.993 | 0 | 0.00 | yes |
| b | N2 | cw | 23 | 12 | 24.51 | 24.25 | 0.989 | 0 | 0.00 | yes |
| b | N3 | ccw | 21 | 12 | 15.96 | 10.94 | 0.685 | 0 | 0.00 | yes |
| b | N3 | ccw | 22 | 12 | 11.26 | 10.94 | 0.971 | 0 | 0.00 | yes |
| b | N3 | ccw | 23 | 12 | 11.26 | 10.95 | 0.973 | 0 | 0.00 | yes |
| b | N3 | cw | 21 | 12 | 11.14 | 10.82 | 0.971 | 0 | 0.00 | yes |
| b | N3 | cw | 22 | 12 | 11.20 | 10.82 | 0.966 | 0 | 0.00 | yes |
| b | N3 | cw | 23 | 12 | 11.16 | 10.81 | 0.969 | 0 | 0.00 | yes |
| c | N1 | ccw | 21 | 12 | 30.65 | 15.35 | 0.501 | 0 | 0.00 | yes |
| c | N1 | ccw | 22 | 12 | 30.64 | 15.35 | 0.501 | 0 | 0.00 | yes |
| c | N1 | ccw | 23 | 12 | 30.66 | 15.32 | 0.500 | 0 | 0.00 | yes |
| c | N1 | cw | 21 | 12 | 30.50 | 15.34 | 0.503 | 0 | 0.00 | yes |
| c | N1 | cw | 22 | 12 | 30.50 | 15.32 | 0.502 | 0 | 0.00 | yes |
| c | N1 | cw | 23 | 12 | 30.47 | 15.33 | 0.503 | 0 | 0.00 | yes |
| c | N2 | ccw | 21 | 12 | 30.30 | 24.91 | 0.822 | 0 | 0.00 | yes |
| c | N2 | ccw | 22 | 12 | 30.33 | 24.90 | 0.821 | 0 | 0.00 | yes |
| c | N2 | ccw | 23 | 12 | 30.37 | 24.88 | 0.819 | 0 | 0.00 | yes |
| c | N2 | cw | 21 | 12 | 30.17 | 24.33 | 0.807 | 0 | 0.00 | yes |
| c | N2 | cw | 22 | 12 | 30.19 | 24.31 | 0.805 | 0 | 0.00 | yes |
| c | N2 | cw | 23 | 12 | 30.18 | 24.29 | 0.805 | 0 | 0.00 | yes |
| c | N3 | ccw | 21 | 12 | 26.62 | 11.01 | 0.414 | 0 | 0.00 | yes |
| c | N3 | ccw | 22 | 12 | 26.65 | 10.97 | 0.412 | 0 | 0.00 | yes |
| c | N3 | ccw | 23 | 12 | 26.61 | 11.01 | 0.414 | 0 | 0.00 | yes |
| c | N3 | cw | 21 | 12 | 26.27 | 10.90 | 0.415 | 0 | 0.00 | yes |
| c | N3 | cw | 22 | 12 | 26.22 | 10.86 | 0.414 | 0 | 0.00 | yes |
| c | N3 | cw | 23 | 12 | 26.22 | 10.88 | 0.415 | 0 | 0.00 | yes |
| d | N1 | ccw | 21 | 12 | 30.68 | 14.66 | 0.478 | 0 | 0.00 | yes |
| d | N1 | ccw | 22 | 12 | 30.60 | 14.55 | 0.476 | 0 | 0.00 | yes |
| d | N1 | ccw | 23 | 12 | 30.62 | 14.60 | 0.477 | 0 | 0.00 | yes |
| d | N1 | cw | 21 | 12 | 30.45 | 14.54 | 0.477 | 0 | 0.00 | yes |
| d | N1 | cw | 22 | 12 | 30.50 | 14.56 | 0.477 | 0 | 0.00 | yes |
| d | N1 | cw | 23 | 12 | 30.50 | 14.61 | 0.479 | 0 | 0.00 | yes |
| d | N2 | ccw | 21 | 12 | 30.33 | 20.87 | 0.688 | 0 | 0.00 | yes |
| d | N2 | ccw | 22 | 12 | 30.33 | 20.98 | 0.692 | 0 | 0.00 | yes |
| d | N2 | ccw | 23 | 12 | 30.36 | 20.85 | 0.687 | 0 | 0.00 | yes |
| d | N2 | cw | 21 | 12 | 30.15 | 20.36 | 0.675 | 0 | 0.00 | yes |
| d | N2 | cw | 22 | 12 | 35.66 | 20.37 | 0.571 | 0 | 0.00 | yes |
| d | N2 | cw | 23 | 12 | 30.20 | 20.31 | 0.673 | 0 | 0.00 | yes |
| d | N3 | ccw | 21 | 12 | 26.61 | 11.11 | 0.417 | 0 | 0.00 | yes |
| d | N3 | ccw | 22 | 12 | 26.58 | 11.07 | 0.417 | 0 | 0.00 | yes |
| d | N3 | ccw | 23 | 12 | 26.61 | 11.07 | 0.416 | 0 | 0.00 | yes |
| d | N3 | cw | 21 | 12 | 26.26 | 10.94 | 0.417 | 0 | 0.00 | yes |
| d | N3 | cw | 22 | 12 | 26.25 | 10.91 | 0.416 | 0 | 0.00 | yes |
| d | N3 | cw | 23 | 12 | 29.65 | 20.74 | 0.699 | 0 | 0.00 | yes |
| e | N1 | ccw | 21 | 12 | 30.67 | 14.59 | 0.476 | 0 | 0.00 | yes |
| e | N1 | ccw | 22 | 12 | 30.59 | 14.58 | 0.477 | 0 | 0.00 | yes |
| e | N1 | ccw | 23 | 12 | 30.69 | 14.53 | 0.473 | 0 | 0.00 | yes |
| e | N1 | cw | 21 | 12 | 30.48 | 14.55 | 0.477 | 0 | 0.00 | yes |
| e | N1 | cw | 22 | 12 | 30.46 | 14.52 | 0.477 | 0 | 0.00 | yes |
| e | N1 | cw | 23 | 12 | 30.38 | 14.59 | 0.480 | 0 | 0.00 | yes |
| e | N2 | ccw | 21 | 12 | 30.33 | 20.82 | 0.686 | 0 | 0.00 | yes |
| e | N2 | ccw | 22 | 12 | 30.34 | 20.82 | 0.686 | 0 | 0.00 | yes |
| e | N2 | ccw | 23 | 12 | 30.41 | 20.85 | 0.686 | 0 | 0.00 | yes |
| e | N2 | cw | 21 | 12 | 30.18 | 20.35 | 0.674 | 0 | 0.00 | yes |
| e | N2 | cw | 22 | 12 | 30.21 | 20.30 | 0.672 | 0 | 0.00 | yes |
| e | N2 | cw | 23 | 2 | 33.52 | nan | nan | 0 | nan | NO  |
| e | N3 | ccw | 21 | 12 | 26.60 | 11.13 | 0.418 | 0 | 0.00 | yes |
| e | N3 | ccw | 22 | 2 | 30.07 | nan | nan | 0 | nan | NO  |
| e | N3 | ccw | 23 | 12 | 26.59 | 11.07 | 0.416 | 0 | 0.00 | yes |
| e | N3 | cw | 21 | 12 | 26.25 | 10.97 | 0.418 | 0 | 0.00 | yes |
| e | N3 | cw | 22 | 12 | 26.22 | 10.89 | 0.415 | 0 | 0.00 | yes |
| e | N3 | cw | 23 | 12 | 26.28 | 10.91 | 0.415 | 0 | 0.00 | yes |



## Attempts (runs that needed more than one)
- a_N1_ccw_22: [a1 starved=True gap=0.0447, a2 starved=False gap=0.0266]
- a_N1_ccw_23: [a1 starved=True gap=0.0417, a2 starved=False gap=0.0272]
- a_N1_cw_21: [a1 starved=True gap=0.0476, a2 starved=False gap=0.024]
- a_N1_cw_23: [a1 starved=True gap=0.0863, a2 starved=True gap=0.0655, a3 starved=False gap=0.0258]
- a_N2_ccw_21: [a1 starved=True gap=0.0836, a2 starved=False gap=0.0297]
- a_N2_ccw_22: [a1 starved=True gap=0.0973, a2 starved=True gap=0.0663, a3 starved=False gap=0.0237]
- a_N2_ccw_23: [a1 starved=True gap=0.3142, a2 starved=False gap=0.0397]
- a_N2_cw_21: [a1 starved=True gap=0.0681, a2 starved=False gap=0.0227]
- a_N2_cw_22: [a1 starved=True gap=0.0466, a2 starved=False gap=0.0271]
- a_N2_cw_23: [a1 starved=True gap=0.0444, a2 starved=False gap=0.0399]
- a_N3_ccw_22: [a1 starved=True gap=0.0517, a2 starved=False gap=0.0237]
- a_N3_cw_21: [a1 starved=True gap=0.087, a2 starved=False gap=0.0366]
- a_N3_cw_22: [a1 starved=True gap=0.0857, a2 starved=False gap=0.024]
- a_N3_cw_23: [a1 starved=True gap=0.0813, a2 starved=False gap=0.0326]
- b_N1_ccw_21: [a1 starved=True gap=0.0647, a2 starved=False gap=0.024]
- b_N1_ccw_22: [a1 starved=True gap=0.0795, a2 starved=False gap=0.0235]
- b_N1_ccw_23: [a1 starved=True gap=0.0508, a2 starved=False gap=0.032]
- b_N2_ccw_22: [a1 starved=True gap=0.0511, a2 starved=False gap=0.0244]
- b_N2_ccw_23: [a1 starved=True gap=0.0702, a2 starved=False gap=0.0368]
- b_N2_cw_21: [a1 starved=True gap=0.0861, a2 starved=False gap=0.0244]
- b_N3_ccw_23: [a1 starved=True gap=0.0429, a2 starved=False gap=0.0228]
- c_N1_ccw_23: [a1 starved=True gap=0.0457, a2 starved=False gap=0.0287]
- c_N1_cw_21: [a1 starved=True gap=0.0431, a2 starved=False gap=0.0228]
- c_N1_cw_22: [a1 starved=True gap=0.0714, a2 starved=False gap=0.0272]
- c_N1_cw_23: [a1 starved=True gap=0.0693, a2 starved=False gap=0.0265]
- c_N2_ccw_21: [a1 starved=True gap=0.0696, a2 starved=False gap=0.0263]
- c_N2_ccw_22: [a1 starved=True gap=0.0722, a2 starved=True gap=0.048, a3 starved=True gap=0.0605]
- c_N2_ccw_23: [a1 starved=True gap=0.0734, a2 starved=False gap=0.0237]
- c_N2_cw_21: [a1 starved=True gap=0.0743, a2 starved=False gap=0.0338]
- c_N3_cw_22: [a1 starved=True gap=0.0528, a2 starved=False gap=0.0244]
- c_N3_cw_23: [a1 starved=True gap=0.0812, a2 starved=False gap=0.0231]
- d_N1_ccw_21: [a1 starved=True gap=0.0774, a2 starved=False gap=0.0325]
- d_N1_ccw_22: [a1 starved=True gap=0.0586, a2 starved=False gap=0.0261]
- d_N1_ccw_23: [a1 starved=True gap=0.0547, a2 starved=False gap=0.0234]
- d_N2_cw_23: [a1 starved=True gap=0.0612, a2 starved=False gap=0.0274]
- d_N3_ccw_21: [a1 starved=True gap=0.0677, a2 starved=False gap=0.026]
- d_N3_ccw_22: [a1 starved=True gap=0.0667, a2 starved=False gap=0.0227]
- d_N3_ccw_23: [a1 starved=True gap=0.0666, a2 starved=True gap=0.0477, a3 starved=False gap=0.0248]
- d_N3_cw_21: [a1 starved=True gap=0.0698, a2 starved=False gap=0.022]
- e_N2_ccw_21: [a1 starved=True gap=0.0459, a2 starved=True gap=0.0629, a3 starved=True gap=0.0593]
- e_N2_ccw_22: [a1 starved=True gap=0.0665, a2 starved=False gap=0.0317]
- e_N2_ccw_23: [a1 starved=True gap=0.0731, a2 starved=False gap=0.0361]
- e_N2_cw_21: [a1 starved=True gap=0.0641, a2 starved=False gap=0.0235]
- e_N2_cw_22: [a1 starved=True gap=0.0603, a2 starved=False gap=0.0335]

## What goes on the car under each owner answer

D3-S0 failed, so D does not go on the car with this lap source, whatever the owner decides about
the zone floor and merge. For the record, the table of section 1.4 would give: owner confirms the
deviation: e (d missed its 0.95 by 0.002, e passed); owner rejects it: e; no decision: D v0 only.
None of this applies while D3-S0 stands failed.

What failed is the lap gate's timing, not the cap schedule: lap 1 of conditions d and e is up to
2.4 s longer in the system than in the ground truth, and laps 3 and 4 of three condition c runs
are 0.4 to 0.56 s off. A fix is a new brief and a second, labelled run.

## Development regression (labelled development, no gate)

Conditions a and c on the development cases T, T2, S1 to S4, seeds 11 to 13, ccw and cw, 72 runs,
same commit, in `/Users/cameronjim/code/car-data/pnp/battery/v0.3-b4-development/`.

- Condition c: 36 of 36 runs complete, 0 wall contacts, system lap count equal to ground truth in
  every run, 431 of 432 laps within 0.3 s, largest error 2.37 s. Median r by case: T 0.637,
  T2 0.624, S1 0.424, S2 0.646, S3 0.798, S4 0.387.
- Condition a: 4 of 36 runs were invalid, not results: T2 ccw seeds 11, 12, 13 and T cw seed 13.
  Each shows two publishers on /drive_raw and /drive and sim time at about 1.95 times wall time,
  and ended with 0 laps (T2 ccw 11: 2 wall contacts; T cw 13: 33). The first attempt of each had a
  gap of about 50 s (the host slept); leftover processes of that attempt most likely shared the
  ROS domain with the rerun. This was not seen in any of the 90 battery runs (one publisher each,
  sim time 1.0 times wall time in all).
- Rerun of condition a on T and T2 (12 runs) with no stray processes, into
  `.../v0.3-b4-development-rerun/`: 12 of 12 complete 12 laps, 0 contacts, lap counts equal, no
  run starved after its accepted attempt. Largest system lap error 0.39 s (T2 ccw seed 12); every
  other run is within 0.16 s. Both the original and the rerun are kept; the original 4 are not
  to be read as a regression.
