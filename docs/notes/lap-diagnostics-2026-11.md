# M-D1b: what the lap memory buys, development ablations of brief 53 (2026-11-23)

Development cases and seeds only (yaml `development`): decides no gate, never evidence that D works. Truth lap
source through the start gate, 13 laps, runs at commit `bf4ff89` of `feat/dv1-b11`, files
`car-data/pnp/battery/v0.6-dv1-dev/`. Tuned values chosen before any run and not changed: station uncertainty
0.02 per metre, cap 0.5 m; reset length share 0.08; curvature bound a_lat 2.0 m/s^2.

Part 1 (the candidate rule's input): c, K1, K2, K3 on the minimum cases T, S1, S2, S3, N1, N3, seeds 11 to 13,
both directions, 144 runs. 39 attempts were starved by host load and rerun alone; K2 T/cw/11 was starved on all
three attempts, so it counts as failed to execute and K2 is not eligible.

| condition | runs | contacts | not 12 laps | stalls > 5 s | late brakes/lap | median T_late / c | median sum 2-10 / c |
|---|---|---|---|---|---|---|---|
| c | 36 | 0 | 0 | 0 | 0.000 | 1.000 | 1.000 |
| K1 | 36 | 0 | 0 | 0 | 0.000 | 1.027 | 0.998 |
| K2 | 36 | 0 | 1 (failed to execute) | 0 | 0.000 | 0.944 | 0.920 |
| K3 | 36 | 0 | 0 | 0 | 0.000 | 0.801 | 0.855 |

Candidate rule (yaml `candidate_rule`, applied as written): K1 eligible, score 1.027; K2 not eligible; K3 eligible,
score 0.801. Pick K3 (below 0.97). B11 was run (`docs/notes/pnp-lap-battery-b11-2026-11.md`).

Not in the rule, seen while reading: K3 raises TTC ENGAGE records on late laps where c has none (B11 then failed
on exactly that), and its gain is largest on the cases where c is far below the ceiling.

Part 2 (conditions a, b, m, f, u, t, z) was not run before B11; brief 44's reading (a learned value profile is
part of a later D only if u or f has a median late-lap time at most 0.97 times m's with no more contacts) needs
m, u and f and is therefore not yet answered. Status: see the brief's result section.
