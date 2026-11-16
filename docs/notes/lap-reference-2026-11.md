# lapref: the offline lap reference (2026-11)

Pre-registration PnP v0.4 sections 4.2 and 4.3, brief 36. Tool: `tools/lapref.py`, method in
`tools/lapref/method.py`, measurements by `tools/lapref_measure.py`, tables and verdict code in
`tools/lapref/compare.py`.

## Method

Inputs are the steering angle of the gated `/drive` and the forward speed of `/odom/wheel` inside
one run, nothing else (no scan, camera, pose, planner or lap memory output; `/scan` is read for
the log time of its first message only, the zero of the bag axis).

1. Steering resampled onto wheel distance (0.02 m grid), the first 1.0 m dropped.
2. Lap length L: the lag of the highest interior peak of the biased normalised autocorrelation
   over [2.0 m, 0.6 of the run's distance], parabola refined. Peak under 0.5: no laps returned.
3. Template: the signal folded at L. Phase of each lap: the shift in +-0.15 L (window of +-0.5 lap
   around the predicted crossing, Pearson correlation, parabola refined) that best matches the
   template; the prediction follows the previous lap's length.
4. Crossing times interpolated on the odometry clock; lap time = difference of crossings. A lap
   counts if its closing crossing is inside the run's wheel distance; no lap under 2.0 m.

Deviations from the brief's wording, needed to pass its own synthetic test (a line that shortens
2 percent per lap smears a fold over many laps): the first template folds only the first 2 laps,
then the template is rebuilt twice from the tracked laps (each lap resampled to its own length)
and pinned in phase to the run's start so the first lap stays a whole lap. No constant of the
brief was changed on the development data (a sweep of the window shares gave no gain).

## M-LR (development)

lapref against the pose detector of `racer_laps.replay` (differences in s, lapref minus pose):

| segment | pose laps | lapref laps | per lap | pose mean | lapref mean |
|---|---|---|---|---|---|
| G1a | 26 | 26 | -0.44 -0.08 -0.16 -0.10 -0.11 -0.03 -0.16 -0.12 -0.20 -0.11 -0.12 -0.08 -0.27 -0.13 -0.17 -0.34 -0.21 -0.16 -0.22 -0.12 -0.19 -0.14 -0.26 -0.24 -0.13 -0.16 | 16.48 | 16.31 |
| G1b | 6 | 6 | -0.13 -0.10 -0.06 -0.01 -0.19 -0.11 | 16.38 | 16.28 |
| T0 | 13 | 13 | +0.25 +0.42 -0.12 +0.02 +0.01 +0.06 -0.03 -0.06 +0.09 -0.06 -0.08 +0.06 +0.00 | 16.96 | 17.01 |
| T1 | 7 | 7 | -0.02 +0.00 -0.01 +0.01 +0.00 +0.00 -0.01 | 8.32 | 8.31 |
| T2a+T2b | 3 | 3 | -0.37 -0.06 +0.08 | 6.00 | 5.89 |

Counts equal on all five. The bias of about -0.15 s on G1a is the one gate v1 and v2 also show
against the pose detector (PR 178). Lap 1 differs most because the start lines differ.

L0: 20 laps found (owner observed 20), mean 5.64 s, peak 0.842. Other dev runs: X1 9 laps (mean
5.94 s), X3 11 (13.84 s), X5 8 (9.03 s), X6 6 (5.10 s). X0: no periodic lap found (peak 0.222,
the run drove only 51 m in 102 s); reported, not tuned.

Frozen at git `29225deb1ddb4205e5d617e74ceadb58e02cc538`.

## M-L1 (scored once, 2026-11-14, at the freeze SHA above)

lapref against lap gate v2 (`racer_laps` unchanged, last touched by `3d1550b`, the merge of PR
178; whether B4 ran on that commit is not stated on PR 185, so not confirmed). Whole runs, gate
replay with a 1.0 s tail (the bag had it for all five; no gate lap closed in a tail).

| run | lapref laps | gate laps | counts | within 0.3 s |
|---|---|---|---|---|
| K1 | 12 | 12 | equal | 10 of 12 |
| K2 | 11 | 11 | equal | 9 of 11 |
| X2 | 4 | 4 | equal | 3 of 4 |
| X4 | 9 | 9 | equal | 7 of 9 |
| X7 | 7 | 14 | DIFFER | 0 of 14 |

**M-L1: FAIL** (reading A, per run, and reading B, pooled 58.0 percent, both fail). Differing laps:
K1 1 (+0.60 s gate minus lapref), 2 (-0.38); K2 1 (+0.57), 11 (+0.45); X2 1 (+0.48); X4 1 (+1.98),
3 (+0.36); X7 every lap. Cause in X7: the autocorrelation peaked at 10.49 m, twice the 5.25 m
lap, so lapref returned double laps (about 9.2 s) and 7 instead of 14. The gate's 14 laps of about
4.6 s are consistent with the wheel distance (75.9 m). Lap 1 is the other pattern (K1, K2, X2, X4):
the gate's lap 1 starts at standstill and includes the launch, lapref's starts 1 m later.

Per pre-registration 4.3 neither source is trusted on the strength of the other; the phone video
of the next session decides. No change was made after reading these numbers.
