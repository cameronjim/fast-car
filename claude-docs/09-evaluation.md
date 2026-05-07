# 09 — Evaluation Protocol (pre-registered)

The protocol is what makes the headline number credible. It is fixed BEFORE data collection.
Anything less is an anecdote.

## Pre-registration (evaluation/prereg/)

Committed to git before the first evaluation lap: the comparison, the primary endpoint, the
analysis, and the three named outcomes:

- **Improvement** → headline number with CI
- **Null** → the fidelity study explains what the sim work did and didn't buy
- **Regression** → the failure-diagnosis writeup (07 checklist) is the result

All three are planned writeups. No post-hoc endpoint changes; post-hoc analyses are allowed
but labeled as such.

## Design

- **Paired**: base vs. base + residual — same tracker, same raceline, same tuning.
- **Interleaved, never blocked**: alternate conditions within each session (randomized
  ABBA-style order). Tire wear, motor/ESC temperature, battery, and surface dust drift
  monotonically; blocked A-then-B converts drift into a fake controller effect.
- ≥20 laps per condition per track; **minimum two track layouts**, one held out per the
  semantics in `08-learning.md`.
- **Identical speed caps** across conditions; tuning budget parity, in logged hours.

## Measurement

- Lap time from the **independent timing gate** (hardware in `11-hardware.md`), never from
  the onboard estimator — the estimator is part of the system under test.
- DNF handling, pre-stated: lap-time stats over completed laps only, reported ALONGSIDE
  completion rate. No imputation; no dropping crashes from the narrative.
- Safety interventions counted from `/safety/events`: every TTC brake, OOD fallback,
  covariance slowdown, manual kill.

## Controlled and logged per run

- Battery: same voltage window for every run (a fresh pack and a 20% pack are different cars)
- Surface + ambient temperature logged; tire set + wear session count logged
- Re-ID battery at session start (07) certifies the car is the same vehicle
- Track direction fixed per prereg

## Statistics

- Primary endpoint: lap time — mean, 95% CI, paired comparison across interleaved runs.
- Rates (crash, DNF, interventions): report with Wilson intervals. At n≈20 one crash moves
  the rate 5 points — REPORT rates, do not hypothesis-test rate differences the sample
  cannot support.
- Zero-shot numbers (pre-adaptation) reported alongside, clearly labeled, same protocol.

## Session checklist (evaluation/protocol/ runner enforces)

1. Kill-switch check → 2. re-ID battery → 3. battery window check → 4. timing gate check →
5. randomized run order printed → 6. every lap bagged with rail voltage → 7. session notes
committed same day to `docs/notes/`.
