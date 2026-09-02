# Stage 10: Phase 4, the classical baseline on hardware (roadmap 4.1 to 4.3, gate G3)

Time: 2 to 4 weeks. This stack is both the evaluation baseline and the base the learned
residual rides on, so underinvesting here weakens the headline result directly.

## Steps

1. Raceline optimization (4.1) for the venue track from the SLAM map, with the existing
   optimizer under `tools/raceline`, using the fitted friction values from Phase 3. Commit
   under `config/tracks/<venue>_<layout>/raceline.csv`. Repeat for the second layout.
2. Tracker tuning on hardware (4.2): lookahead, speed profile scaling, the speed ramp limits
   found in sim milestone 3. Every tuning hour is logged in `docs/notes/tuning-log.md`; the
   budget spent here is reported alongside the results.
3. Timing gate: mount the IR break-beam pair at the start/finish line, read by a Pico or the
   ingest board, timestamped, published as the independent lap clock. Lap times never come from
   the onboard estimator.
4. Evaluation machinery dry run (4.3): the session runner in `evaluation/protocol/` prints the
   randomized interleaved run order, enforces the checklist (kill check, re-ID battery, battery
   window, timing gate check), and bags every lap. Run at least 20 baseline laps on each layout
   under the full protocol as if it were the real evaluation.
5. Widen the VESC layer-2 limits only if the tracker needs it, with the change logged and the
   exported config re-committed.

## Done when

Baseline laps complete reliably under the full protocol on both layouts with the timing gate
producing the lap times. Record gate G3 as passed. If the baseline cannot lap reliably, the
evaluation is impossible for any controller; fix before Phase 5.
