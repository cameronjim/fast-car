# Stage 12: Phase 6 evaluation and Phase 7 writeup (roadmap 6.0 to 7.1)

Time: 3 to 5 weeks. The protocol is fixed before the first evaluation lap; that is what makes
the number credible.

## Steps

1. Analysis code tested (6.0) in `evaluation/analysis/`: paired confidence intervals, Wilson
   intervals for rates, DNF handling, tested against hand-computed answers and against
   synthetic data with a known effect size that the pipeline must recover. The interleaving
   generator is property-tested (balanced, no condition more than twice in a row).
2. Pre-registration (6.1) committed under `evaluation/prereg/` before any evaluation lap: the
   comparison (base versus base plus residual), the primary endpoint (lap time), the analysis,
   the sample size (at least 20 laps per condition per layout), and the three named outcomes
   with their planned writeups: improvement, null, regression.
3. Evaluation sessions (6.2): paired and interleaved in randomized ABBA-style order within each
   session, identical speed caps, same battery voltage window, tire set and wear session
   logged, surface and temperature logged, re-ID battery at session start, lap times from the
   gate only. Two layouts; on the second the residual's weights are frozen. Interventions
   counted from `/safety/events`. About 10 percent of calendar is repair time; crashes are
   scheduled.
4. Analysis (6.3): the committed notebook produces the reported numbers: lap-time means and
   95 percent CIs, completion rates with Wilson intervals, intervention counts, zero-shot
   numbers labeled alongside. No post-hoc endpoint changes; post-hoc sections are additive and
   labeled.
5. Writeup assembly (7.1): assemble from the per-phase notes in `docs/notes/`, the drift
   record, the fidelity curve, and the analysis output. Phase 7 is assembly, not authorship.

## Done when

The headline sentence from `claude-docs/00-project-overview.md` can be written with real
numbers in every slot, defensible whether the effect is positive, zero, or negative.
