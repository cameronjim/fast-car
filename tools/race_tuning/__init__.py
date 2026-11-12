"""Auto-tune harness for gap_follow's race profile (docs/notes/gap-follow-race-2026-11-12.md).

    tools/race_tune.py search    sample parameter sets, run them on the TUNING tracks, rank them
    tools/race_tune.py evaluate  run named parameter sets on the HELD-OUT tracks, report vs baseline
    tools/race_tune.py replay    replay real bags through the baseline and the race profile
    tools/race_tune.py select    apply the pre-stated selection rule to a search directory

Modules (all pure, tested in tools/tests/test_race_tune.py, except `runs`, which starts the
simulator through tests/l5_reactive_bench/track_family_runner.py):

  tracks     randomised track family: a seed -> a track, tuning seeds and held-out seeds disjoint
  stats      bootstrap and Wilson intervals, paired ratios
  selection  the selection rule, written down before the search was run
  space      the parameter space and the seeded sampler
  runs       run specs, execution (resumable), per-candidate aggregation
  replay_check  the real-bag check: nothing the race profile commands may be crazy
"""
