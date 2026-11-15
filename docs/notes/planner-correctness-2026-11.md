# Planner correctness and the timing chain (brief 42, 2026-11-14)

Branch `fix/pnp-planner-correctness`. Design: `claude-docs/13-plug-and-play.md` 17.1 and 17.4.
Measurements: addendum 1 section 4 (M-C1, M-C2, M-C3, M-T2). Nothing here tunes a weight, a
timeout, the sample count or the threading, and nothing touches `racer_safety`.

## The four faults, and what fixed each

| Fault on main f5db7f1 | Change | Test that fails on main |
|---|---|---|
| The smoothed weighted mean is evaluated, `pbd.collided` is never read | `mppi.cpp`: a colliding or non finite final sequence is replaced by the cheapest evaluated collision free sample of the chosen class; `SolveOutput::final_sequence` says which | `FinalSequence.TheMeanOfTwoCleanFamiliesCollidesOnTheOldPathAndFallsBackOnTheNewOne` (the old path is `check_final_sequence = false`) |
| Odometry and lap guidance freshness from arrival time | `judge_stamp` (`planner_logic`), used by the node: age from the header stamp, a stamp not later than the last accepted refreshes nothing, a zero stamp falls back to arrival; both counted and logged every 10 s | `test_planner_node_launch` `test_f_a_repeated_old_stamp_refreshes_nothing` (odometry keeps arriving with one old stamp: DRIVING on main, NO_ODOM here) and the `StampFreshness` L1 tables |
| Actuator estimate follows published commands | `Planner::apply_gated`: the previous tick's ring entry (or, with no delay ring, the estimate of that step) takes the gated `/drive` value matched to it; counted corrected, equal, fallback | `GatedEstimate.*` (a gated 0 against a published speed, a gated steering hold) |
| Dead reckoning omits the learned yaw | `EgoMotion(const DynamicsModel*)` takes the yaw rate from `DynamicsModel::yaw_rate_for`, residual included; the ring stores the acting commands | `EgoMotionModel.WithAResidualTheDeadReckonedYawEqualsTheModelRollouts` |

Bit identity (tests): a clean mean gives the same outputs with the check on and off; `/drive`
equal to `/drive_raw` leaves the planner state bit identical to a planner that is never told a
gated sequence number; a bundle without a residual gives bit identical dead reckoning.

### How a gated message is matched to a tick

`/drive` carries float32 values. A gated message received after tick k's publish is compared
(exact float32 equality, both fields) with k's published command: equal means the double stays as
published (no correction, counted `equal`); equal to an older tick's command means it is that
tick's late message (counted `fallback` for k); anything else is the gate's output for k and
replaces k's ring entry (counted `corrected`). With no gated message since k's publish the
published value stands (`fallback`, as before). A gated message that stopped arriving therefore
gives exactly the old behaviour. Limits: a gate output that arrives later than the next tick's
publish is attributed to the wrong tick; with a ring of zero ticks the estimate is moved by the
difference the gated command makes to the one step it already acted for.

### A hazard the owner should know about

A source whose header stamps jump backwards for good (a clock reset, a looped bag replay) is
never accepted again by the stamp rule, so NO_ODOM follows. This is the rule as written in the
brief. It does not occur with a live car or the sim; it matters for bag replay tools that loop.

## Speed limiter attribution (PlannerTiming.speed_limiter)

The rule is stated in `mppi.hpp` and `PlannerTiming.msg` and tested (`SpeedLimiter.*`): the final
sequence is re-evaluated with every speed command raised by 0.1 m/s; clearance, terminal, gate and
lap_trust are the terms that can limit; if the sum of their positive changes is at least the fall
of the progress terms, the largest is the limiter; else the cap (plan speed within 0.05 m/s of it),
else the sample choice; none for a plan that does not move; validity when the cap was lowered by
the track confidence. Cost: two more evaluations per solve (of 128 to 768).

## Measurements

The v0.4 pre-registration (PR 186) and addendum 1 (PR 189, yaml SHA-256
`ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824`) were not on main when this
was written, so by the brief's rule no number they govern was produced and the data roles of the
2026-11-14 segments were not read. Table to fill once both are on main:

| Item | Status |
|---|---|
| Stamp age of `/odom/wheel` at receipt on L0, X0, X1 (p99 plus one period against the present timeout) | not run, waits for PR 186 and 189 |
| M-C1, share of ticks where the unchecked final sequence collided although a clean sample existed, on L0, X0, X1 | not run, same reason; the tool is `planner_replay_cli --set check_final_sequence=0` (main's behaviour) against the default, columns `final_sequence` and `unchecked_final_collided` |
| M-C1 on the L5 canary | not run, no L5 before the B4 report |
| M-C2, M-C3, M-T2 on a 2026-11-14 bag (echo part) | tools built and tested on synthetic streams and the R2 fixture; `python -m racer_eval.timing_chain BAG` refuses until both pre-registration files are on the checkout with the registered SHA-256 |

The stamp age measurement is the gate of change 2 in the brief: if p99 plus one period exceeds
the present odometry timeout, stop and report instead of loosening. Until it is run the stamp rule
is built and tested, but the brief's own precondition is open.

## Tools

- `racer_eval.timing_chain`: per command the first safety cycle that echoes it, the legs, the share
  identified, the ambiguous commands (equal consecutive, gate changed every changed field), the
  latched command per cycle (M-T2 input), M-C3. Takes the latch from `/safety/cycle` when the bag
  has it (brief 45) and says so.
- `pnp_session_report timing --m-t2`: the limiter replay of the M-T code with the latched command
  from the echo; "reproduces" when every recorded `rate_limit` ENGAGE is replayed at the same cycle.
  The M-T result is untouched next to it.
