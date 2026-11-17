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

### Looped replay hits NO_ODOM, by design (owner decision 2026-11-14)

A source whose header stamps jump backwards for good (a clock reset, a looped bag replay) is never
accepted again by the stamp rule, so NO_ODOM follows. The owner kept the rule: on the car stamps
come from system time and do not go backwards. The 10 s log line of the counters says so.
A gated message later than the next tick's publish is attributed to the wrong tick; accepted for
now. The safety receipt record (brief 45, `/safety/cycle`) is approved, and `timing_chain` prefers
it where the bag has it.

## Speed limiter attribution (PlannerTiming.speed_limiter)

The rule is stated in `mppi.hpp` and `PlannerTiming.msg` and tested (`SpeedLimiter.*`): the final
sequence is re-evaluated with every speed command raised by 0.1 m/s; clearance, terminal, gate and
lap_trust are the terms that can limit; if the sum of their positive changes is at least the fall
of the progress terms, the largest is the limiter; else the cap (plan speed within 0.05 m/s of it),
else the sample choice; none for a plan that does not move; validity when the cap was lowered by
the track confidence. Cost: two more evaluations per solve (of 128 to 768).

## Measurements

The v0.4 pre-registration and addendum 1 are on main (yaml SHA-256 `78510c14...` and
`ada5a836...`, checked by the tool on every run). Read in the roles the yaml gives: the L0, X0 and
X1 runs are lap `dev` runs, read without limit; no held-out or test segment was read. Code: this
branch after merging main (planner built in the ros-dev container, 432 tests, 0 failures).

**Stamp age of `/odom/wheel` at receipt** (recorder receive time minus header stamp, the gate of
change 2; present `odom_timeout_s` is 0.10 s). p99 plus one period (p50 20 ms) is below the
timeout on all three, so the gate passes and nothing is loosened.

| Segment | n | age p50 (ms) | age p99 (ms) | age max (ms) | p99 plus period (ms) |
|---|---|---|---|---|---|
| L0 | 5741 | 2.9 | 26.2 | 62.6 | 46.2 |
| X0 | 5090 | 2.6 | 24.8 | 147.9 | 44.8 |
| X1 | 2915 | 2.5 | 18.5 | 41.0 | 38.5 |

**M-C1** (`pnp_replay.py` with dyn-v0.1-2026-11-12, geometric perception, cap 1.0, whole PnP run
of each segment; "main" is `--set check_final_sequence=0`, which is main's behaviour). Share of
solved ticks where the unchecked final sequence collided although a clean sample existed:

| Run | Solved ticks | Main: unchecked collided | Branch: replaced by fallback |
|---|---|---|---|
| L0 | 5800 | 0 (0.00 %) | 0 |
| X0 | 5139 | 8 (0.16 %) | 5 (0.10 %) |
| X1 | 3033 | 0 (0.00 %) | 0 |

The open loop replay does not feed commands back, so the two columns count different ticks of
a plan stream that is the same up to the first replaced tick; on the branch every flagged tick was
replaced (flag count equals fallback count). No tick had all samples colliding. No threshold.
M-C1 on the L5 canary: not run in this note.

**M-C2, M-C3 (echo part) and M-T2 on L0's bag** (`2026-11-14T21-49-21_car_teleop`, no
`/pnp/planner/timing` and no `/safety/cycle` in it, so the latch is the echo):

| Item | Result |
|---|---|
| Commands, identified by echo | 5802, 5665 identified (97.6 %), 35 equal to the previous command, 102 unmatched |
| publish to first echoing cycle (s) | p50 0.0118, p99 0.0209, max 0.0655 |
| M-C3 distinct commanded speeds | 5681; only the 0.0 m/s command was held for 1 s (wheel 0.0 m/s); no steady wheel speed can be paired with the others |
| M-T2 replay with the echoed latch | reproduces: rate_limit recorded 2, replayed 2, matched 2, extra 0 (dt detail within 0.002 ms); watchdog recorded 3, replayed 1, matched 1, missed 2 |
| M-T2 cycles with an ambiguous echo | 47.8 % of cycles (listed in the JSON) |

The watchdog misses are the receive-time effect already known from M-T (the gate's own receive
time is not recorded); `/safety/cycle` removes it at the next session. M-T2 as registered
"reproduces" because every `rate_limit` ENGAGE is replayed at the same cycle. The limiter replay
ran `limiter_whatif_cli` built from `racer_session_tools` in the container.

Still open: M-C1 on the L5 canary (no L5 run here) and M-C2 legs that need `/pnp/planner/timing`
(first bag recorded by this code is the next session).

## Tools

- `racer_eval.timing_chain`: per command the first safety cycle that echoes it, the legs, the share
  identified, the ambiguous commands (equal consecutive, gate changed every changed field), the
  latched command per cycle (M-T2 input), M-C3. Takes the latch from `/safety/cycle` when the bag
  has it (brief 45) and says so.
- `pnp_session_report timing --m-t2`: the limiter replay of the M-T code with the latched command
  from the echo; "reproduces" when every recorded `rate_limit` ENGAGE is replayed at the same cycle.
  The M-T result is untouched next to it.
