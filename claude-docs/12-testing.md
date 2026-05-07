# 12 — Testing Strategy

Testing on this project is layered like the safety architecture: cheap, exhaustive tests on
pure logic; replay tests on real data; sim-in-the-loop on the full stack; scripted bench
procedures on hardware. **No roadmap task is `[x]` until its tests from this doc exist and
pass in CI** — that is the definition of done, and it applies to Claude Code's work
specifically.

## The layers

| Layer | What | Runs | Gate |
|---|---|---|---|
| L1 Unit | Pure logic, no ROS, no hardware | every push, `sim-cpu` + `ros-dev` | merge-blocking |
| L2 Property | Invariants under generated inputs (hypothesis) | every push | merge-blocking |
| L3 Node | Single ROS node, mocked topics (`launch_testing`) | every push, `ros-dev` | merge-blocking |
| L4 Replay | Recorded rosbags through real pipelines, golden outputs | every push (small bags), nightly (full) | merge-blocking |
| L5 Sim-in-loop | Full stack vs. `racer_gym`, headless, seeded | every push (short), nightly (long) | merge-blocking |
| L6 Bench (HIL) | Scripted wheels-off-ground procedures, human present | before any on-track session touching that code | session-blocking |
| L7 Track | Kill test, envelope-fallback live demo | roadmap gates G1/5.4 | phase gate |

## L1 — Unit tests (exhaustive on the critical set)

**Critical packages — 100% branch coverage on decision logic, enforced in CI:**

- `envelope/`: bounds clipping, rate limiting, OOD fallback trigger, speed envelope. Every
  branch, every boundary value (exactly at bound, epsilon over, NaN, inf).
- `racer_safety` gate logic (separated from node plumbing precisely so it is testable
  without ROS): TTC math, staleness watchdog, covariance gate, command sanity — table-driven,
  every gate condition × (pass / marginal / fail / garbage input).
- Contract loader (`racer_policy`): one test per mismatch class — bad checksum, schema
  version, obs schema, params version, missing field, extra field, wrong dtype, wrong units
  string. Each must produce a hard refusal with its specific error. **A test proving it
  refuses is as important as one proving it loads.**
- `tools/gen_params.py`: generated Python/C++/C bindings agree with each other on every
  field of a fixture params file (round-trip equality test across languages).

**Normal packages** (tracker math, raceline optimizer, fitting, transforms): standard pytest
/ gtest coverage of the math against hand-computed cases, including sign-convention cases
(left turn positive, slip angle sign) taken verbatim from `06-vehicle-params.md`.

## L2 — Property-based tests (hypothesis)

- Envelope: for ANY input command and ANY internal state, output is always within bounds and
  within rate limits of the previous output. This is the single most important test in the
  repo — the layer-4 safety claim rests on it.
- Params schema: serialize → validate → deserialize round-trips for generated valid configs;
  generated invalid configs always rejected.
- Unit conversions and frame transforms: inverse(f(x)) ≈ x, composition identities.

## L3 — Node tests (launch_testing, mocked topics)

Per node, at minimum: correct behavior on nominal input; correct behavior on silence
(watchdog paths); correct QoS (a `reliable` subscriber actually rejects a `best_effort`
mock); clean shutdown. `safety_node` additionally: publishes brake on internal exception
(fail-closed test — inject a fault, assert brake on `/drive`), emits `/safety/events` on
every intervention path.

## L4 — Replay / golden tests (the robotics workhorse)

- A curated set of small rosbags lives in `tests/bags/` (seconds long, committed via LFS or
  fetched by CI): nominal lap segment, LiDAR dropout, reflective-surface segment, brownout
  (real rail-sag recording once one exists), stale-sensor segment.
- Pipelines under test: EKF config, particle-filter config, safety node, full
  estimation→control chain. Outputs compared to committed golden results **with stated
  tolerances** (never exact float equality).
- Golden updates are deliberate: regenerating a golden file requires a PR note explaining
  WHY the output changed. A silent golden refresh is a defect.
- **Fault injection lives here**: bag-mutation fixtures inject NaNs, timestamp jumps
  (forward and backward), dropped frames, out-of-order messages, and frozen sensors into any
  bag. The estimator must not crash; the safety node must degrade per `04-architecture.md`
  (covariance gate → slowdown, staleness → brake). One test per fault × per consumer.

## L5 — Sim-in-the-loop (headless, seeded, CI)

- **Tracker lap test**: base controller completes N laps of a reference track in
  `racer_gym`, no wall contact, lap time within a committed band. This is the regression
  canary for the whole classical stack.
- **Envelope-in-env test**: run a deliberately hostile policy (max residual, max rate) —
  assert the env-side envelope clips identically to the deployment library (same `envelope/`
  import, same numbers; a divergence test, not two implementations).
- **Determinism test**: same seed → identical trajectory, to keep training reproducible.
- **Model-upgrade regression**: after any change to `racer_gym` dynamics, a fixed battery of
  maneuvers (step, circle, coastdown — the sysid battery in sim) produces outputs within
  tolerance of committed references, or the change is intentional and the references are
  updated with a stated reason.
- Nightly long run: training smoke test — a tiny SAC run (minutes) completes, loss is
  finite, checkpoint loads through the deployment contract. Catches pipeline rot early.

## L6 — Bench / HIL procedures (scripted, human present)

Scripted in `tests/bench/` as executable checklists (script drives, human confirms):

- Wheels-off-ground actuation sweep after ANY change to the command path: commands in,
  measured wheel/steering response out, compared to expected direction/magnitude/signs.
- Ingest-board timestamp sanity: monotonic, expected rates, cross-sensor offset within the
  sync-design budget.
- VESC config diff: exported config matches the committed layer-2 config byte-for-byte
  before every session.
- Rail brownout drill (bench PSU): sag the rail, assert logging captured it and the mux MCU
  stayed alive.

## L7 — Track-level proofs (phase gates)

- G1 kill test: Jetson actually frozen, mux cuts drive (roadmap 1.3).
- Envelope fallback live (roadmap 5.4): force OOD, observe fallback to base controller and
  the `/safety/events` record.

## Evaluation and sysid code are tested too

- `evaluation/analysis/`: stats functions (paired CI, Wilson intervals, DNF handling) tested
  against hand-computed known answers AND against synthetic data with a known effect size
  (the pipeline must recover it). The reported numbers come from tested code or they are
  not reported.
- `sysid/fitting/`: fit a model to trajectories generated from KNOWN parameters + noise;
  assert recovery within tolerance. Held-out validation split logic unit-tested.
- Interleaving/run-order generator: property test — balanced orders, no condition ever runs
  >2 consecutively.

## CI wiring

- Every push: L1–L3, L4-small, L5-short. Merge is blocked on green; there are no skip labels.
- Nightly (Desktop A runner): L4-full, L5-long, training smoke.
- Coverage report per package; `envelope/` and `racer_safety` gate at 100% branch on
  decision logic, correctness-critical Python (`racer_policy`, fitting, analysis) at ≥90%,
  the rest reported but not gated.
- A flaky test is quarantined within one day with an issue filed — never deleted, never
  retried-until-green.

## Rules for Claude Code

- Every roadmap task's PR contains its tests from the relevant layer(s); the task list in
  `01-roadmap.md` is only ticked when they pass in CI.
- Never weaken a tolerance, delete a golden file, or relax a coverage gate to make a build
  pass. If a test seems wrong, say so and ask — the test might be the bug, but that is a
  human decision.
- When you fix a bug, first write the failing test that reproduces it (replay bag or unit
  case), then fix, then keep the test.
- New fault modes discovered on the bench or track become L4 fixtures within the same week.
