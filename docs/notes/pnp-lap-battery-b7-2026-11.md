# Lap battery B7, 2026-11: not run (stop rule of brief 48 item 5)

B7 (pre-registration `docs/notes/preregistration-pnp-v0.5-laps.md`, constants
`evaluation/prereg/pnp-v0.5-laps.yaml`, SHA-256
`35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de`) was not run. No P case and no
seed 51, 52 or 53 has run, the freeze file `evaluation/prereg/freeze/b7.json` does not exist, and
there is no D7-S0 or D7-S1 result.

## Why

Item 5 of the brief ends with a stop rule: if the cause of the lap-3 early crossing (S0b(c) of B4)
is none of h1 to h3, or the fix needs more than the one new constant, nothing is changed and B7 runs
only on the launcher's word. `lap-gate-lap3-2026-11.md` has the evidence. In 13 development runs
of B4's three failing condition c cases (and the good control) on the fixed harness no lap-3 early
crossing occurred, and the candidate tables give none of h1, h2 or h3 as its cause. What reproduces
B4's signature (crossing 3 early by 0.33 to 0.46 s, lap 3 short and lap 4 long by the same amount)
is the wheel odometry reaching the lap node about 0.7 s after it was stamped, which the gate
cannot see (`StartScanGate._at` extrapolates 0.2 s at most). That is outside the allowed fix, so the
gate is as it was after items 3 and 4.

The data of B7 is single use: its cases and seeds run once. Running them on a gate that may be
changed by what the launcher decides next would burn them.

## What is ready

- The registered truth and its validity rules in the runner, the two-phase start (the planner
  starts last, after the reset and after the lap node answers) and the battery decisions that read
  them (B4, B5 and B7).
- `tests/l5_pnp_laps/lap_battery_b7.py` (D7-S0, D7-S1, the lap-1 table) and
  `tests/l5_pnp_laps/battery_b7.py` (run, report, the drop rule, the freeze guard), tested with a fake
  runner, table-driven per item.

## To run it, once the code under test is final

1. Commit `evaluation/prereg/freeze/b7.json`:
   `{"freeze_commit": "<40 digit SHA of the last code commit>", "yaml_sha256":
   "35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de"}`. The commit must contain
   items 1 to 4 (and the fix of item 5 if one is made) and be what the car will run.
2. On the host: `python3 tests/l5_pnp_laps/battery_b7.py check-freeze` (the freeze commit is an
   ancestor of HEAD and `ros_ws/src/racer_laps`, `ros_ws/src/racer_planner` and
   `tests/l5_reactive_bench` are unchanged since it).
3. In the ros-dev container with `B4_GIT_SHA` set to that SHA: `python3
   tests/l5_pnp_laps/battery_b7.py run --out <car-data>/pnp/battery/v0.5-b7 --workers 3`, then
   `report`. The seed 51 condition a runs go first and decide the drop rule (16 of 18 instead of 22
   of 24 when a case is dropped).

D may drive at the next car session only if D7-S0 and D7-S1 pass on that run and D7-C0 passes
earlier in the same session (pre-registration section 5). Until then: no.
