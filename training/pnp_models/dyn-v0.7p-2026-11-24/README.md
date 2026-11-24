# dyn-v0.7p-2026-11-24

Prototype, unregistered (owner decision 29). Not a gate result: GP2g-R (brief 54) and every earlier
dynamics gate are untouched, and dyn-v0.1 stays the default on the car.

dyn-v0.1 nominal, unchanged, plus a residual MLP (speed and yaw, 4-32-32-2, output bounded by its
weights to |dv/dt| <= 2.0 m/s2 and |dyaw/dt| <= 0.5 rad/s), residual enabled. Validity 1.3 m/s
(the dyn-v0.1 range, not widened). Bundle schema 1.1.0, vehicle_params 0.17.0.

## Data

Every session bag with odometry since 2026-11-15, truth T2 (wheel seeded ICP), split by
`dev-split.yaml` (committed before the fit, `racer_pnp_dynamics.proto`):

- each run (`tools/pnp_data.py runs-v05`, any driver, at least 15 s and 1 m): first 75 percent
  minus a 2 s guard is train (its last 15 percent validation: early stopping, config and seed),
  last 25 percent is the development held-out part;
- 2026-11-15 session (bags 2026-11-16T*): 14 runs, PnP and excitation, its spent v0.5 truth reused;
- 2026-11-23 session (bag 2026-11-24T04-36-54, tonight's 1.3 m/s laps): 4 runs;
- 2026-11-22 bag: ONLY the first 55 percent of each run (the GP2g-R train role) is used, as train
  and validation; nothing of it is scored, so the GP2g-R held-out set stays unread. Note: the
  2026-11-23 bag is trained on here, so GP2g-C's "fresh driving after 2026-11-22" must come from a
  later session.

Train 1083 s, validation 191 s, development 343 s (712 windows of 1.0 s).

Fit: `pipeline_v02.strict_fit_stage` on dyn-v0.1 (the R-series recipe), configs one step, multi
step, multi step from one step, all capped, seeds 0 to 2. Shipping: multi_step_from_one_capped,
seed 1, validation P1 0.0735 m against dyn-v0.1's 0.1012 m. Training device cpu, float64.

## Development held-out, 1.0 s horizon (macro over direction x speed strata)

| group | windows | P1 ratio | Y1 ratio | S1 ratio |
|---|---|---|---|---|
| all | 712 | 0.779 (bootstrap 0.728 to 0.829) | 0.684 | 0.828 |
| cw | 283 | 0.785 | 0.706 | 0.853 |
| ccw | 429 | 0.767 | 0.654 | 0.787 |
| 2026-11-15 session | 566 | 0.764 | 0.670 | 0.815 |
| 2026-11-23 session (1.3 m/s laps) | 146 | 0.802 | 0.553 | 0.982 |
| start speed >= 1.0 m/s | 219 | 0.810 | 0.750 | 0.715 |

Ratios are prototype / dyn-v0.1 on identical windows (P1 0.0877 m vs 0.1126 m, Y1 0.067 rad vs
0.098 rad, S1 0.073 m/s vs 0.088 m/s). Each of the three seeds is within 0.78 to 0.80 on P1.
Residual output at or above 0.9 of its cap: 0 percent of development steps (largest |dv/dt| 1.40
m/s2, |dyaw/dt| 0.22 rad/s). These are development numbers on a split chosen tonight, scored once
after the fit; they are not the pre-registered GP2g-R comparison.

## Online speed adaptation (planner switch `dynamics_adaptation`, default off)

Replayed causally on the same development windows (`racer_pnp_dynamics.adaptation`, the C++
estimator's reference; window 0.5 s and forgetting 0.98 chosen on the validation parts):

| model | P1 ratio | Y1 ratio | S1 ratio |
|---|---|---|---|
| dyn-v0.1 + adaptation | 0.959 | 0.940 | 0.952 |
| prototype + adaptation | 0.884 | 0.693 | 0.759 |

On the prototype it improves the wheel speed prediction (S1 0.76 against 0.83) and makes the
position worse (P1 0.88 against 0.78): contract 1 integrates the wheel speed as ground speed, and
fitting the wheel speed better moves the ground position away. Keep it off with this bundle unless
the run is about the adaptation itself. A first design (per sample acceleration regression) was
replayed on the development set, made everything worse (P1 1.10 on dyn-v0.1) and was replaced;
the current numbers are therefore a second look at the same development set.

## Planner cost

`mppi_bench` (512 samples, horizon 30, speed 1.3, aarch64 container on the Mac): planner solve
p50 9.1 ms with this bundle against 0.58 ms with dyn-v0.1 (the 4-32-32-2 residual on every rollout
step). The Jetson is slower: expect the sample budget to halve samples (solve over 15 ms) on the
car; at 128 samples the bench solve is 4.6 ms. Watch `/pnp/planner/timing` and the sample count
log on the first run.

## On the car

    rsync -a /Users/cameronjim/code/car-data/pnp/models/dyn-v0.7p-2026-11-24 jetson:~/car/data/pnp/models/
    # inside car-stack: dynamics_bundle:=/workspace/data/pnp/models/dyn-v0.7p-2026-11-24
    # optional, prototype: dynamics_adaptation:=true (status /pnp/dynamics/adaptation,
    # reset: ros2 topic pub --once /pnp/dynamics/adaptation_reset std_msgs/msg/Empty {})

Weights are not in git; the bundle is under `car-data/pnp/models/` (residual_mlp.f32 sha256
4b8a8ad4b83d22063bbbc1f98b8a5aa037480cc93a9bc602ef7e9c584a2c2049).
