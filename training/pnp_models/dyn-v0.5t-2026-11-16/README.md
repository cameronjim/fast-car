do-not-deploy; contract 2.1: no planner on main can load this; candidate only

# dyn-v0.5t-2026-11-16

T5: dyn-v0.1 plus the alpha and gamma tables (step contract 2.1).

Status: candidate, not confirmed: needs contract 2.1 in the planner and the fresh confirmation GP2f-C. Schema 2.1.0 is the owner's decision; until then the bundle is do-not-deploy whatever the gates say.

### GP2f-S the tables may be trusted: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1 supported nodes within 0.005 of a bound (alpha and gamma) | 0 | <= 0 | PASS | none |
| 2a split half: largest alpha difference at a supported node | 0.0316512 | <= 0.05 | PASS |  |
| 2b split half: largest gamma difference at a supported node | 0.0662605 | <= 0.08 | PASS |  |
| 3 share of held-out rollout steps in the cell of a supported node | 0.908482 | >= 0.95 | FAIL |  |

### GP2f-T T5 is a candidate: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0 GP2f-S passed | 0 | >= 1 | FAIL |  |
| 1a P1 ratio T5 / dyn-v0.1 | 0.742838 | <= 0.9 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.827618 | < 0.95 | PASS |  |
| 2a A1 ratio | 0.662265 | <= 1 | PASS |  |
| 2b C1 ratio | 0.844292 | <= 1 | PASS |  |
| 3a Y1 ratio | 0.830531 | <= 0.9 | PASS |  |
| 3b S1(T5) minus S1(dyn-v0.1) | 0 | <= 1e-06 | PASS | absolute m/s |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.77571 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.810501 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.691541 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.721243 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.965244 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85-1.25 position RMSE ratio | 0.743053 | <= 1.05 | PASS |  |
| 4 stratum cw:1.25+ position RMSE ratio | 0.623262 | <= 1.05 | PASS |  |
| 5 ccw P1 ratio | 0.756885 | <= 0.95 | PASS |  |
| 5 ccw Y1 ratio | 0.818689 | <= 1 | PASS |  |
| 5 cw P1 ratio | 0.736585 | <= 0.95 | PASS |  |
| 5 cw Y1 ratio | 0.837798 | <= 1 | PASS |  |
| 6 ccw absolute yaw rate bias (rad/s) | 0.0206483 | <= 0.03 | PASS | at most 0.03 rad/s or 0.5 of the dyn-v0.1 bias -0.0485 rad/s |
| 6 cw absolute yaw rate bias (rad/s) | 0.008956 | <= 0.03 | PASS | at most 0.03 rad/s or 0.5 of the dyn-v0.1 bias -0.0279 rad/s |
| 7 near-limit left position RMSE ratio | 0 | at least 30 | CANNOT BE APPLIED | cannot be applied: 0 windows, at least 30 needed |
| 7 near-limit right position RMSE ratio (164 windows) | 0.689369 | <= 1 | PASS |  |
| 8 transition position RMSE ratio (226 windows) | 0.719066 | <= 1 | PASS |  |
| 9 P1 ratio under T2s | 0.735399 | <= 1 | PASS |  |
| 10a Pp ratio on G1b and T2b | 0.679817 | <= 1 | PASS |  |
| 10b Yp ratio on G1b and T2b | 0.777142 | <= 1 | PASS |  |

## Data

- train role under T2, session split 4824ec40750db64d2de98b16259149290238a681d5d51f86e80313333af4cbcd, constants 2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82
- selection: the 2026-11-14 held-out segments, never fitted on
