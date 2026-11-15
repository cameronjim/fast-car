do-not-deploy

# dyn-v0.4r-2026-11-14

R4: the yaw-only residual on dyn-v0.1, shipping seed 2.

Status: candidate, not confirmed: needs the fresh confirmation of pre-registration v0.4 section 2.5.

### GP2d-R yaw-only residual: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio R / nominal | 0.951719 | <= 0.95 | FAIL |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.964552 | < 1 | PASS |  |
| 2a Y1 ratio R / nominal | 0.548404 | <= 1 | PASS |  |
| 2b S1(R) minus S1(nominal) | 0 | <= 1e-06 | PASS | absolute difference in m/s |
| 3 seed 0 P1 ratio to nominal | 0.951438 | <= 1 | PASS |  |
| 3 seed 1 P1 ratio to nominal | 0.951343 | <= 1 | PASS |  |
| 3 seed 2 P1 ratio to nominal | 0.951719 | <= 1 | PASS |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.991184 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.995744 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.851947 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.955615 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.974512 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85+ position RMSE ratio | 0.957011 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.300 m/s |
| 6 Yp ratio R / nominal | 0.707254 | <= 1 | PASS |  |

## Data

- train role under T2, split 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63, addendum ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824

Training device: cpu, float64 (Apple MPS is available but has no float64 and the residual recipe is float64).
