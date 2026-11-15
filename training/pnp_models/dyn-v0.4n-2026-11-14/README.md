do-not-deploy

# dyn-v0.4n-2026-11-14

N4: the classical refit of dyn-v0.1 on the rollout objective.

Status: candidate, not confirmed: needs the fresh confirmation of pre-registration v0.4 section 2.5.

### GP2d-N classical refit: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio N4 / v0.1 | 0.968161 | <= 1 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.980003 | < 1 | PASS |  |
| 2a Y1 ratio N4 / v0.1 | 0.653842 | <= 1 | PASS |  |
| 2b S1 ratio N4 / v0.1 | 1.00256 | <= 1 | FAIL |  |
| 3 stratum ccw:0.25-0.6 position RMSE ratio | 0.995187 | <= 1.05 | PASS |  |
| 3 stratum ccw:0.6-0.85 position RMSE ratio | 1.01047 | <= 1.05 | PASS |  |
| 3 stratum ccw:0.85+ position RMSE ratio | 0.900398 | <= 1.05 | PASS |  |
| 3 stratum cw:0.25-0.6 position RMSE ratio | 0.958478 | <= 1.05 | PASS |  |
| 3 stratum cw:0.6-0.85 position RMSE ratio | 0.985764 | <= 1.05 | PASS |  |
| 3 stratum cw:0.85+ position RMSE ratio | 0.971264 | <= 1.05 | PASS |  |
| 4 refit parameters on a bound | 1 | <= 0 | FAIL | understeer_s2_per_m2 |
| 5 Yp ratio N4 / v0.1 | 0.925342 | <= 1 | PASS |  |

## Data

- train role under T2, split 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63, addendum ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824
