do-not-deploy; contract 2: no planner on main can load this; candidate only

# dyn-v0.4t-2026-11-14

T4: dyn-v0.1 plus the ground motion table (step contract 2).

Status: candidate, not confirmed: needs the fresh confirmation of pre-registration v0.4 section 2.5.

### GP2e-T ground motion table: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio T4 / nominal | 0.62654 | <= 0.95 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.663672 | < 1 | PASS |  |
| 2a A1 ratio T4 / nominal | 0.597678 | <= 1 | PASS |  |
| 2b C1 ratio T4 / nominal | 0.680265 | <= 1 | PASS |  |
| 3a Y1(T4) minus Y1(nominal) | 0 | <= 1e-06 | PASS | absolute rad |
| 3b S1(T4) minus S1(nominal) | 0 | <= 1e-06 | PASS | absolute m/s |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.560937 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.642149 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.763553 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.587841 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.560879 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85+ position RMSE ratio | 0.630477 | <= 1.05 | PASS |  |
| 5 supported table nodes on a bound | 2 | <= 0 | FAIL | s+0.0000/v0.25, s+0.6667/v1.5 |
| 6 P1 ratio T4 / nominal under T2s | 0.613508 | <= 1 | PASS |  |
| 7 Pp ratio T4 / nominal | 0.80947 | <= 1 | PASS |  |

Validity: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at min(rule, 1.3).

- bundle model passed its gate: 0 >= 1 FAIL
- fast ccw: P1 ratio to dyn-v0.1 (36 windows): 0.715384 <= 1 PASS
- fast cw: windows with start speed >= 1.15 m/s: 24 >= 30 FAIL the fast item cannot be applied

## Data

- train role under T2, split 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63, addendum ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824
