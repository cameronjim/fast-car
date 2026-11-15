# M-B3: error budget of dyn-v0.1 on the train windows at 1 s

Pre-registration v0.4 section 2.2. A measurement on TRAIN data only (train and the internal validation part of the train segments, truth T2), produced and committed before any v0.4 fit. It decides nothing.

- Windows: 7161 (train and the internal validation part of the train segments of the v0.2 split, the legacy train part and the 2026-11-14 train segments), strata ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85-1.25, ccw:1.25+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+, legacy:0.25-0.85, legacy:0.85+.
- Constants file evaluation/prereg/pnp-v0.4.yaml, SHA-256 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63.
- Model: dyn-v0.1 nominal read from its committed pointer manifest, residual disabled.

Along and across are the components of the position error in the true heading at the window start (model minus truth). Per stratum position^2 = along^2 + across^2; the macro row is the mean of the per stratum RMSEs and does not add in quadrature. Oracles: perfect yaw puts the true yaw into the rollout at every step, perfect speed the true wheel speed. Ratio = oracle P1 over the model's P1.

| stratum | windows | P1 (m) | along (m) | across (m) | perfect yaw P1 | perfect speed P1 | ratio perfect yaw | ratio perfect speed |
|---|---|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 3167 | 0.0658 | 0.0596 | 0.0280 | 0.0653 | 0.0480 | 0.991 | 0.729 |
| ccw:0.6-0.85 | 188 | 0.0700 | 0.0585 | 0.0384 | 0.0698 | 0.0658 | 0.998 | 0.940 |
| ccw:0.85-1.25 | 908 | 0.0819 | 0.0703 | 0.0420 | 0.0696 | 0.0856 | 0.849 | 1.045 |
| ccw:1.25+ | 105 | 0.1481 | 0.1336 | 0.0639 | 0.1032 | 0.1418 | 0.696 | 0.957 |
| cw:0.25-0.6 | 931 | 0.0782 | 0.0675 | 0.0396 | 0.0762 | 0.0645 | 0.974 | 0.825 |
| cw:0.6-0.85 | 163 | 0.0982 | 0.0658 | 0.0728 | 0.0919 | 0.0959 | 0.936 | 0.977 |
| cw:0.85+ | 298 | 0.1335 | 0.0784 | 0.1080 | 0.1220 | 0.1423 | 0.914 | 1.066 |
| legacy:0.25-0.85 | 1268 | 0.0578 | 0.0516 | 0.0260 | 0.0570 | 0.0505 | 0.986 | 0.874 |
| legacy:0.85+ | 133 | 0.1447 | 0.0987 | 0.1058 | 0.1312 | 0.1580 | 0.907 | 1.092 |
| macro | 7161 | 0.0976 | 0.0760 | 0.0583 | 0.0873 | 0.0947 | 0.895 | 0.971 |

Perfect yaw ratio (macro): 0.895.

The perfect yaw ratio is not above 0.95: the pre-registered sentence about item 1 of GP2d-R is not written.
