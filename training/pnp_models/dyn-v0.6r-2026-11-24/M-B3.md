# M-B3: error budget of dyn-v0.1 on the train windows at 1 s

Brief 54, pnp-v0.6-dynamics-yaw.yaml `measurements.m_b3` (the v0.4 section 2.2 method). A measurement on the TRAIN data of this attempt only (train and the internal validation part of every train role, truth T2), committed before any R6 fit. It decides nothing.

- Windows: 9115 (train and internal validation parts of every v0.5 train role and of the 2026-11-22 train parts), strata ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85-1.25, ccw:1.25+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85-1.25, cw:1.25+, legacy:0.25-0.85, legacy:0.85+; the direction of a session window is its measured T2 heading.
- Constants file evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml, SHA-256 db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c.
- Session file evaluation/prereg/sessions/2026-11-22-dynamics-v06.yaml, SHA-256 6b791ac5bcc5c0b150b86420574f4d704453963e9d1fdb1502aa09f0447516f5.
- Model: dyn-v0.1 nominal read from its committed pointer manifest, residual disabled.

Along and across are the components of the position error in the true heading at the window start (model minus truth). Per stratum position^2 = along^2 + across^2; the macro row is the mean of the per stratum RMSEs and does not add in quadrature. Oracles: perfect yaw puts the true yaw into the rollout at every step, perfect speed the true wheel speed. Ratio = oracle P1 over the model's P1.

| stratum | windows | P1 (m) | along (m) | across (m) | perfect yaw P1 | perfect speed P1 | ratio perfect yaw | ratio perfect speed |
|---|---|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 3333 | 0.0656 | 0.0592 | 0.0283 | 0.0649 | 0.0481 | 0.990 | 0.733 |
| ccw:0.6-0.85 | 339 | 0.0811 | 0.0684 | 0.0436 | 0.0792 | 0.0684 | 0.976 | 0.843 |
| ccw:0.85-1.25 | 1665 | 0.0858 | 0.0741 | 0.0432 | 0.0767 | 0.0853 | 0.894 | 0.994 |
| ccw:1.25+ | 122 | 0.1752 | 0.1552 | 0.0812 | 0.1443 | 0.1433 | 0.824 | 0.818 |
| cw:0.25-0.6 | 1414 | 0.0821 | 0.0692 | 0.0442 | 0.0800 | 0.0699 | 0.974 | 0.851 |
| cw:0.6-0.85 | 313 | 0.1009 | 0.0720 | 0.0707 | 0.0957 | 0.0949 | 0.948 | 0.940 |
| cw:0.85-1.25 | 472 | 0.1430 | 0.0910 | 0.1103 | 0.1329 | 0.1406 | 0.929 | 0.983 |
| cw:1.25+ | 56 | 0.2203 | 0.1601 | 0.1513 | 0.2142 | 0.1835 | 0.972 | 0.833 |
| legacy:0.25-0.85 | 1268 | 0.0578 | 0.0516 | 0.0260 | 0.0570 | 0.0505 | 0.986 | 0.874 |
| legacy:0.85+ | 133 | 0.1447 | 0.0987 | 0.1058 | 0.1312 | 0.1580 | 0.907 | 1.092 |
| macro | 9115 | 0.1156 | 0.0900 | 0.0705 | 0.1076 | 0.1042 | 0.930 | 0.901 |

Perfect yaw ratio (macro): 0.930.

The perfect yaw ratio is not above 0.95: the registered sentence is not written.
