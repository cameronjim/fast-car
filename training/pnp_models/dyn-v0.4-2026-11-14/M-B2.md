# M-B2: direction asymmetry, train segments of 2026-11-14, against dyn-v0.1

Pre-registration v0.4 section 2.3. A measurement on the TRAIN segments of 2026-11-14 only, produced and committed before any v0.4 fit. No threshold, it decides nothing.

- Windows: 1711 on the train segments of 2026-11-14 only.
- Lap radius from wheel distance and the T2 yaw of each segment (brief 36's lap reference was not on main).

Mean yaw rate columns are the 1.0 s window yaw change divided by 1.0 s (rad/s); bias is model minus truth (T2), RMS is of the same difference. Speed bins are the start speed bins of the strata, unmerged, and bin the 50 Hz samples by their own wheel speed for the first two columns.

| direction | speed bin | seconds | mean wheel speed | steering at limit | windows | truth yaw rate | model yaw rate | bias | RMS |
|---|---|---|---|---|---|---|---|---|---|
| ccw | 0.25-0.6 | 107.3 | 0.452 | 0.000 | 398 | 0.446 | 0.469 | 0.023 | 0.048 |
| ccw | 0.6-0.85 | 57.2 | 0.676 | 0.000 | 181 | 0.788 | 0.777 | -0.011 | 0.059 |
| ccw | 0.85-1.25 | 175.6 | 0.969 | 0.000 | 582 | 1.146 | 1.076 | -0.069 | 0.093 |
| ccw | 1.25+ | 2.7 | 1.280 | 0.000 | 7 | 1.164 | 1.040 | -0.124 | 0.135 |
| cw | 0.25-0.6 | 45.4 | 0.451 | 0.033 | 153 | -0.462 | -0.525 | -0.063 | 0.071 |
| cw | 0.6-0.85 | 44.7 | 0.673 | 0.040 | 157 | -0.743 | -0.806 | -0.063 | 0.083 |
| cw | 0.85-1.25 | 65.2 | 0.971 | 0.165 | 223 | -1.042 | -1.104 | -0.062 | 0.093 |
| cw | 1.25+ | 3.0 | 1.285 | 0.000 | 10 | -1.170 | -1.232 | -0.062 | 0.106 |

Lap radius per segment (wheel distance over the yaw turned, T2 intervals that matched):

| segment | direction | cap (m/s) | wheel distance (m) | yaw turned (rad) | radius (m) |
|---|---|---|---|---|---|
| L0 | ccw | 1.0 | 88.00 | 105.61 | 0.833 |
| L0 | ccw | 1.0 | 15.89 | 18.90 | 0.841 |
| X0 | ccw | 1.0 | 2.83 | 3.48 | 0.814 |
| X0 | ccw | 1.0 | 11.04 | 10.73 | 1.029 |
| X0 | ccw | 1.0 | 5.68 | 5.33 | 1.066 |
| X0 | ccw | 1.0 | 4.83 | 4.81 | 1.005 |
| X0 | ccw | 1.0 | 8.57 | 9.26 | 0.926 |
| X0 | ccw | 1.0 | 14.01 | 17.20 | 0.815 |
| X1 | cw | 1.0 | 45.06 | -49.45 | 0.911 |
| X1 | cw | 1.0 | 7.97 | -8.60 | 0.927 |
| X2a | cw | 0.5 | 14.30 | -14.10 | 1.014 |
| X2a | cw | 0.5 | 2.49 | -2.39 | 1.043 |
| X3a | ccw | 0.5 | 34.20 | 32.95 | 1.038 |
| X3a | ccw | 0.5 | 6.12 | 5.70 | 1.073 |
| X4a | ccw | 0.7 | 22.89 | 26.47 | 0.865 |
| X4a | ccw | 0.7 | 4.20 | 5.01 | 0.837 |
| X5a | cw | 0.7 | 23.17 | -24.81 | 0.934 |
| X5a | cw | 0.7 | 4.19 | -4.57 | 0.917 |
| X6a | cw | 1.3 | 16.33 | -17.74 | 0.920 |
| X6a | cw | 1.3 | 3.15 | -3.63 | 0.869 |
| X7a | ccw | 1.3 | 33.73 | 40.64 | 0.830 |
| X7a | ccw | 1.3 | 5.87 | 7.01 | 0.837 |
