# dyn-v0.1-2026-11-12

Fitted dynamics bundle for the Plug-and-Play planner. Weights are not committed; the bundle lives under car-data/pnp/models/.

vehicle_params 0.14.0, sysid session none-preliminary. Validity: v up to 1.3 m/s.

## Fitted nominal parameters

| parameter | value |
|---|---|
| speed_gain | 0.9477 |
| speed_deadband_mps | 0.1600 |
| speed_tau_s | 0.2754 |
| accel_max_mps2 | 2.0000 |
| decel_max_mps2 | 0.7528 |
| steer_gain_left | 0.9166 |
| steer_gain_right | 1.0115 |
| steer_offset_rad | 0.0152 |
| steer_tau_s | 0.0279 |
| command_delay_s | 0.0000 |
| understeer_s2_per_m2 | 0.1094 |

Command delay chosen by: held-out windows (speed RMSE at the chosen delay 0.0838 m/s, 1 s windows).

Held-out error: wheel speed 0.0831 m/s, yaw change 0.0081 rad per 0.1 s (n/a means the split holds no data of that kind).

Steering gains tied (one gain, offset 0): False.

Yaw channel trained on: odom_only; blocked CV yaw RMSE per option {"odom_only": 0.006221863294060304, "all": 0.006270418771007515}.

Training counts: {"speed_windows_train": 2063, "speed_windows_heldout": 867, "yaw_intervals_train": 4844, "yaw_intervals_heldout": 3608, "yaw_left_train": 2134, "yaw_right_train": 2619}.

Flags (parameters on a bound, or that the data cannot move):

- accel_max_mps2: not supported while rolling (launch windows only), kept at its start

## Held-out error, open loop

556 windows. Cells are wheel speed RMSE (m/s) / yaw RMSE (rad) / position RMSE (m) at 0.5 s, 1.0 s, 1.5 s.

| variant | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|
| placeholder defaults | 0.068 / 0.139 / 0.097 | 0.072 / 0.261 / 0.193 | 0.078 / 0.369 / 0.289 |
| fitted nominal (shipped) | 0.070 / 0.085 / 0.090 | 0.073 / 0.160 / 0.169 | 0.078 / 0.225 / 0.234 |
| fitted nominal, other side tying (not shipped) | 0.070 / 0.081 / 0.090 | 0.073 / 0.154 / 0.169 | 0.078 / 0.217 / 0.234 |
| fitted + residual (seed 0) | 0.074 / 0.070 / 0.090 | 0.077 / 0.127 / 0.167 | 0.082 / 0.172 / 0.231 |
| fitted + residual (seed 1) | 0.072 / 0.072 / 0.091 | 0.074 / 0.131 / 0.172 | 0.080 / 0.178 / 0.239 |
| fitted + residual (seed 2) | 0.073 / 0.072 / 0.093 | 0.076 / 0.135 / 0.177 | 0.082 / 0.184 / 0.248 |

## Planner geometry

Minimum path radius at the command limits (exported vehicle): left 0.726 m, right 0.703 m; kinematic at the steering limit 0.685 m.

Held-out yaw RMSE per 0.1 s interval, shipped nominal: driven by ICP ground speed 0.0086 rad, driven by wheel speed 0.0179 rad.

Acceleration limit rule: {"rolling_windows": 1772, "rmse_fitted": 0.04017561682578197, "rmse_start": 0.039943659267508515, "kept_start": true, "fitted_value": 0.7774792361632757} (speed RMSE on the rolling windows with the fitted value and with the start value; the start value is kept when it is not worse).

## Dead zone

Shipped `speed_deadband_mps`: 0.16 m/s (median command at which the wheels turned over the resolved launches from rest, 18 launches).

- (a) Launches from rest, command at which the wheel speed first exceeds 0.03 m/s: n 18, median 0.250, 10th to 90th percentile 0.136 to 0.589 m/s. Launches whose command interval is resolved: n 8, median 0.156 m/s.
- (b) Steady state: the largest command bin with the wheels still ends at 0.08 m/s, the smallest bin with a wheel speed above 0.05 m/s starts at 0.38 m/s (bins with at least 20 samples; teleop and gap follow hold no steady command in between). Shipped value inside the bracket: True.
- Held-out speed RMSE over all windows is smallest at a dead zone of n/a m/s (n/a m/s) against 0.0831 m/s at the shipped value.
- Rolling below the dead zone (windows that start at 0.15 m/s or more and are commanded between 0.02 m/s and the dead zone throughout): 0 windows, median start n/a m/s, median end n/a m/s, share still turning at the end n/a, model median end n/a m/s.

| bag | bag time (s) | command at launch (m/s) | highest command while still (m/s) |
|---|---|---|---|
| 2026-11-11T03-54-11.. ar_teleop_cw | 222 | 0.572 | 0.572 |
| 2026-11-11T03-54-11.. ar_teleop_cw | 851 | 0.509 | 0.509 |
| 2026-11-11T03-54-11.. ar_teleop_cw | 914 | 0.149 | 0.068 |
| 2026-11-11T04-20-19.. ar_teleop_v3 | 132 | 0.149 | 0.068 |
| 2026-11-11T04-20-19.. ar_teleop_v3 | 656 | 0.149 | 0.068 |
| 2026-11-11T04-20-19.. ar_teleop_v3 | 1251 | 0.628 | 0.628 |
| 2026-11-11T04-20-19.. ar_teleop_v3 | 1377 | 0.163 | 0.081 |
| 2026-11-12T03-04-56.. 6_car_teleop | 200 | 0.186 | 0.076 |
| 2026-11-12T03-04-56.. 6_car_teleop | 297 | 0.216 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 475 | 0.330 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1088 | 0.238 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1098 | 0.072 | 0.087 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1115 | 0.263 | 0.285 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1132 | 0.265 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1140 | 0.105 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1175 | 0.261 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1733 | 0.360 | 0.000 |
| 2026-11-12T03-04-56.. 6_car_teleop | 1748 | 1.751 | 1.766 |

## Identification

Not identified by the data (shipped as fitted, delay at the grid minimum):

- command_delay_s: on its lower bound 0
- speed_deadband_mps: profile flat within 2% of its minimum over 90% of its range (linear measure)

Steering lag cross-check (peak of the cross-correlation of the steering command with ICP path curvature, speed over 0.4 m/s, independent of the fit): 0.057 s (correlation 0.99, 4742 intervals) against fitted delay plus steer_tau 0.028 s; difference +0.029 s.

Profiles (held-out RMSE against the parameter, the others refit): profiles.png.

command_delay_s (dead zone at its chosen value, continuous speed parameters refit)

| value | RMSE | over the minimum |
|---|---|---|
| 0.000 | 0.0838 | +0.0% |
| 0.020 | 0.0845 | +0.8% |
| 0.040 | 0.1030 | +23.0% |
| 0.060 | 0.1025 | +22.4% |
| 0.080 | 0.1003 | +19.8% |
| 0.100 | 0.0995 | +18.8% |
| 0.120 | 0.0929 | +10.9% |
| 0.140 | 0.0935 | +11.6% |
| 0.160 | 0.0940 | +12.2% |
| 0.180 | 0.0958 | +14.4% |
| 0.200 | 0.0960 | +14.7% |
| 0.220 | 0.0976 | +16.5% |
| 0.240 | 0.0989 | +18.1% |
| 0.260 | 0.1024 | +22.2% |
| 0.280 | 0.1054 | +25.8% |
| 0.300 | 0.1089 | +30.1% |

speed_tau_s (gain, accel and decel refit; delay and dead zone at chosen values)

| value | RMSE | over the minimum |
|---|---|---|
| 0.005 | 0.1054 | +33.8% |
| 0.010 | 0.1054 | +33.8% |
| 0.020 | 0.1048 | +33.1% |
| 0.050 | 0.1014 | +28.7% |
| 0.100 | 0.0959 | +21.7% |
| 0.200 | 0.0866 | +9.9% |
| 0.300 | 0.0788 | +0.0% |
| 0.500 | 0.0879 | +11.6% |
| 0.750 | 0.1023 | +29.9% |
| 1.000 | 0.1141 | +44.8% |

steer_tau_s (yaw gains and understeer refit (yaw RMSE per 0.1 s interval))

| value | RMSE | over the minimum |
|---|---|---|
| 0.005 | 0.0080 | +0.0% |
| 0.010 | 0.0080 | +0.0% |
| 0.020 | 0.0080 | +0.3% |
| 0.050 | 0.0084 | +5.1% |
| 0.100 | 0.0100 | +24.2% |
| 0.200 | 0.0137 | +70.8% |
| 0.300 | 0.0166 | +106.5% |
| 0.500 | 0.0201 | +150.3% |
| 0.750 | 0.0224 | +179.6% |
| 1.000 | 0.0237 | +195.9% |

speed_deadband_mps (held-out RMSE over all windows with the dead zone varied and every other parameter fixed at its shipped value; the shipped value is the launch side median (deadband_choice), not this argmin)

| value | RMSE | over the minimum |
|---|---|---|
| 0.000 | 0.0832 | +0.1% |
| 0.010 | 0.0832 | +0.1% |
| 0.020 | 0.0832 | +0.1% |
| 0.030 | 0.0832 | +0.1% |
| 0.040 | 0.0832 | +0.1% |
| 0.050 | 0.0832 | +0.1% |
| 0.060 | 0.0832 | +0.1% |
| 0.070 | 0.0832 | +0.1% |
| 0.080 | 0.0832 | +0.1% |
| 0.090 | 0.0832 | +0.1% |
| 0.100 | 0.0832 | +0.1% |
| 0.110 | 0.0831 | +0.0% |
| 0.120 | 0.0831 | +0.0% |
| 0.130 | 0.0831 | +0.0% |
| 0.140 | 0.0831 | +0.0% |
| 0.150 | 0.0831 | +0.0% |
| 0.160 | 0.0831 | +0.0% |
| 0.170 | 0.0831 | +0.0% |
| 0.180 | 0.0831 | +0.0% |
| 0.190 | 0.0831 | +0.0% |
| 0.200 | 0.0831 | +0.0% |
| 0.210 | 0.0831 | +0.0% |
| 0.220 | 0.0831 | +0.0% |
| 0.230 | 0.0831 | +0.0% |
| 0.240 | 0.0831 | +0.0% |
| 0.250 | 0.0831 | +0.0% |
| 0.260 | 0.0831 | +0.0% |
| 0.270 | 0.0831 | +0.0% |
| 0.280 | 0.0831 | +0.0% |
| 0.290 | 0.0832 | +0.1% |
| 0.300 | 0.0831 | +0.0% |
| 0.310 | 0.0831 | +0.0% |
| 0.320 | 0.0833 | +0.2% |
| 0.330 | 0.0833 | +0.2% |
| 0.340 | 0.0834 | +0.3% |
| 0.350 | 0.0834 | +0.4% |
| 0.360 | 0.0836 | +0.6% |
| 0.370 | 0.0973 | +17.1% |
| 0.380 | 0.0974 | +17.2% |
| 0.390 | 0.1484 | +78.6% |
| 0.400 | 0.1607 | +93.4% |

## Prediction error on the planner runs (bag 2026-11-12T03-04-56)

Open loop from the recorded /drive commands, speed against the wheel, pose against chained ICP; windows that start standing included. Cells: wheel speed RMSE (m/s) / yaw RMSE (rad) / position RMSE (m).

| variant | windows | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|---|
| old_dyn-v0-2026-11-12 | 122 | 0.098 / 0.026 / 0.046 | 0.094 / 0.052 / 0.095 | 0.088 / 0.078 / 0.141 |
| new_nominal | 122 | 0.075 / 0.023 / 0.032 | 0.077 / 0.046 / 0.072 | 0.070 / 0.069 / 0.111 |
| new_shipped | 122 | 0.075 / 0.023 / 0.032 | 0.077 / 0.046 / 0.072 | 0.070 / 0.069 / 0.111 |

## Residual verdict

Median relative improvement at 1.0 s: speed -4.6%, yaw +18.1% (rule: both at least 5 percent). Residual NOT enabled: zeros in residual_mlp.f32.

## Decisions and findings

- Split (brief 14): within every bag the first 70 percent of its moving data trains and the last 30 percent is held out; no bag is held out whole and there is no validation split. The delay is therefore scored on the held-out part, which makes the held-out numbers below in-sample for the delay and the dead zone profile. The bags are nested copies of three recordings (only the longest of each group is used). The two 2026-11-12 bags are in current units and are not converted; every older bag is converted once.
- Windows where every command was at least 0.5 m/s and the wheels never turned (the car held, blocked, or the layers downstream of /drive not passing the command) are dropped from the speed fit and the evaluation: a 1.75 m/s gap follow command with the car standing was 90 percent of the squared speed error.
- The dead zone shipped is the median command of the resolved launches from rest (the launch side value), not the argmin of a speed error curve: the held-out windows hold almost no command inside the dead zone band, so held-out error cannot identify it (its profile table moves with the refit, not with the dead zone), and the error over windows that start at rest prefers 0.4 m/s because of windows where the car was held still under 0.36 to 0.6 m/s. A launch whose command jumped from nothing gives only an upper bound on the threshold and is not in the shipped estimate; the launch table lists every launch. The planner adds its own margin.
- A parameter on a bound or with a flat profile is listed under unidentified and is shipped as fitted, except the delay: an unidentified delay ships the grid minimum (0.00 s). The speed time constant, delay and the accel limit are an effective description of the drivetrain, not measured actuator constants.
- The acceleration limit is kept at its start value unless the fitted value fits the rolling windows (start speed and every command at least 0.15 m/s) better. The yaw step is driven by ICP ground speed (steering gain = path curvature per command); ground speed falls to about 0.8 of wheel speed at full lock (table below), so in a full-lock turn the model over-predicts distance and yaw rate per second, the conservative side against walls.
- Windows with a reverse command or a wheel speed below -0.03 m/s are skipped (the model has v >= 0). Converted steering reaches 0.504 rad to the left, beyond the planning limit: the fit and the held-out numbers clamp at the physical stops, the exported bundle uses the vehicle_params limits.
- Gains are tied (one gain, offset 0) when a side has fewer than 300 training turning intervals. The residual ships enabled only if the median of its relative improvement over the fitted nominal at 1.0 s is at least 5 percent on both speed and yaw.

## Ground speed against wheel speed

| steering command (rad) | n | median ICP ground speed / wheel speed | p25 | p75 |
|---|---|---|---|---|
| 0.00 to 0.05 | 317 | 0.935 | 0.904 | 0.963 |
| 0.05 to 0.10 | 410 | 0.919 | 0.886 | 0.948 |
| 0.10 to 0.20 | 981 | 0.900 | 0.854 | 0.941 |
| 0.20 to 0.30 | 888 | 0.858 | 0.783 | 0.918 |
| 0.30 to 0.40 | 860 | 0.820 | 0.740 | 0.888 |
| 0.40 to 0.55 | 3658 | 0.783 | 0.698 | 0.845 |

## Data

- bag 2026-10-24T23-13-18_car_teleop: 806 s, odometry no
- bag 2026-10-25T19-27-08_car_teleop: 409 s, odometry no
- bag 2026-11-11T03-54-11_car_teleop_cw: 1020 s, odometry yes
- bag 2026-11-11T04-20-19_car_teleop_v3: 1557 s, odometry yes
- bag 2026-11-12T02-57-50_car_teleop: 415 s, odometry yes
- bag 2026-11-12T03-04-56_car_teleop: 1988 s, odometry yes
- dropped as a nested copy of a longer bag: 2026-11-11T03-54-11_car_teleop
- dropped as a nested copy of a longer bag: 2026-11-11T04-20-19_car_teleop
- dropped as a nested copy of a longer bag: 2026-11-11T04-20-19_car_teleop_latest

Seconds of driving per bag (train / val / held out):

- 2026-10-24T23-13-18_car_teleop: 313.5 / 0.0 / 132.8
- 2026-10-25T19-27-08_car_teleop: 44.8 / 0.0 / 16.4
- 2026-11-11T03-54-11_car_teleop_cw: 178.0 / 0.0 / 69.6
- 2026-11-11T04-20-19_car_teleop_v3: 175.1 / 0.0 / 71.2
- 2026-11-12T03-04-56_car_teleop: 223.1 / 0.0 / 97.4
