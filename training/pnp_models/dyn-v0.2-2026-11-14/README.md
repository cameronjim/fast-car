not better than dyn-v0.1 on held-out, do not deploy

# dyn-v0.2-2026-11-14

Refit of the identified bicycle model on the 2026-11-13 bags with the pre-registered split (docs/notes/preregistration-pnp-v0.2.md section 3, split file SHA-256 `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`). Weights are not committed; the bundle lives under car-data/pnp/models/.

- 3.1 refit ships: **False**. 3.2 residual ships enabled: **True**.
- vehicle_params 0.17.0 / none-preliminary; validity v up to 1.45 m/s.

## Fitted nominal parameters

| parameter | dyn-v0.2 | dyn-v0.1 |
|---|---|---|
| speed_gain | 0.9313 | 0.9482 |
| speed_deadband_mps | 0.3400 | 0.2200 |
| speed_tau_s | 0.1320 | 0.2246 |
| accel_max_mps2 | 1.0022 | 2.0000 |
| decel_max_mps2 | 0.3745 | 0.6731 |
| steer_gain_left | 0.9346 | 0.9166 |
| steer_gain_right | 0.9832 | 1.0115 |
| steer_offset_rad | 0.0097 | 0.0152 |
| steer_tau_s | 0.0228 | 0.0279 |
| command_delay_s | 0.0000 | 0.0000 |
| understeer_s2_per_m2 | 0.0555 | 0.1094 |

Unidentified (on a bound or a flat profile):

- command_delay_s: on its lower bound 0

## Addendum

Addendum 2026-11-13, written before stage 2 scoring. Segment T3b (bag 2026-11-13T23-27-23, 1312.1 to 1347.6 s) has no driving: the car is at rest there (speed command 0, wheel speed 0), so it contributes no windows. Held-out driving is 403 s, not the 441 s of the pre-registration. Held-out windows (1453 in all, counted before any model was scored): ccw 641 / 93 / 46 / 1 and cw 527 / 91 / 51 / 3 in the bins 0.25-0.6 / 0.6-0.85 / 0.85-1.25 / 1.25+. The 1.25+ bins are merged into 0.85-1.25 as the rule says (ccw 1 window, cw 3 windows), so no stratum above 1.25 m/s exists: the fast held-out data is too thin to decide anything about speeds above 1.25 m/s. Strata after merging: ccw 0.25-0.6, 0.6-0.85, 0.85+; cw 0.25-0.6, 0.6-0.85, 0.85+. The pre-registration and the split file are unchanged.

## 3.1 Refit decision (held-out segments)

| item | value | threshold | result |
|---|---|---|---|
| P1 ratio v0.2 / v0.1 | 0.9660 | <= 1 | pass |
| S1 ratio v0.2 / v0.1 | 1.0729 | <= 1.05 | FAIL |
| Y1 ratio v0.2 / v0.1 | 1.0550 | <= 1.05 | FAIL |

Gate 3.1 refit: FAIL.

Macro RMSE (mean over strata of the per stratum RMSE) at each horizon, position (m) / yaw (rad) / wheel speed (m/s):

| model | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|
| dyn-v0.1-2026-11-12 | 0.075 / 0.071 / 0.037 | 0.148 / 0.140 / 0.037 | 0.218 / 0.206 / 0.037 |
| dyn-v0.2 nominal | 0.072 / 0.076 / 0.041 | 0.143 / 0.147 / 0.040 | 0.212 / 0.217 / 0.040 |
| dyn-v0.2 + residual seed 0 | 0.072 / 0.032 / 0.036 | 0.140 / 0.052 / 0.036 | 0.204 / 0.068 / 0.036 |
| dyn-v0.2 + residual seed 1 | 0.069 / 0.036 / 0.036 | 0.134 / 0.057 / 0.036 | 0.195 / 0.077 / 0.036 |
| dyn-v0.2 + residual seed 2 | 0.070 / 0.037 / 0.035 | 0.134 / 0.061 / 0.035 | 0.195 / 0.083 / 0.037 |

Paired block bootstrap (5.0 s blocks, 2000 resamples, seed 0), P1 ratio dyn-v0.2 nominal over dyn-v0.1: 0.9660, 95 percent interval [0.9496, 0.9847].

## 3.2 GP2a residual decision (held-out segments)

Shipping config `multi_step_from_one_wd1e-2`, seed 2 (lowest validation P1, fixed before held-out was read).

| item | value | threshold | result |
|---|---|---|---|
| 1a P1 ratio residual / nominal | 0.9388 | <= 0.95 | pass |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.9632 | < 1 | pass |
| 2a Y1 ratio residual / nominal | 0.4140 | <= 1 | pass |
| 2b S1 ratio residual / nominal | 0.8864 | <= 1.02 | pass |
| 3 seed 0 P1 ratio to nominal | 0.9848 | <= 1 | pass |
| 3 seed 1 P1 ratio to nominal | 0.9377 | <= 1 | pass |
| 3 seed 2 P1 ratio to nominal | 0.9388 | <= 1 | pass |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.8299 | <= 1.05 | pass |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.9531 | <= 1.05 | pass |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.9883 | <= 1.05 | pass |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.8473 | <= 1.05 | pass |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 1.0038 | <= 1.05 | pass |
| 4 stratum cw:0.85+ position RMSE ratio | 0.9489 | <= 1.05 | pass |
| 5 random sweep of the validity box: finite, inside the state bounds | 0.0000 | <= 0 | pass |

Gate 3.2 GP2a residual: PASS.

Paired block bootstrap (5.0 s blocks, 2000 resamples, seed 0), P1 ratio shipping residual over dyn-v0.2 nominal: 0.9388, 95 percent interval [0.9129, 0.9632].

Validation P1 per candidate (metres, macro, on the internal validation part):

| config | mean | min | seed 0 | seed 1 | seed 2 |
|---|---|---|---|---|---|
| one_step_wd1e-2 | 0.1207 | 0.1187 | 0.1215 | 0.1187 | 0.1219 |
| multi_step_wd1e-3 | 0.1143 | 0.1140 | 0.1147 | 0.1140 | 0.1141 |
| multi_step_wd1e-2 | 0.1143 | 0.1127 | 0.1150 | 0.1150 | 0.1127 |
| multi_step_from_one_wd1e-2 | 0.1134 | 0.1128 | 0.1141 | 0.1132 | 0.1128 |
| multi_step_capped_wd1e-2 | 0.1144 | 0.1141 | 0.1147 | 0.1145 | 0.1141 |
| multi_step_nonorm_wd1e-2 | 0.1150 | 0.1129 | 0.1129 | 0.1148 | 0.1173 |

## Strata (direction by start speed, after merging)

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1-2026-11-12 | dyn-v0.2 nominal | dyn-v0.2 + residual seed 0 | dyn-v0.2 + residual seed 1 | dyn-v0.2 + residual seed 2 |
|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.104 | 0.099 | 0.083 | 0.081 | 0.082 |
| ccw:0.6-0.85 | 93 | 0.105 | 0.098 | 0.118 | 0.103 | 0.094 |
| ccw:0.85+ | 47 | 0.185 | 0.179 | 0.177 | 0.174 | 0.177 |
| cw:0.25-0.6 | 527 | 0.120 | 0.116 | 0.102 | 0.101 | 0.098 |
| cw:0.6-0.85 | 91 | 0.144 | 0.135 | 0.145 | 0.125 | 0.135 |
| cw:0.85+ | 54 | 0.227 | 0.229 | 0.217 | 0.217 | 0.217 |

Merges: ccw 1.25+ (1 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25

Strata under 30 windows with no lower bin to merge into (kept, listed): none.

Windows by original bin: ccw:0.25-0.6 641, ccw:0.6-0.85 93, ccw:0.85-1.25 46, ccw:1.25+ 1, cw:0.25-0.6 527, cw:0.6-0.85 91, cw:0.85-1.25 51, cw:1.25+ 3

## Command delay

The command delay is chosen on the internal validation part (role val), NOT on held-out as dyn-v0.1 did (it had no validation split), so the comparison with dyn-v0.1 is slightly generous to dyn-v0.1. Chosen 0 s (scored on the validation windows). Independent cross-correlation estimate of steering lag (steering command against ICP path curvature, speed above 0.4 m/s): 0.056 s against fitted delay plus steer_tau 0.023 s. Not tuned to.

Profile of command_delay_s (dead zone at its chosen value, continuous speed parameters refit); validation RMSE:

| value | RMSE | over the minimum |
|---|---|---|
| 0.000 | 0.0439 | +0.0% |
| 0.020 | 0.0457 | +4.0% |
| 0.040 | 0.0542 | +23.4% |
| 0.060 | 0.0471 | +7.1% |
| 0.080 | 0.0472 | +7.5% |
| 0.100 | 0.0447 | +1.7% |
| 0.120 | 0.0442 | +0.5% |
| 0.140 | 0.0572 | +30.1% |
| 0.160 | 0.0455 | +3.5% |
| 0.180 | 0.0456 | +3.8% |
| 0.200 | 0.0461 | +5.0% |
| 0.220 | 0.0483 | +9.9% |
| 0.240 | 0.0613 | +39.5% |
| 0.260 | 0.0508 | +15.7% |
| 0.280 | 0.0516 | +17.4% |
| 0.300 | 0.0533 | +21.3% |

Profile of speed_tau_s (gain, accel and decel refit; delay and dead zone at chosen values); validation RMSE:

| value | RMSE | over the minimum |
|---|---|---|
| 0.005 | 0.0453 | +3.4% |
| 0.010 | 0.0453 | +3.4% |
| 0.020 | 0.0453 | +3.3% |
| 0.050 | 0.0450 | +2.6% |
| 0.100 | 0.0443 | +1.0% |
| 0.200 | 0.0438 | +0.0% |
| 0.300 | 0.0444 | +1.3% |
| 0.500 | 0.0469 | +6.9% |
| 0.750 | 0.0510 | +16.4% |
| 1.000 | 0.0543 | +23.9% |

Profile of steer_tau_s (yaw gains and understeer refit (yaw RMSE per 0.1 s interval)); validation RMSE:

| value | RMSE | over the minimum |
|---|---|---|
| 0.005 | 0.0071 | +0.0% |
| 0.010 | 0.0071 | +0.0% |
| 0.020 | 0.0071 | +0.1% |
| 0.050 | 0.0072 | +1.7% |
| 0.100 | 0.0077 | +9.0% |
| 0.200 | 0.0092 | +29.5% |
| 0.300 | 0.0105 | +47.7% |
| 0.500 | 0.0122 | +72.0% |
| 0.750 | 0.0134 | +89.1% |
| 1.000 | 0.0141 | +99.1% |

Independent steering lag estimate (cross-correlation of the steering command with ICP path curvature at speed above 0.4 m/s, train and val segments): 0.056 s (correlation 0.98, 5088 intervals) against fitted delay plus steer_tau 0.023 s, difference +0.033 s. Reported, not tuned to.

## Validity speed

v_max_mps 1.45: 99th percentile of the wheel speed in the train and val data 1.461 m/s, rounded down 1.45, not capped. ICP ground speed p99 1.164 m/s.

Seconds of train and val data by wheel speed bin and direction:

| bin (m/s) | ccw | cw | legacy |
|---|---|---|---|
| 0.00-0.25 | 6.1 | 2.6 | 80.3 |
| 0.25-0.50 | 650.1 | 198.3 | 308.3 |
| 0.50-0.75 | 64.6 | 11.6 | 106.4 |
| 0.75-1.00 | 79.4 | 19.9 | 13.6 |
| 1.00-1.25 | 18.5 | 7.8 | 22.3 |
| 1.25-1.50 | 29.9 | 1.7 | 9.0 |
| 1.50-1.75 | 10.6 | 0.0 | 0.4 |
| 1.75-2.00 | 0.0 | 0.0 | 0.0 |
| 2.00-2.25 | 0.0 | 0.0 | 0.0 |
| 2.25-2.50 | 0.0 | 0.0 | 0.0 |

## Dead zone

Shipped `speed_deadband_mps` 0.34 m/s (median command at which the wheels turned over the launches from rest; 17 used of 19 launches in the train and val segments). Launches from rest: n 19, median 0.338 m/s. Steady state bracket: the largest still bin ends at 0.37, the smallest moving bin starts at 0.38 (m/s). Shipped value inside the bracket: False.

## Prediction error per planner run (bag 2026-11-13T22-20-18, R0 to R8)

Open loop from the recorded /drive commands, standing starts included, speed vs the wheel, pose vs chained ICP. Cells: wheel speed RMSE (m/s) / yaw RMSE (rad) / position RMSE (m). `train` runs are in-sample for dyn-v0.2.

| run | role | windows | model | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|---|---|---|
| R0 | train | 153 | dyn-v0.1-2026-11-12 | 0.027 / 0.064 / 0.057 | 0.026 / 0.127 / 0.114 | 0.026 / 0.188 / 0.172 |
| R0 | train | 153 | dyn-v0.2 nominal | 0.034 / 0.059 / 0.054 | 0.039 / 0.116 / 0.107 | 0.033 / 0.173 / 0.161 |
| R0 | train | 153 | dyn-v0.2 + residual seed 2 | 0.027 / 0.026 / 0.051 | 0.028 / 0.048 / 0.100 | 0.028 / 0.069 / 0.148 |
| R1 | train | 164 | dyn-v0.1-2026-11-12 | 0.028 / 0.049 / 0.048 | 0.028 / 0.096 / 0.096 | 0.030 / 0.141 / 0.144 |
| R1 | train | 164 | dyn-v0.2 nominal | 0.048 / 0.049 / 0.045 | 0.056 / 0.097 / 0.091 | 0.049 / 0.146 / 0.138 |
| R1 | train | 164 | dyn-v0.2 + residual seed 2 | 0.027 / 0.027 / 0.041 | 0.028 / 0.042 / 0.079 | 0.030 / 0.053 / 0.116 |
| R2 | train | 48 | dyn-v0.1-2026-11-12 | 0.038 / 0.076 / 0.087 | 0.039 / 0.141 / 0.172 | 0.040 / 0.203 / 0.252 |
| R2 | train | 48 | dyn-v0.2 nominal | 0.048 / 0.091 / 0.083 | 0.048 / 0.172 / 0.165 | 0.049 / 0.248 / 0.248 |
| R2 | train | 48 | dyn-v0.2 + residual seed 2 | 0.036 / 0.039 / 0.086 | 0.039 / 0.063 / 0.163 | 0.040 / 0.087 / 0.232 |
| R3 | heldout | 181 | dyn-v0.1-2026-11-12 | 0.024 / 0.049 / 0.048 | 0.027 / 0.095 / 0.096 | 0.025 / 0.141 / 0.144 |
| R3 | heldout | 181 | dyn-v0.2 nominal | 0.022 / 0.050 / 0.046 | 0.024 / 0.096 / 0.090 | 0.022 / 0.141 / 0.135 |
| R3 | heldout | 181 | dyn-v0.2 + residual seed 2 | 0.024 / 0.026 / 0.041 | 0.027 / 0.038 / 0.079 | 0.024 / 0.044 / 0.117 |
| R4 | heldout | 119 | dyn-v0.1-2026-11-12 | 0.022 / 0.064 / 0.055 | 0.023 / 0.124 / 0.111 | 0.023 / 0.183 / 0.168 |
| R4 | heldout | 119 | dyn-v0.2 nominal | 0.022 / 0.060 / 0.053 | 0.023 / 0.115 / 0.105 | 0.023 / 0.168 / 0.158 |
| R4 | heldout | 119 | dyn-v0.2 + residual seed 2 | 0.026 / 0.028 / 0.049 | 0.027 / 0.051 / 0.096 | 0.027 / 0.073 / 0.143 |
| R5 | heldout | 82 | dyn-v0.1-2026-11-12 | 0.024 / 0.076 / 0.072 | 0.027 / 0.145 / 0.142 | 0.024 / 0.213 / 0.211 |
| R5 | heldout | 82 | dyn-v0.2 nominal | 0.030 / 0.073 / 0.068 | 0.032 / 0.137 / 0.134 | 0.031 / 0.201 / 0.198 |
| R5 | heldout | 82 | dyn-v0.2 + residual seed 2 | 0.041 / 0.038 / 0.067 | 0.048 / 0.071 / 0.129 | 0.047 / 0.105 / 0.190 |
| R6 | heldout | 79 | dyn-v0.1-2026-11-12 | 0.025 / 0.043 / 0.050 | 0.024 / 0.081 / 0.100 | 0.028 / 0.115 / 0.148 |
| R6 | heldout | 79 | dyn-v0.2 nominal | 0.029 / 0.048 / 0.047 | 0.026 / 0.089 / 0.093 | 0.031 / 0.126 / 0.138 |
| R6 | heldout | 79 | dyn-v0.2 + residual seed 2 | 0.030 / 0.055 / 0.046 | 0.030 / 0.106 / 0.090 | 0.030 / 0.158 / 0.138 |
| R7 | heldout+train | 54 | dyn-v0.1-2026-11-12 | 0.053 / 0.111 / 0.104 | 0.051 / 0.218 / 0.204 | 0.046 / 0.323 / 0.300 |
| R7 | heldout+train | 54 | dyn-v0.2 nominal | 0.061 / 0.118 / 0.100 | 0.061 / 0.229 / 0.196 | 0.055 / 0.339 / 0.291 |
| R7 | heldout+train | 54 | dyn-v0.2 + residual seed 2 | 0.051 / 0.038 / 0.099 | 0.052 / 0.060 / 0.189 | 0.055 / 0.076 / 0.272 |
| R8 | heldout | 44 | dyn-v0.1-2026-11-12 | 0.040 / 0.078 / 0.087 | 0.032 / 0.151 / 0.174 | 0.034 / 0.221 / 0.258 |
| R8 | heldout | 44 | dyn-v0.2 nominal | 0.048 / 0.095 / 0.083 | 0.040 / 0.184 / 0.168 | 0.044 / 0.270 / 0.256 |
| R8 | heldout | 44 | dyn-v0.2 + residual seed 2 | 0.039 / 0.045 / 0.086 | 0.032 / 0.073 / 0.165 | 0.034 / 0.093 / 0.236 |

## Legacy held-out (last 30 percent of the older bags; decides nothing)

The dyn-v0.1 pooled report, cells: speed RMSE (m/s) / yaw RMSE (rad) / position RMSE (m).

| model | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|
| dyn-v0.1-2026-11-12 | 0.072 / 0.085 / 0.090 | 0.073 / 0.160 / 0.170 | 0.080 / 0.226 / 0.236 |
| dyn-v0.2 nominal | 0.080 / 0.090 / 0.091 | 0.086 / 0.171 / 0.173 | 0.093 / 0.243 / 0.244 |
| dyn-v0.2 + residual seed 2 | 0.065 / 0.046 / 0.081 | 0.069 / 0.076 / 0.148 | 0.072 / 0.100 / 0.206 |

## Data

Seconds of data per role per bag, then per role by direction and start speed bin (wheel speed of the samples; `legacy` is the older bags' 70/30 rule):

```json
{
 "seconds_by_bag": {
  "2026-10-24T23-13-18_car_teleop": {
   "train": 257.8,
   "val": 41.0,
   "legacy_heldout": 132.9
  },
  "2026-10-25T19-27-08_car_teleop": {
   "train": 18.8,
   "legacy_heldout": 16.3
  },
  "2026-11-11T03-54-11_car_teleop_cw": {
   "train": 151.2,
   "val": 25.0,
   "legacy_heldout": 69.6
  },
  "2026-11-11T04-20-19_car_teleop_v3": {
   "train": 148.6,
   "val": 7.7,
   "legacy_heldout": 71.3
  },
  "2026-11-12T03-04-56_car_teleop": {
   "train": 187.0,
   "val": 29.5,
   "legacy_heldout": 97.5
  },
  "2026-11-13T22-00-40_car_teleop": {
   "train": 516.6,
   "val": 91.5,
   "heldout": 123.4
  },
  "2026-11-13T22-20-18_car_teleop": {
   "heldout": 271.1,
   "train": 115.3,
   "val": 20.3
  },
  "2026-11-13T23-19-51_car_teleop": {
   "train": 11.2
  },
  "2026-11-13T23-27-23_car_teleop": {
   "train": 295.8,
   "val": 50.1,
   "heldout": 8.8
  }
 },
 "seconds_by_role_direction_speed_bin": {
  "train": {
   "legacy": {
    "<0.25": 70.3,
    "0.25-0.6": 365.6,
    "0.6-0.85": 2.7,
    "0.85-1.25": 30.5,
    "1.25+": 8.9
   },
   "ccw": {
    "<0.25": 5.7,
    "0.25-0.6": 608.2,
    "0.6-0.85": 2.4,
    "0.85-1.25": 82.2,
    "1.25+": 35.4
   },
   "cw": {
    "<0.25": 2.4,
    "0.25-0.6": 178.1,
    "0.6-0.85": 2.1,
    "0.85-1.25": 20.9,
    "1.25+": 1.7
   }
  },
  "val": {
   "legacy": {
    "<0.25": 9.7,
    "0.25-0.6": 46.7,
    "0.6-0.85": 0.4,
    "0.85-1.25": 4.5,
    "1.25+": 0.4
   },
   "ccw": {
    "<0.25": 0.4,
    "0.25-0.6": 105.9,
    "0.6-0.85": 0.2,
    "0.85-1.25": 13.9,
    "1.25+": 5.1
   },
   "cw": {
    "<0.25": 0.1,
    "0.25-0.6": 31.3,
    "0.6-0.85": 0.1,
    "0.85-1.25": 5.1,
    "1.25+": 0.0
   }
  },
  "legacy_heldout": {
   "legacy": {
    "<0.25": 13.3,
    "0.25-0.6": 140.0,
    "0.6-0.85": 8.9,
    "0.85-1.25": 62.6,
    "1.25+": 9.9
   }
  },
  "heldout": {
   "ccw": {
    "<0.25": 2.3,
    "0.25-0.6": 165.0,
    "0.6-0.85": 25.9,
    "0.85-1.25": 16.9,
    "1.25+": 6.7
   },
   "cw": {
    "<0.25": 1.6,
    "0.25-0.6": 137.3,
    "0.6-0.85": 24.9,
    "0.85-1.25": 21.4,
    "1.25+": 1.4
   }
  }
 }
}
```
