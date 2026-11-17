# Lap battery B7, 2026-11: D7-S0 FAIL, D7-S1 PASS

Run once at freeze commit 71ae56dd3de72799ace4fa7af97be148f2fa2039 (PR 205 and 206 merged), 3 workers,
yaml sha256 35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de, results in
car-data/pnp/battery/v0.5-b7. Thresholds unchanged, nothing re-scored or investigated after reading.
D does not drive. D7-C0 was not computed (no tool in the paths of this brief).

By case, laps outside 0.3 s of truth (largest error), conditions a and c: P1 0 of 144 (0.06 s),
P2 0 of 144 (0.13 s), P3 0 of 144 (0.20 s), P4 144 of 144 (2.01 s). Every D7-S0 failure is a P4 run.

# Battery B7 report

constants file sha256 now: 35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de (registered 35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de)
constants file sha256 in the run files: ['35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de']
freeze commit in the run files: ['71ae56dd3de72799ace4fa7af97be148f2fa2039']
git sha of the checkout in the run files: ['7e3a99f77bfc4c429144bc380b7b079cdb733832']
runs: 72
drop rule: dropped [], seed 51 laps {'a_P1_ccw_51': 12, 'a_P1_cw_51': 12, 'a_P2_ccw_51': 12, 'a_P2_cw_51': 12, 'a_P3_ccw_51': 12, 'a_P3_cw_51': 12, 'a_P4_ccw_51': 12, 'a_P4_cw_51': 12}

## Attempts

a_P1_cw_53: [{'attempt': 1, 'file': 'a_P1_cw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0595, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P1_cw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0259, 'real_time_factor': 1.0, 'error': ''}]
a_P2_ccw_51: [{'attempt': 1, 'file': 'a_P2_ccw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0536, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P2_ccw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0258, 'real_time_factor': 1.0, 'error': ''}]
a_P2_ccw_52: [{'attempt': 1, 'file': 'a_P2_ccw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0728, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P2_ccw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0391, 'real_time_factor': 1.0, 'error': ''}]
a_P2_ccw_53: [{'attempt': 1, 'file': 'a_P2_ccw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0643, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P2_ccw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0245, 'real_time_factor': 1.0, 'error': ''}]
a_P2_cw_52: [{'attempt': 1, 'file': 'a_P2_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0659, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P2_cw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0254, 'real_time_factor': 1.0, 'error': ''}]
a_P2_cw_53: [{'attempt': 1, 'file': 'a_P2_cw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0763, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P2_cw_53.attempt2.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0608, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'a_P2_cw_53.attempt3.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0241, 'real_time_factor': 1.0, 'error': ''}]
a_P3_ccw_51: [{'attempt': 1, 'file': 'a_P3_ccw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0646, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P3_ccw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0381, 'real_time_factor': 1.0, 'error': ''}]
a_P3_cw_51: [{'attempt': 1, 'file': 'a_P3_cw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0721, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P3_cw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0345, 'real_time_factor': 1.0, 'error': ''}]
a_P4_ccw_53: [{'attempt': 1, 'file': 'a_P4_ccw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0646, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P4_ccw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0333, 'real_time_factor': 1.0, 'error': ''}]
a_P4_cw_52: [{'attempt': 1, 'file': 'a_P4_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0515, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_P4_cw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0279, 'real_time_factor': 1.0, 'error': ''}]
b_P2_ccw_53: [{'attempt': 1, 'file': 'b_P2_ccw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0543, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_P2_ccw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0254, 'real_time_factor': 1.0, 'error': ''}]
b_P3_cw_52: [{'attempt': 1, 'file': 'b_P3_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0424, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_P3_cw_52.attempt2.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0501, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'b_P3_cw_52.attempt3.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0278, 'real_time_factor': 1.0, 'error': ''}]
c_P1_cw_51: [{'attempt': 1, 'file': 'c_P1_cw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.058, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P1_cw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0216, 'real_time_factor': 1.0, 'error': ''}]
c_P1_cw_52: [{'attempt': 1, 'file': 'c_P1_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0715, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P1_cw_52.attempt2.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0425, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'c_P1_cw_52.attempt3.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0263, 'real_time_factor': 1.0, 'error': ''}]
c_P1_cw_53: [{'attempt': 1, 'file': 'c_P1_cw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0714, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P1_cw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0246, 'real_time_factor': 1.0, 'error': ''}]
c_P2_cw_51: [{'attempt': 1, 'file': 'c_P2_cw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0738, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P2_cw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0332, 'real_time_factor': 1.0, 'error': ''}]
c_P2_cw_52: [{'attempt': 1, 'file': 'c_P2_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0438, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P2_cw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0237, 'real_time_factor': 1.0, 'error': ''}]
c_P2_cw_53: [{'attempt': 1, 'file': 'c_P2_cw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0606, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P2_cw_53.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0361, 'real_time_factor': 1.0, 'error': ''}]
c_P3_ccw_52: [{'attempt': 1, 'file': 'c_P3_ccw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0437, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P3_ccw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0255, 'real_time_factor': 1.0, 'error': ''}]
c_P3_cw_51: [{'attempt': 1, 'file': 'c_P3_cw_51.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0553, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P3_cw_51.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0387, 'real_time_factor': 1.0, 'error': ''}]
c_P3_cw_52: [{'attempt': 1, 'file': 'c_P3_cw_52.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0514, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P3_cw_52.attempt2.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.025, 'real_time_factor': 1.0, 'error': ''}]
c_P3_cw_53: [{'attempt': 1, 'file': 'c_P3_cw_53.attempt1.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0583, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_P3_cw_53.attempt2.json', 'starved': True, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0426, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'c_P3_cw_53.attempt3.json', 'starved': False, 'invalid_truth': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0239, 'real_time_factor': 1.0, 'error': ''}]

## Runs

| cond | case | dir | seed | laps | T1 s | T_late s | r | contacts | obstacle brakes per lap (late) | complete |
|---|---|---|---|---|---|---|---|---|---|---|
| a | P1 | ccw | 51 | 12 | 29.23 | 29.02 | 0.993 | 0 | 0.00 | yes |
| a | P1 | ccw | 52 | 12 | 29.19 | 29.02 | 0.994 | 0 | 0.00 | yes |
| a | P1 | ccw | 53 | 12 | 29.24 | 29.03 | 0.993 | 0 | 0.00 | yes |
| a | P1 | cw | 51 | 12 | 28.97 | 28.74 | 0.992 | 0 | 0.00 | yes |
| a | P1 | cw | 52 | 12 | 28.96 | 28.74 | 0.992 | 0 | 0.00 | yes |
| a | P1 | cw | 53 | 12 | 28.97 | 28.74 | 0.992 | 0 | 0.00 | yes |
| a | P2 | ccw | 51 | 12 | 28.58 | 28.34 | 0.991 | 0 | 0.00 | yes |
| a | P2 | ccw | 52 | 12 | 28.56 | 28.35 | 0.993 | 0 | 0.00 | yes |
| a | P2 | ccw | 53 | 12 | 28.57 | 28.42 | 0.995 | 0 | 0.00 | yes |
| a | P2 | cw | 51 | 12 | 28.46 | 28.21 | 0.991 | 0 | 0.00 | yes |
| a | P2 | cw | 52 | 12 | 28.40 | 28.25 | 0.995 | 0 | 0.00 | yes |
| a | P2 | cw | 53 | 12 | 28.43 | 28.22 | 0.993 | 0 | 0.00 | yes |
| a | P3 | ccw | 51 | 12 | 34.38 | 34.19 | 0.995 | 0 | 0.00 | yes |
| a | P3 | ccw | 52 | 12 | 34.41 | 34.27 | 0.996 | 0 | 0.00 | yes |
| a | P3 | ccw | 53 | 12 | 34.39 | 34.21 | 0.995 | 0 | 0.00 | yes |
| a | P3 | cw | 51 | 12 | 34.26 | 34.04 | 0.994 | 0 | 0.00 | yes |
| a | P3 | cw | 52 | 12 | 34.20 | 34.01 | 0.994 | 0 | 0.00 | yes |
| a | P3 | cw | 53 | 12 | 34.89 | 34.02 | 0.975 | 0 | 0.00 | yes |
| a | P4 | ccw | 51 | 12 | 12.53 | 12.31 | 0.982 | 0 | 0.00 | yes |
| a | P4 | ccw | 52 | 12 | 12.53 | 12.31 | 0.982 | 0 | 0.00 | yes |
| a | P4 | ccw | 53 | 12 | 12.55 | 12.35 | 0.984 | 0 | 0.00 | yes |
| a | P4 | cw | 51 | 12 | 12.36 | 12.13 | 0.982 | 0 | 0.00 | yes |
| a | P4 | cw | 52 | 12 | 12.34 | 12.10 | 0.981 | 0 | 0.00 | yes |
| a | P4 | cw | 53 | 12 | 12.36 | 12.10 | 0.979 | 0 | 0.00 | yes |
| b | P1 | ccw | 51 | 12 | 13.01 | 12.68 | 0.975 | 0 | 0.00 | yes |
| b | P1 | ccw | 52 | 12 | 13.02 | 12.71 | 0.976 | 0 | 0.00 | yes |
| b | P1 | ccw | 53 | 12 | 13.00 | 12.73 | 0.979 | 0 | 0.00 | yes |
| b | P1 | cw | 51 | 12 | 12.98 | 12.65 | 0.975 | 0 | 0.00 | yes |
| b | P1 | cw | 52 | 12 | 12.96 | 12.64 | 0.975 | 0 | 0.00 | yes |
| b | P1 | cw | 53 | 12 | 12.93 | 12.63 | 0.977 | 0 | 0.00 | yes |
| b | P2 | ccw | 51 | 12 | 18.30 | 18.14 | 0.991 | 0 | 0.00 | yes |
| b | P2 | ccw | 52 | 12 | 18.26 | 18.11 | 0.992 | 0 | 0.00 | yes |
| b | P2 | ccw | 53 | 12 | 18.26 | 18.09 | 0.991 | 0 | 0.00 | yes |
| b | P2 | cw | 51 | 12 | 18.23 | 18.03 | 0.989 | 0 | 0.00 | yes |
| b | P2 | cw | 52 | 12 | 18.27 | 18.00 | 0.985 | 0 | 0.00 | yes |
| b | P2 | cw | 53 | 12 | 18.27 | 17.95 | 0.982 | 0 | 0.00 | yes |
| b | P3 | ccw | 51 | 12 | 21.42 | 21.40 | 0.999 | 0 | 0.00 | yes |
| b | P3 | ccw | 52 | 12 | 21.49 | 21.31 | 0.992 | 0 | 0.00 | yes |
| b | P3 | ccw | 53 | 12 | 21.51 | 21.36 | 0.993 | 0 | 0.00 | yes |
| b | P3 | cw | 51 | 12 | 21.51 | 21.35 | 0.993 | 0 | 0.00 | yes |
| b | P3 | cw | 52 | 12 | 21.63 | 21.42 | 0.990 | 0 | 0.00 | yes |
| b | P3 | cw | 53 | 12 | 21.59 | 21.43 | 0.993 | 0 | 0.00 | yes |
| b | P4 | ccw | 51 | 12 | 6.17 | 5.91 | 0.958 | 0 | 0.00 | yes |
| b | P4 | ccw | 52 | 12 | 6.24 | 5.95 | 0.954 | 0 | 0.00 | yes |
| b | P4 | ccw | 53 | 12 | 6.18 | 5.95 | 0.963 | 0 | 0.00 | yes |
| b | P4 | cw | 51 | 12 | 6.34 | 6.08 | 0.959 | 0 | 0.00 | yes |
| b | P4 | cw | 52 | 12 | 6.25 | 5.99 | 0.958 | 0 | 0.00 | yes |
| b | P4 | cw | 53 | 12 | 6.30 | 6.02 | 0.955 | 0 | 0.00 | yes |
| c | P1 | ccw | 51 | 12 | 29.17 | 12.74 | 0.437 | 0 | 0.00 | yes |
| c | P1 | ccw | 52 | 12 | 29.20 | 12.70 | 0.435 | 0 | 0.00 | yes |
| c | P1 | ccw | 53 | 12 | 29.22 | 12.70 | 0.435 | 0 | 0.00 | yes |
| c | P1 | cw | 51 | 12 | 28.96 | 12.66 | 0.437 | 0 | 0.00 | yes |
| c | P1 | cw | 52 | 12 | 28.98 | 12.65 | 0.437 | 0 | 0.00 | yes |
| c | P1 | cw | 53 | 12 | 28.96 | 12.68 | 0.438 | 0 | 0.00 | yes |
| c | P2 | ccw | 51 | 12 | 28.55 | 18.15 | 0.636 | 0 | 0.00 | yes |
| c | P2 | ccw | 52 | 12 | 28.58 | 18.07 | 0.632 | 0 | 0.00 | yes |
| c | P2 | ccw | 53 | 12 | 28.59 | 18.13 | 0.634 | 0 | 0.00 | yes |
| c | P2 | cw | 51 | 12 | 28.45 | 18.11 | 0.637 | 0 | 0.00 | yes |
| c | P2 | cw | 52 | 12 | 28.42 | 18.04 | 0.635 | 0 | 0.00 | yes |
| c | P2 | cw | 53 | 12 | 28.43 | 18.01 | 0.633 | 0 | 0.00 | yes |
| c | P3 | ccw | 51 | 12 | 34.39 | 21.34 | 0.621 | 0 | 0.00 | yes |
| c | P3 | ccw | 52 | 12 | 34.39 | 21.29 | 0.619 | 0 | 0.00 | yes |
| c | P3 | ccw | 53 | 12 | 34.38 | 21.33 | 0.620 | 0 | 0.00 | yes |
| c | P3 | cw | 51 | 12 | 34.24 | 21.34 | 0.623 | 0 | 0.00 | yes |
| c | P3 | cw | 52 | 12 | 34.23 | 21.37 | 0.624 | 0 | 0.00 | yes |
| c | P3 | cw | 53 | 12 | 34.23 | 21.39 | 0.625 | 0 | 0.00 | yes |
| c | P4 | ccw | 51 | 12 | 12.52 | 5.94 | 0.474 | 0 | 0.00 | yes |
| c | P4 | ccw | 52 | 12 | 12.52 | 5.92 | 0.473 | 0 | 0.00 | yes |
| c | P4 | ccw | 53 | 12 | 12.50 | 5.92 | 0.474 | 0 | 0.00 | yes |
| c | P4 | cw | 51 | 12 | 12.30 | 6.06 | 0.493 | 0 | 0.00 | yes |
| c | P4 | cw | 52 | 12 | 12.31 | 5.97 | 0.485 | 0 | 0.00 | yes |
| c | P4 | cw | 53 | 12 | 12.33 | 6.01 | 0.488 | 0 | 0.00 | yes |

## Decisions

D7-S0: FAIL
  FAIL: S0a: system lap count equals ground truth in every condition (a) run: 18 of 24 runs agree ['P4/ccw/51: system 13 vs truth 12', 'P4/ccw/52: system 13 vs truth 12', 'P4/ccw/53: system 13 vs truth 12', 'P4/cw/51: system 13 vs truth 12', 'P4/cw/52: system 13 vs truth 12', 'P4/cw/53: system 13 vs truth 12']
  FAIL: S0a: every system lap time within 0.3 s of ground truth: 72 of 288 laps outside ['P4/ccw/51 lap 1: system 11.59 vs truth 12.53', 'P4/ccw/51 lap 2: system 10.82 vs truth 12.32', 'P4/ccw/51 lap 3: system 10.83 vs truth 12.41', 'P4/ccw/51 lap 4: system 10.89 vs truth 12.23', 'P4/ccw/51 lap 5: system 10.70 vs truth 12.44', 'P4/ccw/51 lap 6: system 10.64 vs truth 12.21', 'P4/ccw/51 lap 7: system 10.85 vs truth 12.32', 'P4/ccw/51 lap 8: system 10.50 vs truth 12.31']
  ok: S0a: no condition (a) run failed to execute: 0 errors []
  FAIL: S0b(c): lap count equal in every run: 18 of 24 runs agree ['P4/ccw/51: system 14 vs truth 12', 'P4/ccw/52: system 14 vs truth 12', 'P4/ccw/53: system 14 vs truth 12', 'P4/cw/51: system 14 vs truth 12', 'P4/cw/52: system 14 vs truth 12', 'P4/cw/53: system 14 vs truth 12']
  FAIL: S0b(c): at least 98% of laps within 0.3 s: 216 of 288 laps within (0.7500); outside: ['P4/ccw/51 lap 1: system 11.52 vs truth 12.52', 'P4/ccw/51 lap 2: system 8.80 vs truth 10.27', 'P4/ccw/51 lap 3: system 7.72 vs truth 8.81', 'P4/ccw/51 lap 4: system 6.96 vs truth 7.62', 'P4/ccw/51 lap 5: system 6.30 vs truth 7.00', 'P4/ccw/51 lap 6: system 5.82 vs truth 6.38', 'P4/ccw/51 lap 7: system 5.40 vs truth 6.08', 'P4/ccw/51 lap 8: system 5.24 vs truth 5.99']
  FAIL: S0b(c): no lap off by more than 1.0 s: largest error 1.47 s
  ok: S0b(c): no run failed to execute: 0 errors []

D7-S1: PASS
  ok: 1. 0 wall contacts in the 24 runs of condition c: 0 contacts, runs with a contact: []
  ok: 2. runs of condition c complete 10 laps: 24 of 24 complete, not complete: []
  ok: 3. reach: T_late(c) <= 1.05 x T_late(b) in at least 22 of 24 runs: 24 of 24 reach; ratios [0.995, 0.995, 0.997, 0.997, 0.997, 0.998, 0.998, 0.998, 0.998, 0.999, 0.999, 0.999, 0.999, 0.999, 1.001, 1.001, 1.001, 1.002, 1.002, 1.003, 1.004, 1.004, 1.005, 1.005]
  ok: 4a. T_late below T1 in every completing run: 0 runs not below []
  ok: 4b. median r <= 0.8: median r 0.556 over 24 completing runs

## What this lets D do

D7-S0 or D7-S1 FAILED: D does not drive, the cap stays fixed (pre-registration section 5). A failed gate is a result: no tuning, no re-scoring.

## Reported without a threshold

Lap 1, system against the registered truth:

| run | system lap 1 s | truth lap 1 s | difference s | launch dwell s | truth valid |
|---|---|---|---|---|---|
| a_P1_ccw_51 | 29.27 | 29.23 | +0.04 | 0.03 | True |
| a_P1_ccw_52 | 29.25 | 29.19 | +0.06 | 0.01 | True |
| a_P1_ccw_53 | 29.29 | 29.24 | +0.05 | 0.01 | True |
| a_P1_cw_51 | 29.02 | 28.97 | +0.05 | 0.01 | True |
| a_P1_cw_52 | 28.99 | 28.96 | +0.03 | 0.01 | True |
| a_P1_cw_53 | 29.03 | 28.97 | +0.06 | 0.03 | True |
| a_P2_ccw_51 | 28.61 | 28.58 | +0.03 | 0.05 | True |
| a_P2_ccw_52 | 28.60 | 28.56 | +0.04 | 0.01 | True |
| a_P2_ccw_53 | 28.63 | 28.57 | +0.06 | 0.01 | True |
| a_P2_cw_51 | 28.52 | 28.46 | +0.06 | 0.01 | True |
| a_P2_cw_52 | 28.44 | 28.40 | +0.04 | 0.01 | True |
| a_P2_cw_53 | 28.50 | 28.43 | +0.07 | 0.03 | True |
| a_P3_ccw_51 | 34.43 | 34.38 | +0.05 | 0.01 | True |
| a_P3_ccw_52 | 34.44 | 34.41 | +0.03 | 0.03 | True |
| a_P3_ccw_53 | 34.42 | 34.39 | +0.03 | 0.03 | True |
| a_P3_cw_51 | 34.29 | 34.26 | +0.03 | 0.03 | True |
| a_P3_cw_52 | 34.27 | 34.20 | +0.07 | 0.01 | True |
| a_P3_cw_53 | 34.99 | 34.89 | +0.10 | 0.01 | True |
| a_P4_ccw_51 | 11.59 | 12.53 | -0.94 | 0.01 | True |
| a_P4_ccw_52 | 11.52 | 12.53 | -1.01 | 0.03 | True |
| a_P4_ccw_53 | 11.59 | 12.55 | -0.96 | 0.03 | True |
| a_P4_cw_51 | 11.27 | 12.36 | -1.09 | 0.03 | True |
| a_P4_cw_52 | 11.23 | 12.34 | -1.11 | 0.03 | True |
| a_P4_cw_53 | 10.99 | 12.36 | -1.37 | 0.03 | True |
| b_P1_ccw_51 | 13.03 | 13.01 | +0.02 | 0.03 | True |
| b_P1_ccw_52 | 13.03 | 13.02 | +0.01 | 0.01 | True |
| b_P1_ccw_53 | 13.03 | 13.00 | +0.03 | 0.03 | True |
| b_P1_cw_51 | 13.00 | 12.98 | +0.02 | 0.01 | True |
| b_P1_cw_52 | 12.99 | 12.96 | +0.03 | 0.01 | True |
| b_P1_cw_53 | 12.95 | 12.93 | +0.02 | 0.03 | True |
| b_P2_ccw_51 | 18.32 | 18.30 | +0.02 | 0.01 | True |
| b_P2_ccw_52 | 18.28 | 18.26 | +0.02 | 0.01 | True |
| b_P2_ccw_53 | 18.30 | 18.26 | +0.04 | 0.01 | True |
| b_P2_cw_51 | 18.25 | 18.23 | +0.02 | 0.01 | True |
| b_P2_cw_52 | 18.31 | 18.27 | +0.04 | 0.01 | True |
| b_P2_cw_53 | 18.29 | 18.27 | +0.02 | 0.03 | True |
| b_P3_ccw_51 | 21.40 | 21.42 | -0.02 | 0.01 | True |
| b_P3_ccw_52 | 21.50 | 21.49 | +0.01 | 0.01 | True |
| b_P3_ccw_53 | 21.53 | 21.51 | +0.02 | 0.01 | True |
| b_P3_cw_51 | 21.52 | 21.51 | +0.01 | 0.01 | True |
| b_P3_cw_52 | 21.65 | 21.63 | +0.02 | 0.01 | True |
| b_P3_cw_53 | 21.63 | 21.59 | +0.04 | 0.03 | True |
| b_P4_ccw_51 | 5.58 | 6.17 | -0.59 | 0.01 | True |
| b_P4_ccw_52 | 5.46 | 6.24 | -0.78 | 0.01 | True |
| b_P4_ccw_53 | 5.48 | 6.18 | -0.70 | 0.01 | True |
| b_P4_cw_51 | 5.58 | 6.34 | -0.76 | 0.01 | True |
| b_P4_cw_52 | 5.57 | 6.25 | -0.68 | 0.01 | True |
| b_P4_cw_53 | 5.46 | 6.30 | -0.84 | 0.01 | True |
| c_P1_ccw_51 | 29.23 | 29.17 | +0.06 | 0.01 | True |
| c_P1_ccw_52 | 29.24 | 29.20 | +0.04 | 0.01 | True |
| c_P1_ccw_53 | 29.28 | 29.22 | +0.06 | 0.01 | True |
| c_P1_cw_51 | 29.02 | 28.96 | +0.05 | 0.03 | True |
| c_P1_cw_52 | 29.01 | 28.98 | +0.03 | 0.03 | True |
| c_P1_cw_53 | 29.01 | 28.96 | +0.06 | 0.03 | True |
| c_P2_ccw_51 | 28.58 | 28.55 | +0.03 | 0.01 | True |
| c_P2_ccw_52 | 28.64 | 28.58 | +0.06 | 0.01 | True |
| c_P2_ccw_53 | 28.65 | 28.59 | +0.06 | 0.01 | True |
| c_P2_cw_51 | 28.50 | 28.45 | +0.05 | 0.01 | True |
| c_P2_cw_52 | 28.47 | 28.42 | +0.05 | 0.01 | True |
| c_P2_cw_53 | 28.46 | 28.43 | +0.03 | 0.01 | True |
| c_P3_ccw_51 | 34.41 | 34.39 | +0.02 | 0.01 | True |
| c_P3_ccw_52 | 34.48 | 34.39 | +0.09 | 0.01 | True |
| c_P3_ccw_53 | 34.38 | 34.38 | +0.00 | 0.01 | True |
| c_P3_cw_51 | 34.25 | 34.24 | +0.01 | 0.03 | True |
| c_P3_cw_52 | 34.30 | 34.23 | +0.07 | 0.03 | True |
| c_P3_cw_53 | 34.22 | 34.23 | -0.02 | 0.01 | True |
| c_P4_ccw_51 | 11.52 | 12.52 | -1.00 | 0.01 | True |
| c_P4_ccw_52 | 11.57 | 12.52 | -0.95 | 0.03 | True |
| c_P4_ccw_53 | 11.39 | 12.50 | -1.12 | 0.01 | True |
| c_P4_cw_51 | 11.11 | 12.30 | -1.19 | 0.01 | True |
| c_P4_cw_52 | 11.09 | 12.31 | -1.22 | 0.01 | True |
| c_P4_cw_53 | 11.29 | 12.33 | -1.04 | 0.03 | True |

lap 1, system minus truth: median |difference| 0.050 s, largest 1.370 s over 72 runs

- condition a: 0 laps flagged missed gate in 0 of 24 runs
- condition b: 0 laps flagged missed gate in 0 of 24 runs
- condition c: 0 laps flagged missed gate in 0 of 24 runs
