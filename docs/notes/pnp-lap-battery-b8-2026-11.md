# Lap battery B8, 2026-11: D8-S0 FAIL, D8-S1 PASS

Run once at freeze commit 7098fcda5906f56287188e0be41d1dc676cf48e0 (PR 213 merged), 3 workers, yaml sha256
74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36, results in
car-data/pnp/battery/v0.5-b8. Thresholds and code unchanged since the freeze, nothing re-scored, rerun or
investigated after reading. D does not drive, the cap stays fixed (pre-registration section 6). D8-C0 is the
car session and was not computed. B9 freezes from the same commit and runs after this.

108 runs (R1 to R4 and Q1, Q2, seeds 61 to 63, both directions, conditions a, b, c). The drop rule decided on
the condition a runs of seed 61 and dropped no case (13 laps in all twelve). 28 runs had a starved first
attempt (a /drive_raw gap over 40 ms) and were rerun alone as registered, at most 3 attempts in all, every
attempt kept; c_R3_cw_63 was starved on all three attempts (the last at a real-time factor of 0.18) and its
last attempt is the one scored, as the rule says. Its late-lap ratio (T_late of c over b, 1.17) is the one
reach failure in D8-S1 item 3 (35 of 36 against the 33 required).

D8-S0 FAIL. Condition a: the count rule holds in 36 of 36 runs, 5 of 432 judged laps are outside their class
tolerance, all of them lap 1 of case R4 (five of its six runs, errors 0.90 to 1.30 s against 0.62 to 0.65 s).
Condition c: the count rule holds in 36 of 36 runs, 422 of 432 laps within their class tolerance (0.9769
against the 0.98 required); the first six of the ten outside, which is all the report lists, are R4 laps 1
(three runs) and R4/ccw/63 laps 6, 7 and 10 (0.75 to 0.87 s); no lap is beyond its single-error limit. R4
closes by `start_scan` in 64 of its 76 condition a laps and by `winding` in the rest; in condition c by
`anchor` (4), `start_scan` (48) and `winding` (26). R1 to R3 close every lap by `winding`, Q1 and Q2 by
`start_scan`.

D8-S1 PASS: 0 wall contacts in the 36 condition c runs, 36 of 36 complete, 35 of 36 reach (33 required), T_late
below T1 in every run, median r 0.496.

What the result means is the owner's call; nothing here proposes a change.

# Battery B8 report

constants file sha256 now: 74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36 (registered 74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36)
constants file sha256 in the run files: ['74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36']
freeze commit in the run files: ['7098fcda5906f56287188e0be41d1dc676cf48e0']
git sha of the checkout in the run files: ['cf7c60f38040b1fa28eb5d38e74d2366d54fc9e0']
runs: 108
drop rule: {'dropped': [], 'decided': True, 'reason': '', 'laps': {'R1_ccw': 13, 'R1_cw': 13, 'R2_ccw': 13, 'R2_cw': 13, 'R3_ccw': 13, 'R3_cw': 13, 'R4_ccw': 13, 'R4_cw': 13, 'Q1_ccw': 13, 'Q1_cw': 13, 'Q2_ccw': 13, 'Q2_cw': 13}}

## Attempts

a_Q1_ccw_62: [{'attempt': 1, 'file': 'a_Q1_ccw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0624, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q1_ccw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0249, 'real_time_factor': 1.0, 'error': ''}]
a_Q1_ccw_63: [{'attempt': 1, 'file': 'a_Q1_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0518, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q1_ccw_63.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0502, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'a_Q1_ccw_63.attempt3.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0225, 'real_time_factor': 1.0, 'error': ''}]
a_Q1_cw_61: [{'attempt': 1, 'file': 'a_Q1_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0634, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q1_cw_61.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0595, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'a_Q1_cw_61.attempt3.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0518, 'real_time_factor': 1.0, 'error': ''}]
a_Q1_cw_62: [{'attempt': 1, 'file': 'a_Q1_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0434, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q1_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0246, 'real_time_factor': 1.0, 'error': ''}]
a_Q2_ccw_63: [{'attempt': 1, 'file': 'a_Q2_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0584, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q2_ccw_63.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0322, 'real_time_factor': 1.0, 'error': ''}]
a_Q2_cw_62: [{'attempt': 1, 'file': 'a_Q2_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0448, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_Q2_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0223, 'real_time_factor': 1.0, 'error': ''}]
a_R1_ccw_61: [{'attempt': 1, 'file': 'a_R1_ccw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0555, 'real_time_factor': 0.997, 'error': ''}, {'attempt': 2, 'file': 'a_R1_ccw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0236, 'real_time_factor': 1.0, 'error': ''}]
a_R1_ccw_62: [{'attempt': 1, 'file': 'a_R1_ccw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0657, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R1_ccw_62.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0514, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'a_R1_ccw_62.attempt3.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0237, 'real_time_factor': 1.0, 'error': ''}]
a_R1_ccw_63: [{'attempt': 1, 'file': 'a_R1_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0667, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R1_ccw_63.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0226, 'real_time_factor': 1.0, 'error': ''}]
a_R1_cw_61: [{'attempt': 1, 'file': 'a_R1_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0637, 'real_time_factor': 0.997, 'error': ''}, {'attempt': 2, 'file': 'a_R1_cw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0247, 'real_time_factor': 1.0, 'error': ''}]
a_R1_cw_62: [{'attempt': 1, 'file': 'a_R1_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0591, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R1_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0296, 'real_time_factor': 1.0, 'error': ''}]
a_R2_ccw_61: [{'attempt': 1, 'file': 'a_R2_ccw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0522, 'real_time_factor': 0.998, 'error': ''}, {'attempt': 2, 'file': 'a_R2_ccw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0278, 'real_time_factor': 1.0, 'error': ''}]
a_R2_cw_61: [{'attempt': 1, 'file': 'a_R2_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0797, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R2_cw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0296, 'real_time_factor': 1.0, 'error': ''}]
a_R2_cw_62: [{'attempt': 1, 'file': 'a_R2_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0642, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R2_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0252, 'real_time_factor': 1.0, 'error': ''}]
a_R3_ccw_61: [{'attempt': 1, 'file': 'a_R3_ccw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0603, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R3_ccw_61.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0612, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'a_R3_ccw_61.attempt3.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0219, 'real_time_factor': 1.0, 'error': ''}]
a_R3_ccw_63: [{'attempt': 1, 'file': 'a_R3_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0792, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R3_ccw_63.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0222, 'real_time_factor': 1.0, 'error': ''}]
a_R3_cw_61: [{'attempt': 1, 'file': 'a_R3_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0453, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R3_cw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0227, 'real_time_factor': 1.0, 'error': ''}]
a_R3_cw_62: [{'attempt': 1, 'file': 'a_R3_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0712, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'a_R3_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0351, 'real_time_factor': 1.0, 'error': ''}]
b_Q1_ccw_63: [{'attempt': 1, 'file': 'b_Q1_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0402, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_Q1_ccw_63.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0503, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'b_Q1_ccw_63.attempt3.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0223, 'real_time_factor': 1.0, 'error': ''}]
b_Q1_cw_61: [{'attempt': 1, 'file': 'b_Q1_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0657, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_Q1_cw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.023, 'real_time_factor': 1.0, 'error': ''}]
b_Q1_cw_62: [{'attempt': 1, 'file': 'b_Q1_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0679, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_Q1_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.025, 'real_time_factor': 1.0, 'error': ''}]
b_R3_ccw_61: [{'attempt': 1, 'file': 'b_R3_ccw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0485, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_R3_ccw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0266, 'real_time_factor': 1.0, 'error': ''}]
b_R3_ccw_63: [{'attempt': 1, 'file': 'b_R3_ccw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0475, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'b_R3_ccw_63.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0256, 'real_time_factor': 1.0, 'error': ''}]
c_Q1_cw_61: [{'attempt': 1, 'file': 'c_Q1_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0756, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_Q1_cw_61.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0332, 'real_time_factor': 1.0, 'error': ''}]
c_Q1_cw_62: [{'attempt': 1, 'file': 'c_Q1_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0627, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_Q1_cw_62.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.12, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'c_Q1_cw_62.attempt3.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.1221, 'real_time_factor': 1.0, 'error': ''}]
c_R1_cw_61: [{'attempt': 1, 'file': 'c_R1_cw_61.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0603, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_R1_cw_61.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0594, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 3, 'file': 'c_R1_cw_61.attempt3.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0269, 'real_time_factor': 1.0, 'error': ''}]
c_R1_cw_62: [{'attempt': 1, 'file': 'c_R1_cw_62.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0605, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_R1_cw_62.attempt2.json', 'starved': False, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0255, 'real_time_factor': 1.0, 'error': ''}]
c_R3_cw_63: [{'attempt': 1, 'file': 'c_R3_cw_63.attempt1.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 0.0613, 'real_time_factor': 1.0, 'error': ''}, {'attempt': 2, 'file': 'c_R3_cw_63.attempt2.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 965.46, 'real_time_factor': 0.066, 'error': ''}, {'attempt': 3, 'file': 'c_R3_cw_63.attempt3.json', 'starved': True, 'invalid_truth': False, 'recording_too_short': False, 'truth_invalid_reason': '', 'max_gap_s': 559.1289, 'real_time_factor': 0.179, 'error': ''}]

## Runs

| cond | case | dir | seed | laps | T1 s | T_late s | r | contacts | obstacle brakes per lap (late) | complete |
|---|---|---|---|---|---|---|---|---|---|---|
| a | Q1 | ccw | 61 | 13 | 26.37 | 26.06 | 0.988 | 0 | 0.00 | yes |
| a | Q1 | ccw | 62 | 13 | 26.29 | 26.09 | 0.992 | 0 | 0.00 | yes |
| a | Q1 | ccw | 63 | 13 | 26.26 | 26.12 | 0.995 | 0 | 0.00 | yes |
| a | Q1 | cw | 61 | 13 | 26.05 | 25.84 | 0.992 | 0 | 0.00 | yes |
| a | Q1 | cw | 62 | 13 | 26.05 | 25.83 | 0.992 | 0 | 0.00 | yes |
| a | Q1 | cw | 63 | 13 | 26.05 | 25.83 | 0.992 | 0 | 0.00 | yes |
| a | Q2 | ccw | 61 | 13 | 21.98 | 21.72 | 0.988 | 0 | 0.00 | yes |
| a | Q2 | ccw | 62 | 13 | 21.88 | 21.72 | 0.993 | 0 | 0.00 | yes |
| a | Q2 | ccw | 63 | 13 | 21.89 | 21.70 | 0.991 | 0 | 0.00 | yes |
| a | Q2 | cw | 61 | 13 | 21.76 | 21.56 | 0.991 | 0 | 0.00 | yes |
| a | Q2 | cw | 62 | 13 | 21.74 | 21.54 | 0.991 | 0 | 0.00 | yes |
| a | Q2 | cw | 63 | 13 | 21.75 | 21.53 | 0.990 | 0 | 0.00 | yes |
| a | R1 | ccw | 61 | 13 | 15.23 | 15.05 | 0.988 | 0 | 0.00 | yes |
| a | R1 | ccw | 62 | 13 | 15.21 | 15.03 | 0.988 | 0 | 0.00 | yes |
| a | R1 | ccw | 63 | 13 | 15.21 | 15.02 | 0.988 | 0 | 0.00 | yes |
| a | R1 | cw | 61 | 13 | 15.05 | 14.86 | 0.987 | 0 | 0.00 | yes |
| a | R1 | cw | 62 | 13 | 15.07 | 14.87 | 0.987 | 0 | 0.00 | yes |
| a | R1 | cw | 63 | 13 | 15.07 | 14.87 | 0.987 | 0 | 0.00 | yes |
| a | R2 | ccw | 61 | 13 | 19.70 | 19.46 | 0.988 | 0 | 0.00 | yes |
| a | R2 | ccw | 62 | 13 | 19.72 | 19.46 | 0.987 | 0 | 0.00 | yes |
| a | R2 | ccw | 63 | 13 | 19.74 | 19.43 | 0.984 | 0 | 0.00 | yes |
| a | R2 | cw | 61 | 13 | 19.48 | 19.18 | 0.985 | 0 | 0.00 | yes |
| a | R2 | cw | 62 | 13 | 19.45 | 19.19 | 0.987 | 0 | 0.00 | yes |
| a | R2 | cw | 63 | 13 | 19.46 | 19.18 | 0.986 | 0 | 0.00 | yes |
| a | R3 | ccw | 61 | 13 | 14.66 | 14.35 | 0.979 | 0 | 0.00 | yes |
| a | R3 | ccw | 62 | 13 | 14.63 | 14.34 | 0.980 | 0 | 0.00 | yes |
| a | R3 | ccw | 63 | 13 | 14.67 | 14.35 | 0.978 | 0 | 0.00 | yes |
| a | R3 | cw | 61 | 13 | 14.50 | 14.18 | 0.978 | 0 | 0.00 | yes |
| a | R3 | cw | 62 | 13 | 14.48 | 14.18 | 0.979 | 0 | 0.00 | yes |
| a | R3 | cw | 63 | 13 | 14.51 | 14.18 | 0.977 | 0 | 0.00 | yes |
| a | R4 | ccw | 61 | 13 | 15.71 | 15.51 | 0.987 | 0 | 0.00 | yes |
| a | R4 | ccw | 62 | 13 | 15.71 | 15.49 | 0.986 | 0 | 0.00 | yes |
| a | R4 | ccw | 63 | 13 | 16.28 | 15.50 | 0.952 | 0 | 0.00 | yes |
| a | R4 | cw | 61 | 13 | 15.58 | 15.32 | 0.983 | 0 | 0.00 | yes |
| a | R4 | cw | 62 | 13 | 15.55 | 15.35 | 0.987 | 0 | 0.00 | yes |
| a | R4 | cw | 63 | 13 | 15.61 | 15.33 | 0.982 | 0 | 0.00 | yes |
| b | Q1 | ccw | 61 | 13 | 11.79 | 11.42 | 0.969 | 0 | 0.00 | yes |
| b | Q1 | ccw | 62 | 13 | 11.78 | 11.42 | 0.969 | 0 | 0.00 | yes |
| b | Q1 | ccw | 63 | 13 | 11.74 | 11.48 | 0.978 | 0 | 0.00 | yes |
| b | Q1 | cw | 61 | 13 | 11.66 | 11.40 | 0.978 | 0 | 0.00 | yes |
| b | Q1 | cw | 62 | 13 | 11.66 | 11.35 | 0.974 | 0 | 0.00 | yes |
| b | Q1 | cw | 63 | 13 | 11.69 | 11.37 | 0.973 | 0 | 0.00 | yes |
| b | Q2 | ccw | 61 | 13 | 11.55 | 11.31 | 0.979 | 0 | 0.00 | yes |
| b | Q2 | ccw | 62 | 13 | 11.55 | 11.31 | 0.979 | 0 | 0.00 | yes |
| b | Q2 | ccw | 63 | 13 | 11.55 | 11.27 | 0.976 | 0 | 0.00 | yes |
| b | Q2 | cw | 61 | 13 | 11.48 | 11.26 | 0.981 | 0 | 0.00 | yes |
| b | Q2 | cw | 62 | 13 | 11.54 | 11.25 | 0.975 | 0 | 0.00 | yes |
| b | Q2 | cw | 63 | 13 | 11.50 | 11.23 | 0.976 | 0 | 0.00 | yes |
| b | R1 | ccw | 61 | 13 | 9.58 | 9.35 | 0.976 | 0 | 0.00 | yes |
| b | R1 | ccw | 62 | 13 | 9.49 | 9.39 | 0.989 | 0 | 0.00 | yes |
| b | R1 | ccw | 63 | 13 | 9.52 | 9.31 | 0.978 | 0 | 0.00 | yes |
| b | R1 | cw | 61 | 13 | 9.67 | 9.52 | 0.984 | 0 | 0.00 | yes |
| b | R1 | cw | 62 | 13 | 9.72 | 9.47 | 0.974 | 0 | 0.00 | yes |
| b | R1 | cw | 63 | 13 | 9.74 | 9.54 | 0.979 | 0 | 0.00 | yes |
| b | R2 | ccw | 61 | 13 | 9.19 | 8.85 | 0.963 | 0 | 0.00 | yes |
| b | R2 | ccw | 62 | 13 | 9.14 | 8.90 | 0.974 | 0 | 0.00 | yes |
| b | R2 | ccw | 63 | 13 | 9.18 | 8.85 | 0.964 | 0 | 0.00 | yes |
| b | R2 | cw | 61 | 13 | 8.82 | 8.50 | 0.964 | 0 | 0.00 | yes |
| b | R2 | cw | 62 | 13 | 8.84 | 8.56 | 0.968 | 0 | 0.00 | yes |
| b | R2 | cw | 63 | 13 | 8.85 | 8.53 | 0.964 | 0 | 0.00 | yes |
| b | R3 | ccw | 61 | 13 | 6.94 | 6.57 | 0.946 | 0 | 0.00 | yes |
| b | R3 | ccw | 62 | 13 | 6.89 | 6.56 | 0.952 | 0 | 0.00 | yes |
| b | R3 | ccw | 63 | 13 | 6.89 | 6.54 | 0.949 | 0 | 0.00 | yes |
| b | R3 | cw | 61 | 13 | 6.94 | 6.58 | 0.948 | 0 | 0.00 | yes |
| b | R3 | cw | 62 | 13 | 6.91 | 6.61 | 0.957 | 0 | 0.00 | yes |
| b | R3 | cw | 63 | 13 | 6.90 | 6.58 | 0.954 | 0 | 0.00 | yes |
| b | R4 | ccw | 61 | 13 | 8.02 | 7.74 | 0.965 | 0 | 0.00 | yes |
| b | R4 | ccw | 62 | 13 | 7.97 | 7.72 | 0.969 | 0 | 0.00 | yes |
| b | R4 | ccw | 63 | 13 | 7.93 | 7.70 | 0.971 | 0 | 0.00 | yes |
| b | R4 | cw | 61 | 13 | 8.14 | 7.90 | 0.970 | 0 | 0.00 | yes |
| b | R4 | cw | 62 | 13 | 8.15 | 7.85 | 0.963 | 0 | 0.00 | yes |
| b | R4 | cw | 63 | 13 | 8.11 | 7.86 | 0.969 | 0 | 0.00 | yes |
| c | Q1 | ccw | 61 | 13 | 26.36 | 11.49 | 0.436 | 0 | 0.00 | yes |
| c | Q1 | ccw | 62 | 13 | 26.27 | 11.49 | 0.437 | 0 | 0.00 | yes |
| c | Q1 | ccw | 63 | 13 | 26.27 | 11.53 | 0.439 | 0 | 0.00 | yes |
| c | Q1 | cw | 61 | 13 | 26.05 | 11.44 | 0.439 | 0 | 0.00 | yes |
| c | Q1 | cw | 62 | 13 | 26.06 | 11.43 | 0.439 | 0 | 0.00 | yes |
| c | Q1 | cw | 63 | 13 | 26.05 | 11.40 | 0.438 | 0 | 0.00 | yes |
| c | Q2 | ccw | 61 | 13 | 21.97 | 11.29 | 0.514 | 0 | 0.00 | yes |
| c | Q2 | ccw | 62 | 13 | 21.91 | 11.31 | 0.516 | 0 | 0.00 | yes |
| c | Q2 | ccw | 63 | 13 | 21.87 | 11.29 | 0.516 | 0 | 0.00 | yes |
| c | Q2 | cw | 61 | 13 | 21.73 | 11.24 | 0.517 | 0 | 0.00 | yes |
| c | Q2 | cw | 62 | 13 | 21.76 | 11.25 | 0.517 | 0 | 0.00 | yes |
| c | Q2 | cw | 63 | 13 | 21.76 | 11.20 | 0.515 | 0 | 0.00 | yes |
| c | R1 | ccw | 61 | 13 | 15.23 | 9.33 | 0.613 | 0 | 0.00 | yes |
| c | R1 | ccw | 62 | 13 | 15.20 | 9.39 | 0.618 | 0 | 0.00 | yes |
| c | R1 | ccw | 63 | 13 | 15.23 | 9.39 | 0.616 | 0 | 0.00 | yes |
| c | R1 | cw | 61 | 13 | 15.08 | 9.43 | 0.625 | 0 | 0.00 | yes |
| c | R1 | cw | 62 | 13 | 15.09 | 9.51 | 0.630 | 0 | 0.00 | yes |
| c | R1 | cw | 63 | 13 | 15.08 | 9.55 | 0.633 | 0 | 0.00 | yes |
| c | R2 | ccw | 61 | 13 | 19.70 | 8.92 | 0.453 | 0 | 0.00 | yes |
| c | R2 | ccw | 62 | 13 | 19.72 | 8.96 | 0.454 | 0 | 0.00 | yes |
| c | R2 | ccw | 63 | 13 | 19.72 | 8.93 | 0.453 | 0 | 0.00 | yes |
| c | R2 | cw | 61 | 13 | 19.46 | 8.62 | 0.443 | 0 | 0.00 | yes |
| c | R2 | cw | 62 | 13 | 19.43 | 8.61 | 0.443 | 0 | 0.00 | yes |
| c | R2 | cw | 63 | 13 | 19.47 | 8.54 | 0.439 | 0 | 0.00 | yes |
| c | R3 | ccw | 61 | 13 | 14.63 | 6.59 | 0.450 | 0 | 0.00 | yes |
| c | R3 | ccw | 62 | 13 | 14.59 | 6.60 | 0.452 | 0 | 0.00 | yes |
| c | R3 | ccw | 63 | 13 | 14.67 | 6.64 | 0.453 | 0 | 0.00 | yes |
| c | R3 | cw | 61 | 13 | 14.45 | 6.62 | 0.458 | 0 | 0.00 | yes |
| c | R3 | cw | 62 | 13 | 14.42 | 6.62 | 0.459 | 0 | 0.00 | yes |
| c | R3 | cw | 63 | 13 | 14.51 | 7.70 | 0.531 | 0 | 0.00 | yes |
| c | R4 | ccw | 61 | 13 | 15.70 | 7.75 | 0.494 | 0 | 0.00 | yes |
| c | R4 | ccw | 62 | 13 | 15.69 | 7.81 | 0.498 | 0 | 0.00 | yes |
| c | R4 | ccw | 63 | 13 | 15.68 | 8.04 | 0.513 | 0 | 0.00 | yes |
| c | R4 | cw | 61 | 13 | 15.59 | 7.88 | 0.505 | 0 | 0.00 | yes |
| c | R4 | cw | 62 | 13 | 15.54 | 7.88 | 0.507 | 0 | 0.00 | yes |
| c | R4 | cw | 63 | 13 | 15.56 | 7.84 | 0.504 | 0 | 0.00 | yes |

## Decisions

D8-S0: FAIL
  ok: S0a: the count rule holds in every condition (a) run: 36 of 36 runs agree []
  FAIL: S0a: every judged lap within its class tolerance: 5 of 432 laps outside ['R4/ccw/61 lap 1: error 0.90 s, tolerance 0.63 s', 'R4/ccw/62 lap 1: error 0.94 s, tolerance 0.63 s', 'R4/ccw/63 lap 1: error 1.30 s, tolerance 0.65 s', 'R4/cw/61 lap 1: error 0.92 s, tolerance 0.62 s', 'R4/cw/62 lap 1: error 1.00 s, tolerance 0.62 s']
  ok: S0a: no condition (a) run failed to execute: 0 errors []
  ok: S0b(c): the count rule holds in every run: 36 of 36 runs agree []
  FAIL: S0b(c): at least 98% of judged laps within their class tolerance: 422 of 432 laps within (0.9769); outside ['R4/ccw/61 lap 1: error 1.03 s, tolerance 0.63 s', 'R4/ccw/62 lap 1: error 0.99 s, tolerance 0.63 s', 'R4/ccw/63 lap 1: error 1.07 s, tolerance 0.63 s', 'R4/ccw/63 lap 6: error 0.82 s, tolerance 0.37 s', 'R4/ccw/63 lap 7: error 0.87 s, tolerance 0.37 s', 'R4/ccw/63 lap 10: error 0.75 s, tolerance 0.32 s']
  ok: S0b(c): no lap beyond its single-error limit: 0 laps beyond []
  ok: S0b(c): no run failed to execute: 0 errors []

D8-S1: PASS
  ok: 1. 0 wall contacts in the 36 runs of condition c: 0 contacts, runs with a contact: []
  ok: 2. runs of condition c complete 10 laps: 36 of 36 complete, not complete: []
  ok: 3. reach: T_late(c) <= 1.05 x T_late(b) in at least 33 of 36 runs: 35 of 36 reach; ratios [0.991, 0.997, 0.997, 0.997, 0.998, 0.998, 0.998, 1.0, 1.0, 1.0, 1.001, 1.001, 1.001, 1.002, 1.002, 1.003, 1.003, 1.004, 1.004, 1.004, 1.005, 1.006, 1.006, 1.006, 1.006, 1.006, 1.007, 1.007, 1.008, 1.009, 1.009, 1.012, 1.014, 1.015, 1.044, 1.17]
  ok: 4a. T_late below T1 in every completing run: 0 runs not below []
  ok: 4b. median r <= 0.8: median r 0.496 over 36 completing runs

## What this lets D do

D8-S0 or D8-S1 FAILED: D does not drive, the cap stays fixed (pre-registration section 6). A failed gate is a result: no tuning, no re-scoring.

## Reported without a threshold

Per case and condition:

| case | class | cond | runs | laps by line source | alias share (last lap, mean) | fixes last lap (mean) | stale scans | on_scan mean ms | on_scan p99 max ms |
|---|---|---|---|---|---|---|---|---|---|
| Q1 | placed | a | 6 | {'start_scan': 78} | 0.000 | 15.33 | 0 | 19.8 | 57.1 |
| Q1 | placed | b | 6 | {'start_scan': 78} | 0.000 | 14.83 | 0 | 17.9 | 54.5 |
| Q1 | placed | c | 6 | {'start_scan': 78} | 0.000 | 12.33 | 529 | 33.1 | 307.2 |
| Q2 | placed | a | 6 | {'start_scan': 77} | 0.061 | 11.50 | 0 | 19.9 | 55.1 |
| Q2 | placed | b | 6 | {'start_scan': 78} | 0.044 | 11.00 | 0 | 20.8 | 54.2 |
| Q2 | placed | c | 6 | {'start_scan': 78} | 0.069 | 11.17 | 0 | 20.6 | 51.8 |
| R1 | aliased | a | 6 | {'winding': 72} | 1.000 | 0.00 | 0 | 26.7 | 54.6 |
| R1 | aliased | b | 6 | {'winding': 75} | 1.000 | 0.00 | 0 | 25.8 | 83.1 |
| R1 | aliased | c | 6 | {'winding': 76} | 1.000 | 0.00 | 0 | 24.7 | 55.7 |
| R2 | aliased | a | 6 | {'winding': 72} | 1.000 | 0.00 | 0 | 21.8 | 54.2 |
| R2 | aliased | b | 6 | {'winding': 76} | 1.000 | 0.00 | 0 | 26.2 | 64.7 |
| R2 | aliased | c | 6 | {'winding': 77} | 1.000 | 0.00 | 0 | 25.3 | 54.5 |
| R3 | aliased | a | 6 | {'winding': 72} | 1.000 | 0.00 | 1 | 27.0 | 55.9 |
| R3 | aliased | b | 6 | {'winding': 78} | 1.000 | 0.00 | 0 | 23.7 | 59.5 |
| R3 | aliased | c | 6 | {'winding': 78} | 1.000 | 0.00 | 0 | 23.5 | 59.2 |
| R4 | aliased | a | 6 | {'start_scan': 64, 'winding': 12} | 0.440 | 5.83 | 0 | 23.9 | 55.9 |
| R4 | aliased | b | 6 | {'start_scan': 65} | 0.269 | 5.50 | 0 | 27.3 | 83.7 |
| R4 | aliased | c | 6 | {'anchor': 4, 'start_scan': 48, 'winding': 26} | 0.416 | 4.50 | 0 | 24.1 | 57.8 |

Lap 1, system against the registered truth:

| run | system lap 1 s | truth lap 1 s | difference s | launch dwell s | truth valid |
|---|---|---|---|---|---|
| a_Q1_ccw_61 | 26.31 | 26.37 | -0.06 | 0.01 | True |
| a_Q1_ccw_62 | 26.36 | 26.29 | +0.08 | 0.01 | True |
| a_Q1_ccw_63 | 26.31 | 26.26 | +0.05 | 0.01 | True |
| a_Q1_cw_61 | 26.10 | 26.05 | +0.06 | 0.01 | True |
| a_Q1_cw_62 | 26.09 | 26.05 | +0.04 | 0.01 | True |
| a_Q1_cw_63 | 26.10 | 26.05 | +0.05 | 0.01 | True |
| a_Q2_ccw_61 | 21.92 | 21.98 | -0.06 | 0.01 | True |
| a_Q2_ccw_62 | 21.94 | 21.88 | +0.06 | 0.01 | True |
| a_Q2_ccw_63 | 21.93 | 21.89 | +0.03 | 0.03 | True |
| a_Q2_cw_61 | 21.79 | 21.76 | +0.03 | 0.03 | True |
| a_Q2_cw_62 | 21.81 | 21.74 | +0.07 | 0.01 | True |
| a_Q2_cw_63 | 21.76 | 21.75 | +0.01 | 0.01 | True |
| a_R1_ccw_61 | 15.30 | 15.23 | +0.07 | 0.01 | True |
| a_R1_ccw_62 | 15.28 | 15.21 | +0.07 | 0.01 | True |
| a_R1_ccw_63 | 15.26 | 15.21 | +0.05 | 0.01 | True |
| a_R1_cw_61 | 15.01 | 15.05 | -0.04 | 0.01 | True |
| a_R1_cw_62 | 15.08 | 15.07 | +0.01 | 0.01 | True |
| a_R1_cw_63 | 15.11 | 15.07 | +0.04 | 0.01 | True |
| a_R2_ccw_61 | 19.91 | 19.70 | +0.21 | 0.01 | True |
| a_R2_ccw_62 | 19.96 | 19.72 | +0.24 | 0.05 | True |
| a_R2_ccw_63 | 19.98 | 19.74 | +0.24 | 0.03 | True |
| a_R2_cw_61 | 19.52 | 19.48 | +0.04 | 0.03 | True |
| a_R2_cw_62 | 19.51 | 19.45 | +0.06 | 0.01 | True |
| a_R2_cw_63 | 19.51 | 19.46 | +0.05 | 0.01 | True |
| a_R3_ccw_61 | 14.77 | 14.66 | +0.11 | 0.01 | True |
| a_R3_ccw_62 | 14.76 | 14.63 | +0.13 | 0.01 | True |
| a_R3_ccw_63 | 14.79 | 14.67 | +0.11 | 0.03 | True |
| a_R3_cw_61 | 14.57 | 14.50 | +0.07 | 0.03 | True |
| a_R3_cw_62 | 14.59 | 14.48 | +0.11 | 0.01 | True |
| a_R3_cw_63 | 14.54 | 14.51 | +0.03 | 0.01 | True |
| a_R4_ccw_61 | 14.81 | 15.71 | -0.90 | 0.01 | True |
| a_R4_ccw_62 | 14.77 | 15.71 | -0.94 | 0.03 | True |
| a_R4_ccw_63 | 14.98 | 16.28 | -1.30 | 0.01 | True |
| a_R4_cw_61 | 14.66 | 15.58 | -0.92 | 0.05 | True |
| a_R4_cw_62 | 14.55 | 15.55 | -1.00 | 0.01 | True |
| a_R4_cw_63 | 15.13 | 15.61 | -0.48 | 0.03 | True |
| b_Q1_ccw_61 | 11.76 | 11.79 | -0.03 | 0.01 | True |
| b_Q1_ccw_62 | 11.80 | 11.78 | +0.02 | 0.03 | True |
| b_Q1_ccw_63 | 11.75 | 11.74 | +0.02 | 0.01 | True |
| b_Q1_cw_61 | 11.69 | 11.66 | +0.04 | 0.01 | True |
| b_Q1_cw_62 | 11.68 | 11.66 | +0.02 | 0.03 | True |
| b_Q1_cw_63 | 11.71 | 11.69 | +0.02 | 0.03 | True |
| b_Q2_ccw_61 | 11.51 | 11.55 | -0.04 | 0.01 | True |
| b_Q2_ccw_62 | 11.56 | 11.55 | +0.01 | 0.03 | True |
| b_Q2_ccw_63 | 11.56 | 11.55 | +0.01 | 0.01 | True |
| b_Q2_cw_61 | 11.51 | 11.48 | +0.03 | 0.03 | True |
| b_Q2_cw_62 | 11.56 | 11.54 | +0.02 | 0.05 | True |
| b_Q2_cw_63 | 11.51 | 11.50 | +0.00 | 0.01 | True |
| b_R1_ccw_61 | 9.57 | 9.58 | -0.01 | 0.01 | True |
| b_R1_ccw_62 | 9.46 | 9.49 | -0.03 | 0.01 | True |
| b_R1_ccw_63 | 9.51 | 9.52 | -0.01 | 0.03 | True |
| b_R1_cw_61 | 9.59 | 9.67 | -0.08 | 0.01 | True |
| b_R1_cw_62 | 9.64 | 9.72 | -0.08 | 0.01 | True |
| b_R1_cw_63 | 9.68 | 9.74 | -0.06 | 0.03 | True |
| b_R2_ccw_61 | 9.20 | 9.19 | +0.01 | 0.01 | True |
| b_R2_ccw_62 | 9.15 | 9.14 | +0.01 | 0.01 | True |
| b_R2_ccw_63 | 9.19 | 9.18 | +0.01 | 0.01 | True |
| b_R2_cw_61 | 8.70 | 8.82 | -0.12 | 0.01 | True |
| b_R2_cw_62 | 8.78 | 8.84 | -0.06 | 0.01 | True |
| b_R2_cw_63 | 8.75 | 8.85 | -0.10 | 0.03 | True |
| b_R3_ccw_61 | 6.88 | 6.94 | -0.06 | 0.01 | True |
| b_R3_ccw_62 | 6.84 | 6.89 | -0.05 | 0.01 | True |
| b_R3_ccw_63 | 6.77 | 6.89 | -0.12 | 0.01 | True |
| b_R3_cw_61 | 6.82 | 6.94 | -0.12 | 0.03 | True |
| b_R3_cw_62 | 6.81 | 6.91 | -0.10 | 0.01 | True |
| b_R3_cw_63 | 6.79 | 6.90 | -0.11 | 0.01 | True |
| b_R4_ccw_61 | 7.45 | 8.02 | -0.57 | 0.01 | True |
| b_R4_ccw_62 | 7.46 | 7.97 | -0.51 | 0.01 | True |
| b_R4_ccw_63 | nan | 7.93 | +nan | 0.01 | True |
| b_R4_cw_61 | 7.66 | 8.14 | -0.48 | 0.01 | True |
| b_R4_cw_62 | 7.81 | 8.15 | -0.34 | 0.01 | True |
| b_R4_cw_63 | 7.56 | 8.11 | -0.55 | 0.01 | True |
| c_Q1_ccw_61 | 26.29 | 26.36 | -0.07 | 0.01 | True |
| c_Q1_ccw_62 | 26.32 | 26.27 | +0.05 | 0.01 | True |
| c_Q1_ccw_63 | 26.31 | 26.27 | +0.04 | 0.03 | True |
| c_Q1_cw_61 | 26.11 | 26.05 | +0.06 | 0.01 | True |
| c_Q1_cw_62 | 26.10 | 26.06 | +0.04 | 0.03 | True |
| c_Q1_cw_63 | 26.11 | 26.05 | +0.07 | 0.01 | True |
| c_Q2_ccw_61 | 21.92 | 21.97 | -0.05 | 0.01 | True |
| c_Q2_ccw_62 | 21.96 | 21.91 | +0.05 | 0.01 | True |
| c_Q2_ccw_63 | 21.91 | 21.87 | +0.04 | 0.01 | True |
| c_Q2_cw_61 | 21.79 | 21.73 | +0.06 | 0.01 | True |
| c_Q2_cw_62 | 21.82 | 21.76 | +0.06 | 0.01 | True |
| c_Q2_cw_63 | 21.79 | 21.76 | +0.03 | 0.03 | True |
| c_R1_ccw_61 | 15.37 | 15.23 | +0.14 | 0.01 | True |
| c_R1_ccw_62 | 15.30 | 15.20 | +0.10 | 0.01 | True |
| c_R1_ccw_63 | 15.35 | 15.23 | +0.12 | 0.01 | True |
| c_R1_cw_61 | 15.08 | 15.08 | +0.00 | 0.01 | True |
| c_R1_cw_62 | 15.17 | 15.09 | +0.08 | 0.05 | True |
| c_R1_cw_63 | 15.08 | 15.08 | +0.00 | 0.01 | True |
| c_R2_ccw_61 | 19.84 | 19.70 | +0.14 | 0.01 | True |
| c_R2_ccw_62 | 19.94 | 19.72 | +0.22 | 0.03 | True |
| c_R2_ccw_63 | 19.89 | 19.72 | +0.17 | 0.01 | True |
| c_R2_cw_61 | 19.50 | 19.46 | +0.04 | 0.01 | True |
| c_R2_cw_62 | 19.53 | 19.43 | +0.10 | 0.01 | True |
| c_R2_cw_63 | 19.49 | 19.47 | +0.02 | 0.01 | True |
| c_R3_ccw_61 | 14.77 | 14.63 | +0.14 | 0.01 | True |
| c_R3_ccw_62 | 14.72 | 14.59 | +0.13 | 0.01 | True |
| c_R3_ccw_63 | 14.84 | 14.67 | +0.17 | 0.05 | True |
| c_R3_cw_61 | 14.54 | 14.45 | +0.09 | 0.01 | True |
| c_R3_cw_62 | 14.50 | 14.42 | +0.08 | 0.01 | True |
| c_R3_cw_63 | 14.57 | 14.51 | +0.05 | 0.03 | True |
| c_R4_ccw_61 | 14.67 | 15.70 | -1.03 | 0.01 | True |
| c_R4_ccw_62 | 14.70 | 15.69 | -0.99 | 0.01 | True |
| c_R4_ccw_63 | 14.61 | 15.68 | -1.07 | 0.01 | True |
| c_R4_cw_61 | 14.99 | 15.59 | -0.60 | 0.01 | True |
| c_R4_cw_62 | 14.60 | 15.54 | -0.94 | 0.01 | True |
| c_R4_cw_63 | 15.19 | 15.56 | -0.37 | 0.01 | True |

lap 1, system minus truth: median |difference| 0.063 s, largest 1.304 s over 107 runs

The offset of crossing 12:

| run | crossing 12, system minus truth s | share of a lap |
|---|---|---|
| a_Q1_ccw_61 | -0.00 | -0.000 |
| a_Q1_ccw_62 | +0.11 | +0.004 |
| a_Q1_ccw_63 | -0.02 | -0.001 |
| a_Q1_cw_61 | +0.12 | +0.005 |
| a_Q1_cw_62 | +0.11 | +0.004 |
| a_Q1_cw_63 | +0.09 | +0.004 |
| a_Q2_ccw_61 | +0.12 | +0.006 |
| a_Q2_ccw_62 | -0.01 | -0.001 |
| a_Q2_ccw_63 | -0.02 | -0.001 |
| a_Q2_cw_61 | +0.09 | +0.004 |
| a_Q2_cw_62 | +0.01 | +0.001 |
| a_Q2_cw_63 | -0.04 | -0.002 |
| a_R1_ccw_61 | +1.24 | +0.082 |
| a_R1_ccw_62 | +1.12 | +0.075 |
| a_R1_ccw_63 | +1.22 | +0.081 |
| a_R1_cw_61 | +0.33 | +0.022 |
| a_R1_cw_62 | +0.35 | +0.024 |
| a_R1_cw_63 | +0.54 | +0.037 |
| a_R2_ccw_61 | +1.41 | +0.073 |
| a_R2_ccw_62 | +1.53 | +0.079 |
| a_R2_ccw_63 | +1.54 | +0.079 |
| a_R2_cw_61 | +0.61 | +0.032 |
| a_R2_cw_62 | +0.70 | +0.036 |
| a_R2_cw_63 | +0.59 | +0.031 |
| a_R3_ccw_61 | +1.06 | +0.074 |
| a_R3_ccw_62 | +1.13 | +0.079 |
| a_R3_ccw_63 | +1.04 | +0.073 |
| a_R3_cw_61 | +0.55 | +0.039 |
| a_R3_cw_62 | +0.41 | +0.029 |
| a_R3_cw_63 | +0.49 | +0.035 |
| a_R4_ccw_61 | -0.95 | -0.061 |
| a_R4_ccw_62 | -0.96 | -0.062 |
| a_R4_ccw_63 | +0.62 | +0.040 |
| a_R4_cw_61 | -1.01 | -0.066 |
| a_R4_cw_62 | -0.96 | -0.063 |
| a_R4_cw_63 | +0.16 | +0.011 |
| b_Q1_ccw_61 | +0.03 | +0.003 |
| b_Q1_ccw_62 | +0.07 | +0.006 |
| b_Q1_ccw_63 | +0.03 | +0.002 |
| b_Q1_cw_61 | +0.07 | +0.006 |
| b_Q1_cw_62 | +0.06 | +0.006 |
| b_Q1_cw_63 | +0.08 | +0.007 |
| b_Q2_ccw_61 | +0.06 | +0.006 |
| b_Q2_ccw_62 | -0.04 | -0.003 |
| b_Q2_ccw_63 | -0.00 | -0.000 |
| b_Q2_cw_61 | +0.08 | +0.007 |
| b_Q2_cw_62 | +0.03 | +0.002 |
| b_Q2_cw_63 | +0.01 | +0.001 |
| b_R1_ccw_61 | +0.52 | +0.056 |
| b_R1_ccw_62 | +0.48 | +0.051 |
| b_R1_ccw_63 | +0.56 | +0.060 |
| b_R1_cw_61 | -0.01 | -0.001 |
| b_R1_cw_62 | -0.02 | -0.002 |
| b_R1_cw_63 | +0.04 | +0.004 |
| b_R2_ccw_61 | +0.37 | +0.042 |
| b_R2_ccw_62 | +0.47 | +0.053 |
| b_R2_ccw_63 | +0.40 | +0.045 |
| b_R2_cw_61 | +0.22 | +0.026 |
| b_R2_cw_62 | +0.13 | +0.015 |
| b_R2_cw_63 | +0.17 | +0.020 |
| b_R3_ccw_61 | +0.12 | +0.018 |
| b_R3_ccw_62 | +0.18 | +0.028 |
| b_R3_ccw_63 | +0.06 | +0.009 |
| b_R3_cw_61 | -0.06 | -0.008 |
| b_R3_cw_62 | -0.02 | -0.003 |
| b_R3_cw_63 | -0.09 | -0.013 |
| b_R4_ccw_61 | -0.57 | -0.073 |
| b_R4_ccw_62 | -0.43 | -0.056 |
| b_R4_ccw_63 | +nan | +nan |
| b_R4_cw_61 | -0.44 | -0.056 |
| b_R4_cw_62 | -0.41 | -0.052 |
| b_R4_cw_63 | -0.49 | -0.062 |
| c_Q1_ccw_61 | +0.01 | +0.001 |
| c_Q1_ccw_62 | +0.07 | +0.005 |
| c_Q1_ccw_63 | +0.02 | +0.001 |
| c_Q1_cw_61 | +0.06 | +0.005 |
| c_Q1_cw_62 | +0.05 | +0.004 |
| c_Q1_cw_63 | +0.07 | +0.006 |
| c_Q2_ccw_61 | +0.07 | +0.006 |
| c_Q2_ccw_62 | +0.02 | +0.002 |
| c_Q2_ccw_63 | -0.03 | -0.003 |
| c_Q2_cw_61 | +0.13 | +0.011 |
| c_Q2_cw_62 | +0.05 | +0.005 |
| c_Q2_cw_63 | -0.07 | -0.006 |
| c_R1_ccw_61 | +0.58 | +0.062 |
| c_R1_ccw_62 | +0.43 | +0.046 |
| c_R1_ccw_63 | +0.64 | +0.068 |
| c_R1_cw_61 | +0.11 | +0.012 |
| c_R1_cw_62 | +0.11 | +0.011 |
| c_R1_cw_63 | +0.09 | +0.010 |
| c_R2_ccw_61 | +0.45 | +0.045 |
| c_R2_ccw_62 | +0.52 | +0.053 |
| c_R2_ccw_63 | +0.50 | +0.051 |
| c_R2_cw_61 | +0.16 | +0.016 |
| c_R2_cw_62 | +0.25 | +0.026 |
| c_R2_cw_63 | +0.20 | +0.021 |
| c_R3_ccw_61 | +0.26 | +0.036 |
| c_R3_ccw_62 | +0.24 | +0.033 |
| c_R3_ccw_63 | +0.22 | +0.030 |
| c_R3_cw_61 | +0.09 | +0.012 |
| c_R3_cw_62 | -0.00 | -0.000 |
| c_R3_cw_63 | -0.74 | -0.080 |
| c_R4_ccw_61 | -0.40 | -0.047 |
| c_R4_ccw_62 | -0.46 | -0.054 |
| c_R4_ccw_63 | -0.45 | -0.049 |
| c_R4_cw_61 | +0.00 | +0.000 |
| c_R4_cw_62 | -0.43 | -0.050 |
| c_R4_cw_63 | +0.18 | +0.022 |

Condition b against the same tolerances: {'runs': 36, 'laps': 432, 'within': 415, 'count_bad': 1}

- condition a: 0 laps flagged missed gate in 0 of 36 runs
- condition b: 0 laps flagged missed gate in 0 of 36 runs
- condition c: 0 laps flagged missed gate in 0 of 36 runs
