# Runs and blocks of the 2026-11-15 session, and how much of each bag was readable

Written before any v0.5 fit (brief 47 step 4). It maps what the tools find in the five bags to the
blocks of the excitation plan. Nothing in the split depends on this table: the rule
`session_2026_11_15` is one rule for every run (see `dynamics-v05-session-identity.md`). It is for
the reader and for the report.

## How much of each bag was readable

`mcap` StreamReader, every file read to its end; the footer and summary are checked separately.

| Bag (UTC name) | File size | Footer and summary | Messages read | Span of `/drive` | `/scan` messages |
|---|---|---|---|---|---|
| 2026-11-16T02-18-54_car_teleop | 118 323 976 B | yes | 367 695 of 367 695 | 620.0 s | 6201 |
| 2026-11-16T02-29-27_car_teleop | 22 835 200 B | NO, the file ends inside a record | 93 250 (all that is there) | 164.8 s | 1636 (163.2 s) |
| 2026-11-16T22-38-47_car_teleop | 76 578 435 B | yes | 236 735 of 236 735 | 385.4 s | 3855 |
| 2026-11-16T22-45-39_car_teleop | 208 814 191 B | yes | 659 832 of 659 832 | 1100.0 s | 10 989 |
| 2026-11-16T23-04-09_car_teleop | 148 791 486 B | yes | 430 203 of 430 203 | 711.0 s | 7121 |

Four bags are complete. `02-29-27` has no footer: it was read up to the truncated tail
(`unpack requires a buffer of 8 bytes`), which is the end of the file, so nothing readable was
left out. It holds no `/drive_raw` and no planner status, every `/drive` command is zero and the
wheel odometry never moves (maximum 0.0 m/s): the car did not drive in the part that exists. It is
kept in the session (nothing is dropped) and contributes no run, because the run definition needs a
non-zero command or a status stretch and there is none.

## Runs found by `tools/pnp_data.py runs-v05` and the block each belongs to

Times are bag seconds after the first `/scan`. The block is the launch log of
`/Users/cameronjim/code/car-data/pnp/exc-2026-11-16/` whose ROS-time interval overlaps the run most
(`tools/pnp_data.py map-v05`). The two GP3 runs and the aborted launch have no launch log.
"Command direction" is the sign of the mean steering command; the direction used by the scoring is
the measured T2 heading of each window, not this column and not the block name.

| Bag | Run | Kind | From | To | Duration s | Wheel distance m | Command direction | Block | Note |
|---|---|---|---|---|---|---|---|---|---|
| 02-18-54 | R0 | pnp | 276.1 | 390.0 | 113.9 | 103.1 | ccw | none (no log) | PnP cap 1.0 ccw, 20 laps (owner) |
| 02-29-27 | none | | | | | | | | car did not drive |
| 22-38-47 | R0 | pnp | 40.4 | 157.4 | 117.0 | 106.0 | ccw | none (no log) | GP3-R1, PnP cap 1.0 ccw, 20 laps (owner) |
| 22-45-39 | R0 | pnp | 190.9 | 194.6 | 3.7 | 1.1 | ccw | none (no log) | the second aborted P launch (lap counter requirement refusal after about 3 s of driving, owner) |
| 22-45-39 | R1 | pnp | 262.5 | 365.0 | 102.4 | 77.1 | ccw | P_ccw | planned 96 s |
| 22-45-39 | R2 | drive_raw | 439.3 | 471.4 | 32.1 | 18.5 | ccw | G_ccw_1.0 | extra to the plan; safety refusals and reverse escapes (owner log) |
| 22-45-39 | R3 | drive_raw | 612.6 | 660.0 | 47.4 | 28.3 | ccw | G_ccw_1.3 | safety refusals and reverse escapes (owner log) |
| 22-45-39 | R4 | drive_raw | 733.7 | 775.7 | 42.0 | 14.4 | ccw | L_ccw_0.6 | rear of the car near the wall about 3 times, no contact (owner) |
| 22-45-39 | R5 | drive_raw | 844.8 | 867.2 | 22.4 | 10.8 | ccw | F_ccw_1.4 | 2 reverse escapes, no contact (owner) |
| 22-45-39 | R6 | drive_raw | 967.2 | 1030.3 | 63.1 | 35.1 | ccw | F_ccw_1.5 | 9 reverse escapes; three in a row turned the car round and it finished driving CLOCKWISE (owner) |
| 23-04-09 | R0 | pnp | 68.3 | 170.0 | 101.6 | 77.0 | cw | P_cw | planned 96 s |
| 23-04-09 | R1 | drive_raw | 357.9 | 390.3 | 32.4 | 25.8 | cw | G_cw_1.0 | |
| 23-04-09 | R2 | drive_raw | 401.7 | 450.5 | 48.8 | 40.5 | cw | G_cw_1.3 | |
| 23-04-09 | R3 | drive_raw | 502.3 | 546.2 | 43.9 | 15.1 | cw | L_cw_0.6 | |
| 23-04-09 | R4 | drive_raw | 563.8 | 586.8 | 23.0 | 20.3 | cw | F_cw_1.4 | |
| 23-04-09 | R5 | drive_raw | 603.1 | 667.0 | 64.0 | 63.3 | cw | F_cw_1.5 | |

The first aborted P launch (a bad bundle path) drove nothing and has no run.

## The run definition in bags that mix drivers

The yaml defines a run as a stretch of `/pnp/planner/status` with no gap above 2 s, "or in a bag
without planner status a stretch of non-zero `/drive_raw` commands". `22-45-39` and `23-04-09` hold
both: a PnP block and then gap follow blocks with no status. Read literally, only the PnP stretch is
a run there and every gap follow block (G, L, F) would fall outside the rule, which contradicts
"every run of every bag of the session, whatever drove it". The tool therefore applies the second
clause to the non-zero command stretches that no status stretch covers (`run_rows_all`); in a bag
with no status this is exactly the yaml's text. This is an interpretation, listed in the first PR
comment for the owner.

## Deviations from the excitation plan (reported, not fixed)

From `DEVIATIONS.txt` and the owner's observations, unchanged:

- The counter-clockwise blocks were driven first (the plan says clockwise first).
- G at 1.0 was also driven counter-clockwise (extra to the plan).
- Block P used `require_lap_counter:=false`. Two P launches were aborted early in `22-45-39`: a bad
  bundle path, then a lap counter requirement refusal after about 3 s of driving.
- Gap follow blocks G and F had many safety refusals and reverse escapes: G 1.0 ccw 3, G 1.3 ccw 4
  forward refusals each followed by a reverse escape; F 1.4 ccw 2 reverse escapes; F 1.5 ccw 9.
  No contact in any block.
- L 0.6 ccw: the rear of the car came very close to the wall about 3 times; no stall, no refusal.
- F 1.5 ccw ended driving clockwise after three reverse escapes in a row turned the car round, so a
  label from the block name is wrong for its tail; the direction of every window comes from the
  measured heading.
