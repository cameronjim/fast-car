# Lap node replay of the 2026-11-16 car bags (M-R2, brief 48)

Development and regression data: these bags were replayed while brief 48 was designed, so this
decides nothing and no gate reads it. It shows what the lap node's logic does with them after the
fixes of brief 48 (lap 1 as the launch lap, the false missed gate, the parked last lap) against
main as it was (PR 191 and PR 201). Produced by
`ros_ws/src/racer_laps/tests/fixtures_car_2026_11_16/m_r2_table.py` (`LapMemory` fed every message
of the bag from its start, D v0 defaults, the car's LiDAR mount, `racer_laps.replay`). Times are
bag seconds after the first `/scan`. "Camera" is the bag's `/lap/event`, the camera lap counter.

## 2026-11-16T22-38-47 (GP3-R1, cap 1.0, counter-clockwise)

| | expected after the fixes | after (this PR) | on main today |
|---|---|---|---|
| laps | 20 | 20 | 20 |
| start scan | the first scan after DRIVING (about 42.9 s) | 42.95 s | 42.95 s (node rule); 43.19 s from the replay CLI, which cut at the last standstill odometry sample |
| lap 1 | 5.99 s within 0.05 | 5.99 s (launch dwell 0.30 s) | 5.99 s (node rule); 5.70 s from the replay CLI |
| laps 2 to 20 against the camera | each within 0.3 s | 19 of 19, largest 0.139 s | 19 of 19, largest 0.139 s |
| lap-1 crossing offset against the camera | within 0.3 s of the median offset of crossings 2 to 20 | median offset -0.340 s, lap 1 -0.394 s, difference -0.054 s | the same |
| missed gate flags | 0, and 20 clean | 0 missed, 0 unclean | laps 15 and 17 missed and unclean |
| cap | 1.0 from lap 6 and never lower | 1.0 from lap 6, final 1.0 | falls to 0.8 after lap 15 |

The camera's lap 1 is 8.00 s because its reference frame was taken 1.9 s before the car moved: lap
1 is compared by crossing instant, not duration (pre-registration D7-C0 item iii).

## 2026-11-16T02-18-54 (the earlier 20 lap attempt, camera counted 10)

| | expected after the fixes | after (this PR) | on main today |
|---|---|---|---|
| laps | 20, the last one closed within 1 s of standstill | 20 | 19 |
| last lap decided at | within 1 s of standstill | bag 390.77 s, standstill 390.76 s (0.02 s after) | the 19th at 384.98 s, then 16 to 17 live candidates every 1.5 s for minutes |
| missed gate flags | 0 | 0 | lap 13 |

The camera counted 10 events for the 20 laps, so no per lap comparison with it is made. Lap 1 is
6.90 s (launch dwell 0.17 s).

## 2026-11-16T22-45-39, T23-04-09, T02-29-27 (reported, no expectation)

These bags hold several launches of the planner and gap follow blocks; one lap node run over all
of one bag is not what the car did, so the counts below mean little. `22-45-39`: 16 laps, 4
flagged missed, 11 unclean. `23-04-09`: 34 laps, 17 flagged missed, 22 unclean. `02-29-27` (the bag the recorder did not close): 0 laps, an idle car.

## The 2026-11-13 M-R set

`python -m racer_laps.replay --m-r` on the pre-registered split gives the same table on main and
after this PR, line by line: no lap count lower than main, no lap time moved, so the M-R golden is
not re-derived. (Its verdict lines read FAIL R6, G1a and T0 on both, as they did when v0.3 was
registered: they are not gates of brief 48.)

## The two L4 fixtures

`ros_ws/src/racer_laps/tests/fixtures_car_2026_11_16/` holds two slices of these bags, cut so that
their start line is the full run's (see `make_car_fixtures.py`): GP3-R1 bag seconds 116.5 to 141.0
(brief 48 says 115 to 141, but at 115 s the slice's start line is 1.6 m before the full run's and
main flags nothing there) and 378.8 to 395.8 of `02-18-54` (12 s of driving and 5 s parked; the
brief says the last 14 s). On main the first slice flags its last lap missed and the cap falls;
the second never closes its last lap. After this PR the first gives 4 clean laps and the second 2
closed laps.
