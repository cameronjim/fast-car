# Lap counter: driving reference and alias guard (S4)

Launch arg (lap_counter.launch.py, node params): `driving_reference` (default false), `lap_gate_ratio` (default 0.85).
Off: counting, events and status are the registered 51c ones. On: the peak frame of lap 1 becomes a second reference, similarity is the max of both from lap 2, the distance gate is `lap_gate_ratio` x the median of the last 3 lap distances (lap 1 on the fixed 3 m gate), events carry `reference`.

Replay (exact /lap/thumb, tools/lap_replay.py; truth = crossings every L m from the standing-reference peak grid; false = event more than 0.6 m off the grid, missed = grid crossing with no event):

| run | before false/missed | ratio 0.75 | ratio 0.85 |
|---|---|---|---|
| 11-23 D v0 cw geometric (27 crossings) | 14 / 13 | 15 / 14 | 0 / 0 |
| 11-23 observe cw | 1 / 0 | 5 / 4 | 1 / 0 |
| 11-23 net ccw | 0 / 0 | 0 / 0 | 0 / 0 |
| 11-23 net cw | 0 / 0 | 3 / 3 | 0 / 0 |
| 11-22 D v0 cw (record stream, approx) | 0 / 0 | 12 / 12 | 0 / 0 |
| 11-22 observe 210414 (approx) | 0 / 2 | 0 / 6 | 0 / 6 |
| 11-22 cw-check (approx) | 3 / 4 | 2 / 3 | 3 / 4 |

Findings: the cw alias sits at 0.77 to 0.78 of the lap, so decision 21's 0.75 does not stop it, and the driving reference adds its own early alias that 0.75 lets through. 0.85 clears tonight's data. It loses crossings when lap 1 swallowed a missed crossing (11-22 observe 210414: lap 1 = 2 laps raises the gate). Hence default off.
