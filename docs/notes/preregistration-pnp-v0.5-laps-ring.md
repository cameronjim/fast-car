**Pre-registration: lap gate v3 (keyframe ring), fourth sim attempt (B8) and car gate D8-C0.**
Written 2026-11-17, before any code of brief 50 and before any B8 run.

**0. Known when written.** B7 (v0.5 laps, sections 2 and 3): D7-S0 FAIL, D7-S1 PASS. Every
failing item is case P4 (13 laps in condition a and 14 in condition c against 12, in all 12
runs; worst lap error 2.01 s; lap 1 short by 0.94 to 1.37 s). P1 to P3: 0 of 432 laps
outside 0.3 s. D7-C0 was not computed. Those results stand and are not re-scored. B7's raw
run files were not opened to design this attempt and are not opened to build it: the design
rests on the B7 note, on P4's track definition (a round island in a round wall, no straight,
so a scan does not say where along the lap the car is) and on development data (B4's stored
runs, the 13 development dumps of brief 48, the 2026-11-16 car bags). From those dumps: the
gate's odometry heading over one true lap is 0.982 to 1.025 of a full turn in the simulator.
P1 to P4 are development geometry from now on, with seeds 11 to 13 only. Owner decision of
2026-11-17, item 8: the keyframe ring is a new registered attempt and B7 stands.

**1. System under test.** Lap gate v3: gate v2.1 plus the uniqueness test, the keyframe ring
and the winding closure of brief 50, at its default configuration, frozen at one commit
recorded in `evaluation/prereg/freeze/b8.json` before the first run.

**2. Battery B8.** Cases R1 to R4, Q1 and Q2 of the yaml, never run by anyone; seeds 61, 62,
63; ccw and cw; a standing start; lap-1 cap 0.5 m/s; ceiling, plant, noise, lap source and
the node's yaw gains as B7. Conditions a (D off, cap 0.5), b (D off, ceiling), c (D v0). 36
runs per condition. Each run drives 13 ground-truth laps and laps 1 to 12 are judged. Two
classes, fixed here by the track and not by anything the system reports. Aliased: the
centreline has no straight longer than the lane is wide (R1 to R4), so one place cannot be
told from its neighbours by a scan, or only at one spot (R3's box). Placed: Q1 and Q2. A
case is dropped if condition a cannot complete 3 laps on it in either direction with seed
61; each dropped case removes 6 runs per condition. At most two cases may be dropped, and
at least two aliased cases and one placed case must be left; otherwise nothing is decided
and the owner is told.

**3. Truth and counting.** Ground truth as v0.5 laps section 2, same words: crossing 0 is
the stamp of the first `/scan` at or after the planner's first DRIVING report, the start
line is the ground-truth pose at that stamp, lap k is the time between ground-truth
crossings k-1 and k, lap 1 is the launch lap for the system and the truth alike, and the
truth never reads the system's lap events. The system's crossing 0 is its start scan stamp
and its lap k is the time between its crossings k-1 and k. Count rule: the number of system
crossings stamped at or before truth crossing 12 plus half of truth lap 12 is exactly 12.
That is "no lap gained and no lap lost" without asking an odometry-closed lap to keep its
phase to a scan's precision. A system lap that does not exist counts as outside every
tolerance. Invalid runs as v0.5 laps, and also a run whose recording ends before the count
window does: rerun alone, at most 3 attempts, all kept; still invalid counts as failed to
execute. Starved runs as B4.

**4. Gates.** Tolerance of a lap against its truth lap time T: placed class 0.3 s; aliased
class the larger of 0.3 s and 0.04 T. Largest single error allowed in condition c: placed
class 1.0 s; aliased class the larger of 1.0 s and 0.08 T. Why a share of the lap for the
aliased class: where no place is distinctive the line can only be carried by odometry, whose
error is a share of a lap and not a distance. 0.04 was chosen before any code: above the
0.025 seen in development, below the 0.058 that the station memory's own filter (0.4 m either
side) allows on the shortest aliased case. D8-S0: S0a on condition a (the count rule in
every run, every judged lap within tolerance) and S0b on condition c (the count rule in
every run, at least 98 percent of judged laps within tolerance, none beyond the single-error
limit), no run failed to execute. Lap 1 is inside both items, with no allowance. D8-S1: the
four items of D3-S1 on conditions c and b, with 33 of 36 where D3 says 22 of 24 (28 of 30
after one dropped case, 22 of 24 after two). Reported with no threshold: per case the laps
by line source (start scan, anchor, winding), the share of start-scan matches judged
ambiguous, keyframe fixes per lap, scans skipped for stale odometry, `on_scan` cost; per run
lap 1 against truth, launch dwell, and the offset of crossing 12 as a share of a lap;
condition b's laps against the same tolerances; missed-gate flags per condition.

**5. On the car, D8-C0** (D3-C0 and the D7-C0 rule are computed next to it by the same tool,
`pnp_session lap-c0`). The freeze commit, lap memory in observe mode, two runs of at least
10 laps, one per direction, the first 10 laps of each judged, from the recorded lap events:
(i) the lap count equals the reference's in both runs; (ii) of the 18 lap times of laps 2 to
10, at least 17 are within 0.3 s of the reference; (iii) system crossing 1 minus reference
crossing 1 is within 0.3 s of the median of that difference over crossings 2 to 10 of the
same run, in both runs; (iv) no judged lap is flagged missed gate; (v) the lap node's
`on_scan` p99 is at most 80 ms in both runs. Reference and its validity rule as v0.5 laps
section 4 and owner decision 4 of 2026-11-16 (camera lap counter, phone video as backup); a
reference that fails the rule means the run is repeated, neither pass nor fail. Reported:
the line source of every lap. The tolerance on the car is 0.3 s whatever the source: a
winding lap gets no allowance there.

**6. What lets D drive.** Nothing in this file by itself. If the owner answers yes to
question 1 below: D v0 may drive on the plain donut in a session when D8-S0 and D8-S1 passed
on B8 at the freeze commit, that commit is what runs on the car, and D8-C0 passed earlier in
the same session; lap-1 cap 0.5 m/s, 10 laps per direction. D v1 does not drive under this
file. Any fail: D does not drive; the cap stays fixed.

**7. Stop rules.** (a) B8 runs once, on the freeze commit. A failed gate is a result: no
tuning, no re-scoring, no reading of B8's run files beyond the report; what comes next is
the owner's call. (b) There is no freeze and no B8 run unless the six development checks of
brief 50 (A1 to A6, written in the brief before any code) all hold; a failed check is
reported to the launcher. (c) If the synthetic ring test of the brief is green on gate v2.1,
the reading of B7 behind this attempt is wrong and the work stops there. (d) No R or Q case
and no seed 61 to 63 runs before the freeze file exists; a test enforces it.

**8. Owner decisions.** Given: 2026-11-17 item 8 (this attempt; B7 stands). Asked with this
file and not yet given, so until they are a D8 result decides nothing about driving: (1) a
D8-S0 plus D8-S1 pass, with D8-C0 in the session, lets D v0 drive, D3-S0 and D7-S0 staying
FAIL as registered; (2) that pass stands in for "D3-S0 and D3-S1 passed" in front of B5 and
B6; (3) the aliased-class tolerance (a share of the lap, and the count rule of section 3) is
accepted as the measure of a lap gate where no start line can be seen; (4) the D8-C0
reference is the camera lap counter with the phone video as backup, as for D7-C0.
