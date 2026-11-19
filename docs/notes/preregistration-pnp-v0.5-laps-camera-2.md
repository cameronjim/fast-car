**Pre-registration: second attempt at lap learning on an external lap source (camera on the
car, truth oracle in sim), with a start gate; battery B10, gates D10-S0, D10-S1, D10-S2, car
gates D10-P0 and D10-C0.**
Written 2026-11-18, before any code of brief 51c and before any B10 run.

**0. Known when written.** B9 (freeze `7098fcd`, run once): D9-S0 FAIL and D9-S1 FAIL, both
from one run (c U2 ccw seed 72) that started not ready; D9-S2 PASS; D9-B0 INSUFFICIENT
(items i and ii PASS over 23 laps, no `lapref` period). B8: D8-S0 FAIL, D8-S1 PASS. D7-S1
PASS, D7-S0 and D3-S0 FAIL. All stand. No B9 run file was opened and none will be. Read to
design this attempt: the B9 report; the code at the freeze (the sim oracle's first status is
published after the planner's first `/drive_raw`; the lap node fixes ready at the first
odometry after the first DRIVING report; on the car the launch starts the planner once the
counter is in the graph); the 16 development runs of brief 51 (6 or 7 STARTING reports before
DRIVING in 3 of 10 c and f runs); development bag `2026-11-16T02-18-54` (the camera's first
status 2.4 s after the first DRIVING report). Why the B9 run started not ready is a
hypothesis and stays one.

**1. System under test.** As pre-registration v0.5 laps camera section 1 (the source rule
and its constants, unchanged, repeated in this yaml), plus the start gate of brief 51c with
the yaml's `start_gate`: the starter (the launch on the car, the runner in sim) does not
start the planner until the lap node on the external source has said `source_ready_now` and
`source_alive` on 2 consecutive statuses at most 2.5 s apart, waits at most 30 s, and then
refuses with the planner never started. One pure rule serves both starters. The node's own
rule is unchanged: a run that starts not ready is doubtful on every lap. Frozen at one
commit recorded in `evaluation/prereg/freeze/b10.json` before the first B10 run.

**2. Battery B10 (sim, truth oracle).** Cases V1, V2, V3 of the yaml, never run by anyone;
seeds 81, 82; ccw and cw; standing start; lap-1 cap 0.5 m/s; ceiling, plant and noise as B9.
Conditions a (D off, cap 0.5; seed 81 only, for the drop rule), b (D off at the ceiling), c
(D v0, `lap_source truth`), f (D v0, `lap_source truth`, B9's fault plan, unchanged), 12
runs in each of b, c and f; and r (D v0, `lap_source truth`, the oracle never ready), 3 runs:
each case, ccw, seed 81. In c and f the oracle is not ready until 0 s (seed 81) or 6 s (seed
82) of sim time after the lap node's first status. Each run of a, b, c, f drives 13
ground-truth laps and laps 1 to 12 are judged. Drop rule, ground truth, validity, starved
and invalid runs as B9. A c or f run the starter refused is a run that failed to execute and
is not repeated. U1 to U3, seeds 71 and 72 and B9's run files are not run or opened.

**3. Sim gates.** Thresholds are D9's.
D10-S0 (the start, the oracle and the wiring). Condition c, every run: (1) the planner was
started only after the start gate said start, the first DRIVING report is later than that,
and the lap node started ready; (2) the oracle publishes exactly one event per registered
truth crossing 1 to 12, each within 0.3 s of it; (3) the system closes laps 1 to 12, all
confirmed, none doubtful, no event rejected; (4) every system crossing is within 0.3 s of
the truth crossing of the same number; (5) no run failed to execute. Condition r, every run:
(6) the starter refused within 35 s of sim time of the lap node's first status, the planner
was never started, no message was published on `/drive_raw`, the ground-truth path is at
most 0.05 m, and the refusal names the source's reason.
D10-S1 (the learning arrives through the path): the four items of D3-S1 on conditions c and
b, with 11 of 12 where D3 says 22 of 24 (7 of 8 after one dropped case). As D9-S1.
D10-S2 (the fallback in closed loop), condition f, every run: items (1) to (6) of D9-S2 as
registered, and (7) the run started ready through the gate as D10-S0 item (1).
Reported with no threshold: the start trace of every run, the largest source status gap,
late lap times of f against c, event latency, the reference length per run, every doubtful
lap with its reasons.

**4. Offline.** There is no D10-B0. D9-B0 stands as scored once: (i) and (ii) PASS over 23
judged laps, (iii) INSUFFICIENT. The two evaluation bags are read and are not scored again.
D9-B0 carries to this attempt only if brief 51c's check G1 holds: byte-identical lap results
on the L4 fixtures and on development bags `22-38-47` and `02-18-54`. If G1 fails there is
no freeze. Item (iii)'s question is asked on the car by D10-C0, whose reference is the
owner's count. Reported with no threshold: `lapref`'s period and autocorrelation peak on
every D10-C0 bag.

**5. On the car.**
D10-P0 (every launch with a camera-source lap node; enforced by the launch and the node, not
by a person): the planner is started only after the counter is in the graph, the reset
returned, and the start gate said start; at the lap node's start the camera status is at
most 3.0 s old, says `running`, 0 laps and a similarity of at least 0.90. Gate not met: the
launch refuses and the car does not drive. Node rule not met: every lap of that run is
doubtful and the cap stays at the lap-1 cap.
D10-C0 (the test pass; per direction): items (i) to (v) of D9-C0 as registered, with the
freeze commit of B10 on the car, and (vi) in the bag the planner's first status is later
than the second of two consecutive ready statuses of the camera-source lap node. Attempts as
D9-C0: at most 3 per direction per session, a new one only after the start view was changed,
every attempt reported.

**6. What lets D v0 drive on the camera source.** All of: D10-S0, D10-S1 and D10-S2 passed
on B10 at the freeze commit; G1 held, so D9-B0 stands as PASS on (i) and (ii) and
INSUFFICIENT on (iii); that commit is what runs on the car; D10-C0 passed earlier in the
same session in the direction to be driven, from the same mark, with the lights unchanged;
the run is launched with `lap_mode:=v0 lap_source:=camera`, lap-1 cap 0.5 m/s, on the plain
donut, 10 laps per direction, the owner holding the kill switch. D7-S1 is the standing
evidence for the cap schedule. D3-S0, D7-S0, D8-S0, D9-S0 and D9-S1 stay FAIL as registered
and this path does not depend on the lidar gate. D v1 does not drive under this file. This
pass does not stand in for D3 in front of B5 or B6. Any fail: D does not drive on the camera
source; the cap stays fixed.

**7. Stop rules.** (a) B10 runs once on the freeze commit. A failed gate is a result: no
tuning, no re-scoring, no reading of B10's run files beyond the report. The report carries
the start trace of every run so that a start failure needs no file opened. (b) No freeze
unless brief 51c's checks G1 to G6 (written before any code) all hold. (c) `source_rule`,
`start_gate` and every threshold are fixed by this file; development data does not move
them. (d) No V case and no seed 81 or 82 before the freeze file exists; no U case, seed 71
or 72 or B9 run file ever; a test enforces both. (e) On the car a second doubtful lap, a wall
contact or a second obstacle brake ends the D run.

**8. Owner decisions.** Given: 2026-11-17 items 11, 12 and 13 (for D9). Recorded by the
coordinator 2026-11-18 item 15 (this attempt). Asked with this file and not yet given, so
until they are a D10 result decides nothing about driving: (1) a D10 pass as section 6 lets
D v0 drive on the camera source on decision 13's terms; (2) D9-B0 carries as scored and no
new offline gate is run; (3) the lidar lap gate is parked and runs only as a shadow.
