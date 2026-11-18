**Pre-registration: lap learning fed by the camera lap counter (car) and a truth oracle
(sim); battery B9, offline gate D9-B0, car gates D9-P0 and D9-C0.**
Written 2026-11-17, before any code of brief 51, before any B9 run and before either
evaluation bag was opened.

**0. Known when written.** B7 (v0.5 laps): D7-S0 FAIL (lap counting by the lidar gate on the
ring case P4), D7-S1 PASS (0 contacts, 24 of 24 reach, median r 0.556). Those results stand.
B8 (lap gate v3, brief 50) is registered and not run. Owner decision 2026-11-17 item 12: on
the car D takes its laps from the camera lap counter; in sim the battery feeds D the truth
line crossing as an oracle, which tests the learning only; a new registered gate before
code. Read to design this: car bags `2026-11-16T22-38-47` (camera 20 of 20, peaks 0.92 to
0.95, event latency 0.09 to 0.26 s, reference taken standing) and `2026-11-16T02-18-54`
(10 events for 20 laps, peaks 0.80 to 0.87, every miss a lap of 2 or 4 lap lengths,
reference taken at 0.8 m/s). Both are development data from now on, with every 2026-11-13
and 2026-11-14 bag. Of bags `2026-11-16T22-45-39` and `2026-11-16T23-04-09` only the topic
counts were read (12 and 13 camera lap events) and what the session note says was driven.
No real bag exists in which the lap memory ran on the camera source.

**1. System under test.** `lap_memory_node` with `lap_source` `camera` (car) or `truth`
(sim), lap mode v0, the source rule of brief 51 with the constants of the yaml's
`source_rule`, frozen at one commit recorded in `evaluation/prereg/freeze/b9.json` before
the first B9 run and before either evaluation bag is opened. The source rule in one
paragraph: the lap ends at the event's `crossing_time_s` (the peak frame), never at the
event's arrival and never from the 1 Hz status; a lap is confirmed only if the run started
ready, the event numbers are consecutive, the odometry covers the crossing, the source
stayed alive, the wheel length is at least 3.0 m and inside the band of the reference
length, and the heading is within 1.2 rad of one turn; every other lap is doubtful. A
doubtful lap is the engine's existing missed-gate lap: not clean, not stored, the cap takes
one step down and needs two clean laps to rise. With no crossing past the reference length
plus its band, or with a silent source, the cap handed out is the lap-1 cap until the next
crossing. The camera counter itself is not changed.

**2. Battery B9 (sim, truth oracle).** Cases U1, U2, U3 of the yaml, never run by anyone;
seeds 71, 72; ccw and cw; standing start; lap-1 cap 0.5 m/s; ceiling, plant and noise as
B7. Conditions: a (D off, cap 0.5; seed 71 only, for the drop rule), b (D off at the
ceiling), c (D v0, `lap_source truth`), f (D v0, `lap_source truth` with the fault plan of
the yaml: crossings 4 and 7 not published, one false event at 0.6 of lap 9, crossing 10
published twice). 12 runs in each of b, c and f. Each run drives 13 ground-truth laps and
laps 1 to 12 are judged. A case is dropped if condition a cannot complete 3 laps on it in
either direction with seed 71; at most one case may be dropped; with fewer than two cases
left nothing is decided and the owner is told. Ground truth, validity, starved and invalid
runs as the v0.5 laps ring file section 3. The oracle is computed from the simulator's
ground-truth pose while the run goes on; the registered truth is computed afterwards from
the recording and reads neither the oracle nor the system. This battery says nothing about
lap counting: D7-S0 stays FAIL and D8 is brief 50's.

**3. Sim gates.**
D9-S0 (the oracle and the wiring), condition c, every run: the oracle publishes exactly one
event per registered truth crossing 1 to 12, each within 0.3 s of it; the system closes
laps 1 to 12, all confirmed, none doubtful, no event rejected; every system crossing is
within 0.3 s of the truth crossing of the same number; no run failed to execute.
D9-S1 (the learning arrives through the new path): the four items of D3-S1 on conditions c
and b, with 11 of 12 where D3 says 22 of 24 (7 of 8 after one dropped case).
D9-S2 (the fallback in closed loop), condition f, every run: (1) 0 wall contacts; (2) 12
truth laps completed; (3) every confirmed system lap is one truth lap: its two crossings are
within 0.3 s of two consecutive truth crossings; (4) the laps that close at truth crossings
5 and 8, the lap closed by the false event and the lap after it are doubtful, and the
duplicate of crossing 10 is rejected; (5) the cap of the lap after a doubtful lap is at most
the cap of that doubtful lap; (6) between truth crossing 4 plus half a truth lap and truth
crossing 5, and the same for 7 and 8, the lowest `LapGuidance.v_cap_mps` is the lap-1 cap.
Reported with no threshold: late lap times of condition f against c, event latency, the
reference length per run, every doubtful lap with its reasons.

**4. Offline, D9-B0 (the camera's times and lengths through D's rule, on real bags never
read).** Bags `2026-11-16T22-45-39` and `2026-11-16T23-04-09`, opened only after the
freeze, scored once by `pnp_session lap-src --offline`. Per run of each bag (as
`tools/pnp_data` splits runs), every camera lap event after the first of its run is a
judged lap. (i) Time: the replayed lap's crossing stamp equals the event's
`crossing_time_s` within 1 ms and its lap time equals the event's within 0.02 s, every
judged lap. (ii) Length: the replayed wheel length is within 0.10 m of the event's
`distance_m`, every judged lap. (iii) Verdict against `lapref` (steering and wheel distance
only, brief 36), on the runs where `lapref` finds a period: no lap whose wheel length is
outside 0.85 to 1.15 of the `lapref` lap length is confirmed (not one), and at least 90
percent of the laps inside it are confirmed. For (iii) the replay sets ready true, because
these runs were not started for this purpose; the ready rule is judged by D9-P0 and D9-C0.
Fewer than 10 judged laps in all, or no run with a `lapref` period: INSUFFICIENT, which is
neither a pass nor a fail and is said so. Development bags, never scored: `22-38-47` and
`02-18-54` (brief 51's checks E2 and E3).

**5. On the car.**
D9-P0 (every launch whose laps D will use; enforced by the node, not by a person): at the
start the camera status is at most 3.0 s old, says `running`, 0 laps, and a similarity of
at least 0.90 with the car standing. Not met: every lap of that run is doubtful and the cap
stays at the lap-1 cap.
D9-C0 (the test pass; per direction): one run of at least 10 laps in observe mode at the
ceiling D will be given, from the taped start mark, with a lap node on the camera source
(primary or shadow), the freeze commit on the car, the first 10 camera laps judged from the
recorded topics by `pnp_session lap-src`: (i) the camera's count equals the count the
owner called at the stop, and each of laps 1 to 10 has a camera `distance_m` within 10
percent of the median of laps 2 to 10; (ii) every `peak_similarity` of laps 1 to 10 is at
least 0.88; (iii) the lap node started ready, confirmed laps 1 to 10, rejected no event,
and its lap lengths of laps 2 to 10 are within 0.10 m of the events' `distance_m`; (iv) no
event arrived more than 0.5 s after its `crossing_time_s`; (v) no gap above 3.0 s in the
camera status. If the owner's count was not taken, the wheel rule of v0.5 laps section 4
stands in for it and the phone video is the backup. A direction that fails may be attempted
again after the start view was changed, at most 3 attempts per direction per session, every
attempt reported. 0.88 was fixed from the development bags: the run that counted 20 of 20
had no peak under 0.92, the run that missed half had none over 0.87.

**6. What lets D v0 drive on the camera source.** All of: D9-S0, D9-S1 and D9-S2 passed on
B9 at the freeze commit; D9-B0 is PASS or INSUFFICIENT (a FAIL stops this path); that
commit is what runs on the car; D9-C0 passed earlier in the same session in the direction
to be driven, from the same mark, with the lights unchanged; the run is launched with
`lap_mode:=v0 lap_source:=camera`, lap-1 cap 0.5 m/s, on the plain donut, 10 laps per
direction, the owner holding the kill switch (owner decision 2026-11-17 item 11's terms).
D7-S1 is the standing evidence for the cap schedule; D3-S0 and D7-S0 stay FAIL as
registered and this path does not depend on D8. D v1 does not drive under this file. This
pass does not stand in for D3 in front of B5 or B6. Any fail: D does not drive on the
camera source; the cap stays fixed.

**7. Stop rules.** (a) B9 runs once and the evaluation bags are scored once, on the freeze
commit. A failed gate is a result: no tuning, no re-scoring, no reading of B9's run files or
of the evaluation bags' lap topics beyond the report. (b) No freeze unless the six
development checks of brief 51 (E1 to E6, written before any code) all hold. (c) The
constants of `source_rule` and every threshold here are fixed by this file; development
data does not move them. (d) No U case, no seed 71 or 72 and neither evaluation bag before
the freeze file exists; a test enforces it. (e) On the car a second doubtful lap, a wall
contact or a second obstacle brake ends the D run.

**8. Owner decisions.** Given: 2026-11-17 item 12 (this path), item 11 (the terms of a D v0
drive). Asked with this file and not yet given, so until they are a D9 result decides
nothing about driving: (1) a D9 pass as section 6 lets D v0 drive on the camera source
while D3-S0 and D7-S0 stay FAIL and whatever D8 says; (2) the fallback is the existing
missed-gate path (lap-1 cap while overdue, one step down after a doubtful lap), which is
stricter than holding the cap; (3) B8's and B9's freeze are taken from one main commit that
holds both briefs, or, if B8 was frozen earlier, the car runs the later commit and brief
51's test that the start-scan path is unchanged is accepted in place of "the B8 freeze
commit runs on the car".
