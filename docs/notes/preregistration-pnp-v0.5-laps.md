**Pre-registration: lap bookkeeping, third sim attempt (B7) and car gate D7-C0.**
Written 2026-11-16, before any code of brief 48 and before any B7 run.

**0. Known when written.** B4 (v0.3 section 1): D3-S0 FAIL, D3-S1 PASS, D3-S2(d) FAIL
(0.952), D3-S2(e) PASS (0.949). Those results stand and are not re-scored. Reading of the
failure: 4 lap-1 errors of 2.23 to 2.37 s came from the scoring harness starting its truth
clock after a late simulator reset; 6 laps in 3 condition c runs came from the crossing
ending lap 3 being placed about 0.3 m early. B4's cases and seeds are development data from
now on. The 2026-11-16 car bags `T22-38-47` and `T02-18-54` were replayed during design
(20 and 19 laps, 3 false missed-gate flags, laps 2 to 20 of the first within 0.15 s of the
camera counter): they are regression data, not evidence.

**1. System under test.** Lap gate v2.1: gate v2 of PR 185 plus the fixes of brief 48,
frozen at one commit recorded in `evaluation/prereg/freeze/b7.json` before the first run.

**2. Battery B7.** Cases P1 to P4 of the yaml, never run with the lap memory; seeds 51, 52,
53; ccw and cw; 12 laps from a standing start; lap-1 cap 0.5 m/s; ceiling, plant, noise and
lap source as B4. Conditions a (D off, cap 0.5), b (D off, ceiling), c (D v0). 24 runs per
condition. A case is dropped if condition a cannot complete 3 laps on it in either direction
with seed 51; the battery is then 18 runs per condition and 22 of 24 becomes 16 of 18.
Ground truth as v0.3 section 1.3, same words: lap k is the time between ground-truth
crossings k-1 and k of the start line, crossing 0 is the stamp of the first `/scan` after the
planner first reported DRIVING, the start line is the ground-truth pose at that stamp. Lap 1
is the launch lap for the system and for the truth alike. Truth is computed without reading
the system's lap events. A run whose truth recording does not cover crossing 0, or whose
startup reset completed after the DRIVING report, is invalid: rerun alone, at most 3
attempts, all kept; still invalid counts as failed to execute. Starved runs as B4.

**3. Gates.** D7-S0: S0a on condition a (count equal in every run, every lap within 0.3 s)
and S0b on condition c (count equal in every run, at least 98 percent of laps within 0.3 s,
none off by more than 1.0 s), no run failed to execute. Lap 1 is inside both items, with no
allowance. D7-S1: the four items of D3-S1 on conditions c and b, unchanged. Reported with no
threshold: the per-run lap-1 difference and launch dwell; missed-gate flags per condition.

**4. On the car, D7-C0** (D3-C0 stays registered and is computed and reported next to it).
Lap memory in observe mode, two runs of at least 10 laps, one per direction, the first 10
laps of each judged: (i) the lap count equals the reference's in both runs; (ii) of the 18
lap times of laps 2 to 10, at least 17 are within 0.3 s of the reference; (iii) lap 1 is
judged by its crossing instant, not its duration: system crossing 1 minus reference crossing
1 is within 0.3 s of the median of that difference over crossings 2 to 10 of the same run,
in both runs; (iv) no lap of either run is flagged missed gate. Reference: the camera lap
counter, or the video if the owner does not accept it. A reference whose count over the run
differs from the wheel-distance count (driven wheel distance over the median reference lap
distance, rounded down) is not a reference: the run is repeated, neither pass nor fail.
Why (iii): the system starts lap 1 at its start scan and the camera at its reference frame
(1.9 s before the car moved in one bag of 2026-11-16, 2.6 s after 0.3 m/s in the other), so
the two lap-1 durations do not measure the same interval. Disclosure: on bag `T22-38-47`
items (i) to (iii) would hold and (iv) would fail on main.

**5. What lets D drive.** D v0 (the cap schedule) may drive on the plain donut in a session
when D7-S0 and D7-S1 passed on B7 at the freeze commit, that commit is what runs on the car,
and D7-C0 passed earlier in the same session. Then: lap-1 cap 0.5 m/s, 10 laps per
direction. D v1 does not drive under this file. Any fail: D does not drive; the cap stays
fixed.

**6. Owner decisions, 2026-11-16** (recorded in `claude-docs/owner-decisions-2026-11-16.md`, given
before any code of brief 48 and before any B7 run; all four YES).

1. A D7-S0 plus D7-S1 pass on B7 is the new registered attempt that lets D v0 drive on the car.
   D3-S0 stays FAIL as registered.
2. That pass stands in for "D3-S0 and D3-S1 passed" as the precondition of B5 and B6, as a
   recorded decision, not an edit of the v0.3 or v0.4 files.
3. The small-fix path (brief 48) comes before the keyframe ring of 15.2. The ring waits, and
   comes back if item 5 of the brief hits its stop rule or D7-S0 fails.
4. The reference of D7-C0 is the camera lap counter, with the phone video as the backup
   reference. Section 4 therefore reads: reference = camera lap counter; if its count is not a
   valid reference by the rule of section 4, or the owner does not accept a given run's counter
   reading, the video of that run is the reference.
