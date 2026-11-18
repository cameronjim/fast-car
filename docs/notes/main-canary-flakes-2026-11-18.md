# Main's red L5 canaries, 2026-11-18

Main CI runs 44199589402 (on 6483291) and 44202710978 (on 16e491d) failed three L5 jobs that were
green on PR 209's own CI. Nothing in racer_safety, racer_drivers, firmware or vehicle_params is
touched here: the fixes are in the test harness and the CI script.

## 1. L5 Plug-and-Play canary, contract 2 planner bundle, cw: `rate_limit_events` 1 != 0

Not the startup fault of PRs 208 and 209 (those are fixed: no `rate_limit` at t = 0.3 s in any of
the 26 traces of run 44199589402). The `l5-pnp-canary-trace` artefact of that run shows:

    /drive stamps  ... 3.65, 3.67, 3.69, 3.69, 4.03, 4.05 ...        a 0.34 s gap
    /drive_raw     3.71, 3.73, ... 4.01, 4.03 (20 ms cadence, no gap)
    safety events  watchdog engage "age=0.341653s" (stamp 3.69), release 4.03,
                   rate_limit engage dt=0.013541 s at 4.03

safety_node was not scheduled for 0.34 s while the planner kept publishing every 20 ms. It measures
its /drive_raw watchdog on the steady clock and ticks on a wall timer, and the simulator's clock
never runs faster than the wall, so the first cycle after the stall reads a 0.34 s old command: the
watchdog brakes (one brake, inside the allowed 3) and the ramp back to speed from 0 is clipped by the
rate limit (the one `rate_limit` engagement). Both are the gate working as designed on a late
cycle. The other 25 traces have a longest /drive gap of 0.01 to 0.03 s. Under a 2 CPU limit with a
70 to 100 per cent stress load the same watchdog plus rate_limit pairs appear in bulk (local runs).

Fix (harness): `pnp_timing_summary` now reports `safety_drive` (the gaps of /drive). A run that
fails its judgement and shows a gap of four sim steps (40 ms) or more in /drive_raw or /drive was starved by the machine
and is run again, at most 3 attempts, with the same judgement every time
(`tests/l5_reactive_bench/starvation.py`, the rule the planner timing cases already had for
/drive_raw alone, now shared). A failing run without such a gap fails at once, a defect that fails
every attempt still fails the test, and nothing is relaxed.

## 2. L5 lap learning canary, lap 1: system 21.24 s, truth 20.84 s

The printed lines of the failing job (the only data there is: that job kept no artefact):

    truth   [20.84, 17.73, 15.58, 14.23, 13.50, 13.35]
    system  [21.24, 18.05, 15.73, 14.33, 13.56, 13.34]   differences .40 .31 .15 .10 .06 -.01

and of a passing local run: truth [21.26, 18.01 ...], system [21.31, 17.98 ...], lap 1 difference
0.03 to 0.055 in four clean 4 CPU runs. The system's laps are the normal ones; the truth's are
short, by an amount that shrinks as the lap speed settles, which points at the truth's start line
(its first odometry sample) and not at the lap node. Which sample the truth starts from is the first
odometry sample at or after the system's start scan, recorded by the runner's own spin loop. That is
a guess, not a measurement: the root cause of this one is NOT proven. Under a 2 CPU limit and a
stress load the truth validity checks of the runner fire instead (`truth_starts_after_crossing_0`,
`no_scan_at_start`, and watchdog brakes), which are the starved-machine failures.

Fix (harness, what can be done without the data): the two lap time cases run through the same
starvation rule (also a truth recording the runner itself calls invalid), the lap time assertion
now prints the per lap differences, both crossing lists, the truth's start, the lap lengths, the
/drive_raw and /drive gaps and the brake count, and the lap job uploads its event traces
(`l5-pnp-laps-canary-trace`) like the Plug-and-Play job does. The next occurrence will say what it
was. If it recurs without a gap, it is a measurement problem in the truth start and wants its own
brief.

## 3. L5 localisation canary, mapping: `wall_err_mean_m` 0.1258 > 0.075

A bad map: coverage 0.78, 2266 occupied cells against about 1870 in a good one, slam pose mean error
0.256 m against 0.10. The trajectory is the same as in a good run (lap times [17.51, 34.76], 684
samples in both), so the sim did the same thing and slam_toolbox built a different map from it. It
is the "wrong loop closure on the plain lane" of docs/notes/sim-time-lockstep-2026-11-14.md.

Measured locally (ros-dev, arm64, 2 CPUs, stress-ng 70 per cent): 2 bad maps in 8 runs at the real
time factor 1, 0 in 14 more, then 1 in 8 at the real time factor 0.5, so pacing the sim slower does
not cure it. The lockstep gate timeouts of the bad CI run (20) are the same as in good runs (17 to
24), the number of /pose updates slam publishes varies by a few between good runs (170 to 178), so
no signal in the run separates a bad map from a good one, and a starved-run rule cannot be written.

Fix (CI script): the mapping canary is run on its own with ctest `--repeat until-pass:3`
(`.github/scripts/l5_localization_test.sh`); the AMCL canaries are judged once as before. This is a
bounded rerun, not a looser threshold: a stack that builds a wrong map every time fails all three
attempts. It is the one judgement call in this change; it matches the mapping canary's own header
("a rerun is the right answer"). The cause (slam_toolbox's single-slot scan queue dropping scans while
the node is busy, or the order the EKF's odometry and the scans pair up) is still open under issue
156. The canary now records `slam_pose_updates` so a future change can look for a signal.
