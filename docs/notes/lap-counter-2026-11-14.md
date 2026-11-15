# Camera lap counter, 2026-11-14: why 10 of 20 (brief 39, M-CAM)

Pre-registration `docs/notes/preregistration-pnp-v0.4.md` section 4.1, yaml SHA-256
`78510c14...a1d63` (checked by `tools/lap_replay.py --split-run` on every run). Frames are the
`/camera/usb/record/compressed` stream of each bag (the counter's camera, usb), 640 x 360 JPEG,
**4.73 Hz** in these bags (not 2 Hz), against the full-rate frames the live node saw.

## 1. The hypothesis, tested

Hypothesis: `min_lap_time_s` 6.0 refuses a crossing less than 6.0 s after the last counted one,
so only every second crossing counts.

**The live record confirms it.** The `/lap/event` stream recorded on 2026-11-14 (live node, full
rate, old defaults):

| Run | Real lap | Live events (lap time, s) | Reading |
|---|---|---|---|
| L0 (GP3) | 5.5 to 5.8 s | 10 events: 11.1, 6.3, 10.5, 6.5, 10.4, 11.1, 11.2, 11.3, 11.2, 11.3 | double laps (about 11 s, peak similarity 0.96 to 0.99) alternate with 6.3 to 6.5 s laps just over the gate: 10 of 20 |
| X0 | 5.6 s | 34.7, 38.8, 7.5, 10.8 | car crept for the first 100 m (distance gate), then doubles |
| X1 | 6.1 s | 6.3, 10.8, 7.5, 6.2, 6.2, 10.9, 6.2 | laps near 6.1 s: some refused, some counted |
| X3 (cap 0.5) | 13.9 s | 11 events, 13.0 to 14.4 s | every lap counted |
| X5 (cap 0.7) | 9.0 s | 8 events, 8.9 to 9.1 s | every lap counted |
| X6 (cap 1.3) | 5.0 s | 10.2, 10.0, 9.9 | every second lap |

Laps over 6 s count, laps under 6 s count every second one, exactly as predicted. The
hypothesis holds.

**It does not hold alone for the replay of the recorded frames.** Replay of L0 with
`tools/lap_replay.py --split-run L0 --image-topic /camera/usb/record/compressed`:

| Replay | min_lap_time_s 6.0 | min_lap_time_s 2.0 |
|---|---|---|
| L0 as the tool starts it (reference at the first moving frame, 0.48 s, during the launch) | 1 lap (79.2 s) | 1 lap (79.2 s) |
| L0 from 4.9 s (reference after the launch, where the live node's was) | 12 laps (11.0, 6.3, 10.5, 6.4, ...: the live pattern) | 19 laps, 5.44 to 5.75 s |

Same frames, same thresholds: the 2.0 s floor takes 12 to 19 laps, lap times 5.5 s, no extra
lap. That is the fix shown on recorded frames. A second and third cause show too:

1. **Recording rate.** 4.73 Hz against about 15 Hz live. Live crossing peaks are 0.96 to 0.99;
   the same crossings in the recorded stream peak at 0.72 to 0.90, around the 0.80 threshold.
   Dev runs X5 (live 8 of 8 laps) and X6 give 2 of 8 and 1 of 3 in replay. A replay at this rate
   is a lower bound on the live counter, as the brief says. The fix is the 10 Hz record
   argument below, not a threshold.
2. **The reference frame.** The first moving frame of L0 is during a launch lurch (speed 0.5, 0.78,
   then 0.11 m/s at 2 s). As the reference its crossings peak at 0.72 to 0.80 (1 lap), while a
   reference at 1.2 s gives 17, at 4.9 s gives 19 (0.7 s: 7, 1.3 s: 12, 1.5 s: 8, 2.0 s: 2). The
   count is not stable against the reference frame at 4.73 Hz. Not fixed: a later reference loses
   the first crossing (19 where lapref and the owner say 20), a count definition that is the
   owner's. Thresholds were NOT lowered: on L0 as started, 0.60 gives 17 and still misses real
   crossings at 0.596 and 0.600, with between-lap bumps at 0.554 and 0.559; a threshold tuned to
   the degraded stream would be wrong for the 15 Hz live counter.

## 2. The fix (all of it)

- `min_lap_time_s` 6.0 to 2.0 in `lap_counter.py`, `lap_counter.launch.py`,
  `cameras.launch.py` (`lap_min_lap_time_s`). The 3.0 m wheel distance gate is unchanged.
- `cameras.launch.py` `lap_record_rate_hz` (default 0.0 = the camera records at
  `record_rate_hz`, so nothing changes): the camera on `lap_image_topic` records at this rate
  while a drive is up. Cost, from the 2026-11-14 bags (usb record stream 15.2 kB per frame):
  5 Hz 4.3 MB/min, 10 Hz 9.1 MB/min, so +4.8 MB/min. The default stays; brief 41 switches it
  on; the owner decides the default.
- `tools/lap_replay.py`: `--split-run NAME` (the v0.4 yaml, SHA checked, brief 36's loader was not
  on main), `--reference LAPREF.json` (matched, missed, false counts).

Development result against the acceptance line: **L0 does not give 20 at the recorded rate** (best
19, and it depends on the reference frame). Not tuned further.

## 3. M-CAM on the lap test runs

Candidate frozen at git `387f7548c9a3618ba73f7cf176e4bed7f299508e` (code unchanged since). One
replay of K1, K2, X2, X4, X7 with `lap_replay.py --split-run <run> --image-topic
/camera/usb/record/compressed`, default parameters (2.0 s floor), the recorded 4.73 Hz stream,
the tool's own reference capture. Not repeated.

**Which reference.** Section 4.1 says the count of 4.2's reference (lapref). lapref then failed
M-L1 (PR 194: K1 10 of 12, K2 9 of 11, X2 3 of 4, X4 7 of 9 within 0.3 s, X7 double laps), and
4.3 says that then neither lapref nor the gate is trusted on the strength of the other and the
phone video decides; 9.3 leaves "whether lapref may be a reference" to the owner. No test run was
filmed. The registration therefore does not name a usable reference for this case. **No pass or
fail is declared.** Counts side by side:

| Run | Camera counter (this replay) | lapref | Gate v2 | Wheel distance check |
|---|---|---|---|---|
| K1 | 7 (16.8, 5.5, 5.6, 11.2, 11.4, 5.6, 5.6 s) | 12 | 12 | |
| K2 | 8 (7.3, 6.2, 6.1, 12.3, 6.1, 6.1, 6.3, 12.1 s) | 11 | 11 | |
| X2 | 1 (14.7 s) | 4 | 4 | |
| X4 | 9 (7.96 to 9.15 s) | 9 | 9 | |
| X7 | 3 (18.9, 9.1, 27.4 s) | 7 (double laps, wrong) | 14 (consistent with 75.9 m) | 14 laps of about 5.2 m |

Reading, no threshold applied: the counter equals both candidates on X4 only. On K1, K2, X2 and
X7 it is below every candidate, with no count above any of them (no false count against either
reference). Its misses at the recorded rate are the same effect as the dev runs (section 1: the
recorded stream peaks 0.72 to 0.90 against 0.96 to 0.99 live; near misses 0.68 to 0.79 are listed
in the replay output of each run), and the dev table showed the recorded rate cannot decide
runs that the live counter counted fully (X5 8 of 8 live, 2 in replay). By 4.1's own rule these
four runs are therefore "cannot be applied at the recorded rate", not a pass and not a measured
failure of the live counter. X4 matches but is one run. The test of the counter at full rate
needs the next session's 10 Hz stream (`lap_record_rate_hz:=10`, brief 41) and a filmed
reference.
