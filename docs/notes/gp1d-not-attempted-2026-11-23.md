# GP1d outcome: not attempted (2026-11-23)

Outcome by the rule of the pre-registration `docs/notes/preregistration-pnp-v0.6-perception-fused.md`
(constants `evaluation/prereg/pnp-v0.6-perception-fused.yaml`, SHA-256
`22d65de750ef56bcc90e33cc0a7a5a85eb69b5291d606ebd30fd452ed1e3e0b5`): no setting of the free
parameters meets the development margin (mean coverage loss and excess at most 0.005, p99 of each
at most 0.025) on every development set. Nothing was frozen, no `trackperc-v2g` bundle was exported,
no 2026-11-14 `heldout` scan and no 2026-11-22 scan was read, and GP1d and GP1d-K were not scored.
GP1 stays "better where it trained, not shown elsewhere", GP1b stays FAIL, GP1c stays "not
attempted". M-A2 measured under 3 percent of lap time for a perfect track in sim, so even a pass
would have claimed a more accurate centre, not a faster lap.

## What was run

Development sets (yaml `data_roles.development`): sim (1500 scans, seed 777), the v0.2 train roles
with the full model (21037 moving scans), A-run (4009), A-layout 2026-11-12 (2715), folds F1 (4826)
and F2 (5201) with their fold models, the five 2026-11-16 bags (8027 moving scans, no self-labels,
so the margin only). Sweep (`racer_pnp_perception.tune_v2`): stage 1 slope (0.05, 0.10, 0.15, 0.20,
0.30, none) by edge extension (0, 0.05, 0.10, 0.20), stage 2 reach (0.25, 0.35, 0.50) by margin
(0.10, 0.15, 0.20) by blend (0.5, 0.75, 1.0) around the best of stage 1, 51 settings on an evenly
spaced subsample of 1200 scans per set; the best three, ranked by the rule, repeated on the full
sets. `append_far` does not touch the measured stations and stays on. Table:
`docs/notes/gp1d-tuning-table-2026-11-23.json`.

Smallest value over all 51 settings of the worst set: mean loss 0.073, p99 loss 0.757, mean excess
0.0145, p99 excess 0.188 (limits 0.005 and 0.025). The three best settings on the full sets (they
differ only in the edge margin, which does not bind) have the development benefit (E_c ratio 0.754
to 0.944 on every labelled set, mean 0.840, so the net's centre does halve most of the tracer's
error) and fail the margin on every set:

| set | scans | loss mean | loss p99 | excess mean | excess p99 | E_c ratio |
|---|---|---|---|---|---|---|
| sim | 1500 | 0.031 | 0.505 | 0.004 | 0.038 | 0.903 |
| train roles | 21037 | 0.063 | 0.622 | 0.014 | 0.144 | 0.786 |
| A-run | 4009 | 0.051 | 0.261 | 0.016 | 0.119 | 0.754 |
| A-layout 2026-11-12 | 2715 | 0.046 | 0.350 | 0.011 | 0.128 | 0.944 |
| fold F1 | 4826 | 0.067 | 0.745 | 0.011 | 0.154 | 0.891 |
| fold F2 | 5201 | 0.088 | 0.576 | 0.016 | 0.263 | 0.764 |
| 2026-11-16 bags | 8027 | 0.065 | 0.704 | 0.020 | 0.161 | n/a |

(slope 0.15, extension 0, reach 0.35, margin 0.10, blend 0.5.)

## Why (measured, not a bug of the tuning)

The ray cast does what the brief says: every rebuilt edge point lies on the tracer's edge polyline
(property test, 1e-6). The loss and excess come from the stations, not the edges:

* The tracer's stations are 0.20 m apart along its raw marching polyline, but the chord between
  neighbouring stations is shorter: the 16 stations of a full track are 2.90 to 2.98 m apart end to
  end, not 3.00 m. A resample at exactly 0.20 m along the centre therefore either gives one station
  fewer (A-run, extension 0: 34 percent of scans lose 1 to 4 stations, mean loss 0.139 on those) or,
  with the extension that recovers it, runs 1 to 3 cm per station beyond the tracer's polygon.
  Where no station is lost the loss is still mean 0.009 (p99 0.073) and the excess mean 0.016 (p99
  0.133) on A-run: the area of the polygon over the first `valid_count` stations changes by about
  the length difference (2 to 3 percent of a 3 m corridor) times the width.
* The extension (up to 0.20 m) buys the lost station back and costs excess: at 0.20 the mean excess
  is 0.036 to 0.044.
* No free parameter acts on this. Reach, margin, blend and the slope limit change the centre and
  barely the station spacing.

A diagnostic outside the registration (stations kept at the tracer's indices with the ray cast
widths) is worse, not better: loss mean 0.10 to 0.21 on A-run, from rays at the first and last
stations that miss the polyline's end after the normal turns. It is not a candidate and was not
tuned.

## For the owner

The margin as registered compares the polygon over the tracer's own `valid_count` stations, and a
fused track at the 0.20 m contract cannot match the tracer's polygon to 0.5 percent while the
tracer's stations are 2 to 3 percent short of 0.20 m. Passing would take a change outside this
brief: the tracer's resample (`geometric.py`, not in scope) or a coverage measure that compares the
corridors by area rather than by station count. Whether either is worth doing is the owner's call;
the net's centre gain is real on development data (ratio 0.75 to 0.94).
