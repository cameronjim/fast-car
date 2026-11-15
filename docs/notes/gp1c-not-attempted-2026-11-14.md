# GP1c outcome: not attempted, fusion cannot keep the corridor (2026-11-14)

Outcome by the rule of pre-registration v0.4 addendum 1 section 3.2 (constants
`evaluation/prereg/pnp-v0.4-addendum-1.yaml`, SHA-256
`ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824`): no value of
`max_shift_slope` in the grid (0.05, 0.10, 0.15, 0.20, 0.30) meets the development margin (mean
loss and excess at most 0.005, p99 of each at most 0.025) on every development set. Nothing was
frozen, no `trackperc-v2f` bundle was exported, and no 2026-11-14 scan was read. GP1c and GP1c-K
were not scored and M-A1 was not measured.

Development sets: A-run, A-layout 2026-11-12, folds F1 and F2 (full model seed 1, F1 seed 0, F2
seed 1). The full table (loss, excess, and E_c for information only, which was not used to choose)
is `docs/notes/gp1c-slope-table-2026-11-14.json`. The best slope, 0.05, gives mean loss 0.0069 to
0.0100 and p99 loss up to 0.079 on the four sets.

`fused-v1` stays in the tree as code and tests, enabled nowhere by default. Geometric stays the
launch default. Perception is parked until the oracle experiment (M-A2) shows headroom.
