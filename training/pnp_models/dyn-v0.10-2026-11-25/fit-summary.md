# dyn-v0.10-2026-11-25 fit, frozen before its held-out pass

Validation numbers only; no H1 or H2 number of this version exists when this is written.

- Train segments: 38 (data 'pnp'; iter_roles: dyn-v0.7p's 2026-11-15 and 2026-11-22 train and validation parts; no session 2 bag).
- Validation windows: 300.
- Configs: ['multi_step_from_one_capped_wd1e-2_gs', 'multi_step_from_one_capped_wd1e-2_gs_h16', 'multi_step_from_one_capped_wd1e-2_gs_h8']; seeds [0, 1, 2]; device cpu, float64, 4 threads.
- Fit pickle SHA-256: 7e74621fa99af1ea23c4bd41bb3c4518e421b5fdc1537f71bcaf901710ec3639.

Shipping: multi_step_from_one_capped_wd1e-2_gs_h8 seed 0 (hidden 8), by `iter_rule.select_cost_aware` (width tolerance 0.03).

Validation P1: dyn-v0.1 0.1244 m, dyn-v0.7p 0.0841 m.

| config | mean validation P1 (m) | min | per seed |
|---|---|---|---|
| multi_step_from_one_capped_wd1e-2_gs | 0.0608 | 0.0601 | seed0 0.0608, seed1 0.0601, seed2 0.0613 |
| multi_step_from_one_capped_wd1e-2_gs_h16 | 0.0606 | 0.0603 | seed0 0.0613, seed1 0.0603, seed2 0.0603 |
| multi_step_from_one_capped_wd1e-2_gs_h8 | 0.0616 | 0.0611 | seed0 0.0611, seed1 0.0619, seed2 0.0618 |
