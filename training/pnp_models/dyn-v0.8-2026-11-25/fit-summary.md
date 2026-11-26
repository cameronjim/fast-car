# dyn-v0.8-2026-11-25 fit, frozen before its held-out pass

Validation numbers only; no H1 or H2 number of this version exists when this is written.

- Train segments: 168 (iter_roles: older train roles, dyn-v0.7p's 2026-11-15 and 2026-11-22 train and validation parts; no session 2 bag).
- Validation windows: 1119.
- Configs: ['multi_step_from_one_capped_wd1e-2', 'multi_step_from_one_capped_wd1e-2_h16', 'multi_step_from_one_capped_wd1e-2_h8']; seeds [0, 1, 2]; device cpu, float64, 4 threads.
- Fit pickle SHA-256: c2fefb6769d7a5553330f315e88d52d02fc399daa8ed5b7c1b48b468083efa98.

Shipping: multi_step_from_one_capped_wd1e-2 seed 1 (hidden 32), by `iter_rule.select_cost_aware` (width tolerance 0.01).

Validation P1: dyn-v0.1 0.0947 m, dyn-v0.7p 0.0689 m.

| config | mean validation P1 (m) | min | per seed |
|---|---|---|---|
| multi_step_from_one_capped_wd1e-2 | 0.0680 | 0.0670 | seed0 0.0689, seed1 0.0670, seed2 0.0681 |
| multi_step_from_one_capped_wd1e-2_h16 | 0.0694 | 0.0688 | seed0 0.0688, seed1 0.0705, seed2 0.0689 |
| multi_step_from_one_capped_wd1e-2_h8 | 0.0696 | 0.0691 | seed0 0.0691, seed1 0.0692, seed2 0.0706 |
