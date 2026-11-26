# dyn-v0.9-2026-11-25 fit, frozen before its held-out pass

Validation numbers only; no H1 or H2 number of this version exists when this is written.

- Train segments: 168 (iter_roles: older train roles, dyn-v0.7p's 2026-11-15 and 2026-11-22 train and validation parts; no session 2 bag).
- Validation windows: 1119.
- Configs: ['multi_step_from_one_capped_wd1e-2_gs', 'multi_step_from_one_capped_wd1e-2_gs_h16', 'multi_step_from_one_capped_wd1e-2_gs_h8']; seeds [0, 1, 2]; device cpu, float64, 4 threads.
- Fit pickle SHA-256: fb6dcd86e7bf46130977294735f78fc76e185a1b99969e9349128d4c04e16250.

Shipping: multi_step_from_one_capped_wd1e-2_gs seed 0 (hidden 32), by `iter_rule.select_cost_aware` (width tolerance 0.01).

Validation P1: dyn-v0.1 0.0947 m, dyn-v0.7p 0.0689 m.

| config | mean validation P1 (m) | min | per seed |
|---|---|---|---|
| multi_step_from_one_capped_wd1e-2_gs | 0.0509 | 0.0505 | seed0 0.0505, seed1 0.0512, seed2 0.0510 |
| multi_step_from_one_capped_wd1e-2_gs_h16 | 0.0516 | 0.0512 | seed0 0.0519, seed1 0.0512, seed2 0.0515 |
| multi_step_from_one_capped_wd1e-2_gs_h8 | 0.0517 | 0.0515 | seed0 0.0517, seed1 0.0515, seed2 0.0519 |
