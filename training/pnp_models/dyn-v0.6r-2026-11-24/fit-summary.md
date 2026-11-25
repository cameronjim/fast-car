# R6 fit, frozen before the held-out pass

Selection and train numbers only; no 2026-11-22 held-out number exists when this is written.

- Constants SHA-256 db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c.
- Session file SHA-256 6b791ac5bcc5c0b150b86420574f4d704453963e9d1fdb1502aa09f0447516f5.
- Train segments: 173 (every v0.5 train role plus the 2026-11-22 train parts).
- Selection: the 2026-11-14 held-out segments and the 2026-11-15 session held-out parts.
- Recipe: R4's (stages_v04.r4_fit) on dyn-v0.1, configs ['yaw_one_step_wd1e-2', 'yaw_multi_step_wd1e-3', 'yaw_multi_step_wd1e-2', 'yaw_multi_step_from_one_wd1e-2', 'yaw_multi_step_capped_wd1e-2', 'yaw_multi_step_nonorm_wd1e-2'], seeds [0, 1, 2].
- Training device: cpu, float64, 4 threads.
- Frozen fit pickle SHA-256: 37253735d66f60e7ff7b765adee89d7e502aee4df9fd926fe67db9491ffa686e.
- Samples: {'one_step_samples_train': 22724, 'one_step_samples_val': 3707, 'windows_train': 20255, 'windows_val_loss': 2669, 'windows_val_p1': 1068}.

Shipping: yaw_multi_step_from_one_wd1e-2, seed 2 (chosen by selection P1).

dyn-v0.1 selection P1 0.1111 m on 2287 selection windows; internal validation P1 0.0872 m.

| config | mean selection P1 (m) | min | per seed | mean internal validation P1 (m) |
|---|---|---|---|---|
| yaw_one_step_wd1e-2 | 0.1086 | 0.1086 | seed0 0.1086, seed1 0.1087, seed2 0.1086 | 0.0843 |
| yaw_multi_step_wd1e-3 | 0.1082 | 0.1079 | seed0 0.1082, seed1 0.1086, seed2 0.1079 | 0.0839 |
| yaw_multi_step_wd1e-2 | 0.1084 | 0.1083 | seed0 0.1083, seed1 0.1086, seed2 0.1083 | 0.0840 |
| yaw_multi_step_from_one_wd1e-2 | 0.1079 | 0.1077 | seed0 0.1079, seed1 0.1080, seed2 0.1077 | 0.0836 |
| yaw_multi_step_capped_wd1e-2 | 0.1083 | 0.1082 | seed0 0.1082, seed1 0.1084, seed2 0.1082 | 0.0840 |
| yaw_multi_step_nonorm_wd1e-2 | 0.1087 | 0.1085 | seed0 0.1085, seed1 0.1087, seed2 0.1088 | 0.0843 |
