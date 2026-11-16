# v0.4 fits, frozen before the held-out pass

## N4

| parameter | dyn-v0.1 | N4 | lower | upper | free in yaw fit | on a bound | flag |
|---|---|---|---|---|---|---|---|
| speed_gain | 0.94824 | 0.94804 | 0.5 | 1.5 | False | False |  |
| speed_deadband_mps | 0.22 | 0.33 | nan | nan | False | False |  |
| speed_tau_s | 0.22461 | 0.23593 | 0.005 | 1 | False | False |  |
| accel_max_mps2 | 2 | 1.1498 | 0.2 | 20 | False | False |  |
| decel_max_mps2 | 0.67311 | 1.7593 | 0.2 | 20 | False | False |  |
| steer_gain_left | 0.91662 | 0.8978 | 0.5 | 1.5 | True | False |  |
| steer_gain_right | 1.0115 | 0.87042 | 0.5 | 1.5 | True | False |  |
| steer_offset_rad | 0.015223 | 0.0081042 | -0.1 | 0.1 | True | False |  |
| steer_tau_s | 0.027877 | 0.014046 | 0.005 | 1 | True | False | on a bound |
| command_delay_s | 0 | 0.02 | nan | nan | False | False |  |
| understeer_s2_per_m2 | 0.10938 | 3.1986e-15 | 0 | 2 | True | True | on a bound |

Rollout objective on the validation windows: {'dyn-v0.1': 1.3695558708149254, 'speed fit only': 1.3652676328312459, 'n4': 1.090105264799243}.
Unidentified: {'understeer_s2_per_m2': 'on its lower bound 0; on a bound'}.

## R4 on dyn-v0.1

Shipping yaw_multi_step_wd1e-3, seed 2.

| config | mean validation P1 | min | per seed |
|---|---|---|---|
| yaw_one_step_wd1e-2 | 0.0804 | 0.0803 | seed0 0.0804, seed1 0.0804, seed2 0.0803 |
| yaw_multi_step_wd1e-3 | 0.0796 | 0.0795 | seed0 0.0797, seed1 0.0797, seed2 0.0795 |
| yaw_multi_step_wd1e-2 | 0.0798 | 0.0797 | seed0 0.0799, seed1 0.0797, seed2 0.0797 |
| yaw_multi_step_from_one_wd1e-2 | 0.0798 | 0.0797 | seed0 0.0797, seed1 0.0798, seed2 0.0798 |
| yaw_multi_step_capped_wd1e-2 | 0.0799 | 0.0798 | seed0 0.0799, seed1 0.0798, seed2 0.0799 |
| yaw_multi_step_nonorm_wd1e-2 | 0.0804 | 0.0804 | seed0 0.0804, seed1 0.0804, seed2 0.0804 |

## R4 on N4

Shipping yaw_multi_step_from_one_wd1e-2, seed 0.

| config | mean validation P1 | min | per seed |
|---|---|---|---|
| yaw_one_step_wd1e-2 | 0.0802 | 0.0801 | seed0 0.0801, seed1 0.0804, seed2 0.0803 |
| yaw_multi_step_wd1e-3 | 0.0798 | 0.0797 | seed0 0.0798, seed1 0.0797, seed2 0.0798 |
| yaw_multi_step_wd1e-2 | 0.0799 | 0.0799 | seed0 0.0799, seed1 0.0799, seed2 0.0799 |
| yaw_multi_step_from_one_wd1e-2 | 0.0794 | 0.0793 | seed0 0.0793, seed1 0.0795, seed2 0.0794 |
| yaw_multi_step_capped_wd1e-2 | 0.0799 | 0.0798 | seed0 0.0798, seed1 0.0800, seed2 0.0799 |
| yaw_multi_step_nonorm_wd1e-2 | 0.0802 | 0.0801 | seed0 0.0802, seed1 0.0802, seed2 0.0801 |

## T4 on dyn-v0.1

Smoothness weight 0.1; validation P1 by weight {0.0: 0.05849135349584428, 0.01: 0.058503515578145406, 0.1: 0.057710567960763424, 1.0: 0.05795662990952821}; base validation P1 0.0929; supported nodes on a bound: ['s+0.0000/v0.25', 's+0.6667/v1.5'].

| steering share | speed (m/s) | alpha | supporting windows | tied to node | on a bound |
|---|---|---|---|---|---|
| -1.0000 | 0.25 | 0.7342 | 953 |  | False |
| -1.0000 | 0.6 | 0.8460 | 1068 |  | False |
| -1.0000 | 0.85 | 0.8583 | 357 |  | False |
| -1.0000 | 1.25 | 0.8487 | 233 |  | False |
| -1.0000 | 1.5 | 0.8487 | 0 | 3 | False |
| -0.6667 | 0.25 | 0.7867 | 1148 |  | False |
| -0.6667 | 0.6 | 0.8716 | 1127 |  | False |
| -0.6667 | 0.85 | 0.9326 | 332 |  | False |
| -0.6667 | 1.25 | 0.8872 | 140 |  | False |
| -0.6667 | 1.5 | 0.8872 | 0 | 8 | False |
| -0.3333 | 0.25 | 0.9854 | 617 |  | False |
| -0.3333 | 0.6 | 0.8868 | 766 |  | False |
| -0.3333 | 0.85 | 0.9424 | 55 |  | False |
| -0.3333 | 1.25 | 0.8872 | 29 | 8 | False |
| -0.3333 | 1.5 | 0.8872 | 2 | 8 | False |
| +0.0000 | 0.25 | 1.0500 | 778 |  | True |
| +0.0000 | 0.6 | 0.9866 | 1073 |  | False |
| +0.0000 | 0.85 | 0.9368 | 54 |  | False |
| +0.0000 | 1.25 | 0.8444 | 52 |  | False |
| +0.0000 | 1.5 | 0.8444 | 10 | 18 | False |
| +0.3333 | 0.25 | 0.9425 | 1715 |  | False |
| +0.3333 | 0.6 | 0.8638 | 1914 |  | False |
| +0.3333 | 0.85 | 0.9352 | 341 |  | False |
| +0.3333 | 1.25 | 0.9275 | 312 |  | False |
| +0.3333 | 1.5 | 0.9925 | 99 |  | False |
| +0.6667 | 0.25 | 0.8200 | 2909 |  | False |
| +0.6667 | 0.6 | 0.8681 | 3134 |  | False |
| +0.6667 | 0.85 | 0.9460 | 816 |  | False |
| +0.6667 | 1.25 | 0.9411 | 769 |  | False |
| +0.6667 | 1.5 | 1.0500 | 99 |  | True |
| +1.0000 | 0.25 | 0.7532 | 1941 |  | False |
| +1.0000 | 0.6 | 0.9321 | 2112 |  | False |
| +1.0000 | 0.85 | 0.9718 | 695 |  | False |
| +1.0000 | 1.25 | 0.9667 | 469 |  | False |
| +1.0000 | 1.5 | 1.0500 | 3 | 29 | False |

## T4 on N4

Smoothness weight 0.1; validation P1 by weight {0.0: 0.05545529105161331, 0.01: 0.05536015802346178, 0.1: 0.05288588408302844, 1.0: 0.05302968461329246}; base validation P1 0.0894; supported nodes on a bound: ['s+0.0000/v0.25', 's+0.6667/v1.5'].

| steering share | speed (m/s) | alpha | supporting windows | tied to node | on a bound |
|---|---|---|---|---|---|
| -1.0000 | 0.25 | 0.7287 | 838 |  | False |
| -1.0000 | 0.6 | 0.8681 | 948 |  | False |
| -1.0000 | 0.85 | 0.8425 | 317 |  | False |
| -1.0000 | 1.25 | 0.8164 | 200 |  | False |
| -1.0000 | 1.5 | 0.8164 | 0 | 3 | False |
| -0.6667 | 0.25 | 0.7776 | 1148 |  | False |
| -0.6667 | 0.6 | 0.8626 | 1266 |  | False |
| -0.6667 | 0.85 | 0.9107 | 357 |  | False |
| -0.6667 | 1.25 | 0.8931 | 232 |  | False |
| -0.6667 | 1.5 | 0.8931 | 0 | 8 | False |
| -0.3333 | 0.25 | 1.0245 | 691 |  | False |
| -0.3333 | 0.6 | 0.8519 | 836 |  | False |
| -0.3333 | 0.85 | 0.9502 | 66 |  | False |
| -0.3333 | 1.25 | 0.8769 | 58 |  | False |
| -0.3333 | 1.5 | 0.8769 | 1 | 13 | False |
| +0.0000 | 0.25 | 1.0500 | 898 |  | True |
| +0.0000 | 0.6 | 0.9945 | 1125 |  | False |
| +0.0000 | 0.85 | 0.9408 | 59 |  | False |
| +0.0000 | 1.25 | 0.8629 | 59 |  | False |
| +0.0000 | 1.5 | 0.8629 | 15 | 18 | False |
| +0.3333 | 0.25 | 0.9810 | 1832 |  | False |
| +0.3333 | 0.6 | 0.8331 | 1980 |  | False |
| +0.3333 | 0.85 | 0.9357 | 357 |  | False |
| +0.3333 | 1.25 | 0.9075 | 354 |  | False |
| +0.3333 | 1.5 | 1.0181 | 100 |  | False |
| +0.6667 | 0.25 | 0.8112 | 2880 |  | False |
| +0.6667 | 0.6 | 0.8789 | 3109 |  | False |
| +0.6667 | 0.85 | 0.9440 | 815 |  | False |
| +0.6667 | 1.25 | 0.9483 | 769 |  | False |
| +0.6667 | 1.5 | 1.0500 | 98 |  | True |
| +1.0000 | 0.25 | 0.7515 | 1789 |  | False |
| +1.0000 | 0.6 | 0.9428 | 1911 |  | False |
| +1.0000 | 0.85 | 0.9720 | 649 |  | False |
| +1.0000 | 1.25 | 0.9849 | 398 |  | False |
| +1.0000 | 1.5 | 1.0500 | 2 | 29 | False |

## TL4 on dyn-v0.1

Shipping wd1e-3, seed 2, device cpu, float64 (MPS available but has no float64 and the recipe is float64).

| config | mean validation P1 | min | per seed |
|---|---|---|---|
| wd1e-3 | 0.0508 | 0.0507 | seed0 0.0509, seed1 0.0508, seed2 0.0507 |
| wd1e-2 | 0.0511 | 0.0508 | seed0 0.0509, seed1 0.0517, seed2 0.0508 |

## TL4 on N4

Shipping wd1e-3, seed 0, device cpu, float64 (MPS available but has no float64 and the recipe is float64).

| config | mean validation P1 | min | per seed |
|---|---|---|---|
| wd1e-3 | 0.0510 | 0.0508 | seed0 0.0508, seed1 0.0512, seed2 0.0510 |
| wd1e-2 | 0.0510 | 0.0508 | seed0 0.0508, seed1 0.0512, seed2 0.0511 |

