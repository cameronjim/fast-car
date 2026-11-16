# M-B6: the frozen v0.4 models on the PnP runs of the 2026-11-15 session, whole

Pre-registration v0.5 section 4. Printed, deciding nothing. The models were frozen before this session existed (freeze commit 416e68501c424caeb1f6dc80b6a0ee9edc7a288c).

- Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82.
- Runs: every PnP run of the session as tools/pnp_data finds it (a /pnp/planner/status stretch): 5 runs in 4 bags.
- Truth T2 of each whole run (wheel seeded ICP, flags of pnp-v0.4.yaml); windows as the gate builds them (1.5 s, start every 0.25 s, start speed at least 0.25 m/s); direction of a window from the measured T2 heading of its 5 s block, not from a block name.
- Models: dyn-v0.1; N4; R4 and TL4 at their shipping seeds, T4, all on dyn-v0.1 (the shipping nominal of the v0.4 pass), loaded from the v0.4 pickles under the cache.

Pickles scored (SHA-256): n4 905e3d154da8e8c9, r4 9fe163c2ca4ad8ca, t4 5d4484ed6956818f, tl4 39ca964ad08a07aa.

Windows: 1576; by direction (measured heading): ccw 1250, cw 326.

## Macro RMSE at 1.0 s, identical windows, T2

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 / dyn-v0.1 | Y1 / dyn-v0.1 |
|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.0932 | 0.0722 | 0.0582 | 0.0876 | 0.0591 | 1.0000 | 1.0000 |
| N4 | 0.0879 | 0.0714 | 0.0512 | 0.0596 | 0.0496 | 0.9429 | 0.6802 |
| R4 | 0.0910 | 0.0748 | 0.0516 | 0.0510 | 0.0591 | 0.9767 | 0.5819 |
| T4 | 0.0610 | 0.0421 | 0.0437 | 0.0876 | 0.0591 | 0.6543 | 1.0000 |
| TL4 | 0.0511 | 0.0342 | 0.0376 | 0.0569 | 0.0591 | 0.5487 | 0.6499 |

## Per direction (macro over that direction's strata)

| model | P1 ccw (m) | Y1 ccw (rad) | P1 cw (m) | Y1 cw (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.0741 | 0.0797 | 0.1123 | 0.0954 |
| N4 | 0.0716 | 0.0640 | 0.1042 | 0.0551 |
| R4 | 0.0716 | 0.0473 | 0.1105 | 0.0546 |
| T4 | 0.0551 | 0.0797 | 0.0669 | 0.0954 |
| TL4 | 0.0484 | 0.0599 | 0.0538 | 0.0540 |

## Per run, pooled position RMSE at 1.0 s (m)

| run | windows | dyn-v0.1 | N4 | R4 | T4 | TL4 |
|---|---|---|---|---|---|---|
| 02-18-54:R0 | 443 | 0.0771 | 0.0725 | 0.0699 | 0.0552 | 0.0495 |
| 22-38-47:R0 | 446 | 0.0777 | 0.0738 | 0.0722 | 0.0551 | 0.0506 |
| 22-45-39:R0 | 5 | 0.0227 | 0.0301 | 0.0140 | 0.0287 | 0.0427 |
| 22-45-39:R1 | 356 | 0.0948 | 0.0839 | 0.0911 | 0.0738 | 0.0622 |
| 23-04-09:R0 | 326 | 0.1171 | 0.1076 | 0.1157 | 0.0688 | 0.0557 |

## Strata (windows)

- ccw:0.25-0.6: 126
- ccw:0.6-0.85: 175
- ccw:0.85+: 949
- cw:0.25-0.6: 126
- cw:0.6-0.85: 84
- cw:0.85+: 116

Merged strata: {'direction': 'ccw', 'from': '1.25+', 'windows': 10, 'into': '0.85-1.25'}; {'direction': 'cw', 'from': '1.25+', 'windows': 20, 'into': '0.85-1.25'}.
