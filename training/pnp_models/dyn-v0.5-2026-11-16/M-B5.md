# M-B5: the T5 method on the data that exists before the 2026-11-15 session

Pre-registration v0.5 section 4. A measurement with one stop rule: a supported node on a bound means the reading of section 1 is wrong. No 2026-11-15 data was read.

- Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82.
- Train windows: 7411 of 0.5 s (start every 0.25 s, no wheel speed sample under 0.3 m/s) from the v0.2 split train segments, the legacy train part and the 2026-11-14 train segments (train and validation-of-train parts).
- Selection windows: 1281 (1.5 s windows of the 2026-11-14 held-out segments, T2).
- Targets: ground distance = sum of the T2 ICP step lengths (the quantity M-B4 measures), yaw change = sum of the T2 yaw changes; inputs: the measured wheel speed and the nominal's steering state; base nominal dyn-v0.1.

## Result against the stop rule

No supported node is on a bound (alpha: none; gamma: none). The work goes on.

Smoothness weight: alpha 0.1 (selection A1 by weight {0.01: 0.03074, 0.1: 0.03073, 1.0: 0.03084, 10.0: 0.0316}), gamma 0.01 (selection Y1 by weight {0.01: 0.04443, 0.1: 0.04495, 1.0: 0.04698, 10.0: 0.05852}).
Supported nodes: 18 of 30; alpha range [0.7, 1.05], gamma range [-0.25, 0.25], a node is on a bound within 0.005.
split half alpha: largest difference 0.0264 at s-1/v0.35 against the limit 0.05; 0 supported node(s) above it
split half gamma: largest difference 0.0367 at s-0.5/v0.35 against the limit 0.08; 0 supported node(s) above it

## Selection set (the 2026-11-14 held-out segments), macro RMSE at 1.0 s

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) |
|---|---|---|---|---|---|
| dyn-v0.1 | 0.0854 | 0.0636 | 0.0542 | 0.0728 | 0.0336 |
| M-B5 model (T5) | 0.0459 | 0.0307 | 0.0332 | 0.0444 | 0.0336 |
| ratio | 0.5377 | 0.4835 | 0.6124 | 0.6100 | 1.0000 |

## Both tables, per node

Node = steering share of the side limit (negative right) by measured wheel speed. A node that is not supported copies the node in `tied to` and is no parameter; the gamma centre column is fixed at 0.

| steering share | speed (m/s) | windows | supported | alpha | gamma | 1 + gamma | tied to | alpha on a bound | gamma on a bound | alpha split half | gamma split half |
|---|---|---|---|---|---|---|---|---|---|---|---|
| -1 | 0.35 | 807 | yes | 0.8457 | -0.1323 | 0.8677 |  |  |  | 0.0264 | 0.0110 |
| -1 | 0.6 | 150 | yes | 0.8766 | -0.1583 | 0.8417 |  |  |  | 0.0010 | 0.0148 |
| -1 | 0.85 | 282 | yes | 0.8805 | -0.0999 | 0.9001 |  |  |  | 0.0027 | 0.0166 |
| -1 | 1.1 | 92 | yes | 0.8516 | -0.0681 | 0.9319 |  |  |  | 0.0021 | 0.0117 |
| -1 | 1.3 | 24 | no | 0.8516 | -0.0681 | 0.9319 | s-1/v1.1 |  |  |  |  |
| -1 | 1.5 | 0 | no | 0.8516 | -0.0681 | 0.9319 | s-1/v1.1 |  |  |  |  |
| -0.5 | 0.35 | 373 | yes | 0.8806 | -0.1194 | 0.8806 |  |  |  | 0.0168 | 0.0367 |
| -0.5 | 0.6 | 246 | yes | 0.8955 | 0.0847 | 1.0847 |  |  |  | 0.0003 | 0.0176 |
| -0.5 | 0.85 | 5 | no | 0.8955 | 0.0847 | 1.0847 | s-0.5/v0.6 |  |  |  |  |
| -0.5 | 1.1 | 58 | no | 0.8955 | 0.0847 | 1.0847 | s-0.5/v0.6 |  |  |  |  |
| -0.5 | 1.3 | 42 | no | 0.8955 | 0.0847 | 1.0847 | s-0.5/v0.6 |  |  |  |  |
| -0.5 | 1.5 | 5 | no | 0.8955 | 0.0847 | 1.0847 | s-0.5/v0.6 |  |  |  |  |
| +0 | 0.35 | 125 | yes | 0.9576 | 0.0000 (fixed) | 1.0000 |  |  |  | 0.0041 |  |
| +0 | 0.6 | 353 | yes | 0.9317 | 0.0000 (fixed) | 1.0000 |  |  |  | 0.0147 |  |
| +0 | 0.85 | 12 | no | 0.9317 | 0.0000 (fixed) | 1.0000 | s+0/v0.6 |  |  |  |  |
| +0 | 1.1 | 17 | no | 0.9317 | 0.0000 (fixed) | 1.0000 | s+0/v0.6 |  |  |  |  |
| +0 | 1.3 | 8 | no | 0.9317 | 0.0000 (fixed) | 1.0000 | s+0/v0.6 |  |  |  |  |
| +0 | 1.5 | 3 | no | 0.9317 | 0.0000 (fixed) | 1.0000 | s+0/v0.6 |  |  |  |  |
| +0.5 | 0.35 | 1624 | yes | 0.8998 | -0.0703 | 0.9297 |  |  |  | 0.0190 | 0.0033 |
| +0.5 | 0.6 | 418 | yes | 0.8929 | 0.0884 | 1.0884 |  |  |  | 0.0026 | 0.0306 |
| +0.5 | 0.85 | 295 | yes | 0.9600 | 0.0354 | 1.0354 |  |  |  | 0.0211 | 0.0155 |
| +0.5 | 1.1 | 103 | yes | 0.8957 | 0.1773 | 1.1773 |  |  |  | 0.0057 | 0.0048 |
| +0.5 | 1.3 | 96 | yes | 0.9117 | 0.1581 | 1.1581 |  |  |  | 0.0135 | 0.0264 |
| +0.5 | 1.5 | 60 | yes | 0.9970 | 0.2195 | 1.2195 |  |  |  | 0.0008 | 0.0077 |
| +1 | 0.35 | 1475 | yes | 0.8872 | -0.1083 | 0.8917 |  |  |  | 0.0202 | 0.0203 |
| +1 | 0.6 | 132 | yes | 0.9484 | -0.0809 | 0.9191 |  |  |  | 0.0186 | 0.0155 |
| +1 | 0.85 | 484 | yes | 0.9398 | 0.0068 | 1.0068 |  |  |  | 0.0018 | 0.0160 |
| +1 | 1.1 | 112 | yes | 0.9925 | 0.0066 | 1.0066 |  |  |  | 0.0013 | 0.0151 |
| +1 | 1.3 | 8 | no | 0.9925 | 0.0066 | 1.0066 | s+1/v1.1 |  |  |  |  |
| +1 | 1.5 | 2 | no | 0.9925 | 0.0066 | 1.0066 | s+1/v1.1 |  |  |  |  |

## 1 + gamma next to alpha at the supported turning nodes

If the yaw rate follows ground speed rather than wheel speed, the two are close.

| steering share | speed (m/s) | alpha | 1 + gamma | (1 + gamma) - alpha |
|---|---|---|---|---|
| -1 | 0.35 | 0.8457 | 0.8677 | 0.0221 |
| -1 | 0.6 | 0.8766 | 0.8417 | -0.0349 |
| -1 | 0.85 | 0.8805 | 0.9001 | 0.0196 |
| -1 | 1.1 | 0.8516 | 0.9319 | 0.0804 |
| -0.5 | 0.35 | 0.8806 | 0.8806 | -0.0000 |
| -0.5 | 0.6 | 0.8955 | 1.0847 | 0.1891 |
| +0.5 | 0.35 | 0.8998 | 0.9297 | 0.0299 |
| +0.5 | 0.6 | 0.8929 | 1.0884 | 0.1955 |
| +0.5 | 0.85 | 0.9600 | 1.0354 | 0.0754 |
| +0.5 | 1.1 | 0.8957 | 1.1773 | 0.2816 |
| +0.5 | 1.3 | 0.9117 | 1.1581 | 0.2464 |
| +0.5 | 1.5 | 0.9970 | 1.2195 | 0.2225 |
| +1 | 0.35 | 0.8872 | 0.8917 | 0.0046 |
| +1 | 0.6 | 0.9484 | 0.9191 | -0.0293 |
| +1 | 0.85 | 0.9398 | 1.0068 | 0.0670 |
| +1 | 1.1 | 0.9925 | 1.0066 | 0.0142 |

