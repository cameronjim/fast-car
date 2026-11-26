# trackperc-v6 (2026-11-25): internal screen failed, not scored on H, v2 stays the published net

Unregistered development candidate (owner decisions 29 and 37a), first of the v6+ iteration loop.
Record: `evaluation/prereg/w2-trackperc-v6-2026-11-25.yaml`. Candidate bundle:
`car-data/pnp/models-candidate/trackperc-v6-2026-11-25` (weights not committed), "(candidate)" in
the Racer app Net bundle picker. No default changes.

**Held-out scoring count: H-layout 3 (v3, v4, v5), H-session 3 (v3, v4, v5). v6 did not add one.**

## A bigger internal validation part (w2-v6)

`train --val-plan w2-v6` / `internal_val --val-plan w2-v6`: last 45 percent of each session 2
train segment, last 40 percent of each 2026-11-16 bag, 15 percent elsewhere. 3159 labels (session
2: 334, was 43). Scored through `TrackPerceiver` with the scan refinement, as the node runs the
bundle (`internal_val --refine` for runs). v5's recipe retrained on this train part (v5-dev)
reproduces the H-session gap: session 2 V 14.86 vs v2 15.34 (H-session 14.85 vs 15.27).

## Diagnosis: the remaining V gap is geometric, not sigma

Refined V split into a geometry-only count (sigma forced valid) and a sigma-only count (no
refinement), session 2 internal:

| net | refined V | geometry-only | sigma-only | net arc (m) |
|---|---|---|---|---|
| v2 | 15.34 | 15.40 | 15.93 | 2.89 |
| v5-dev | 14.87 | 14.99 | 15.86 | 2.81 |
| v4-dev (stage 1) | 13.45 | 14.99 | 14.27 | 2.81 |

The refinement resamples the polyline at 0.20 m from the point nearest the car, so its count is
capped by arc length (16 stations need 2.95 m). v4/v5 far stations shrink towards the car on the
new bags (spacing 0.175 to 0.185 m at stations 8 to 14). v5's sigma refit fixed the sigma part;
the arc length is what is left.

## Recipe

1. Stage 1: v4 recipe on the w2-v6 train part plus a station-spacing prior
   (`train --spacing-weight 0.1`: mean of ((gap - 0.20) / 0.01)^2 over labelled pairs).
2. Stage 2: v5 sigma-head refit with the label noise floor at 0.15 m (0.10 left session 2
   sigma-only V at 15.50 once the geometry was fixed).

Screened on the internal part only: weight 0.1 / 0.3, floor 0.10 / 0.15 / 0.10 plus a far ramp, a
weight average of the 3 seeds (no gain: same init, near-identical nets). Seed 2 by final raw
internal E_c.

## Internal screen (written before the candidate was scored on it)

| group | V v6 | V v5-dev | V v2 | E_c v6 | E_c v5-dev | E_c v2 |
|---|---|---|---|---|---|---|
| session 2 (334) | 15.39 | 14.86 | 15.34 | 0.1672 | 0.1596 | 0.1681 |
| 2026-11-16 (864) | 15.30 | 14.87 | 15.18 | 0.1447 | 0.1481 | 0.1663 |
| older (1961) | 15.57 | 15.36 | 15.40 | 0.0757 | 0.0751 | 0.0743 |

V items pass on every group. E_c fails on two items: older bags 0.0757 > v2 0.0743, and session
2 0.1672 > 1.03 x v5-dev (0.1644). So H was not scored.

Reading: on stations both nets keep, v6's E_c is about the same or better (session 2 0.161 vs
v5-dev 0.158, 2026-11-16 0.138 vs 0.147, older 0.073 vs 0.075). Most of the E_c rise comes from
the far stations v6 now keeps, and E_c is averaged over each net's own valid stations. The
older-bag item compares against a v2 that was trained on most of those labels, and v5-dev fails it
too. That makes it a weak screen item, but it was written down first, so it stands for v6.
