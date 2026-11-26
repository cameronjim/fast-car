# trackperc-v7 attempts (2026-11-25): no recipe beat v6 internally, loop stopped, nothing scored on H

Iteration after v6 (owner decisions 29 and 37a). Every number comes from the w2-v6 internal
validation part (3159 labels; see `docs/notes/trackperc-v6-2026-11-25.md`), scored through
`TrackPerceiver` with the scan refinement. No held-out set was opened. No v7 bundle was made,
because no attempt differed from v6 on any screen item.

**Held-out scoring count: H-layout 3 (v3, v4, v5), H-session 3 (v3, v4, v5). Unchanged since v5.**

## v7 internal screen (written before any v7 run)

Items 1, 2 and 4 are as in v6. Item 3 on the older bags compares with v5-dev instead of v2: v2
trained on most of those bags' internal cuts, and v5-dev failed v6's version of the item itself.

1. V >= V(v5-dev) + 0.20 on session 2 and 2026-11-16
2. V >= V(v2) - 0.10 on session 2 and 2026-11-16, >= V(v2) - 0.25 on the older bags
3. E_c <= E_c(v2) on session 2 and 2026-11-16, E_c <= E_c(v5-dev) on the older bags
4. E_c <= 1.03 E_c(v5-dev) on all three

## Attempts (seed 0, the v6 recipe plus one change each)

| run | V s2 | V 11-16 | V old | E_c s2 | E_c 11-16 | E_c old |
|---|---|---|---|---|---|---|
| v6 seed 0 (reference) | 15.40 | 15.29 | 15.58 | 0.1675 | 0.1447 | 0.0759 |
| + 100k tight-turn sim samples (`dataset --tight`, `train --train-extra`) | 15.37 | 15.33 | 15.59 | 0.1672 | 0.1439 | 0.0757 |
| + deficit down-weighting (`train --deficit-ref 0.02`, floor 0.25) | 15.39 | 15.30 | 15.58 | 0.1679 | 0.1470 | 0.0758 |
| limits (items 3, 4) | | | | 0.1644 | 0.1525 | 0.0751 |

Both fail items 3 and 4 the same way v6 does. Also tried as analysis on v6 and v5-dev:
* Global sigma scaling (temperature) from 0.7 to 1.2: session 2 V moves at most 0.1 and E_c at
  most 0.002. sigma no longer sets V; geometry does.
* A weight average of the 3 seeds: no change (same init, near-identical nets).

## Why the loop stops here

The binding constraint is no longer sigma. V and E_c now trade through geometry:
* V is capped by the refined polyline's arc length. The spacing prior adds stations, and those
  far stations have about 0.27 to 0.35 m error on session 2 (station 15). Since E_c averages each
  net's own valid stations, more range means a higher E_c. On the stations both nets keep, v6 is
  as good as v5-dev or better (session 2 0.161 vs 0.158, 2026-11-16 0.138 vs 0.147, older 0.073
  vs 0.075).
* The real self-labels have near-square corners: heading changes of about 1.5 rad within one
  0.20 m station, and 50 to 85 percent of the labels on the 2026-11-13 and later bags have one.
  The label stations are also compressed in corners (session 2 arc 2.925 m against the contract's
  3.0). The refinement always outputs 0.20 m spacing, so far stations sit about 0.07 m
  along-track from the label after a corner, whatever the net does.
* The simulator's lanes are rounded (curvature test: radius at least half width plus 0.15 m), so
  the tight-turn worlds do not reproduce the labeller's corners. That explains why more sim
  turns did not help.

The next real step is not a recipe tweak. It is labeller or label-convention work: how corners
are labelled, and arc-length-consistent station placement. Alternatively, a decision on whether
an E_c averaged over own-valid stations should compare nets with different valid ranges. Both
are owner questions. Ensembles were not tried: the seeds are near-identical from the shared init,
and a diverse ensemble needs a new bundle type in ros_ws plus 2 to 3 times the net forward cost.

## Code (default off)

`dataset --tight` (`tracks.sample_tight_world`: islands with corner radius under 0.3 m and
higher-amplitude Fourier loops, lanes 0.7 to 1.2 m), `train --train-extra` (more sim sets),
`train --deficit-ref / --deficit-floor` (`real.deficit_weights`). Dataset:
`car-data/pnp/datasets/trackperc-v3dr-tight-{train,val}-s71-*`. Runs:
`car-data/pnp/runs-v6/{v7s1,v7,screen/df02-*}`.
