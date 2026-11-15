do-not-deploy; contract 2: no planner on main can load this; candidate only

# dyn-v0.4tl-2026-11-14

TL4: T4 plus the bounded learned correction, shipping seed 2.

Status: candidate, not confirmed: needs the fresh confirmation of pre-registration v0.4 section 2.5.

GP2e-L learned correction (not evaluated: GP2e-T failed): not evaluated.

Validity: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at min(rule, 1.3).

- bundle model passed its gate: 0 >= 1 FAIL
- fast ccw: P1 ratio to dyn-v0.1 (36 windows): 0.449833 <= 1 PASS
- fast cw: windows with start speed >= 1.15 m/s: 24 >= 30 FAIL the fast item cannot be applied

## Data

- train role under T2, split 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63, addendum ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824

Training device: cpu, float64 (MPS available but has no float64 and the recipe is float64).
