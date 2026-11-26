do-not-deploy (candidate, its rule against dyn-v0.7p not passed)

# dyn-v0.8-2026-11-25

First version of the dynamics iteration on dyn-v0.7p's line. Unregistered development model
(owner decision 29), scored once against its own rule
(`evaluation/prereg/dyn-iter/dyn-v0.8-2026-11-25.yaml`, committed before the pass).

Change: dyn-v0.7p's recipe (dyn-v0.1 nominal plus a capped speed and yaw residual, multi step
from one step) trained on every allowed train bag: the older train roles (v0.2 split, legacy,
2026-11-14 train) plus dyn-v0.7p's 2026-11-15 and 2026-11-22 train and validation parts, and no
bag of the 2026-11-23 session (confirmation only; dyn-v0.7p trained on it). The width was chosen
cost aware among 32, 16 and 8 on validation: 32 (16 and 8 were 2 percent worse on validation P1,
outside the 1 percent tolerance). Shipping seed 1. Schema 1.1.0, validity 1.3 m/s.

Result on H1 (the GP2g-R held-out parts of 2026-11-22, 525 windows): FAIL. P1 1.026 times
dyn-v0.7p (bootstrap 1.017 to 1.035), Y1 1.041, C1 1.063, ccw P1 1.039, ccw Y1 1.041. Item 12
failed on noise (same 4-32-32-2 network as dyn-v0.7p, so the same cost; 1.07 measured on a shared
machine). Against dyn-v0.1, for information: P1 0.904. H2 (reported only): P1 1.007.

H1 scoring count: this pass is the second scoring of H1 (the first was the R6 pass of PR 250); it
is also the first H1 score of dyn-v0.7p (P1 0.880 times dyn-v0.1) that this iteration found. Validation (1119 windows): P1 0.973 times dyn-v0.7p.

Bundle: `/Users/cameronjim/code/car-data/pnp/models-candidate/dyn-v0.8-2026-11-25`, labelled
"(candidate)" in the app's dynamics picker. Development runs only.

Files: `pass-report.md`, `pass-result.json`, `fit-summary.md`, `solve.json` (the solve time used;
`solve-first-run-under-load.json` is a first measurement taken while this iteration's own training
ran, replaced before the pass and reported here).
