do-not-deploy (candidate, its rule against dyn-v0.7p not passed: Y1 only)

# dyn-v0.10-2026-11-25

Third version of the dynamics iteration on dyn-v0.7p's line. Unregistered development model
(owner decision 29), scored once against its own rule
(`evaluation/prereg/dyn-iter/dyn-v0.10-2026-11-25.yaml`, committed before the pass).

Change: dyn-v0.9 (dyn-v0.7p's recipe plus the learned contract 1 ground scale, bundle 1.2.0)
trained on the PnP sessions only (dyn-v0.7p's 2026-11-15 and 2026-11-22 train and validation
parts; no older teleop bag, no 2026-11-23 bag) and the residual width chosen with a 3 percent cost
tolerance: 4-8-8-2 (validation P1 1.6 percent above the best width). Learned ground_scale 0.906.
Shipping seed 0. Validity 1.3 m/s.

Result on H1 (GP2g-R held-out parts of 2026-11-22, 525 windows): FAIL on item 2a only.
P1 0.851 times dyn-v0.7p (bootstrap 0.826 to 0.890), C1 0.908, A1 0.813, S1 0.986, ccw P1 0.899,
ccw Y1 0.981, cw P1 0.684, cw Y1 1.000, T2s P1 0.847, every seed 0.850 to 0.851, transition 0.78,
near-limit right 0.63, Yp 0.98, saturation 0, planner solve 0.48 times dyn-v0.7p's with fast_mlp
(1.90 ms against 3.96 ms). Failed: Y1 0.983 against the GP2g-R bar of 0.90. Against dyn-v0.1, for
information: P1 0.749. H2 (reported only): P1 0.860, Y1 0.99.

H1 scoring count: this pass is the fourth scoring of H1 (R6, dyn-v0.8, dyn-v0.9, dyn-v0.10).

Bundle: `/Users/cameronjim/code/car-data/pnp/models-candidate/dyn-v0.10-2026-11-25`, "(candidate)"
in the app's dynamics picker; needs the planner of this branch (8 wide residual, ground scale).
