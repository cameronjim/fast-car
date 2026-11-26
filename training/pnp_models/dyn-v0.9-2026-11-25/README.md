do-not-deploy (candidate, its rule against dyn-v0.7p not passed)

# dyn-v0.9-2026-11-25

Second version of the dynamics iteration on dyn-v0.7p's line. Unregistered development model
(owner decision 29), scored once against its own rule
(`evaluation/prereg/dyn-iter/dyn-v0.9-2026-11-25.yaml`, committed before the pass).

Change: dyn-v0.8 plus the contract 1 ground scale, one constant learned with the residual by the
multi step loss: the position moves by ground_scale * v while v stays the wheel speed. Learned
ground_scale 0.909 (all nine fits 0.909 to 0.911): the car covers about 0.91 m of ground per metre
of wheel odometry under truth T2. Bundle schema 1.2.0 (`io.residual.ground_scale`); a planner
older than this branch refuses the bundle (golden rollouts). Width 32 (16 and 8 were 1.4 percent
worse on validation, outside the 1 percent tolerance). Shipping seed 0. Validity 1.3 m/s.

Result on H1 (GP2g-R held-out parts of 2026-11-22, 525 windows): FAIL on Y1 and on solve time.
P1 0.860 times dyn-v0.7p (bootstrap 0.834 to 0.900), C1 0.933, A1 0.818, S1 0.979, ccw P1 0.898,
cw P1 0.727, T2s P1 0.857, every seed 0.859 to 0.869, transition 0.80, near-limit right 0.71.
Failed: Y1 1.041 (ccw 1.029, cw 1.113; dyn-v0.8 had the same Y1, so it comes with the training
data, not the scale) and item 12 at 1.075 (same 4-32-32-2 network as dyn-v0.7p: noise on a shared
machine). Against dyn-v0.1, for information: P1 0.757. H2 (reported only): P1 0.857, Y1 0.975.

Speed adaptation (validation replay, `stages_iter adapt-val`): on dyn-v0.7p it worsens P1 by 17
percent; on this model's recipe by 1.3 percent (Y1 and S1 improve). The ground scale removes most
of what made the adaptation hurt; it still does not help position, so it stays off.

H1 scoring count: this pass is the third scoring of H1 (R6, dyn-v0.8, dyn-v0.9).

Bundle: `/Users/cameronjim/code/car-data/pnp/models-candidate/dyn-v0.9-2026-11-25`, "(candidate)"
in the app's dynamics picker; needs the planner of this branch (colcon build racer_planner on the
car).
