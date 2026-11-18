# prereg

`pnp-v0.2-split.yaml` (SHA-256 `bc5185c9...ef83`) is the train and held-out split of the real bags for Plug-and-Play v0.2.
`docs/notes/preregistration-pnp-v0.2.md` holds the gates, thresholds and counting rules the split serves.
Both files are frozen: a correction is a dated appendix by the owner, never an edit.
The analysis code that implements them is `evaluation/analysis` (package `racer_eval`, tested to 90 percent line coverage).
Run `uv run --group bags python -m racer_eval baseline --bag <bag dir>` there to recompute the section 4.2 baseline.

`pnp-v0.3.yaml` (SHA-256 `4415b371...d7a1`) holds the constants of the v0.3 attempts: the second lap learning sim battery, the yaw dynamics gate, the perception folds and the deskew measurement.
`docs/notes/preregistration-pnp-v0.3.md` holds their rules. It adds new gates after the v0.2 failures and changes nothing in the v0.2 files.

`pnp-v0.4.yaml` (SHA-256 `78510c14...1d63`) holds the split of the 2026-11-14 car session bags (train, held-out, reserve and lap roles) and the constants of the v0.4 attempts: the dynamics gate GP2d, the perception gate GP1c, the lap timing measurements, the obstacle variant sim battery B5 and the oracle track measurement.
`docs/notes/preregistration-pnp-v0.4.md` holds their rules. It adds new gates after the v0.3 results and changes nothing in the v0.2 and v0.3 files.

`pnp-v0.5-laps-ring.yaml` (SHA-256 `74b1dccd...dc36`) holds the constants of the lap gate v3 attempt: battery B8 (cases R1 to R4, Q1 and Q2 on seeds 61 to 63, the count rule and the class tolerances) and the car gate D8-C0.
`docs/notes/preregistration-pnp-v0.5-laps-ring.md` holds their rules. It is a new attempt after B7 and changes nothing in the earlier files.
