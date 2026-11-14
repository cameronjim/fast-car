# prereg

`pnp-v0.2-split.yaml` (SHA-256 `bc5185c9...ef83`) is the train and held-out split of the real bags for Plug-and-Play v0.2.
`docs/notes/preregistration-pnp-v0.2.md` holds the gates, thresholds and counting rules the split serves.
Both files are frozen: a correction is a dated appendix by the owner, never an edit.
The analysis code that implements them is `evaluation/analysis` (package `racer_eval`, tested to 90 percent line coverage).
Run `uv run --group bags python -m racer_eval baseline --bag <bag dir>` there to recompute the section 4.2 baseline.

`pnp-v0.3.yaml` (SHA-256 `4415b371...d7a1`) holds the constants of the v0.3 attempts: the second lap learning sim battery, the yaw dynamics gate, the perception folds and the deskew measurement.
`docs/notes/preregistration-pnp-v0.3.md` holds their rules. It adds new gates after the v0.2 failures and changes nothing in the v0.2 files.
