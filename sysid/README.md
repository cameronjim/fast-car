# sysid

System identification for the vehicle model. `batteries` holds the scripted maneuvers used
for a full identification pass and the shorter 10-minute re-identification run; `fitting`
holds the parameter-fitting code and its held-out validation; `drift` holds the committed,
per-session parameter record used to track how the car's dynamics change over time. See
`claude-docs/02-repo-layout.md` and `claude-docs/07-sim-and-sysid.md`.
