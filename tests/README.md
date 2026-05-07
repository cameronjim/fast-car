# tests

Cross-cutting test fixtures that don't live inside a single package. `bags/` holds the
curated small rosbags used for L4 replay and golden tests (nominal segments, LiDAR dropout,
stale-sensor, brownout) plus the bag-mutation fault injectors; `bench/` holds the scripted
L6 bench/HIL checklists that a human runs before any on-track session touching the relevant
code. Per-package unit, property, and node tests (L1-L3) live alongside their packages
instead. See `claude-docs/12-testing.md`.
