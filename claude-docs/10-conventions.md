# 10 — Code Practices and Conventions

These are working conventions, not architecture. They can be swapped by a human decision;
Claude Code follows them as written until then.

## Python

- Tooling: `uv` for env/deps (per-package `pyproject.toml`), `ruff` for lint + format
  (line length 100), `pytest` for tests, type hints on public functions, `mypy` on
  `envelope/` and `racer_policy` (the correctness-critical Python).
- No `print()` in library code — `logging` (or `rclpy` logging in nodes). Scripts under
  `tools/`/`sysid/batteries/` may print.
- Testing is governed by `12-testing.md` (layers, coverage gates, definition of done) —
  that doc is authoritative; this one only covers style-level practice.

## C++

- C++17. `clang-format` (config committed at repo root), `clang-tidy` in CI for
  `racer_safety` and `racer_control`.
- No heap allocation in the 50 Hz control path after init. No blocking calls in callbacks.
- Gate/decision logic is always separated from node plumbing so it is testable without ROS
  (see `12-testing.md` L1/L3 for what each part must cover).

## ROS

- ROS 2 Humble. Parameters declared with descriptors and ranges; no undeclared params.
- Topic names/types are fixed in `04-architecture.md`; changing one is an interface change
  requiring a doc update in the same PR.
- Launch files in `racer_bringup` only. Per-machine config via launch arguments, not edits.
- QoS: sensor data `best_effort`, command path `reliable`, explicit depth — never default.

## Git

- Conventional commits (`feat:`, `fix:`, `docs:`, `sysid:`, `eval:` ...). Small PRs/commits,
  one roadmap task per branch: `task/<id>-<slug>` (e.g. `task/2.4-adopt-ekf`).
- Never commit: `data/` (bags, checkpoints), secrets, generated bindings (CI regenerates).
- Always commit: configs that produced results, prereg docs, drift records, session notes.
- A PR that changes behavior of a safety layer (05) requires an explicit "safety impact"
  section in its description.

## Data and experiments

- Rosbag every run; naming per `02-repo-layout.md`. Bags are immutable once written.
- Experiments are reproducible from: config file + git SHA + seed + `vehicle_params` version.
  Reported numbers come only from committed analysis code in `evaluation/analysis/`.
- W&B (or equivalent) for training curves; the source of truth is still the committed config.

## Documentation

- Writeup is continuous: per-phase notes in `docs/notes/` written when things happen.
  Phase 7 is assembly, not authorship.
- Every measured quantity (latency, jitter, delays, offsets) gets written into a doc or
  `vehicle_params` — a measurement that lives only in a terminal scrollback didn't happen.
- Update `claude-docs/01-roadmap.md` checkboxes in the same PR that completes a task.

## For Claude Code specifically

- One roadmap task per session/PR. Finish, test, update roadmap, stop.
- If a task is ambiguous or seems to require out-of-scope work, ask; do not improvise scope.
- Never "fix" a refuse-on-mismatch failure by loosening the check.
- Prefer adopting maintained packages over writing new code in Phase 2 components (04).
