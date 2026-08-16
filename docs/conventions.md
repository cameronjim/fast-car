# Conventions

Day-to-day working conventions for this repo. Seeded from `claude-docs/10-conventions.md`,
which stays authoritative along with `claude-docs/12-testing.md` (testing layers, coverage
gates, definition of done). If this file and a claude-doc ever disagree, the claude-doc wins.

## Tooling preferences

Swappable by a human decision (see `claude-docs/03-environments.md`); everything below the
next heading is convention, not architecture.

- Python: `uv` for envs and deps (per-package `pyproject.toml`), `ruff` for lint and format
  (line length 100), `pytest` for tests.
- C++: C++17, `clang-format` (config at repo root), `clang-tidy` in CI for `racer_safety`
  and `racer_control`, `colcon` for builds.
- Experiment tracking: W&B or equivalent for training curves. The source of truth is always
  the committed config file.

## Python

- Type hints on public functions. `mypy` runs on `envelope/` and `racer_policy`, the
  correctness-critical Python.
- No `print()` in library code; use `logging`, or `rclpy` logging inside nodes. Scripts
  under `tools/` and `sysid/batteries/` may print.

## C++

- No heap allocation in the 50 Hz control path after init. No blocking calls in callbacks.
- Gate and decision logic always lives apart from node plumbing so it is testable without
  ROS.

## ROS

- ROS 2 Humble, pinned by JetPack 6.
- Parameters declared with descriptors and ranges; no undeclared params.
- Topic names and types are fixed in `claude-docs/04-architecture.md`. Changing one is an
  interface change and requires the doc update in the same PR.
- Launch files live in `racer_bringup`; per-machine config via launch arguments, not edits.
- QoS: sensor data `best_effort`, command path `reliable`, explicit depth, never default.

## Git

- Conventional commits (`feat:`, `fix:`, `docs:`, `sysid:`, `eval:` and so on). Small PRs,
  one roadmap task per branch, named `task/<id>-<slug>`.
- Never commit: `data/` (bags, checkpoints), secrets, generated bindings (CI regenerates).
- Always commit: configs that produced results, prereg docs, drift records, session notes.
- A PR that changes the behavior of a safety layer (`claude-docs/05-safety.md`) carries an
  explicit "safety impact" section in its description.

## Data and experiments

- Rosbag every run, named per `claude-docs/02-repo-layout.md`. Bags are immutable once
  written.
- An experiment is reproducible from config file + git SHA + seed + `vehicle_params`
  version. Reported numbers come only from committed analysis code in
  `evaluation/analysis/`.

## Documentation

- Writeup is continuous: per-phase notes land in `docs/notes/` when things happen, not
  months later.
- Every measured quantity (latency, jitter, delays, offsets) gets written into a doc or
  `vehicle_params`. A measurement that lives only in terminal scrollback did not happen.
- Roadmap checkboxes in `claude-docs/01-roadmap.md` are updated in the same PR that
  completes the task.
