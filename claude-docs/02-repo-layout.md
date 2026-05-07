# 02 — Repo Layout and Naming

Monorepo, shared source tree across all four container images (see `03-environments.md`).

```
racer/
├── CLAUDE.md
├── claude-docs/                 # these docs
├── config/
│   ├── vehicle_params.yaml      # THE single source of physical truth
│   ├── vehicle_params.schema.json
│   └── tracks/                  # per-track: map, raceline, timing-gate position
│       └── <venue>_<layout>/    # e.g. gym_a/, gym_b/
├── docker/
│   ├── sim-cpu/     Dockerfile + lockfile
│   ├── train-cuda/  Dockerfile + lockfile
│   ├── ros-dev/     Dockerfile + lockfile
│   └── car/         Dockerfile + lockfile (Jetson-built)
├── ros_ws/src/                  # ROS 2 packages (see naming below)
│   ├── racer_msgs/              # custom messages ONLY if std/ackermann msgs won't do
│   ├── racer_bringup/           # launch files, per-machine configs
│   ├── racer_safety/            # C++. safety node, covariance gate, watchdogs
│   ├── racer_state/             # C++. EKF config/wrappers, localization launch
│   ├── racer_control/           # C++. tracker (pure pursuit / stanley), low-level cmd path
│   ├── racer_policy/            # deploy node: contract loader + inference (Py first)
│   ├── racer_drivers/           # VESC, LiDAR, ingest-board serial driver
│   └── racer_tools/             # teleop, bag utilities, timing-gate reader
├── sim/
│   ├── racer_gym/               # fork/extension of f1tenth_gym: model upgrades
│   └── bridge/                  # gym <-> ROS bridge (Humble port)
├── training/
│   ├── racer_train/             # SAC/PPO residual training package
│   ├── envelope/                # bounds/rate/OOD library — shared with racer_policy
│   └── configs/                 # experiment configs, hashed into the contract
├── sysid/
│   ├── batteries/               # scripted maneuvers: full ID + 10-min re-ID
│   ├── fitting/                 # parameter fitting, held-out validation
│   └── drift/                   # per-session parameter record (committed data)
├── evaluation/
│   ├── prereg/                  # pre-registrations, committed BEFORE data collection
│   ├── protocol/                # session runner: interleaving, run order, checklists
│   └── analysis/                # notebooks/scripts producing the reported numbers
├── firmware/
│   ├── safety_mux/              # layer-1 MCU (RC mux + cutoff)
│   └── ingest/                  # RP2040/Teensy sensor timestamping board
├── docs/
│   ├── conventions.md
│   ├── sync-design.md           # task 2.2 deliverable
│   └── notes/                   # per-phase running notes (writeup is continuous)
└── data/                        # NOT in git: rosbags, checkpoints (see 10-conventions.md)
```

## Naming rules

- ROS packages: `racer_<noun>`, snake_case. Nodes: `<thing>_node` (e.g. `safety_node`).
- Topics: see `04-architecture.md` — topic names are an interface, never improvised.
- Python packages: `racer_*`, importable, `pyproject.toml` each, managed with `uv`.
- C++: headers in `include/racer_<pkg>/`, `.hpp/.cpp`, one class per file where sane.
- Experiment configs: `configs/<phase>/<name>_<semver>.yaml`; never edit a config that has
  produced a committed result — copy and bump.
- Rosbags: `data/bags/<date>_<session##>_<purpose>/` (purpose ∈ id, tune, train-eval, eval).
- Track dirs are the ID used everywhere a track is referenced (configs, prereg, analysis).

## Rules

- No file may duplicate a physical constant from `config/vehicle_params.yaml`.
- `envelope/` is one library consumed by BOTH training (sim env) and `racer_policy`
  (deployment). Never fork its logic.
- Nothing under `evaluation/analysis/` may be edited after its pre-registered result is
  reported, except additive clearly-labeled post-hoc sections.
