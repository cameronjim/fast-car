# CLAUDE.md — 1/10 Autonomous Racer

Physical 1/10-scale autonomous racing car. **Scope option A: marginal value of learning** —
a learned residual policy on top of a tuned classical raceline-tracking stack, evaluated on
hardware under a pre-registered protocol. Full plan: `claude-docs/00-project-overview.md`.

## How to work in this repo

- Work **one roadmap task at a time** from `claude-docs/01-roadmap.md`. Do not start a task
  whose phase gate has not been passed. Mark tasks done in that file when complete.
- **A task is not done without its tests.** Every task's PR includes the tests required by
  `claude-docs/12-testing.md` for the layers it touches, passing in CI. Never weaken a
  tolerance, golden file, or coverage gate to get green.
- Before touching any subsystem, read its doc (table below). The docs are authoritative over
  your instincts and over generic ROS/RL conventions.
- Ask before adding scope. If a task seems to require building something listed under
  "Explicitly out of scope" below, stop and ask.

## Hard invariants (never violate)

1. **Safety layering is sacred.** Nothing in software may bypass or reconfigure the hardware
   RC mux (layer 1). The safety node gates `/drive_raw` → `/drive`; no node ever publishes
   to the actuator topic directly. See `claude-docs/05-safety.md`.
2. **One source of truth for physical constants.** All vehicle parameters live in
   `config/vehicle_params.yaml` and are consumed via generated bindings. Never hand-write a
   mass, wheelbase, gear ratio, unit conversion, or sign convention in code.
   See `claude-docs/06-vehicle-params.md`.
3. **Deployment refuses on mismatch.** The policy deploy node hard-fails on any
   schema/version mismatch. Never downgrade a refusal to a warning.
4. **SI units everywhere** in code and messages: metres, seconds, radians, newtons, volts.
   Anything else must be converted at the driver boundary and noted in the schema.
5. **Every run is logged** (rosbag + rail voltage). Code paths that drive the car without
   logging are bugs.

## Explicitly out of scope (do not build)

Custom FOC/ESC firmware, custom sensor-hub PCB, TensorRT integration, on-board fine-tuning,
MPCC (stretch only — ask first). See `claude-docs/00-project-overview.md` §Scope.

## Where the details live

| Topic | Doc |
|---|---|
| Thesis, scope, headline result | `claude-docs/00-project-overview.md` |
| Phased task list + go/no-go gates | `claude-docs/01-roadmap.md` |
| Repo layout, package & file naming | `claude-docs/02-repo-layout.md` |
| Dev environments, containers, machines | `claude-docs/03-environments.md` |
| ROS graph, nodes, topics, C++/Python split | `claude-docs/04-architecture.md` |
| Safety architecture (read before any control code) | `claude-docs/05-safety.md` |
| Vehicle params schema, units, sign conventions | `claude-docs/06-vehicle-params.md` |
| Simulator, vehicle model, system ID | `claude-docs/07-sim-and-sysid.md` |
| Residual policy, envelope, deployment contract | `claude-docs/08-learning.md` |
| Evaluation protocol (pre-registered) | `claude-docs/09-evaluation.md` |
| Code practices: style, git, logging | `claude-docs/10-conventions.md` |
| Hardware, wiring, sensor sync, ingest board | `claude-docs/11-hardware.md` |
| Testing strategy: unit → replay → sim-in-loop → bench (definition of done) | `claude-docs/12-testing.md` |

## Quick facts

ROS 2 Humble (pinned by JetPack 6) · Jetson Orin Nano on-vehicle · sim = extended
`f1tenth_gym` · control-critical code in C++, training/analysis in Python · Python tooling:
`uv` + `ruff` + `pytest` · C++17 + `clang-format` + `colcon`.
