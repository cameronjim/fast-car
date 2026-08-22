# 03 — Environments, Containers, Machines

Principle: **separate pinned images, shared source tree.** One image cannot honestly serve
CUDA, JetPack, ROS, and Apple Silicon at once. Each image has its own lockfile; the source
tree is the shared artifact.

## Images

| Image | Runs on | Contains | Never contains |
|---|---|---|---|
| `sim-cpu` | Mac (arm64), any desktop | Python, torch CPU, racer_gym | ROS, CUDA |
| `train-cuda` | Desktop A (amd64, RTX 3060) | CUDA, torch cu126, racer_gym, racer_train | ROS |
| `ros-dev` | Mac, desktops | ROS 2 Humble, colcon, racer ROS pkgs | CUDA |
| `car` | Jetson Orin Nano | JetPack 6 base, ROS 2 Humble, Jetson torch | dev tooling bloat |

- The `car` image is built ON the Jetson (or cross-built); it is a different artifact from
  desktop images and must never be assumed interchangeable.
- ROS distro is pinned by JetPack: **JetPack 6 → Humble.** If JetPack changes, the distro
  decision re-opens; that is a top-level decision, not a dependency bump.

## Machines

| Machine | Role |
|---|---|
| MacBook Pro M5 Pro | Primary dev (`sim-cpu`, `ros-dev`), trackside operation via SSH to car |
| Desktop A (RTX 3060) | `train-cuda`: long training runs; artifact/rosbag storage |
| Desktop B (Windows) | Hardware bench: flashing, VESC Tool, scope, logic analyzer. Pinned setup |
| Jetson Orin Nano | On-vehicle only. Not a dev machine |

- No dual boot anywhere: Docker on macOS, WSL2 on Windows, JetPack native on Jetson.
- The Mac never talks to hardware directly (no USB passthrough into containers); it SSHes to
  the car. This is the intended workflow, not a limitation to work around.

## Rules for Claude Code

- When adding a dependency, add it to the correct image's lockfile ONLY. A package needed in
  two images is added twice, explicitly. Never create a shared requirements file.
- Rebuilds must be reproducible from the Dockerfile + lockfile alone; no `docker exec` state.
- CI runs in `sim-cpu` (Python tests) and `ros-dev` (colcon build + C++ tests). Training and
  car images are not CI targets.
- Tooling preferences (Tailscale, W&B, MinIO, etc.) live in `docs/conventions.md` and are
  swappable; never architect around them.
