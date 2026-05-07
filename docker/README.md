# docker

One image per development or deployment target, each with its own Dockerfile and
dependency lockfile: `sim-cpu` for headless simulation, `train-cuda` for GPU training on
Desktop A, `ros-dev` for ROS 2 Humble development, and `car` for the Jetson Orin Nano
on-vehicle build. All four images share the same source tree in this monorepo. See
`claude-docs/02-repo-layout.md` and `claude-docs/03-environments.md`. The Dockerfiles and
lockfiles themselves are added by roadmap tasks 0.2 through 0.4, not this skeleton.
