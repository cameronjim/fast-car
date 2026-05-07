# ros_ws

ROS 2 Humble workspace. Packages live under `src/`, one per `racer_<noun>` responsibility:
`racer_msgs` (custom messages, only if standard ones won't do), `racer_bringup` (launch
files and per-machine configs), `racer_safety` and `racer_control` (C++, control-critical),
`racer_state` (localization), `racer_policy` (residual-policy deploy node), `racer_drivers`
(VESC, LiDAR, ingest-board serial), and `racer_tools` (teleop, bag utilities). Topic names,
node graph, and the C++/Python split are fixed in `claude-docs/04-architecture.md`; package
and file naming follow `claude-docs/02-repo-layout.md`.
