"""Bag replay for the gap follower (tools/replay_follower.py, docs/notes/replay-follower.md).

Takes a recorded run (mcap, indexed or not), replays its /scan messages through the same
GapFollower code gap_follow_node runs (the gap_follow_replay_cli binary built from
ros_ws/src/racer_control/src/gap_follow_replay_cli.cpp), and compares the result with what the
car did (/drive_raw, /drive, /safety/events).

Modules:
  bag.py       stream an mcap file into plain records; tolerates a bag whose recorder was killed
  replay.py    profiles from gap_follow.launch.py, the scan text file, running the CLI
  analysis.py  per-scan table, summary, CSV
  plots.py     matplotlib figures (optional extra: `uv sync --extra plots`)

Everything except plots.py is stdlib plus the mcap packages, so the logic is unit-tested in
tools/tests/test_replay_follower.py without ROS.
"""
