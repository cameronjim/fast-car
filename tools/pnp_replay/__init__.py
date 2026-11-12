"""Open-loop replay of Plug-and-Play bags (tools/pnp_replay.py, docs/notes/pnp-replay.md).

A recorded bag's /scan and /odom/wheel go through the real perception code (the numpy package in
ros_ws/src/racer_track_perception) and the real planner (planner_replay_cli, a ROS-free build of
racer_planner::Planner), and the numbers that say whether the driver would have stopped, crawled
or chattered at each recorded pose come out as one JSON per run. The scans do not react to the
replayed commands, so this is a measurement of decisions at recorded poses, not a lap.

    bag.py           indexed mcap read of the PnP topics, cached as an .npz
    segments.py      PnP runs (R0, R1, ...) and fast laps (L) found in a bag
    perception.py    run a perceiver over scans, check it against the recorded /pnp/local_track
    planner_stream.py  the planner_replay_cli stream file, running the CLI
    metrics.py       pure metric functions
    report.py        the stdout table and the comparison with a baseline JSON
    cut_fixture.py   regenerates tools/tests/fixtures/pnp_replay_2026-11-12.npz from the bag
"""
