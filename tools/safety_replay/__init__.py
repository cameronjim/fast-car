"""Bag replay of racer_safety's obstacle gate (tools/replay_safety.py).

The recorded /scan + /drive_raw (+ /odom/wheel) stream goes through safety_replay_cli, a
ROS-free build of the real gate (ros_ws/src/racer_safety/src/safety_replay_cli.cpp), once with
the rule the bag was driven with and once with the rule under test. Each recorded brake is then
matched with both replays and with what the car actually did, measured from the LiDAR: how far it
travelled after the brake (scan matching) and how close it came to rest to the nearest return.
docs/notes/safety-floor-review-2026-11-02.md has the method and the numbers.
"""
