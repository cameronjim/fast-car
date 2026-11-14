"""Publishes /clock from the system clock at about 2 kHz, for a simulated stack whose bridge does
not publish one. track_family_runner.py starts it with --wall-clock and gives mppi_planner_node
use_sim_time:=true, which its debug_stall_profile (stall injection into the solve thread, for the
simulation acceptance of the planner's timing) requires: on the car use_sim_time is false and the
node refuses to start with a stall profile. The time it publishes is the system clock, the epoch
every other node of the stack stamps with, so nothing else notices the difference.

Not a library: python wall_clock_node.py [--ros-args ...].
"""

from __future__ import annotations

import contextlib
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

PERIOD_S = 0.0005


def main() -> int:
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("wall_clock_node")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Clock, "/clock", qos)
    msg = Clock()
    try:
        while rclpy.ok():
            ns = time.time_ns()
            msg.clock.sec = ns // 1_000_000_000
            msg.clock.nanosec = ns % 1_000_000_000
            pub.publish(msg)
            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        with contextlib.suppress(Exception):  # already shut down by the signal handler
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
