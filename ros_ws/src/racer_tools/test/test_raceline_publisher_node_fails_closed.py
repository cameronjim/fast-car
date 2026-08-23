"""Fail-closed check for racer_tools' raceline_publisher_node (roadmap milestone 3,
claude-docs/05-safety.md: "fail closed"): a missing `raceline_path` parameter must refuse
to start, not silently publish nothing.

A plain rclpy test (no launch_testing) constructing the node directly -- deliberately kept
out of test_raceline_publisher_node_launch.py, whose launch_testing active-phase machinery
manages rclpy's context/process lifecycle around a REAL launched process; mixing in a
directly-constructed node there would fight that machinery. Guarded with
pytest.importorskip so this skips cleanly under the bare `uv run pytest` L1 run
(racer_tools/pyproject.toml has no rclpy) and runs for real under `colcon test` in the
ros-dev image, same as test_raceline_publisher_node_launch.py."""

from __future__ import annotations

import unittest

import pytest

pytest.importorskip("rclpy")

import rclpy


class TestRacelinePublisherNodeFailsClosedOnMissingRaceline(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def test_missing_raceline_path_parameter_raises_instead_of_publishing_nothing(self):
        from racer_tools.raceline_publisher_node import RacelinePublisherNode

        with self.assertRaises(RuntimeError):
            RacelinePublisherNode()
