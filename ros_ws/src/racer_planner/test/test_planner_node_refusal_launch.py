"""L3: startup refusals. A bad checksum, a vehicle_params mismatch, v_max_mps above the bundle
validity and a debug_stall_profile without use_sim_time (or a malformed one) each make the node
exit non-zero before it publishes anything, naming the error.

Each node gets its own ROS_DOMAIN_ID (170 and up, used by no other test). Seven participants in one
domain, created and torn down within the same moments, hit a race inside Fast-DDS 2.6 (Humble's
libfastrtps): a peer's receive thread crashes in StatefulReader::matched_writer_remove while
handling the dispose of a participant that is exiting (SIGSEGV, exit -11, core dump backtrace in
test_gap_follow_node_params_launch.py's docstring; seen here as `mppi_planner_node process has
died, exit code -11` on the CI run for PR 169). The refusals need no peers."""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "152")

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest

# Skips cleanly under a bare `uv run pytest` (no ROS); runs for real under colcon test.
pytest.importorskip("rclpy")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import planner_launch_common as common
import pytest
from launch_ros.actions import Node as LaunchNode
from launch_testing.util import KeepAliveProc

_ROOT = Path(tempfile.mkdtemp(prefix="racer_planner_refusal_"))
_GOOD = common.make_bundle(_ROOT, "good")
_BAD_SUM = common.make_bad_checksum_bundle(_ROOT)
_BAD_VEHICLE = common.make_vehicle_mismatch_bundle(_ROOT)


_NODE_DOMAINS = iter(range(170, 170 + 16))


def _node(name: str, bundle: Path, **extra) -> LaunchNode:
    params = {
        "dynamics_bundle": str(bundle),
        "v_max_mps": 0.5,
        "laser_yaw_from_vehicle_params": False,
    }
    params.update(extra)
    return LaunchNode(
        package="racer_planner",
        executable="mppi_planner_node",
        name=name,
        output="screen",
        parameters=[params],
        additional_env={"ROS_DOMAIN_ID": str(next(_NODE_DOMAINS))},
    )


_CHECKSUM = _node("planner_bad_checksum", _BAD_SUM)
_VEHICLE = _node("planner_bad_vehicle", _BAD_VEHICLE)
_VALIDITY = _node("planner_over_validity", _GOOD, v_max_mps=0.95)
_MISSING = _node("planner_no_bundle", _ROOT / "does_not_exist")
# The stall injection is for simulation only: refused without use_sim_time, and refused when the
# profile is malformed even with it.
_STALL = _node("planner_stall_profile", _GOOD, debug_stall_profile="1.0:0.4")
_STALL_BAD = _node("planner_stall_bad", _GOOD, use_sim_time=True, debug_stall_profile="abc")
_STALL_RANGE = _node("planner_stall_range", _GOOD, use_sim_time=True, debug_stall_profile="1:99")


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            _CHECKSUM,
            _VEHICLE,
            _VALIDITY,
            _MISSING,
            _STALL,
            _STALL_BAD,
            _STALL_RANGE,
            # every process under test exits at startup; keep the launch alive for the checks
            KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestPlannerNodeRefusals(unittest.TestCase):
    def test_each_refusal_exits_and_names_the_error(self, proc_info, proc_output):
        cases = [
            (_CHECKSUM, "ChecksumMismatch"),
            (_VEHICLE, "VehicleParamsMismatch"),
            (_VALIDITY, "ValidityExceeded"),
            (_MISSING, "ManifestMissing"),
            (_STALL, "StallProfileRefused"),
            (_STALL_BAD, "StallProfileInvalid"),
            (_STALL_RANGE, "StallProfileInvalid"),
        ]
        for action, code in cases:
            proc_info.assertWaitForShutdown(process=action, timeout=30)
            proc_output.assertWaitFor(code, process=action, timeout=10)


@launch_testing.post_shutdown_test()
class TestPlannerNodeRefusalExitCodes(unittest.TestCase):
    def test_exit_codes_are_non_zero(self, proc_info):
        for action in (_CHECKSUM, _VEHICLE, _VALIDITY, _MISSING, _STALL, _STALL_BAD, _STALL_RANGE):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[1], process=action
            )
        common.remove_tree(_ROOT)
