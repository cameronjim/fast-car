"""L3: startup refusals. A bad checksum, a vehicle_params mismatch and v_max_mps above the bundle
validity each make the node exit non-zero before it publishes anything, naming the error."""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "152")

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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
    )


_CHECKSUM = _node("planner_bad_checksum", _BAD_SUM)
_VEHICLE = _node("planner_bad_vehicle", _BAD_VEHICLE)
_VALIDITY = _node("planner_over_validity", _GOOD, v_max_mps=0.95)
_MISSING = _node("planner_no_bundle", _ROOT / "does_not_exist")


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            _CHECKSUM,
            _VEHICLE,
            _VALIDITY,
            _MISSING,
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
        ]
        for action, code in cases:
            proc_info.assertWaitForShutdown(process=action, timeout=30)
            proc_output.assertWaitFor(code, process=action, timeout=10)


@launch_testing.post_shutdown_test()
class TestPlannerNodeRefusalExitCodes(unittest.TestCase):
    def test_exit_codes_are_non_zero(self, proc_info):
        for action in (_CHECKSUM, _VEHICLE, _VALIDITY, _MISSING):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[1], process=action
            )
        common.remove_tree(_ROOT)
