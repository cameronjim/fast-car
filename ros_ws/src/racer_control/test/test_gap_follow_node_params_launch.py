"""L3 parameter range checks for gap_follow_node's gap selection, swept-path and speed
smoothing parameters.

forward_preference and gap_switch_margin (include/racer_control/gap_follow.hpp,
GapPreference) are declared with a [0, 1] floating point range. A value outside it must stop
the node from starting (rclcpp rejects it at declaration, gap_follow_node's main catches the
exception, logs it as FATAL and exits 1), never be clamped or warned about. Both are passed
through the real launch/gap_follow.launch.py launch arguments, so this also covers the
launch file forwarding them as floats. swept_path_lookahead_m (range [0.01, 10] m) gets the
same check with 0. The speed smoothing parameters (2026-10-06 floor checkpoint) get it too:
speed_time_constant_s (range [0, 5] s) with 6, and target_range_median_scans (integer range
[1, 15]) with 0, which also covers the launch file forwarding it as an integer. Lane centring's
centering_gain (range [0, 5] rad) gets it with -0.1.

Each node runs in its own scoped group so the includes' launch arguments do not leak into
each other. They share the node name; neither gets far enough to matter.
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "86")

import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")


def _include(**launch_arguments: str) -> GroupAction:
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    arguments = {"params_file": _PARAMS_FILE, **launch_arguments}
    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
                launch_arguments=arguments.items(),
            )
        ],
        scoped=True,
    )


# keep_alive: every node is EXPECTED to exit at once. Without it the launch service shuts
# down as soon as they do, before the active test has finished, and launch_testing reports
# that as a failure.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            _include(forward_preference="1.5"),
            _include(gap_switch_margin="-0.1"),
            _include(swept_path_lookahead_m="0"),
            _include(speed_time_constant_s="6"),
            _include(target_range_median_scans="0"),
            _include(centering_gain="-0.1"),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGapFollowNodeParameterRanges(unittest.TestCase):
    def test_out_of_range_values_are_reported(self, proc_output, proc_info):
        # The FATAL line carries rclcpp's exception text, which names the parameter.
        proc_output.assertWaitFor("forward_preference", timeout=30, stream="stderr")
        proc_output.assertWaitFor("gap_switch_margin", timeout=30, stream="stderr")
        proc_output.assertWaitFor("swept_path_lookahead_m", timeout=30, stream="stderr")
        proc_output.assertWaitFor("speed_time_constant_s", timeout=30, stream="stderr")
        proc_output.assertWaitFor("target_range_median_scans", timeout=30, stream="stderr")
        proc_output.assertWaitFor("centering_gain", timeout=30, stream="stderr")
        # Let every process exit on their own before this test returns; otherwise launch
        # shuts them down with SIGINT and the exit code check below sees -2, not 1.
        processes = proc_info.processes()
        self.assertEqual(len(processes), 6)
        for process in processes:
            proc_info.assertWaitForShutdown(process=process, timeout=30)


@launch_testing.post_shutdown_test()
class TestGapFollowNodeParameterRangesShutdown(unittest.TestCase):
    def test_every_node_refused_to_start(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1])
