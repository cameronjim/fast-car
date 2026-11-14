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
centering_gain (range [0, 5] rad) gets it with -0.1, and the reverse escape's
escape_probe_distance_m (range [0.02, 3] m) with 0. The lane-centre target's lane_lookahead_m
(range [0.1, 3] m) and lane_default_half_width_m (range [0.1, 2] m) get it with 0 and 5. The
race parameters (2026-11-12) get it too: a_lat_max_mps2 (range [0.1, 20]) with 0, apex_offset_m
and track_out_offset_m (range [0, 1] m) with 2 and -1, stopping_margin_m (range [0, 1] m) with 3,
side_cap_distance_m (range [0, 1] m) with 5 and status_rate_hz (range [0, 10] Hz) with 50.

Each node runs in its own scoped group so the includes' launch arguments do not leak into
each other. They share the node name; neither gets far enough to matter.

Each group also gets its own ROS_DOMAIN_ID. Fifteen participants in ONE domain, all created and
all torn down within the same 100 ms, hit a race inside Fast-DDS 2.6 (Humble's libfastrtps): the
receive thread of one process runs EDPSimplePUBListener::onNewCacheChangeAdded ->
PDP::removeWriterProxyData -> StatefulReader::matched_writer_remove for a peer that is going away
and crashes there, SIGSEGV (exit -11), in roughly 1 process in 450 at 2 CPUs (core dump
backtrace, 2026-11-14; seen in CI as `gap_follow_node process has died, exit code -11` in the L3
job of the runs for PR 161 and PR 168). The crash is in the middleware, not in gap_follow_node,
which has already thrown out of its constructor by then. This test is about the nodes' parameter
refusals, not about discovery, and needs no peers, so no two of them share a domain.
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
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")


# One domain per node, 160 and up: no other test uses them (see the docstring).
_FIRST_NODE_DOMAIN = 160


def _include(domain: int, **launch_arguments: str) -> GroupAction:
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    arguments = {"params_file": _PARAMS_FILE, **launch_arguments}
    return GroupAction(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
                launch_arguments=arguments.items(),
            ),
        ],
        scoped=True,
    )


# keep_alive: every node is EXPECTED to exit at once. Without it the launch service shuts
# down as soon as they do, before the active test has finished, and launch_testing reports
# that as a failure.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    cases = [
        {"forward_preference": "1.5"},
        {"gap_switch_margin": "-0.1"},
        {"swept_path_lookahead_m": "0"},
        {"speed_time_constant_s": "6"},
        {"target_range_median_scans": "0"},
        {"centering_gain": "-0.1"},
        {"escape_probe_distance_m": "0"},
        {"lane_lookahead_m": "0"},
        {"lane_default_half_width_m": "5"},
        {"a_lat_max_mps2": "0"},
        {"apex_offset_m": "2"},
        {"track_out_offset_m": "-1"},
        {"stopping_margin_m": "3"},
        {"side_cap_distance_m": "5"},
        {"status_rate_hz": "50"},
    ]
    return launch.LaunchDescription(
        [
            *[_include(_FIRST_NODE_DOMAIN + i, **case) for i, case in enumerate(cases)],
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
        proc_output.assertWaitFor("escape_probe_distance_m", timeout=30, stream="stderr")
        proc_output.assertWaitFor("lane_lookahead_m", timeout=30, stream="stderr")
        proc_output.assertWaitFor("lane_default_half_width_m", timeout=30, stream="stderr")
        for name in (
            "a_lat_max_mps2",
            "apex_offset_m",
            "track_out_offset_m",
            "stopping_margin_m",
            "side_cap_distance_m",
            "status_rate_hz",
        ):
            proc_output.assertWaitFor(name, timeout=30, stream="stderr")
        # Let every process exit on their own before this test returns; otherwise launch
        # shuts them down with SIGINT and the exit code check below sees -2, not 1.
        processes = proc_info.processes()
        self.assertEqual(len(processes), 15)
        for process in processes:
            proc_info.assertWaitForShutdown(process=process, timeout=30)


@launch_testing.post_shutdown_test()
class TestGapFollowNodeParameterRangesShutdown(unittest.TestCase):
    def test_every_node_refused_to_start(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1])
