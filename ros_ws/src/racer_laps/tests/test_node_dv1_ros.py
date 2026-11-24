"""lap_memory_node with the switches of brief 53 on a live ROS graph (L3): D v1 on the sim truth
source (`use_sim_time`, the harness publishes /clock), each lap node switch on (nominal: laps close
clean, the guidance carries stations and a bound; silence: a silent source gives the lap-1 cap
within 3.5 s), the reset service only with `reset_on_change`, and an unknown mode refused.

Reuses the harness of test_node_external_ros.py (the truth source reads the same two JSON topics).
Needs rclpy and the colcon-built messages; skipped where they are absent.
"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs.msg")
pytest.importorskip("racer_msgs.msg")
pytest.importorskip("racer_tools")

from rosgraph_msgs.msg import Clock
from test_node_external_ros import LAP_S, Harness, node_args

os.environ.setdefault("ROS_DOMAIN_ID", "97")

TRUTH_V1 = ("lap_source:=truth", "use_sim_time:=true", "lap_mode:=v1")
SWITCHES = {
    "defaults": (),
    "value_profile_freeze": ("value_profile_mode:=freeze_after_first_clean_lap",),
    "station_uncertainty": (
        "station_uncertainty:=true",
        "station_uncertainty_rate:=0.05",
        "station_uncertainty_cap_m:=0.5",
    ),
    "reset_on_change": ("reset_on_change:=true", "reset_length_share:=0.08"),
    "cap_wins": ("zone_floor_mode:=cap_wins",),
    "candidate_all": (
        "zone_floor_mode:=cap_wins",
        "station_uncertainty:=true",
        "reset_on_change:=true",
        "value_profile_mode:=freeze_after_first_clean_lap",
    ),
}


class SimHarness(Harness):
    """The external-source harness plus a /clock at wall time, so a use_sim_time node runs."""

    def __init__(self, tmp_path: Path, extra: tuple[str, ...]) -> None:
        super().__init__(tmp_path, extra=extra)
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if hasattr(self, "clock_pub"):
                self.clock_pub.publish(Clock(clock=self.node.get_clock().now().to_msg()))
            rclpy.spin_once(self.node, timeout_sec=0.005)


def _services(h: Harness) -> list[str]:
    return [name for name, _ in h.node.get_service_names_and_types()]


@pytest.mark.parametrize("name", list(SWITCHES))
def test_v1_on_truth_with_each_switch_nominal(tmp_path: Path, name: str) -> None:
    h = SimHarness(tmp_path, TRUTH_V1 + SWITCHES[name])
    try:
        h.wait_for_node()
        h.camera_status()
        h.spin(1.5)
        h.drive(4.0 * LAP_S + 1.0)
        h.spin(0.5)
        events = list(h.events)
        assert [e.lap_index for e in events][:3] == [1, 2, 3], h.close()
        assert all(e.clean and e.header.frame_id == "truth" for e in events[:3])
        with_stations = [g for _, g in h.guidance if len(g.s_m) > 0]
        assert with_stations, "D v1 published no station bound"
        g = with_stations[-1]
        assert len(g.s_m) == len(g.v_ref_mps) == len(g.cost_to_go_s)
        assert min(g.v_ref_mps) >= 0.0 and g.v_cap_mps > 0.5
        status = h.statuses[-1]
        assert status["lap_mode"] == "v1" and status["lap_source"] == "truth"
        assert ("resets" in status) is ("reset_on_change:=true" in SWITCHES[name])
        has_service = "/lap_memory_node/reset_memory" in _services(h)
        assert has_service is ("reset_on_change:=true" in SWITCHES[name])
        assert h.node.get_publishers_info_by_topic("/drive_raw") == []
    finally:
        if h.proc.poll() is None:
            h.close()


@pytest.mark.parametrize("name", ["defaults", "candidate_all"])
def test_v1_on_truth_silence_gives_the_lap_one_cap(tmp_path: Path, name: str) -> None:
    h = SimHarness(tmp_path, TRUTH_V1 + SWITCHES[name])
    try:
        h.wait_for_node()
        h.camera_status()
        h.spin(0.3)
        h.drive(2.0 * LAP_S + 0.5)
        assert h.guidance[-1][1].v_cap_mps > 0.55
        t_silent = time.monotonic()
        h.drive(4.0, status_every_s=None, events=False)
        late = [t for t, g in h.guidance if abs(g.v_cap_mps - 0.5) < 1e-9 and t > t_silent]
        assert late and late[0] - t_silent < 3.5
        quiet = [g for t, g in h.guidance if t > late[0] + 0.2]
        assert quiet and all(len(g.s_m) == 0 for g in quiet), (
            "no bound while the station is unknown"
        )
        h.spin(0.3)
        assert h.statuses[-1]["source_alive"] is False
    finally:
        if h.proc.poll() is None:
            h.close()


def test_the_reset_service_drops_the_laps(tmp_path: Path) -> None:
    from std_srvs.srv import Trigger

    h = SimHarness(tmp_path, (*TRUTH_V1, "reset_on_change:=true"))
    try:
        h.wait_for_node()
        h.camera_status()
        h.spin(0.3)
        h.drive(2.0 * LAP_S + 0.5)
        assert h.guidance[-1][1].v_cap_mps > 0.55
        client = h.node.create_client(Trigger, "/lap_memory_node/reset_memory")
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            h.drive(0.1, events=False)
        assert future.done() and future.result().success
        h.drive(1.5, events=False)
        assert abs(h.guidance[-1][1].v_cap_mps - 0.5) < 1e-9
        assert h.statuses[-1]["resets"][-1]["reason"] == "operator"
        assert h.statuses[-1]["safe_set_laps"] == 0
    finally:
        if h.proc.poll() is None:
            h.close()


@pytest.mark.parametrize("bad", ["value_profile_mode:=sometimes", "zone_floor_mode:=maybe"])
def test_an_unknown_mode_is_refused(tmp_path: Path, bad: str) -> None:
    proc = subprocess.run(
        node_args(tmp_path, *TRUTH_V1, bad),
        env=dict(os.environ),
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode != 0


def test_v1_with_the_camera_source_learns_and_falls_back_on_silence(tmp_path: Path) -> None:
    """Owner decision 29: D v1 on the car's camera lap source (changed expectation: refused
    before). Laps close on camera events, the guidance carries a station bound, and a silent
    camera gives the lap-1 cap with no bound."""
    h = Harness(tmp_path, extra=("lap_source:=camera", "lap_mode:=v1"))
    try:
        h.wait_for_node()
        h.camera_status()
        h.spin(1.5)
        h.drive(3.0 * LAP_S + 0.5)
        h.spin(0.3)
        assert [e.lap_index for e in h.events][:2] == [1, 2]
        assert all(e.clean and e.header.frame_id == "camera" for e in h.events[:2])
        assert [g for _, g in h.guidance if len(g.s_m) > 0], "no station bound"
        assert h.statuses[-1]["lap_mode"] == "v1"
        t_silent = time.monotonic()
        h.drive(4.0, status_every_s=None, events=False)
        late = [t for t, g in h.guidance if abs(g.v_cap_mps - 0.5) < 1e-9 and t > t_silent]
        assert late and late[0] - t_silent < 3.5
        assert all(len(g.s_m) == 0 for t, g in h.guidance if t > late[0] + 0.2)
    finally:
        if h.proc.poll() is None:
            h.close()
