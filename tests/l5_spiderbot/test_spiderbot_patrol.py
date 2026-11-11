"""L5 sim-in-loop: the spiderbot patrols, stops for a person, faces them, alerts, resumes.

The full command path is real: racer_gym_bridge (the car), racer_safety's safety_node (the ONLY
publisher of /drive), racer_control's tracker_node following a patrol loop generated from
waypoints by spiderbot.launch.py, and racer_apps' spiderbot_node publishing /drive_raw. The
sim has no camera, so racer_apps' fake_detector_node stands a synthetic person with raised arms
beside the track and publishes /perception/objects the way fusion_node would, with the bearing
computed from the car's real pose: as the spiderbot steers towards the person the bearing goes
to zero for real.

Pose source: the bridge's ground truth, the same sim-only adapter sim_autopilot.launch.py uses
(`odom_topic:=/sim/ground_truth_odom pose_frame:=`). Localisation on a map is covered by
tests/l5_localization for the tracker; this test is about the behaviour. On the car both stay at
their defaults (AMCL map frame).

What is asserted, in order, each with its own failure message:
  1. the car patrols at speed with no person in view (and nothing publishes /drive but
     safety_node, nothing publishes /drive_raw but spiderbot_node);
  2. a person with raised arms comes into view and, after the rule has held, the car stops;
  3. /spiderbot/state goes patrol -> stopping -> facing -> hold -> patrol;
  4. one alert, with the person's bearing and range, facing_ok true and a final bearing inside
     the tolerance, raised where the car really was (map pose within range of the person);
  5. the car stays stopped for the hold, then drives off again;
  6. the cooldown prevents a second alert.
Timings are wall-clock (the bridge steps in real time) so they are generous.
"""

from __future__ import annotations

import json
import math
import signal
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from racer_msgs.msg import SpiderbotAlert
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RACELINE = _REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
_OUT_DIR = tempfile.mkdtemp(prefix="l5_spiderbot_")
_PATROL_FILE = Path(_OUT_DIR) / "oval_patrol.yaml"

_PATROL_SPEED_MPS = 0.8
_HOLD_S = 4.0
_COOLDOWN_S = 40.0
_FACE_TOLERANCE_RAD = 0.12
# A person with raised arms 1.5 m inside the bottom straight, 0.5 m past the start.
_PERSON_XY = (0.5, -1.5)


def _oval_waypoints(spacing_m: float = 3.0) -> list[list[float]]:
    """Every ~3 m of the committed gym_oval raceline as a patrol waypoint."""
    rows = []
    for line in _RACELINE.read_text().splitlines():
        if not line.strip() or line.startswith(("#", "s_m")):
            continue
        f = line.split(",")
        rows.append((float(f[0]), float(f[1]), float(f[2])))
    waypoints, next_s = [], 0.0
    for s, x, y in rows:
        if s >= next_s:
            waypoints.append([round(x, 3), round(y, 3)])
            next_s = s + spacing_m
    return waypoints


def _reliable() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def generate_test_description():
    _PATROL_FILE.write_text(
        yaml.safe_dump(
            {"name": "oval", "frame": "map", "loop": True, "waypoints": _oval_waypoints()}
        )
    )
    bridge = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[{"seed": 11, "raceline_path": str(_RACELINE)}],
    )
    # The sim's /scan is aligned to the vehicle, not mounted like the C1 (sim_autopilot does the same).
    safety = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[{"laser_yaw_from_vehicle_params": False}],
    )
    spiderbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("racer_apps")) / "launch" / "spiderbot.launch.py")
        ),
        launch_arguments={
            "patrol": str(_PATROL_FILE),
            "speed_mps": str(_PATROL_SPEED_MPS),
            "odom_topic": "/sim/ground_truth_odom",
            "pose_frame": "",
            "speed_topic": "/sim/ground_truth_odom",
            "rule": "arms_raised",
            "rule_frames": "3",
            "hold_s": str(_HOLD_S),
            "cooldown_s": str(_COOLDOWN_S),
            "output_dir": _OUT_DIR,
        }.items(),
    )
    fake = LaunchNode(
        package="racer_apps",
        executable="fake_detector_node",
        name="fake_detector_node",
        parameters=[
            {
                "pose_topic": "/sim/ground_truth_odom",
                "people": json.dumps(
                    [{"x": _PERSON_XY[0], "y": _PERSON_XY[1], "pose": "arms_raised"}]
                ),
            }
        ],
    )
    return launch.LaunchDescription(
        [bridge, safety, spiderbot, fake, launch_testing.actions.ReadyToTest()]
    )


class TestSpiderbotPatrol(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("l5_spiderbot_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_patrol_stop_face_alert_hold_resume(self):
        node = self.node
        speeds: list[tuple[float, float]] = []  # (wall time, ground-truth speed)
        poses: list[tuple[float, float]] = []
        states: list[tuple[float, str]] = []
        alerts: list[SpiderbotAlert] = []
        node.create_subscription(
            Odometry,
            "/sim/ground_truth_odom",
            lambda m: (
                speeds.append(
                    (time.monotonic(), math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
                ),
                poses.append((m.pose.pose.position.x, m.pose.pose.position.y)),
            ),
            _reliable(),
        )
        node.create_subscription(
            String,
            "/spiderbot/state",
            lambda m: states.append((time.monotonic(), m.data)),
            _reliable(),
        )
        node.create_subscription(SpiderbotAlert, "/spiderbot/alert", alerts.append, _reliable())

        reset = node.create_client(Trigger, "/sim/reset")
        self.assertTrue(reset.wait_for_service(timeout_sec=30.0), "/sim/reset not available")
        future = reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), "/sim/reset did not complete")

        # Command-path wiring: /drive only from safety_node, /drive_raw only from spiderbot_node.
        self._spin(2.0)
        drive_pubs = {i.node_name for i in node.get_publishers_info_by_topic("/drive")}
        raw_pubs = {i.node_name for i in node.get_publishers_info_by_topic("/drive_raw")}
        self.assertEqual(drive_pubs, {"safety_node"}, f"/drive publishers: {drive_pubs}")
        self.assertEqual(raw_pubs, {"spiderbot_node"}, f"/drive_raw publishers: {raw_pubs}")

        # 2. The car is patrolling (the person is not in view yet near the start, or is only
        # just coming into view): wait for it to be at speed or for the alert, whichever first.
        self.assertTrue(
            self._spin_until(lambda: any(v > 0.4 for _, v in speeds) or alerts, 60.0),
            "the car never got up to patrol speed",
        )
        self.assertTrue(
            self._spin_until(lambda: len(alerts) >= 1, 120.0),
            f"no alert within 120 s; states seen: {sorted({s for _, s in states})}, "
            f"last speed {speeds[-1][1] if speeds else None}",
        )
        alert_wall = time.monotonic()
        alert = alerts[0]

        # 3. The state sequence (the state topic is 10 Hz: wait for hold to be published).
        self.assertTrue(
            self._spin_until(lambda: any(s == "hold" for _, s in states), 5.0), "no hold state"
        )
        sequence = [s for i, (_, s) in enumerate(states) if i == 0 or s != states[i - 1][1]]
        for needed in ("stopping", "facing", "hold"):
            self.assertIn(needed, sequence, f"state sequence was {sequence}")
        self.assertLess(sequence.index("stopping"), sequence.index("hold"), sequence)

        # 4. The alert's content.
        self.assertEqual(alert.rule, "arms_raised")
        self.assertGreaterEqual(alert.held_frames, 3)
        self.assertTrue(alert.range_valid)
        self.assertTrue(1.0 < alert.range_m <= 6.0, f"range {alert.range_m}")
        self.assertTrue(alert.facing_ok, f"final bearing {alert.final_bearing_rad}")
        self.assertLessEqual(abs(alert.final_bearing_rad), _FACE_TOLERANCE_RAD + 1e-6)
        self.assertTrue(alert.pose_valid, "no map pose on the alert (bridge TF missing?)")
        distance = math.hypot(alert.pose_x_m - _PERSON_XY[0], alert.pose_y_m - _PERSON_XY[1])
        self.assertLess(distance, 6.0)
        self.assertGreater(distance, 0.9, "the car crept to within a metre of the person")
        events = [
            json.loads(line)
            for line in (Path(_OUT_DIR) / "spiderbot_events.jsonl").read_text().splitlines()
        ]
        engagement = [e["kind"] for e in events if e["kind"] in ("engage", "alert")]
        self.assertEqual(engagement[:2], ["engage", "alert"], f"event log: {events}")

        # 5. Stopped for the hold, then moving again.
        self.assertTrue(
            self._spin_until(lambda: speeds and speeds[-1][1] < 0.05, 10.0),
            "the car did not stand still after the alert",
        )
        stopped_at = time.monotonic()
        self._spin(_HOLD_S * 0.5)
        hold_speeds = [v for t, v in speeds if stopped_at <= t]
        self.assertTrue(
            hold_speeds and max(hold_speeds) < 0.05, f"moved during the hold {max(hold_speeds)}"
        )
        self.assertTrue(
            self._spin_until(lambda: states and states[-1][1] == "patrol", _HOLD_S + 15.0),
            f"never returned to patrol; last states {[s for _, s in states[-5:]]}",
        )
        resumed_from = time.monotonic()
        self.assertTrue(
            self._spin_until(lambda: any(v > 0.4 for t, v in speeds if t >= resumed_from), 30.0),
            "the car did not drive off after the hold",
        )
        self.assertGreater(time.monotonic() - alert_wall, _HOLD_S * 0.9)

        # 6. Cooldown: the person is still beside the track and still has their arms up.
        self._spin(8.0)
        self.assertEqual(len(alerts), 1, "re-triggered during the cooldown")


@launch_testing.post_shutdown_test()
class TestSpiderbotPatrolShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
