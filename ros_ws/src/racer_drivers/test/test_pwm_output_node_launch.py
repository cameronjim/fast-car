"""L3 node tests for pwm_output_node (claude-docs/12-testing.md L3).

Checklist covered: neutral on BOTH channels before any /drive arrives; the correct mapped
pulses for a nominal /drive; neutral again when /drive goes silent past the timeout; correct
QoS (a best_effort /drive publisher is genuinely incompatible with the node's reliable
subscription, so the node keeps outputting neutral); neutral on shutdown, checked after the
process is gone; clean exit.

FAKE SYSFS. The node's `sysfs_root` parameter points at a temp directory laid out exactly
like /sys/class/pwm (pwmchip0/export, pwmchip0/pwm0/{period,duty_cycle,enable}, and pwm1 for
the throttle channel), so the real SysfsPwmChannel code path is exercised end to end -- the
same open/lseek/write/ftruncate sequence that runs on the car -- with ordinary files standing
in for the kernel's attributes. Nothing here needs root, a PWM chip, or a Jetson.

The pulse values asserted below are DERIVED from config/vehicle_params.yaml at test time
(see `_vehicle_params()`), not typed in: a test with its own copy of a physical constant is
exactly the duplication CLAUDE.md invariant 2 forbids, and it would silently stop testing the
real mapping the moment the calibration is bench-measured and changed.
"""

from __future__ import annotations

import os

# Pinned BEFORE rclpy is imported, same reasoning as racer_safety's and racer_control's
# launch tests: colcon runs different packages' launch_testing suites as concurrent
# processes in one DDS domain, and this file publishes /drive, which racer_safety's tests
# subscribe to. Distinct from every domain already claimed in this workspace: racer_control
# uses 77 and 79, racer_safety 78, racer_bringup 82.
os.environ.setdefault("ROS_DOMAIN_ID", "81")

import json
import pathlib
import shutil
import signal
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_OUTPUT_RATE_HZ = 50.0  # the servo frame rate; not a test knob (see the node's descriptor)
_DRIVE_TIMEOUT_S = 0.2
_PERIOD_NS = 20_000_000

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_PARAMS_PATH = _REPO_ROOT / "config" / "vehicle_params.yaml"


def _vehicle_params() -> dict:
    with open(_PARAMS_PATH, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


_PARAMS = _vehicle_params()
_STEERING_NEUTRAL_US = _PARAMS["steering"]["pwm_neutral_us"]
_STEERING_MAX_US = _PARAMS["steering"]["pwm_max_us"]
_STEERING_MIN_US = _PARAMS["steering"]["pwm_min_us"]
_STEERING_MAX_ANGLE_RAD = _PARAMS["steering"]["max_angle_rad"]
# steering.pwm_left_bound (CLAUDE.md invariant 2: the sign convention is a vehicle_params
# field now, not a node parameter) names which pulse end a LEFT (positive) angle goes to.
_STEERING_LEFT_US = (
    _STEERING_MAX_US if _PARAMS["steering"]["pwm_left_bound"] == "pwm_max_us" else _STEERING_MIN_US
)
_STEERING_RIGHT_US = _STEERING_MIN_US if _STEERING_LEFT_US == _STEERING_MAX_US else _STEERING_MAX_US
_THROTTLE_NEUTRAL_US = _PARAMS["actuation"]["throttle_pwm_neutral_us"]
_THROTTLE_MAX_US = _PARAMS["actuation"]["throttle_pwm_max_us"]
_SPEED_FULL_SCALE_MPS = _PARAMS["actuation"]["throttle_full_scale_mps"]

# One temp fake sysfs tree for the whole file; the launch description and the tests both need
# its path, and launch_testing gives no clean way to hand state from one to the other besides
# module scope. Cleaned up by the post-shutdown test.
_FAKE_SYSFS = tempfile.mkdtemp(prefix="racer_fake_sysfs_")


def _channel_dir(chip: int, channel: int) -> pathlib.Path:
    return pathlib.Path(_FAKE_SYSFS) / f"pwmchip{chip}" / f"pwm{channel}"


def _make_fake_sysfs() -> None:
    """Create the file layout SysfsPwmChannel expects, pre-exported (pwm<M>/ already
    present) so the node takes the 'already exported' branch, which is also what a second
    run on the real car hits."""
    chip = pathlib.Path(_FAKE_SYSFS) / "pwmchip0"
    chip.mkdir(parents=True, exist_ok=True)
    (chip / "export").write_text("", encoding="utf-8")
    (chip / "unexport").write_text("", encoding="utf-8")
    for channel in (0, 1):
        channel_dir = chip / f"pwm{channel}"
        channel_dir.mkdir(exist_ok=True)
        (channel_dir / "period").write_text("0", encoding="utf-8")
        (channel_dir / "duty_cycle").write_text("0", encoding="utf-8")
        (channel_dir / "enable").write_text("0", encoding="utf-8")


def _read_int(path: pathlib.Path) -> int:
    return int(path.read_text(encoding="utf-8").strip() or "0")


def _duty_ns(chip: int, channel: int) -> int:
    return _read_int(_channel_dir(chip, channel) / "duty_cycle")


def _enabled(chip: int, channel: int) -> int:
    return _read_int(_channel_dir(chip, channel) / "enable")


def _expected_duty_ns(pulse_us: float) -> int:
    return round(pulse_us * 1000.0)


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _make_drive(steering: float, speed: float) -> AckermannDriveStamped:
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = steering
    msg.drive.speed = speed
    return msg


@pytest.mark.launch_test
def generate_test_description():
    _make_fake_sysfs()
    pwm_output_node = LaunchNode(
        package="racer_drivers",
        executable="pwm_output_node",
        name="pwm_output_node",
        parameters=[
            {
                "output_rate_hz": _OUTPUT_RATE_HZ,
                "drive_timeout_s": _DRIVE_TIMEOUT_S,
                "sysfs_root": _FAKE_SYSFS,
                "steering_pwmchip": 0,
                "steering_pwm_channel": 0,
                "throttle_pwmchip": 0,
                "throttle_pwm_channel": 1,
                # steering_left_is_pwm_max is no longer a node parameter: the steering sign
                # now comes from config/vehicle_params.yaml's steering.pwm_left_bound through
                # the generated binding (CLAUDE.md invariant 2). See _STEERING_LEFT_US /
                # _STEERING_RIGHT_US above, derived from that same field.
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [
            pwm_output_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"fake_sysfs": _FAKE_SYSFS}


class TestPwmOutputNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        deadline = time.time() + 30.0
        while time.time() < deadline and _enabled(0, 0) != 1:
            time.sleep(0.1)
        if _enabled(0, 0) != 1:
            raise RuntimeError("pwm_output_node never enabled the steering channel")

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_pwm_output_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_steadily(self, publisher, msg, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_a_neutral_on_start_before_any_drive(self):
        """Runs first (alphabetical method order) because it is the only test that can
        observe the pre-first-/drive state: nothing in this file has published /drive yet."""
        self.assertEqual(_read_int(_channel_dir(0, 0) / "period"), _PERIOD_NS)
        self.assertEqual(_read_int(_channel_dir(0, 1) / "period"), _PERIOD_NS)
        self.assertEqual(_duty_ns(0, 0), _expected_duty_ns(_STEERING_NEUTRAL_US))
        self.assertEqual(_duty_ns(0, 1), _expected_duty_ns(_THROTTLE_NEUTRAL_US))
        self.assertEqual(_enabled(0, 0), 1)
        self.assertEqual(_enabled(0, 1), 1)

    def test_b_nominal_drive_maps_to_expected_pulses(self):
        publisher = self.node.create_publisher(AckermannDriveStamped, "/drive", _reliable_qos())
        self._spin_for(0.3)

        # Full left, half full-scale speed. Expected pulses are computed from
        # config/vehicle_params.yaml, mirroring the node's own linear map.
        steering = _STEERING_MAX_ANGLE_RAD
        speed = _SPEED_FULL_SCALE_MPS / 2.0
        self._publish_steadily(publisher, _make_drive(steering, speed), seconds=1.0)

        expected_steering_us = _STEERING_LEFT_US  # left end per steering.pwm_left_bound
        expected_throttle_us = _THROTTLE_NEUTRAL_US + 0.5 * (
            _THROTTLE_MAX_US - _THROTTLE_NEUTRAL_US
        )
        self.assertAlmostEqual(
            _duty_ns(0, 0) / 1000.0,
            expected_steering_us,
            delta=1.0,
            msg="steering pulse did not follow a nominal /drive",
        )
        self.assertAlmostEqual(
            _duty_ns(0, 1) / 1000.0,
            expected_throttle_us,
            delta=1.0,
            msg="throttle pulse did not follow a nominal /drive",
        )

    def test_c_right_steering_goes_to_the_other_pulse_end(self):
        publisher = self.node.create_publisher(AckermannDriveStamped, "/drive", _reliable_qos())
        self._spin_for(0.3)
        self._publish_steadily(
            publisher, _make_drive(_PARAMS["steering"]["min_angle_rad"], 0.0), seconds=1.0
        )
        self.assertAlmostEqual(_duty_ns(0, 0) / 1000.0, _STEERING_RIGHT_US, delta=1.0)
        self.assertAlmostEqual(_duty_ns(0, 1) / 1000.0, _THROTTLE_NEUTRAL_US, delta=1.0)

    def test_d_neutral_on_drive_silence(self):
        publisher = self.node.create_publisher(AckermannDriveStamped, "/drive", _reliable_qos())
        self._spin_for(0.3)
        self._publish_steadily(publisher, _make_drive(0.2, 2.0), seconds=0.8)
        self.assertNotEqual(
            _duty_ns(0, 1),
            _expected_duty_ns(_THROTTLE_NEUTRAL_US),
            "throttle was already neutral before the silence test began",
        )

        time.sleep(_DRIVE_TIMEOUT_S * 5.0)
        self.assertEqual(_duty_ns(0, 0), _expected_duty_ns(_STEERING_NEUTRAL_US))
        self.assertEqual(_duty_ns(0, 1), _expected_duty_ns(_THROTTLE_NEUTRAL_US))

    def test_e_drive_subscription_is_reliable_not_best_effort(self):
        """A best_effort /drive publisher must be QoS-incompatible with the node's
        subscription (claude-docs/10-conventions.md: the command path is `reliable`), so the
        node never sees these commands and stays neutral throughout."""
        publisher = self.node.create_publisher(AckermannDriveStamped, "/drive", _best_effort_qos())
        self._spin_for(0.3)
        self._publish_steadily(publisher, _make_drive(0.2, 3.0), seconds=1.0)
        self.assertEqual(_duty_ns(0, 0), _expected_duty_ns(_STEERING_NEUTRAL_US))
        self.assertEqual(_duty_ns(0, 1), _expected_duty_ns(_THROTTLE_NEUTRAL_US))

    def test_f_node_subscribes_to_drive_and_never_drive_raw(self):
        """CLAUDE.md invariant 1: a /drive_raw subscription here would bypass safety_node.
        Asserted structurally against the live graph, not by reading the source."""
        self._spin_for(0.5)
        subscriber_names = [
            name
            for name, _ in self.node.get_subscriber_names_and_types_by_node("pwm_output_node", "/")
        ]
        self.assertIn("/drive", subscriber_names)
        self.assertNotIn("/drive_raw", subscriber_names)


@launch_testing.post_shutdown_test()
class TestPwmOutputNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])

    def test_neutral_and_disabled_after_shutdown(self, fake_sysfs):
        """The node is gone by now: whatever is in these files is the last thing it wrote.
        Neutral on both channels, both disabled -- no stale non-neutral pulse survives a
        SIGINT."""
        root = pathlib.Path(fake_sysfs)
        for channel, neutral_us in ((0, _STEERING_NEUTRAL_US), (1, _THROTTLE_NEUTRAL_US)):
            duty = int((root / "pwmchip0" / f"pwm{channel}" / "duty_cycle").read_text().strip())
            enable = int((root / "pwmchip0" / f"pwm{channel}" / "enable").read_text().strip())
            self.assertEqual(
                duty,
                _expected_duty_ns(neutral_us),
                f"channel {channel} did not end at neutral: {json.dumps({'duty_ns': duty})}",
            )
            self.assertEqual(enable, 0, f"channel {channel} was left enabled after shutdown")
        shutil.rmtree(root, ignore_errors=True)
