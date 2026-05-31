"""ROS 2 node wrapping the pinned f1tenth_gym env (roadmap task 0.5).

Bridges the gym simulator to the standard racer topics
(claude-docs/04-architecture.md):

  - publishes ``/scan`` (sensor_msgs/LaserScan, best_effort) from the gym's
    LiDAR model.
  - publishes ``/sim/ground_truth_odom`` (nav_msgs/Odometry, reliable) from
    gym ground truth -- NOT ``/odom``, which claude-docs/04-architecture.md
    reserves for the real EKF (racer_state).
  - subscribes to ``/drive`` (ackermann_msgs/AckermannDriveStamped,
    reliable) and steps the sim from the latest received command.
  - offers ``/sim/reset`` (std_srvs/Trigger) so tests and tooling can reset
    the episode deterministically.

Vehicle physical parameters (mass, wheelbase, friction, ...) are the
f1tenth_gym env's own defaults. ``config/vehicle_params.yaml`` (roadmap
task 0.7) does not exist yet; when it lands, this node's env ``params``
config should be built from it instead of the gym defaults, per CLAUDE.md
hard invariant 2 (one source of truth for physical constants). Track data
(``config/tracks/``, also task 0.7-adjacent) does not exist yet either, so
this node builds a synthetic, network-free reference-line track -- the
same approach ``docker/sim-cpu/smoke_test.py`` uses -- rather than a named
map, which would fetch from api.f1tenth.org on first use.
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_gym.envs.track import Track
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from racer_gym_bridge.conversions import build_odom_fields, build_scan_fields, drive_cmd_to_action

_GYM_ENV_ID = "f1tenth_gym:f1tenth-v0"
_EGO_AGENT_ID = "agent_0"
_OBSERVATION_FEATURES = [
    "scan",
    "pose_x",
    "pose_y",
    "pose_theta",
    "linear_vel_x",
    "linear_vel_y",
    "ang_vel_z",
]


def build_synthetic_track() -> Track:
    """A deterministic, network-free reference-line track.

    Identical construction to ``docker/sim-cpu/smoke_test.py``'s
    ``build_env()``: a named map (e.g. "Spielberg") would fetch from
    api.f1tenth.org on first use, which this bridge -- meant to run
    headlessly in CI and on a laptop with no network -- must not depend on.
    Real venue tracks land with ``config/tracks/`` (roadmap task 0.7 area).
    """
    xs = np.linspace(0, 50, 100)
    ys = np.sin(xs / 3.0) * 3.0
    velxs = np.full_like(xs, 3.0)
    return Track.from_refline(x=xs, y=ys, velx=velxs)


def build_env(seed: int) -> gym.Env:
    """Construct the pinned f1tenth_gym env: single ego agent, headless."""
    return gym.make(
        _GYM_ENV_ID,
        config={
            "seed": seed,
            "map": build_synthetic_track(),
            "num_agents": 1,
            "ego_idx": 0,
            "observation_config": {"type": "features", "features": _OBSERVATION_FEATURES},
        },
        render_mode=None,
    )


class BridgeNode(Node):
    """Steps a pinned f1tenth_gym env and bridges it to ROS topics."""

    def __init__(self) -> None:
        super().__init__("bridge_node")

        seed_descriptor = ParameterDescriptor(
            description="Seed used for env construction and every /sim/reset call.",
            integer_range=[IntegerRange(from_value=0, to_value=2**31 - 1, step=1)],
        )
        self._seed = int(self.declare_parameter("seed", 42, seed_descriptor).value)

        self.env = build_env(self._seed)

        scan_sim = self.env.unwrapped.sim.agents[0].scan_simulator
        self._fov_rad = float(scan_sim.fov)
        self._range_min = 0.0
        self._range_max = float(scan_sim.max_range)
        self._env_timestep = float(self.env.unwrapped.timestep)

        step_rate_descriptor = ParameterDescriptor(
            description=(
                "Sim step rate in Hz. 0.0 (default) means 'use the gym env's own "
                "physics timestep' (1 / env.timestep) rather than a second "
                "hand-typed number."
            ),
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1000.0, step=0.0)],
        )
        requested_rate = float(
            self.declare_parameter("step_rate_hz", 0.0, step_rate_descriptor).value
        )
        self._step_rate_hz = requested_rate if requested_rate > 0.0 else 1.0 / self._env_timestep

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._scan_pub = self.create_publisher(LaserScan, "/scan", scan_qos)
        self._odom_pub = self.create_publisher(Odometry, "/sim/ground_truth_odom", odom_qos)
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, "/drive", self._on_drive, drive_qos
        )
        self._reset_srv = self.create_service(Trigger, "/sim/reset", self._on_reset)

        self._latest_steering_angle = 0.0
        self._latest_speed = 0.0
        self._warned_terminated = False

        self._reset_env()

        self._timer = self.create_timer(1.0 / self._step_rate_hz, self._on_timer)

        self.get_logger().info(
            f"racer_gym_bridge up: stepping at {self._step_rate_hz:.1f} Hz "
            f"(env timestep {self._env_timestep:.4f} s), scan {scan_sim.num_beams} beams "
            f"over {self._fov_rad:.3f} rad fov, range_max {self._range_max:.1f} m."
        )

    # -- ROS callbacks ---------------------------------------------------

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        self._latest_steering_angle = msg.drive.steering_angle
        self._latest_speed = msg.drive.speed

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request  # std_srvs/Trigger takes no fields
        self._reset_env()
        response.success = True
        response.message = "sim reset"
        return response

    def _on_timer(self) -> None:
        action = drive_cmd_to_action(self._latest_steering_angle, self._latest_speed)
        obs, _reward, terminated, truncated, _info = self.env.step(action)
        if terminated or truncated:
            if not self._warned_terminated:
                self.get_logger().warn(
                    f"sim episode ended (terminated={terminated} truncated={truncated}); "
                    "continuing to step in place -- call /sim/reset to start a new episode."
                )
                self._warned_terminated = True
        else:
            self._warned_terminated = False
        self._publish(obs)

    # -- helpers -----------------------------------------------------------

    def _reset_env(self) -> None:
        obs, _info = self.env.reset(seed=self._seed)
        self._warned_terminated = False
        self._publish(obs)

    def _publish(self, obs: dict) -> None:
        agent_obs = obs[_EGO_AGENT_ID]
        now = self.get_clock().now().to_msg()

        scan_fields = build_scan_fields(
            ranges=agent_obs["scan"],
            fov_rad=self._fov_rad,
            range_min=self._range_min,
            range_max=self._range_max,
        )
        scan_msg = LaserScan()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = "laser"
        scan_msg.angle_min = scan_fields.angle_min
        scan_msg.angle_max = scan_fields.angle_max
        scan_msg.angle_increment = scan_fields.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self._step_rate_hz
        scan_msg.range_min = scan_fields.range_min
        scan_msg.range_max = scan_fields.range_max
        scan_msg.ranges = scan_fields.ranges
        self._scan_pub.publish(scan_msg)

        odom_fields = build_odom_fields(
            pose_x=agent_obs["pose_x"],
            pose_y=agent_obs["pose_y"],
            yaw_rad=agent_obs["pose_theta"],
            vx=agent_obs["linear_vel_x"],
            vy=agent_obs["linear_vel_y"],
            yaw_rate=agent_obs["ang_vel_z"],
        )
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        position = odom_msg.pose.pose.position
        position.x, position.y, position.z = odom_fields.position
        orientation = odom_msg.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = odom_fields.orientation
        linear = odom_msg.twist.twist.linear
        linear.x, linear.y, linear.z = odom_fields.linear
        angular = odom_msg.twist.twist.angular
        angular.x, angular.y, angular.z = odom_fields.angular
        self._odom_pub.publish(odom_msg)

    def destroy_node(self) -> bool:
        self.env.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
