"""rail_voltage_node: publishes the Jetson's onboard INA3221 rail readings as SI topics.

WHY THIS NODE EXISTS. CLAUDE.md invariant 5 -- "Every run is logged (rosbag + rail voltage).
Code paths that drive the car without logging are bugs" -- and claude-docs/05-safety.md's
operational rule "Rail voltage logged on every run; a brownout must never be mistakable for a
control failure". Before this node the second half of that invariant had no publisher at all,
so `ros2 bag record` had nothing to capture (docs/notes/car-runtime-plan.md, "How rosbag
recording starts with the stack"). The INA226 on the compute rail that claude-docs/11-
hardware.md specs is NOT fitted; this reads the carrier board's own INA3221 instead, which
covers the same failure mode (a sag on VDD_IN during a throttle transient looks exactly like
a control fault in a bag that has no voltage in it).

WHAT IT PUBLISHES (all std_msgs/Float32, SI per CLAUDE.md invariant 4 -- volts and amps, the
millivolt/milliamp conversion done at the driver boundary in rail_voltage.py):

    /telemetry/rail_voltage_v            the PRIMARY rail (`primary_rail_label`, default
    /telemetry/rail_current_a            VDD_IN) under a fixed, device-independent name
    /telemetry/rail/<slug>/voltage_v     every channel the chip reports, by board label
    /telemetry/rail/<slug>/current_a     (vdd_in, vdd_cpu_gpu_cv, vdd_soc on an Orin Nano)

The fixed pair exists because the per-label topics are a property of the board, not of this
project: analysis and the bench procedures need one name they can rely on across a board
change, and CLAUDE.md invariant 5 names exactly one quantity ("rail voltage"). The per-label
topics exist because a brownout post-mortem wants to see WHICH rail sagged.

std_msgs/Float32 rather than a custom racer_msgs type, per claude-docs/02-repo-layout.md
("racer_msgs/ -- custom messages ONLY if std/ackermann msgs won't do"): a scalar with fixed
units in a named topic is exactly what Float32 is for. The cost is no per-sample stamp; the
bag's own receive timestamps carry the timing, which is adequate at 5 Hz for a brownout
correlation and does not justify a new message contract. Revisit when the INA226 lands and
sample-level sync with the ingest board's clock matters (roadmap 2.2).

NOT FITTED IS NOT A CRASH. If the sysfs tree is absent (any non-Jetson host: the ros-dev
container on the Mac, CI, the sim) the node warns EXACTLY ONCE, publishes nothing, and stays
alive. It must never take the launch down: this is a layer-4-adjacent logging driver and
claude-docs/05-safety.md's layering means logging may not weaken layers 1-3. The launch file
handles the OTHER direction -- a run that is not being RECORDED is a bug and stops the stack
(see car_teleop.launch.py) -- but a rail sensor that is not fitted is a known, documented
hardware state, not a logging failure.

The parsing/discovery half is ROS-free in racer_drivers/rail_voltage.py and unit tested
there (claude-docs/12-testing.md L1); this file is thin rclpy plumbing, the same split as
racer_tools' keyboard_teleop_node.py/keymap.py.
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

from racer_drivers import rail_voltage

PRIMARY_VOLTAGE_TOPIC = "/telemetry/rail_voltage_v"
PRIMARY_CURRENT_TOPIC = "/telemetry/rail_current_a"
CHANNEL_TOPIC_PREFIX = "/telemetry/rail"


class RailVoltageNode(Node):
    def __init__(self) -> None:
        super().__init__("rail_voltage_node")

        self.declare_parameter(
            "ina3221_root",
            rail_voltage.DEFAULT_INA3221_ROOT,
            ParameterDescriptor(
                description=(
                    "Root of the kernel's ina3221 driver sysfs tree. Only ever changed by "
                    "tests, which point it at a fixture tree of ordinary files (same pattern "
                    "as pwm_output_node's sysfs_root). On the car this stays "
                    f"{rail_voltage.DEFAULT_INA3221_ROOT}."
                )
            ),
        )
        self.declare_parameter(
            "publish_rate_hz",
            5.0,
            ParameterDescriptor(
                description=(
                    "Sampling and publish rate. 5 Hz is node tuning, not a physical constant, "
                    "so it is NOT a vehicle_params field (CLAUDE.md invariant 2): it is fast "
                    "enough to catch a brownout against 50 Hz command data and slow enough "
                    "that the i2c reads behind these sysfs files cost nothing."
                ),
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=100.0)],
            ),
        )
        self.declare_parameter(
            "primary_rail_label",
            "VDD_IN",
            ParameterDescriptor(
                description=(
                    "Board label of the channel republished on the fixed "
                    f"{PRIMARY_VOLTAGE_TOPIC} / {PRIMARY_CURRENT_TOPIC} topics. VDD_IN is the "
                    "Orin Nano carrier's total input rail -- the one a brownout shows up on."
                )
            ),
        )

        root = self.get_parameter("ina3221_root").get_parameter_value().string_value
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._primary_label = (
            self.get_parameter("primary_rail_label").get_parameter_value().string_value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._hwmon_dir = rail_voltage.find_hwmon_dir(root)
        self._channels: list[rail_voltage.RailChannel] = []
        # NOT `self._publishers`: rclpy.node.Node already owns an attribute of that name
        # (the list it appends every created publisher to), and shadowing it makes
        # create_publisher fail on the first call.
        self._channel_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._warned_absent = False
        self._warned_primary_missing = False

        if self._hwmon_dir is None:
            self._warn_absent_once(
                f"No INA3221 hwmon directory under '{root}'. Rail voltage will NOT be "
                "published or recorded on this host. That is expected off the Jetson (ros-dev "
                "container, CI, sim) and is a HARDWARE GAP on the car: CLAUDE.md invariant 5 "
                "wants rosbag AND rail voltage on every run."
            )
            return

        self._channels = rail_voltage.discover_channels(self._hwmon_dir)
        if not self._channels:
            self._warn_absent_once(
                f"INA3221 hwmon directory '{self._hwmon_dir}' exposes no labelled channels. "
                "Rail voltage will NOT be published or recorded on this host."
            )
            return

        for channel in self._channels:
            base = f"{CHANNEL_TOPIC_PREFIX}/{channel.slug}"
            self._channel_pubs[f"{base}/voltage_v"] = self.create_publisher(
                Float32, f"{base}/voltage_v", qos
            )
            self._channel_pubs[f"{base}/current_a"] = self.create_publisher(
                Float32, f"{base}/current_a", qos
            )
        self._primary_voltage_pub = self.create_publisher(Float32, PRIMARY_VOLTAGE_TOPIC, qos)
        self._primary_current_pub = self.create_publisher(Float32, PRIMARY_CURRENT_TOPIC, qos)

        self.get_logger().info(
            f"INA3221 at '{self._hwmon_dir}': channels "
            + ", ".join(f"{channel.index}:{channel.label}" for channel in self._channels)
            + f". Publishing at {rate_hz:g} Hz; primary rail '{self._primary_label}'."
        )
        self.create_timer(1.0 / rate_hz, self._sample)

    def _warn_absent_once(self, message: str) -> None:
        if not self._warned_absent:
            self._warned_absent = True
            self.get_logger().warning(message)

    def _sample(self) -> None:
        # A logging driver may never kill the process it shares with the command path
        # (claude-docs/05-safety.md: layer 4 must not weaken layers 1-3), so every sample is
        # wrapped. Individual unreadable files already come back as None from rail_voltage.
        try:
            readings = rail_voltage.read_all(self._hwmon_dir, self._channels)
        except Exception as error:  # noqa: BLE001 - deliberate, see the comment above
            self.get_logger().error(f"INA3221 read failed, skipping this sample: {error!r}")
            return

        primary_seen = False
        for reading in readings:
            base = f"{CHANNEL_TOPIC_PREFIX}/{reading.channel.slug}"
            if reading.volts is not None:
                self._channel_pubs[f"{base}/voltage_v"].publish(Float32(data=float(reading.volts)))
            if reading.amps is not None:
                self._channel_pubs[f"{base}/current_a"].publish(Float32(data=float(reading.amps)))
            if reading.channel.label != self._primary_label:
                continue
            primary_seen = True
            if reading.volts is not None:
                self._primary_voltage_pub.publish(Float32(data=float(reading.volts)))
            if reading.amps is not None:
                self._primary_current_pub.publish(Float32(data=float(reading.amps)))

        if not primary_seen and not self._warned_primary_missing:
            self._warned_primary_missing = True
            self.get_logger().warning(
                f"No INA3221 channel is labelled '{self._primary_label}', so "
                f"{PRIMARY_VOLTAGE_TOPIC} stays empty. Labels present: "
                + ", ".join(channel.label for channel in self._channels)
                + ". Set the primary_rail_label parameter to one of them."
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RailVoltageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
