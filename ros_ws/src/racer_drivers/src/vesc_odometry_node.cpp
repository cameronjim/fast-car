// vesc_odometry_node -- wheel odometry and VESC telemetry from the VESC's own state stream.
//
//   vesc_driver (f1tenth, pinned + read-only patched, docker/car/Dockerfile)
//       --/vesc/sensors/core (vesc_msgs/VescStateStamped)--> vesc_odometry_node
//       --> /odom/wheel               nav_msgs/Odometry (wheel speed + along-track distance)
//       --> /telemetry/vesc/*         std_msgs/Float32 per quantity, SI (+ raw ERPM)
//       --> /telemetry/vesc/fault     std_msgs/String
//
// READ-ONLY. This node has no publisher on any command topic and no path to the VESC: it only
// listens to what the driver already reports. The driver itself is patched so it neither
// subscribes to nor sends any command (docker/car/patches/vesc_driver-readonly.patch). The
// motor's only command path stays the PPM pulse through the layer-1 mux (CLAUDE.md invariant 1).
//
// CONSTANTS. drivetrain.pole_pairs, drivetrain.gear_ratio and tires.nominal_radius_m come from
// the GENERATED vehicle_params binding and nowhere else (CLAUDE.md invariant 2); the node
// refuses to start if any is null or not positive. The conversion itself is the ROS-free
// include/racer_drivers/vesc_odometry.hpp.
//
// /odom/wheel CONTENTS (frame ids are parameters, defaults shown):
//   header.frame_id "odom", child_frame_id "base_link", header.stamp = the driver's stamp.
//   twist.twist.linear.x  wheel surface speed, m/s, positive forward, in base_link. This is the
//                         field a fusing EKF (roadmap 2.4) should take; every other twist
//                         component is zero with a large variance.
//   pose.pose.position.x  SIGNED ALONG-TRACK DISTANCE since the node started, metres: forward
//                         adds, reverse subtracts. It is an arc length along whatever path the
//                         car drove, NOT an x coordinate in the odom frame (a car that drives a
//                         circle reads its circumference here). y, z are 0, orientation is
//                         identity, and the whole pose covariance is pose_variance (1e6 by
//                         default) so nothing that fuses this message can mistake it for a pose.
// No TF is broadcast: odom -> base_link belongs to the EKF (racer_state), not to this node.
//
// QoS. The subscription is best_effort, depth 10 (sensor data, claude-docs/10-conventions.md;
// it accepts the driver's reliable publisher). Publishers are reliable, depth 10, the same as
// rail_voltage_node's telemetry: a reliable publisher is compatible with both reliable and
// best_effort subscribers, so a future robot_localization subscription works either way.

#include <chrono>
#include <cstdint>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "racer_drivers/vesc_odometry.hpp"

namespace racer_drivers {

namespace vo = vesc_odometry;

namespace {

/// Interface topics (claude-docs/04-architecture.md).
constexpr const char* kOdomTopic = "/odom/wheel";
constexpr const char* kTelemetryPrefix = "/telemetry/vesc/";

/// Read the three drivetrain fields from the generated binding, refusing on null.
vo::DrivetrainConfig drivetrain_from_vehicle_params() {
  std::vector<std::string> missing;
  if (!VEHICLE_PARAMS.drivetrain.pole_pairs.has_value()) {
    missing.emplace_back("drivetrain.pole_pairs");
  }
  if (!VEHICLE_PARAMS.drivetrain.gear_ratio.has_value()) {
    missing.emplace_back("drivetrain.gear_ratio");
  }
  if (!VEHICLE_PARAMS.tires.nominal_radius_m.has_value()) {
    missing.emplace_back("tires.nominal_radius_m");
  }
  if (!missing.empty()) {
    std::string names;
    for (const auto& name : missing) {
      names += (names.empty() ? "" : ", ") + name;
    }
    throw std::runtime_error("config/vehicle_params.yaml has null " + names +
                             "; refusing to convert ERPM to wheel speed without it.");
  }
  vo::DrivetrainConfig config;
  config.pole_pairs = static_cast<double>(*VEHICLE_PARAMS.drivetrain.pole_pairs);
  config.gear_ratio = *VEHICLE_PARAMS.drivetrain.gear_ratio;
  config.wheel_radius_m = *VEHICLE_PARAMS.tires.nominal_radius_m;
  const std::optional<std::string> invalid = vo::validate(config);
  if (invalid.has_value()) {
    throw std::runtime_error("config/vehicle_params.yaml is present but unusable: " + *invalid);
  }
  return config;
}

}  // namespace

class VescOdometryNode : public rclcpp::Node {
 public:
  VescOdometryNode()
      : rclcpp::Node("vesc_odometry_node"),
        drivetrain_(drivetrain_from_vehicle_params()),
        integrator_(declare_double("max_integration_gap_s", 0.2, 0.001, 10.0,
                                   "Longest gap between two VESC samples that is still "
                                   "integrated, s. The driver polls at 50 Hz (0.02 s); a longer "
                                   "gap is NOT bridged, its distance is left out rather than "
                                   "guessed.")) {
    state_topic_ = declare_string("state_topic", "/vesc/sensors/core",
                                  "vesc_msgs/VescStateStamped topic published by vesc_driver.");
    odom_frame_id_ =
        declare_string("odom_frame_id", "odom", "header.frame_id of /odom/wheel (REP-105).");
    child_frame_id_ = declare_string("child_frame_id", "base_link",
                                     "child_frame_id of /odom/wheel; the twist is in this frame.");
    stale_timeout_s_ = declare_double("stale_timeout_s", 0.5, 0.01, 10.0,
                                      "No VescStateStamped for this long, s: /telemetry/vesc/"
                                      "fault reports STALE_NO_VESC_DATA.");
    twist_x_variance_ = declare_double("twist_linear_x_variance", 0.01, 1e-9, 1e9,
                                       "Variance of twist.linear.x, (m/s)^2. PROVISIONAL "
                                       "placeholder until roadmap 2.1/2.4 measure it.");
    pose_variance_ = declare_double("pose_variance", 1.0e6, 1.0, 1e12,
                                    "Variance on every pose component (and every twist "
                                    "component but linear.x). Large on purpose: the pose "
                                    "carries along-track distance, not a pose.");

    const rclcpp::QoS sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, pub_qos);
    erpm_pub_ = telemetry("erpm", pub_qos);
    voltage_pub_ = telemetry("voltage_v", pub_qos);
    current_motor_pub_ = telemetry("current_motor_a", pub_qos);
    current_input_pub_ = telemetry("current_input_a", pub_qos);
    temp_fet_pub_ = telemetry("temp_fet_degc", pub_qos);
    temp_motor_pub_ = telemetry("temp_motor_degc", pub_qos);
    fault_pub_ =
        create_publisher<std_msgs::msg::String>(std::string(kTelemetryPrefix) + "fault", pub_qos);

    state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
        state_topic_, sub_qos,
        [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) { on_state(*msg); });

    // Steady clock, not the ROS clock: "did anything arrive recently" is a wall-time question.
    last_arrival_ = std::chrono::steady_clock::now();
    stale_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { check_stale(); });

    RCLCPP_INFO(get_logger(),
                "vesc_odometry_node: %s -> %s, %.6g m/s per ERPM (pole_pairs %.0f, gear_ratio "
                "%.4g, nominal_radius_m %.4g from config/vehicle_params.yaml, schema %s). "
                "Read-only: no command topic is published.",
                state_topic_.c_str(), kOdomTopic, vo::mps_per_erpm(drivetrain_),
                drivetrain_.pole_pairs, drivetrain_.gear_ratio, drivetrain_.wheel_radius_m,
                VEHICLE_PARAMS.meta.schema_version.c_str());
  }

 private:
  double declare_double(const std::string& name, double default_value, double low, double high,
                        const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = low;
    range.to_value = high;
    descriptor.floating_point_range.push_back(range);
    return declare_parameter<double>(name, default_value, descriptor);
  }

  std::string declare_string(const std::string& name, const std::string& default_value,
                             const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return declare_parameter<std::string>(name, default_value, descriptor);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr telemetry(const std::string& leaf,
                                                                 const rclcpp::QoS& qos) {
    return create_publisher<std_msgs::msg::Float32>(std::string(kTelemetryPrefix) + leaf, qos);
  }

  static void publish_float(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub,
                            double value) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    pub->publish(msg);
  }

  void publish_fault(const std::string& text) {
    std_msgs::msg::String msg;
    msg.data = text;
    fault_pub_->publish(msg);
  }

  void on_state(const vesc_msgs::msg::VescStateStamped& msg) {
    last_arrival_ = std::chrono::steady_clock::now();
    received_any_ = true;
    const auto& state = msg.state;

    // Telemetry first, unconditionally: a bad stamp must not hide a voltage or a fault.
    publish_float(erpm_pub_, state.speed);
    publish_float(voltage_pub_, state.voltage_input);
    publish_float(current_motor_pub_, state.current_motor);
    publish_float(current_input_pub_, state.current_input);
    publish_float(temp_fet_pub_, state.temp_fet);
    publish_float(temp_motor_pub_, state.temp_motor);
    publish_fault(vo::fault_name(state.fault_code));

    const double speed_mps = vo::erpm_to_wheel_speed_mps(state.speed, drivetrain_);
    const double stamp_s = rclcpp::Time(msg.header.stamp).seconds();
    const vo::AlongTrackIntegrator::Result result = integrator_.add(stamp_s, speed_mps);
    switch (result) {
      case vo::AlongTrackIntegrator::Result::kRejectedNonFinite:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "non-finite ERPM or stamp from the VESC; sample dropped");
        return;
      case vo::AlongTrackIntegrator::Result::kRejectedNonMonotonic:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "VESC sample stamp did not move forward; sample dropped");
        return;
      case vo::AlongTrackIntegrator::Result::kRestartedAfterGap:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "gap longer than max_integration_gap_s (%.3f s) in the VESC "
                             "stream; distance over the gap NOT added",
                             integrator_.max_gap_s());
        break;
      case vo::AlongTrackIntegrator::Result::kFirstSample:
      case vo::AlongTrackIntegrator::Result::kIntegrated:
        break;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = integrator_.distance_m();
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = speed_mps;
    for (std::size_t i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 6 + i] = pose_variance_;
      odom.twist.covariance[i * 6 + i] = pose_variance_;
    }
    odom.twist.covariance[0] = twist_x_variance_;
    odom_pub_->publish(odom);
  }

  void check_stale() {
    const double silent_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_arrival_).count();
    if (!received_any_ || silent_s >= stale_timeout_s_) {
      publish_fault(vo::kFaultStale);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no VescStateStamped on %s for %.2f s (vesc_driver down, UART not "
                           "wired, or UART app not enabled in VESC Tool?)",
                           state_topic_.c_str(), silent_s);
    }
  }

  vo::DrivetrainConfig drivetrain_;
  vo::AlongTrackIntegrator integrator_;
  std::string state_topic_;
  std::string odom_frame_id_;
  std::string child_frame_id_;
  double stale_timeout_s_ = 0.0;
  double twist_x_variance_ = 0.0;
  double pose_variance_ = 0.0;
  bool received_any_ = false;
  std::chrono::steady_clock::time_point last_arrival_;

  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr erpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_input_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_fet_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
};

}  // namespace racer_drivers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<racer_drivers::VescOdometryNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("vesc_odometry_node"), "refusing to start: %s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
