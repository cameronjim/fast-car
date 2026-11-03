// wheel_twist_node: /odom/wheel speed + modelled steering -> body twist for the EKF.
//
//   /odom/wheel (nav_msgs/Odometry, best_effort sub, depth 10): twist.linear.x is the VESC's
//       wheel speed (racer_drivers/vesc_odometry_node, or racer_gym_bridge in the sim). Its
//       pose is an along-track distance, NOT a pose, and is ignored.
//   /drive (ackermann_msgs/AckermannDriveStamped, reliable, depth 10): the steering angle
//       safety_node let through, which is what the servo is actually being told. Read only;
//       this node is not on the command path and publishes nothing there.
//   -> /odom/wheel_twist (geometry_msgs/TwistWithCovarianceStamped, reliable, depth 10),
//       frame base_link, one per /odom/wheel sample, stamped with that sample's stamp. The
//       robot_localization ekf_node fuses it as twist0 (racer_state/config/ekf.yaml).
//
// The maths and the staleness rules live in racer_state::WheelTwistEstimator
// (include/racer_state/wheel_twist.hpp), which is unit tested without ROS. Physical constants
// (wheelbase, steering limits and rate, servo time constant) come only from the generated
// vehicle_params binding (CLAUDE.md invariant 2); the noise figures are tuning parameters.
//
// Silence. /odom/wheel silent: nothing is published, and the EKF's covariance grows on its own
// (the honest answer: no speed measurement, no twist). /drive silent for drive_timeout_s:
// twists keep coming, with the steering variance raised to stale_steering_std_rad, and one
// WARN on the transition (and one INFO when it comes back).
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_state/servo_model.hpp"
#include "racer_state/wheel_twist.hpp"

namespace racer_state {

namespace {

// Unused axes of the 6x6 twist covariance (z, roll rate, pitch rate). robot_localization
// ignores them in two_d_mode; a large value rather than zero keeps any other consumer honest.
constexpr double kUnusedAxisVariance = 1e6;

}  // namespace

class WheelTwistNode : public rclcpp::Node {
 public:
  WheelTwistNode() : Node("wheel_twist_node"), estimator_(build_estimator()) {
    const std::string wheel_topic =
        declare_string("wheel_odom_topic", "/odom/wheel", "Wheel speed input (Odometry).");
    const std::string drive_topic =
        declare_string("drive_topic", "/drive", "Steering command input (safety_node output).");
    const std::string twist_topic =
        declare_string("twist_topic", "/odom/wheel_twist", "Twist output for the EKF.");
    frame_id_ = declare_string("frame_id", "base_link",
                               "header.frame_id of the twist (REP-105 body frame).");

    const rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic, command_qos);
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_topic, sensor_qos, std::bind(&WheelTwistNode::on_wheel, this, std::placeholders::_1));
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, command_qos,
        std::bind(&WheelTwistNode::on_drive, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "wheel_twist_node up: %s + %s -> %s, wheelbase %.4f m, steering rate %.2f rad/s, "
                "servo time constant %s",
                wheel_topic.c_str(), drive_topic.c_str(), twist_topic.c_str(),
                VEHICLE_PARAMS.chassis.wheelbase_m, VEHICLE_PARAMS.steering.max_rate_rad_per_s,
                VEHICLE_PARAMS.steering.time_constant_s.has_value()
                    ? std::to_string(*VEHICLE_PARAMS.steering.time_constant_s).c_str()
                    : "not measured (rate limit only)");
  }

 private:
  std::string declare_string(const std::string& name, const std::string& default_value,
                             const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return declare_parameter<std::string>(name, default_value, descriptor);
  }

  double declare_double(const std::string& name, double default_value, double low, double high,
                        const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = low;
    range.to_value = high;
    descriptor.floating_point_range = {range};
    return declare_parameter<double>(name, default_value, descriptor);
  }

  WheelTwistEstimator build_estimator() {
    ServoModelConfig servo;
    servo.min_angle_rad = VEHICLE_PARAMS.steering.min_angle_rad;
    servo.max_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    servo.min_rate_rad_per_s = VEHICLE_PARAMS.steering.min_rate_rad_per_s;
    servo.max_rate_rad_per_s = VEHICLE_PARAMS.steering.max_rate_rad_per_s;
    servo.time_constant_s = VEHICLE_PARAMS.steering.time_constant_s;

    WheelTwistConfig twist;
    twist.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
    twist.speed_std_floor_mps =
        declare_double("speed_std_floor_mps", twist.speed_std_floor_mps, 0.0, 10.0,
                       "Wheel speed std at standstill (m/s): quantisation and ERPM noise.");
    twist.speed_std_fraction =
        declare_double("speed_std_fraction", twist.speed_std_fraction, 0.0, 1.0,
                       "Wheel speed std as a fraction of |speed|: tyre radius error, wheelspin.");
    twist.steering_std_rad = declare_double(
        "steering_std_rad", twist.steering_std_rad, 0.0, 1.0,
        "Std of the servo model's steering angle while /drive is fresh (rad): trim, linkage "
        "play, the unmeasured servo lag.");
    twist.stale_steering_std_rad = declare_double(
        "stale_steering_std_rad", VEHICLE_PARAMS.steering.max_angle_rad, 0.0, 2.0,
        "Steering std once /drive is stale (rad). Default the steering lock: the wheels could "
        "be anywhere.");
    twist.yaw_rate_std_floor_rad_per_s = declare_double(
        "yaw_rate_std_floor_rad_per_s", twist.yaw_rate_std_floor_rad_per_s, 0.0, 10.0,
        "Yaw rate std floor (rad/s): tyre slip and the bicycle model's own error.");
    twist.lateral_std_mps = declare_double(
        "lateral_std_mps", twist.lateral_std_mps, 1e-6, 10.0,
        "Std of the vy = 0 pseudo-measurement at the rear axle (m/s): rear tyre slip.");
    const double drive_timeout_s =
        declare_double("drive_timeout_s", 0.25, 1e-3, 10.0,
                       "/drive older than this (s, monotonic clock) counts as stale. safety_node "
                       "publishes at 50 Hz, so this is about 12 missed cycles.");
    return WheelTwistEstimator(servo, twist, drive_timeout_s);
  }

  double steady_seconds() { return steady_clock_.now().seconds(); }

  void on_drive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    estimator_.on_drive(msg->drive.steering_angle, steady_seconds());
  }

  void on_wheel(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // A sample still being handled after SIGINT shut the context down. This node segfaulted once
    // in CI at launch_testing teardown (2026-11-03, about 0.5 s after SIGINT, when the /drive
    // staleness WARN would fire as the other nodes stop); the likely cause, not proven, is
    // publishing or logging on the shut-down context. Nothing useful can be sent now anyway.
    if (!rclcpp::ok()) {
      return;
    }
    const double steady_s = steady_seconds();
    rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    const auto twist =
        estimator_.on_wheel_speed(msg->twist.twist.linear.x, stamp.seconds(), steady_s);
    report_steering_staleness(steady_s);
    if (!twist.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "wheel_twist_node: dropping a non-finite /odom/wheel sample");
      return;
    }
    geometry_msgs::msg::TwistWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;
    out.twist.twist.linear.x = twist->vx_mps;
    out.twist.twist.linear.y = twist->vy_mps;
    out.twist.twist.angular.z = twist->wz_rad_per_s;
    out.twist.covariance[0] = twist->var_vx;
    out.twist.covariance[7] = twist->var_vy;
    out.twist.covariance[14] = kUnusedAxisVariance;
    out.twist.covariance[21] = kUnusedAxisVariance;
    out.twist.covariance[28] = kUnusedAxisVariance;
    out.twist.covariance[35] = twist->var_wz;
    twist_pub_->publish(out);
  }

  void report_steering_staleness(double steady_s) {
    const bool stale = estimator_.steering_stale(steady_s);
    if (stale && !reported_stale_) {
      RCLCPP_WARN(get_logger(),
                  "wheel_twist_node: /drive stale or never received; yaw rate now carries the "
                  "stale steering variance until it returns");
    } else if (!stale && reported_stale_) {
      RCLCPP_INFO(get_logger(), "wheel_twist_node: /drive fresh again");
    }
    reported_stale_ = stale;
  }

  WheelTwistEstimator estimator_;
  std::string frame_id_;
  bool reported_stale_ = false;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
};

}  // namespace racer_state

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_state::WheelTwistNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_twist_node"), "wheel_twist_node: fatal error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
