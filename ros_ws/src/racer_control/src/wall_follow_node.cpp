// wall_follow_node: ROS 2 plumbing around racer_control's wall-follow core (GitHub issue 26;
// core in include/racer_control/wall_follow.hpp, PID in include/racer_control/pid.hpp). Port
// of the old Python reactive_control/wall_follow_node.py.
//
// Topics, QoS, fixed-rate /drive_raw timer, the /scan-silence watchdog (stop publishing,
// safety_node brakes), the dropped SIGINT wind-down, the float32 wire margin and the
// vehicle_params sourcing are identical to gap_follow_node; read that file's header comment,
// it is not repeated here. This node never publishes /drive.
//
// What is specific to this node:
//   * The PID runs in the /scan callback, once per measurement, with dt measured on the
//     STEADY clock between consecutive valid measurements (CLOCK POLICY below). Running it
//     in the 50 Hz timer instead would see the error change only once per 10 Hz scan, so the
//     derivative would be zero on four cycles and a spike on the fifth.
//   * A scan whose wall rays are not valid returns (no wall in view, out of range, NaN) gives
//     no measurement: the steering target goes to 0 (drive straight, the old node's
//     behaviour for an out-of-range ray) and the PID is reset, so the next valid sample does
//     not integrate or differentiate across the gap. The scan still counts as fresh for the
//     watchdog; only an unusable scan (bad geometry) or silence trips it.
//   * Speed: max_speed scaled down by steering (k_steer), floored at min_speed, then rate
//     limited through SpeedRateLimiter. The old node capped this by the old safety node's
//     /speed topic, which is gone.
//
// CLOCK POLICY (same as tracker_node.cpp and gap_follow_node.cpp):
//   * `steady_clock_` (RCL_STEADY_TIME) measures every elapsed time: the /scan watchdog and
//     the PID dt.
//   * `this->now()` (ROS clock) only stamps /drive_raw.
//   * The speed rate limiter advances by the configured control period (see tracker_node.cpp
//     for why a measured dt is wrong there).
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_control/float32_wire_margin.hpp"
#include "racer_control/laser_scan.hpp"
#include "racer_control/pid.hpp"
#include "racer_control/reactive_speed.hpp"
#include "racer_control/speed_rate_limiter.hpp"
#include "racer_control/wall_follow.hpp"
#include "reactive_node_params.hpp"

namespace racer_control {

class WallFollowNode : public rclcpp::Node {
 public:
  WallFollowNode()
      : Node("wall_follow_node"),
        config_(build_config()),
        pid_(build_gains()),
        speed_limiter_(build_speed_rate_limiter()) {
    max_speed_mps_ = declare_ranged_double(
        *this, "max_speed_mps", 1.5, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Speed on a straight (m/s). Range-limited to vehicle_params "
        "limits.global_speed_cap_mps.");
    min_speed_mps_ = declare_ranged_double(*this, "min_speed_mps", 0.5, 0.0,
                                           VEHICLE_PARAMS.limits.global_speed_cap_mps,
                                           "Floor under the steering slowdown (m/s).");
    if (min_speed_mps_ > max_speed_mps_) {
      throw std::invalid_argument("wall_follow_node: min_speed_mps must not exceed max_speed_mps");
    }
    k_steer_ = declare_ranged_double(
        *this, "k_steer", 0.5, 0.0, 1.0,
        "Steering slowdown: speed = max_speed * (1 - k_steer * |steering| / max_angle).");
    const double control_rate_hz = declare_ranged_double(
        *this, "control_rate_hz", 50.0, 1.0, 1000.0,
        "/drive_raw publish rate (Hz); claude-docs/04-architecture.md specifies 50 Hz.");
    control_period_s_ = 1.0 / control_rate_hz;
    scan_timeout_s_ = declare_ranged_double(
        *this, "scan_timeout_s", 0.3, 1e-3, 10.0,
        "Watchdog: stop publishing /drive_raw when no usable /scan has arrived for this long "
        "(s). Default is 3x the RPLIDAR C1's 10 Hz period.");

    scan_.ranges.reserve(4096);

    const rclcpp::QoS scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&WallFollowNode::on_scan, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_raw",
                                                                                    command_qos);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::duration<double>(control_period_s_)),
                                     std::bind(&WallFollowNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "wall_follow_node up: %.1f Hz, %s wall, rays %.3f / %.3f rad, target %.2f m, "
                "lookahead %.2f m, laser yaw %.3f rad",
                control_rate_hz, wall_steering_sign(config_) > 0.0 ? "right" : "left",
                config_.ray_a_bearing_rad, config_.ray_b_bearing_rad, config_.target_distance_m,
                config_.lookahead_m, config_.laser_yaw_offset_rad);
  }

 private:
  WallFollowConfig build_config() {
    WallFollowConfig c;
    c.ray_a_bearing_rad = declare_ranged_double(
        *this, "ray_a_bearing_rad", -20.0 * M_PI / 180.0, -M_PI, M_PI,
        "Forward wall ray, vehicle frame, left positive (rad). Negative = right wall.");
    c.ray_b_bearing_rad = declare_ranged_double(
        *this, "ray_b_bearing_rad", -M_PI / 2.0, -M_PI, M_PI,
        "Wall-normal ray, same side as ray a and further out (rad). +pi/2 with a positive "
        "ray a follows the left wall instead.");
    c.target_distance_m = declare_ranged_double(*this, "target_distance_m", 0.6, 0.0, 10.0,
                                                "Desired distance to the wall (m).");
    c.lookahead_m = declare_ranged_double(
        *this, "lookahead_m", 0.5, 0.0, 10.0,
        "Distance ahead the wall distance is projected to (m). Replaces the old speed * dt.");
    c.deadband_m = declare_ranged_double(*this, "deadband_m", 0.02, 0.0, 1.0,
                                         "Errors smaller than this read as zero (m).");
    const double yaw_param = declare_ranged_double(
        *this, "laser_yaw_offset_rad", 0.0, -M_PI, M_PI,
        "LiDAR mounting yaw in base_link (rad, CCW positive; pi = facing backwards). Only "
        "used while vehicle_params sensors.lidar.mount_yaw_rad is null; once that is "
        "measured it wins and a disagreeing non-zero value here refuses to start.");
    const auto yaw =
        resolve_laser_yaw_offset(VEHICLE_PARAMS.sensors.lidar.mount_yaw_rad, yaw_param);
    if (!yaw) {
      throw std::invalid_argument(
          "wall_follow_node: laser_yaw_offset_rad disagrees with vehicle_params "
          "sensors.lidar.mount_yaw_rad (or is not finite); refusing to start");
    }
    c.laser_yaw_offset_rad = *yaw;
    if (!wall_rays_valid(c)) {
      throw std::invalid_argument(
          "wall_follow_node: ray_a_bearing_rad / ray_b_bearing_rad must be non-zero, on the "
          "same side, with |ray b| > |ray a|");
    }
    return c;
  }

  PidGains build_gains() {
    PidGains g;
    g.kp = declare_ranged_double(*this, "kp", 1.5, 0.0, 100.0,
                                 "Proportional gain (rad of steering per m of error).");
    g.ki = declare_ranged_double(*this, "ki", 0.0, 0.0, 100.0, "Integral gain (rad / (m s)).");
    g.kd = declare_ranged_double(*this, "kd", 0.1, 0.0, 100.0, "Derivative gain (rad s / m).");
    g.integral_limit = declare_ranged_double(*this, "integral_limit_m_s", 1.0, 0.0, 100.0,
                                             "Clamp on the integral of the error (m s).");
    return g;
  }

  SpeedRateLimiter build_speed_rate_limiter() {
    margin_fraction_ = declare_ranged_double(
        *this, "speed_rate_limit_margin_fraction", 0.5, 0.01, 1.0,
        "Fraction of vehicle_params actuation.max_acceleration_mps2 this node ramps speed at "
        "(see tracker_node.cpp).");
    return SpeedRateLimiter(VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction_);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_.ranges.assign(msg->ranges.begin(), msg->ranges.end());
    scan_.angle_min = msg->angle_min;
    scan_.angle_increment = msg->angle_increment;
    scan_.range_min = msg->range_min;
    scan_.range_max = msg->range_max;
    if (!is_usable(scan_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "wall_follow_node: /scan has unusable geometry; treating as no scan.");
      return;
    }
    // Steady clock, never this->now(): see CLOCK POLICY.
    const rclcpp::Time steady_now = steady_clock_.now();
    last_scan_steady_ = steady_now;
    has_scan_ = true;

    const double max_angle = VEHICLE_PARAMS.steering.max_angle_rad;
    const auto measurement = measure_wall(scan_, config_);
    if (!measurement) {
      pid_.reset();
      has_measurement_ = false;
      steering_target_rad_ = 0.0;
      return;
    }
    const double dt_s = has_measurement_ ? (steady_now - last_measurement_steady_).seconds() : 0.0;
    last_measurement_steady_ = steady_now;
    has_measurement_ = true;
    const double command = wall_steering_sign(config_) * pid_.update(measurement->error_m, dt_s);
    steering_target_rad_ =
        std::isfinite(command) ? std::min(std::max(command, -max_angle), max_angle) : 0.0;
  }

  void on_timer() {
    const rclcpp::Time now = this->now();
    const rclcpp::Time steady_now = steady_clock_.now();
    const bool stale = !has_scan_ || (steady_now - last_scan_steady_).seconds() > scan_timeout_s_;
    if (stale) {
      if (!watchdog_active_) {
        RCLCPP_WARN(this->get_logger(),
                    "wall_follow_node: /scan stale (> %.3fs); stopping /drive_raw until a usable "
                    "scan arrives (safety_node brakes on /drive_raw silence).",
                    scan_timeout_s_);
        watchdog_active_ = true;
      }
      has_last_command_time_ = false;
      pid_.reset();
      has_measurement_ = false;
      speed_limiter_ =
          SpeedRateLimiter(VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction_);
      return;
    }
    if (watchdog_active_) {
      RCLCPP_INFO(this->get_logger(), "wall_follow_node: /scan fresh again; resuming /drive_raw.");
      watchdog_active_ = false;
    }

    const double max_angle = VEHICLE_PARAMS.steering.max_angle_rad;
    const double dt_s = has_last_command_time_ ? control_period_s_ : 0.0;
    has_last_command_time_ = true;
    const double steering = steering_target_rad_;
    const double raw_speed = std::max(
        min_speed_mps_, apply_steering_slowdown(max_speed_mps_, steering, k_steer_, max_angle));
    const double speed = speed_limiter_.limit(raw_speed, dt_s);
    if (!std::isfinite(steering) || !std::isfinite(speed)) {
      RCLCPP_ERROR(this->get_logger(),
                   "wall_follow_node: non-finite command computed; not publishing it.");
      return;
    }

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = clamp_for_float32_publish(steering, -max_angle, max_angle);
    drive_msg.drive.speed = clamp_for_float32_publish(
        speed, VEHICLE_PARAMS.limits.min_velocity_mps,
        std::min(max_speed_mps_, VEHICLE_PARAMS.limits.global_speed_cap_mps));
    drive_pub_->publish(drive_msg);
  }

  // Declared before speed_limiter_ (see gap_follow_node.cpp).
  double margin_fraction_{0.5};
  WallFollowConfig config_;
  Pid pid_;
  SpeedRateLimiter speed_limiter_;
  double max_speed_mps_{0.0};
  double min_speed_mps_{0.0};
  double k_steer_{0.0};
  double control_period_s_{0.02};
  double scan_timeout_s_{0.3};
  double steering_target_rad_{0.0};

  ScanInput scan_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Monotonic clock for elapsed-time measurement only (CLOCK POLICY).
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_scan_steady_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_measurement_steady_{0, 0, RCL_STEADY_TIME};
  bool has_scan_{false};
  bool has_measurement_{false};
  bool watchdog_active_{false};
  bool has_last_command_time_{false};
};

}  // namespace racer_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_control::WallFollowNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wall_follow_node"), "wall_follow_node: fatal error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
