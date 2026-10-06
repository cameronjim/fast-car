// gap_follow_node: ROS 2 plumbing around racer_control's follow-the-gap core (GitHub issue
// 26; core in include/racer_control/gap_follow.hpp). Port of the old Python
// reactive_control/gap_follow_node.py with its safety-relevant behaviour replaced by this
// repo's conventions.
//
// Topics (claude-docs/04-architecture.md, interface names not improvised):
//   * subscribes /scan (sensor_msgs/LaserScan, KeepLast(10) best_effort, matching the LiDAR
//     driver, racer_gym_bridge, and racer_safety/safety_node).
//   * publishes /drive_raw (ackermann_msgs/AckermannDriveStamped, KeepLast(10) reliable) at a
//     fixed `control_rate_hz` (default 50 Hz). NEVER /drive: racer_safety/safety_node is the
//     sole publisher of /drive and gates /drive_raw -> /drive (CLAUDE.md invariant 1,
//     claude-docs/05-safety.md). The old node published /drive directly and listened to the
//     old safety node's /speed and /kys topics; none of that is ported.
//
// Why a fixed-rate timer rather than publishing once per scan: the LiDAR runs at 10 Hz on the
// car (RPLIDAR C1) and safety_node brakes after 3 missed /drive_raw cycles at 50 Hz, so a
// scan-driven publisher would trip the watchdog between every pair of scans. The scan
// callback runs the perception half (sanitise, disparity extension, gap selection, raw
// steering); the timer runs the smoothing half (low-pass, speed law, rate limit) at the
// command rate, which also turns 10 Hz steering steps into a smooth 50 Hz command.
//
// Watchdog behaviour on /scan silence: this node STOPS PUBLISHING /drive_raw, exactly like
// tracker_node does on /odom silence, for the same reason (see tracker_node.cpp's header):
// claude-docs/04-architecture.md assigns the staleness watchdog to safety_node ("missing
// /drive_raw for 3 cycles -> brake command") and claude-docs/05-safety.md makes safety_node
// the sole authority for what the car does when inputs go bad. A controller that invents its
// own "safe" command during sensor silence duplicates that logic and can be wrong in ways
// safety_node never sees; silence on /drive_raw is the signal safety_node acts on. A scan the
// core rejects (unusable geometry, no valid returns, forward not covered) counts as no scan:
// the last good result is kept until `scan_timeout_s`, then publication stops. On resume the
// speed ramp restarts from 0 (safety_node will have braked the car in the meantime).
//
// Shutdown: the old node trapped SIGINT and coasted for two seconds before a final zero
// command. That is dropped: on shutdown this node just stops publishing, and safety_node
// brakes on /drive_raw silence.
//
// CLOCK POLICY (same as tracker_node.cpp, read that file's comment before changing time
// arithmetic here):
//   * `steady_clock_` (RCL_STEADY_TIME) is the ONLY clock used to measure elapsed time, here
//     the /scan staleness watchdog. The ROS clock can step backwards (use_sim_time:=false,
//     NTP) or sit at zero (use_sim_time:=true with no /clock), either of which would disable
//     a ROS-clock watchdog (GitHub issue 22).
//   * `this->now()` (the ROS clock) is used ONLY to stamp the outgoing /drive_raw header.
//   * The low-pass filter and the speed rate limiter advance by the CONFIGURED control period
//     per cycle, not a measured dt, for the reason tracker_node gives for its rate limiter:
//     safety_node checks the same acceleration bound on its own independently-clocked timer,
//     and two noisy measured dts disagreeing around a bound is a false gate trip.
//
// Float32 wire margin: published values go through racer_control::clamp_for_float32_publish,
// as in tracker_node, so a command sitting exactly at a vehicle_params bound in double
// precision cannot round past it on the float32 wire. Non-finite values are never published
// (the core never produces them; the timer re-checks before publishing).
//
// Physical constants come only from the generated vehicle_params binding (CLAUDE.md
// invariant 2): half width = chassis.width_m / 2 (the old node hard-coded 0.5 m, wrong for
// this car), steering clamp = steering.max_angle_rad, speed cap = limits.global_speed_cap_mps
// (the `max_speed_mps` parameter is range-limited below it), acceleration =
// actuation.max_acceleration_mps2 via SpeedRateLimiter. LiDAR yaw comes from
// sensors.lidar.mount_yaw_rad once measured (see resolve_laser_yaw_offset). The swept-path
// clamp takes chassis.wheelbase_m, chassis.length_m, chassis.cg_to_rear_axle_m and
// sensors.lidar.mount_x_m / mount_y_m (the node refuses to start while the mount is null).
// Everything else is a tuning parameter with a default and a description.
//
// laser_yaw_from_vehicle_params (default true) is the one exception to the yaw rule above:
// false ignores the binding and uses laser_yaw_offset_rad as given. It exists ONLY for scans
// that are not the real car's LiDAR, i.e. the simulator (racer_gym_bridge publishes /scan
// aligned to the vehicle, yaw 0) and synthetic-scan tests. Never set it false on the car.
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
#include "racer_control/gap_follow.hpp"
#include "racer_control/laser_scan.hpp"
#include "racer_control/reactive_speed.hpp"
#include "racer_control/speed_rate_limiter.hpp"
#include "reactive_node_params.hpp"

namespace racer_control {

class GapFollowNode : public rclcpp::Node {
 public:
  GapFollowNode()
      : Node("gap_follow_node"),
        follower_(build_config()),
        steering_filter_(declare_ranged_double(
            *this, "steering_time_constant_s", 0.1, 0.0, 10.0,
            "First-order low-pass time constant on the steering command (s). 0 disables "
            "the filter. Smoothness over speed: larger is smoother but lags the gap.")),
        speed_limiter_(build_speed_rate_limiter()) {
    max_speed_mps_ = declare_ranged_double(
        *this, "max_speed_mps", 2.0, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Upper speed bound (m/s). Range-limited to vehicle_params limits.global_speed_cap_mps.");
    min_speed_mps_ = declare_ranged_double(
        *this, "min_speed_mps", 0.5, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Lower bound of the range-based speed (m/s), before the steering slowdown.");
    if (min_speed_mps_ > max_speed_mps_) {
      throw std::invalid_argument("gap_follow_node: min_speed_mps must not exceed max_speed_mps");
    }
    k_speed_per_s_ = declare_ranged_double(
        *this, "k_speed_per_s", 1.0, 0.0, 100.0,
        "Speed gain (1/s): speed = clamp(k_speed * range along the target bearing, min, max).");
    k_steer_ = declare_ranged_double(
        *this, "k_steer", 0.5, 0.0, 1.0,
        "Steering slowdown: speed *= 1 - k_steer * |steering| / steering.max_angle_rad.");
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
        "/scan", scan_qos, std::bind(&GapFollowNode::on_scan, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_raw",
                                                                                    command_qos);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::duration<double>(control_period_s_)),
                                     std::bind(&GapFollowNode::on_timer, this));

    const GapFollowConfig& c = follower_.config();
    RCLCPP_INFO(this->get_logger(),
                "gap_follow_node up: %.1f Hz, half width %.3f m + margin %.3f m, cone +/- %.3f "
                "rad, laser yaw %.3f rad, %s target, forward preference %.3f, gap switch "
                "margin %.3f, swept-path clamp %s (lookahead %.2f m, wheelbase %.4f m, "
                "LiDAR at x %.3f y %.3f m, body front x %.3f m), max speed %.2f m/s",
                control_rate_hz, c.half_width_m, c.safety_margin_m, c.cone_half_angle_rad,
                c.laser_yaw_offset_rad, c.target == GapTarget::kDeepest ? "deepest" : "centre",
                c.forward_preference, c.gap_switch_margin, c.swept_path_clamp ? "on" : "off",
                c.swept_path.lookahead_m, c.swept_path.wheelbase_m, c.swept_path.lidar_mount_x_m,
                c.swept_path.lidar_mount_y_m, c.swept_path.body_front_x_m, max_speed_mps_);
  }

 private:
  GapFollowConfig build_config() {
    GapFollowConfig c;
    c.half_width_m = VEHICLE_PARAMS.chassis.width_m / 2.0;
    c.max_steering_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    c.safety_margin_m = declare_ranged_double(
        *this, "safety_margin_m", 0.1, 0.0, 2.0,
        "Added to chassis.width_m / 2 for the disparity-extension bubble (m).");
    c.clip_max_range_m = declare_ranged_double(
        *this, "clip_max_range_m", 5.0, 0.1, 100.0,
        "Ranges are clipped to this (m); inf and above-range_max returns read as this.");
    c.disparity_threshold_m = declare_ranged_double(
        *this, "disparity_threshold_m", 0.5, 0.01, 100.0,
        "Range jump between neighbouring rays that counts as an obstacle edge (m).");
    c.free_space_threshold_m =
        declare_ranged_double(*this, "free_space_threshold_m", 1.5, 0.0, 100.0,
                              "A ray is free when its extended range is above this (m).");
    c.cone_half_angle_rad = declare_ranged_double(
        *this, "cone_half_angle_rad", 1.57, 0.0, M_PI,
        "Gap search cone half-angle around the vehicle's forward direction (rad).");
    c.target = declare_described_bool(
                   *this, "target_deepest_ray", false,
                   "Aim at the deepest ray of the chosen gap instead of its centre ray.")
                   ? GapTarget::kDeepest
                   : GapTarget::kCentre;
    c.forward_preference = declare_ranged_double(
        *this, "forward_preference", 0.0, 0.0, 1.0,
        "Weights gap selection toward the vehicle's forward direction: each gap scores "
        "width * max(0, 1 - p * (1 - cos(centre bearing))). 0 (default) = plain widest gap; "
        "1 = a gap at 90 degrees scores zero. See gap_follow.hpp GapPreference.");
    c.gap_switch_margin = declare_ranged_double(
        *this, "gap_switch_margin", 0.0, 0.0, 1.0,
        "Gap switching hysteresis (fraction). Keep the gap containing last scan's target "
        "bearing unless another gap's score exceeds it by more than this fraction. 0 "
        "(default) = off.");
    c.steering_gain = declare_ranged_double(*this, "steering_gain", 1.0, 0.0, 10.0,
                                            "steering = clamp(gain * target bearing, +/- max).");
    c.corner_sector_inner_rad = declare_ranged_double(
        *this, "corner_sector_inner_rad", M_PI / 2.0, 0.0, M_PI,
        "Inner edge of the turn-in side sector for the corner override (rad, magnitude).");
    c.corner_sector_outer_rad = declare_ranged_double(
        *this, "corner_sector_outer_rad", 3.0 * M_PI / 4.0, 0.0, M_PI,
        "Outer edge of the turn-in side sector for the corner override (rad, magnitude).");
    if (c.corner_sector_inner_rad > c.corner_sector_outer_rad) {
      throw std::invalid_argument(
          "gap_follow_node: corner_sector_inner_rad must not exceed corner_sector_outer_rad");
    }
    c.corner_min_clearance_m = declare_ranged_double(
        *this, "corner_min_clearance_m", 0.2, 0.0, 10.0,
        "Steering is zeroed when every ray in the turn-in sector is closer than this (m).");
    const double yaw_param = declare_ranged_double(
        *this, "laser_yaw_offset_rad", 0.0, -M_PI, M_PI,
        "LiDAR mounting yaw in base_link (rad, CCW positive; pi = facing backwards). With "
        "laser_yaw_from_vehicle_params true, only used while vehicle_params "
        "sensors.lidar.mount_yaw_rad is null; once that is measured it wins and a disagreeing "
        "non-zero value here refuses to start. With it false, used as given.");
    // false is for the simulator and synthetic-scan tests ONLY (their /scan is aligned to the
    // vehicle, yaw 0, not mounted like the real car's LiDAR). The real car keeps the default
    // true so its yaw comes only from the binding (CLAUDE.md invariant 2).
    const bool yaw_from_vehicle_params = declare_described_bool(
        *this, "laser_yaw_from_vehicle_params", true,
        "true (default, the real car): LiDAR yaw comes from vehicle_params "
        "sensors.lidar.mount_yaw_rad when set. false: SIMULATOR AND SYNTHETIC-SCAN TESTS ONLY, "
        "ignore the binding and use laser_yaw_offset_rad as given.");
    if (!yaw_from_vehicle_params) {
      RCLCPP_INFO(this->get_logger(),
                  "gap_follow_node: laser_yaw_from_vehicle_params is false, ignoring "
                  "vehicle_params sensors.lidar.mount_yaw_rad and using laser_yaw_offset_rad "
                  "= %.6f rad (sim or synthetic-scan fixture only)",
                  yaw_param);
    }
    const auto yaw = resolve_laser_yaw_offset(VEHICLE_PARAMS.sensors.lidar.mount_yaw_rad, yaw_param,
                                              yaw_from_vehicle_params);
    if (!yaw) {
      throw std::invalid_argument(
          "gap_follow_node: laser_yaw_offset_rad disagrees with vehicle_params "
          "sensors.lidar.mount_yaw_rad with laser_yaw_from_vehicle_params true (or is not "
          "finite); refusing to start");
    }
    c.laser_yaw_offset_rad = *yaw;

    // Swept-path clamp (gap_follow.hpp clamp_steering_to_swept_path). The LiDAR position
    // relative to the rear axle comes only from the binding; like lidar.launch.py and
    // safety_node, refuse to start while it is unmeasured (null).
    c.swept_path_clamp = declare_described_bool(
        *this, "swept_path_clamp", true,
        "Reduce the steering until the car's swept area (inside flank to outside front "
        "corner, plus safety_margin_m) over swept_path_lookahead_m of arc is clear of every "
        "return beside and ahead of the car on the turn-in side. See gap_follow.hpp.");
    c.swept_path.lookahead_m = declare_ranged_double(
        *this, "swept_path_lookahead_m", 1.0, 0.01, 10.0,
        "Rear-axle arc length (m) ahead of the car within which returns constrain the "
        "swept-path clamp.");
    if (!VEHICLE_PARAMS.sensors.lidar.mount_x_m.has_value() ||
        !VEHICLE_PARAMS.sensors.lidar.mount_y_m.has_value()) {
      throw std::invalid_argument(
          "gap_follow_node: vehicle_params sensors.lidar.mount_x_m / mount_y_m is null; the "
          "swept-path clamp needs the LiDAR position relative to the rear axle. Refusing to "
          "start");
    }
    c.swept_path.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
    c.swept_path.lidar_mount_x_m = *VEHICLE_PARAMS.sensors.lidar.mount_x_m;
    c.swept_path.lidar_mount_y_m = *VEHICLE_PARAMS.sensors.lidar.mount_y_m;
    // The front of the body relative to the rear axle: chassis.length_m is a bounding box,
    // taken as centred on the CG (the f1tenth_gym convention its values come from).
    c.swept_path.body_front_x_m =
        VEHICLE_PARAMS.chassis.cg_to_rear_axle_m + VEHICLE_PARAMS.chassis.length_m / 2.0;
    return c;
  }

  SpeedRateLimiter build_speed_rate_limiter() {
    margin_fraction_ = declare_ranged_double(
        *this, "speed_rate_limit_margin_fraction", 0.5, 0.01, 1.0,
        "Fraction of vehicle_params actuation.max_acceleration_mps2 this node ramps speed at "
        "(headroom against safety_node's independently clocked copy of the same bound, see "
        "tracker_node.cpp).");
    return SpeedRateLimiter(VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction_);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_.ranges.assign(msg->ranges.begin(), msg->ranges.end());
    scan_.angle_min = msg->angle_min;
    scan_.angle_increment = msg->angle_increment;
    scan_.range_min = msg->range_min;
    scan_.range_max = msg->range_max;
    const GapFollowResult result = follower_.process(scan_);
    if (!result.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "gap_follow_node: /scan rejected (unusable geometry, no valid "
                           "returns, or forward not covered); treating as no scan.");
      return;
    }
    if (result.swept_path_clamped) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "gap_follow_node: swept-path clamp reduced steering %.3f -> %.3f rad "
                            "(target bearing %.3f rad)",
                            result.wanted_steering_rad, result.steering_rad,
                            result.target_vehicle_bearing_rad);
    }
    latest_ = result;
    // Steady clock, never this->now(): see CLOCK POLICY.
    last_scan_steady_ = steady_clock_.now();
    has_scan_ = true;
  }

  void on_timer() {
    const rclcpp::Time now = this->now();
    const rclcpp::Time steady_now = steady_clock_.now();
    const bool stale = !has_scan_ || (steady_now - last_scan_steady_).seconds() > scan_timeout_s_;
    if (stale) {
      if (!watchdog_active_) {
        RCLCPP_WARN(this->get_logger(),
                    "gap_follow_node: /scan stale (> %.3fs); stopping /drive_raw until a usable "
                    "scan arrives (safety_node brakes on /drive_raw silence).",
                    scan_timeout_s_);
        watchdog_active_ = true;
      }
      has_last_command_time_ = false;
      // safety_node brakes the car during the silence, so ramp from rest on resume.
      speed_limiter_ =
          SpeedRateLimiter(VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction_);
      return;
    }
    if (watchdog_active_) {
      RCLCPP_INFO(this->get_logger(), "gap_follow_node: /scan fresh again; resuming /drive_raw.");
      watchdog_active_ = false;
    }

    const double max_angle = VEHICLE_PARAMS.steering.max_angle_rad;
    const double dt_s = has_last_command_time_ ? control_period_s_ : 0.0;
    has_last_command_time_ = true;
    const double steering = steering_filter_.update(latest_.steering_rad, dt_s);
    const double raw_speed = apply_steering_slowdown(
        range_based_speed(latest_.target_range_m, k_speed_per_s_, min_speed_mps_, max_speed_mps_),
        steering, k_steer_, max_angle);
    const double speed = speed_limiter_.limit(raw_speed, dt_s);
    if (!std::isfinite(steering) || !std::isfinite(speed)) {
      RCLCPP_ERROR(this->get_logger(),
                   "gap_follow_node: non-finite command computed; not publishing it.");
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

  // Declared before speed_limiter_: build_speed_rate_limiter() writes it during member
  // initialisation, and a later-declared default initialiser would overwrite that value.
  double margin_fraction_{0.5};
  GapFollower follower_;
  FirstOrderLowPass steering_filter_;
  SpeedRateLimiter speed_limiter_;
  double max_speed_mps_{0.0};
  double min_speed_mps_{0.0};
  double k_speed_per_s_{0.0};
  double k_steer_{0.0};
  double control_period_s_{0.02};
  double scan_timeout_s_{0.3};

  ScanInput scan_;
  GapFollowResult latest_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Monotonic clock for elapsed-time measurement only (CLOCK POLICY).
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_scan_steady_{0, 0, RCL_STEADY_TIME};
  bool has_scan_{false};
  bool watchdog_active_{false};
  bool has_last_command_time_{false};
};

}  // namespace racer_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_control::GapFollowNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("gap_follow_node"), "gap_follow_node: fatal error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
