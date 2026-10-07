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
// Speed smoothing (2026-10-06 floor checkpoint, both off by default): the scan callback can
// replace each scan's target range with the median of the last `target_range_median_scans`
// scans' target ranges, and the timer can low-pass the speed command with
// `speed_time_constant_s` BEFORE the rate limiter, so the limiter still bounds acceleration
// (ReactiveSpeedCommand in reactive_speed.hpp). On a /scan watchdog trip both are reset with
// the rate limiter.
//
// REVERSE ESCAPE (2026-10-06 night floor finding (b), `reverse_escape`, default false; true in
// the floor profile). THIS NODE CAN COMMAND THE CAR TO REVERSE ON ITS OWN when it is on. The car
// ended nose-in to a corner tighter than its turning circle in both lap directions and waited
// on safety_node's latch for ever. With reverse_escape on, the node also subscribes the GATED
// /drive (KeepLast(10) reliable, matching safety_node's publisher) so it can see when
// safety_node is refusing its forward request (its own request > 0, /drive speed 0). When that
// has lasted escape_after_s AND the follower itself sees no way forward (corner override, or no
// steering with a clear forward arc for escape_probe_distance_m, gap_follow.hpp
// any_forward_arc_clear), it requests escape_speed_mps (negative) with the steering opposite to
// the follower's wanted steering, for escape_distance_m of commanded travel or escape_max_s,
// then resumes forward. If safety_node refuses the reverse request too (its rear corridor), the
// escape is aborted and retried after escape_retry_after_s, at most escape_max_attempts times
// without forward progress, then the node holds (stops trying; safety_node's latch holds the
// car). The state machine and every rule are in include/racer_control/reverse_escape.hpp; each
// transition is logged at INFO. The escape goes through /drive_raw and safety_node like every
// other command: this node NEVER publishes /drive (CLAUDE.md invariant 1), only reads it.
// Without safety_node in the loop there is no gated /drive, so the escape never fires.
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
//   * The low-pass filters and the speed rate limiter advance by the CONFIGURED control period
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
#include <cstddef>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_control/float32_wire_margin.hpp"
#include "racer_control/gap_follow.hpp"
#include "racer_control/laser_scan.hpp"
#include "racer_control/reactive_speed.hpp"
#include "racer_control/reverse_escape.hpp"
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
            "the filter. Smoothness over speed: larger is smoother but lags the gap.")) {
    speed_command_ = ReactiveSpeedCommand(build_speed_config());
    target_range_median_ = RollingMedian(static_cast<std::size_t>(declare_ranged_int(
        *this, "target_range_median_scans", 1, 1, 15,
        "The speed law uses the median of the last N scans' target ranges instead of this "
        "scan's. 1 (default) = off. Damps the scan-to-scan flicker of the target range.")));
    const double control_rate_hz = declare_ranged_double(
        *this, "control_rate_hz", 50.0, 1.0, 1000.0,
        "/drive_raw publish rate (Hz); claude-docs/04-architecture.md specifies 50 Hz.");
    control_period_s_ = 1.0 / control_rate_hz;
    scan_timeout_s_ = declare_ranged_double(
        *this, "scan_timeout_s", 0.3, 1e-3, 10.0,
        "Watchdog: stop publishing /drive_raw when no usable /scan has arrived for this long "
        "(s). Default is 3x the RPLIDAR C1's 10 Hz period.");

    build_escape();
    scan_.ranges.reserve(4096);

    const rclcpp::QoS scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&GapFollowNode::on_scan, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_raw",
                                                                                    command_qos);
    if (escape_) {
      // The GATED command, read only (reverse escape). Never published here (invariant 1).
      gated_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
          "/drive", command_qos,
          std::bind(&GapFollowNode::on_gated_drive, this, std::placeholders::_1));
    }
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::duration<double>(control_period_s_)),
                                     std::bind(&GapFollowNode::on_timer, this));

    const GapFollowConfig& c = follower_.config();
    RCLCPP_INFO(this->get_logger(),
                "gap_follow_node up: %.1f Hz, half width %.3f m + margin %.3f m, cone +/- %.3f "
                "rad, laser yaw %.3f rad, %s target, forward preference %.3f, gap switch "
                "margin %.3f, swept-path clamp %s (lookahead %.2f m, wheelbase %.4f m, "
                "LiDAR at x %.3f y %.3f m, body front x %.3f m), lane centring gain %.3f "
                "(sector +/- %.3f rad, max range %.2f m), max speed %.2f m/s, speed time "
                "constant %.3f s, target range median over %zu scans",
                control_rate_hz, c.half_width_m, c.safety_margin_m, c.cone_half_angle_rad,
                c.laser_yaw_offset_rad, c.target == GapTarget::kDeepest ? "deepest" : "centre",
                c.forward_preference, c.gap_switch_margin, c.swept_path_clamp ? "on" : "off",
                c.swept_path.lookahead_m, c.swept_path.wheelbase_m, c.swept_path.lidar_mount_x_m,
                c.swept_path.lidar_mount_y_m, c.swept_path.body_front_x_m, c.centering_gain,
                c.centering_sector_half_angle_rad, c.centering_max_range_m,
                speed_command_.config().max_speed_mps,
                speed_command_.config().speed_time_constant_s, target_range_median_.window());
    if (escape_) {
      const ReverseEscapeConfig& e = escape_->config();
      RCLCPP_WARN(this->get_logger(),
                  "gap_follow_node: REVERSE ESCAPE ON: this node will command the car to "
                  "REVERSE on its own when safety_node has refused its forward request for "
                  "%.2f s and no steering gives %.2f m of clear forward arc (%.2f m/s, steering "
                  "opposite to the wanted one at %.3f rad, up to %.2f m or %.1f s; retry after "
                  "%.1f s, at most %d attempts without forward progress). Reverse requests go "
                  "through safety_node's rear corridor like every other command.",
                  e.escape_after_s, escape_probe_distance_m_, e.escape_speed_mps,
                  e.escape_steering_rad, e.escape_distance_m, e.escape_max_s,
                  e.escape_retry_after_s, e.escape_max_attempts);
    } else {
      RCLCPP_INFO(this->get_logger(), "gap_follow_node: reverse escape off (reverse_escape false)");
    }
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
    // Lane centring (gap_follow.hpp measure_lane_walls / centering_steering).
    c.centering_gain = declare_ranged_double(
        *this, "centering_gain", 0.0, 0.0, 5.0,
        "Lane centring gain (rad). Adds centering_gain * (d_left - d_right) / (d_left + "
        "d_right) to steering_gain * target bearing before the steering clamp: a push away from "
        "the nearer side wall, left when the right wall is nearer. d_left / d_right are the "
        "median perpendicular distances of the returns on each side of the front sector. No "
        "push while either side has no return within centering_max_range_m (an open side). 0 "
        "(default) = off, bit-identical to the follower without centring.");
    c.centering_sector_half_angle_rad = declare_ranged_double(
        *this, "centering_sector_half_angle_rad", 1.0, 0.01, M_PI / 2.0,
        "Lane centring: returns with vehicle bearing in (0, this] are the left wall and in "
        "[-this, 0) the right wall (rad).");
    c.centering_max_range_m = declare_ranged_double(
        *this, "centering_max_range_m", 1.5, 0.05, 20.0,
        "Lane centring: only returns within this range count (m); a side with none is open.");
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

  ReactiveSpeedConfig build_speed_config() {
    ReactiveSpeedConfig s;
    const double margin_fraction = declare_ranged_double(
        *this, "speed_rate_limit_margin_fraction", 0.5, 0.01, 1.0,
        "Fraction of vehicle_params actuation.max_acceleration_mps2 this node ramps speed at "
        "(headroom against safety_node's independently clocked copy of the same bound, see "
        "tracker_node.cpp).");
    s.max_acceleration_mps2 = VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction;
    s.max_speed_mps = declare_ranged_double(
        *this, "max_speed_mps", 2.0, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Upper speed bound (m/s). Range-limited to vehicle_params limits.global_speed_cap_mps.");
    s.min_speed_mps = declare_ranged_double(
        *this, "min_speed_mps", 0.5, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Lower bound of the range-based speed (m/s), before the steering slowdown.");
    if (s.min_speed_mps > s.max_speed_mps) {
      throw std::invalid_argument("gap_follow_node: min_speed_mps must not exceed max_speed_mps");
    }
    s.k_speed_per_s = declare_ranged_double(
        *this, "k_speed_per_s", 1.0, 0.0, 100.0,
        "Speed gain (1/s): speed = clamp(k_speed * range along the target bearing, min, max).");
    s.k_steer = declare_ranged_double(
        *this, "k_steer", 0.5, 0.0, 1.0,
        "Steering slowdown: speed *= 1 - k_steer * |steering| / steering.max_angle_rad.");
    s.max_steering_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    s.speed_time_constant_s = declare_ranged_double(
        *this, "speed_time_constant_s", 0.0, 0.0, 5.0,
        "First-order low-pass time constant on the speed command (s), applied before the "
        "acceleration rate limit so the limit still bounds what is published. 0 (default) = "
        "off.");
    return s;
  }

  // Reverse escape parameters (reverse_escape.hpp). Declared whether or not the escape is on,
  // so a params file or the profile is range-checked either way.
  void build_escape() {
    const bool enabled = declare_described_bool(
        *this, "reverse_escape", false,
        "true: when safety_node has refused this node's forward request for escape_after_s "
        "and no steering gives a clear forward arc, REVERSE on its own (escape_* parameters; "
        "include/racer_control/reverse_escape.hpp). Subscribes the gated /drive to see the "
        "refusal. false (default): never reverses.");
    ReverseEscapeConfig e;
    e.escape_after_s = declare_ranged_double(
        *this, "escape_after_s", 1.5, 0.1, 60.0,
        "Reverse escape: forward request refused by safety_node (gated /drive speed 0) for this "
        "long before an escape may start (s).");
    escape_probe_distance_m_ = declare_ranged_double(
        *this, "escape_probe_distance_m", 0.3, 0.02, 3.0,
        "Reverse escape: the forward path counts as blocked when no steering lets the body "
        "(bounding box plus safety_margin_m) travel this far (m, rear-axle arc) without touching "
        "a return, or the corner override fired.");
    e.escape_speed_mps = declare_ranged_double(
        *this, "escape_speed_mps", -0.5, VEHICLE_PARAMS.limits.min_velocity_mps, -0.01,
        "Reverse escape speed (m/s, negative). On the car keep its magnitude at or above about "
        "0.44 m/s: the VESC speed loop does nothing below its s_pid_min_erpm.");
    e.escape_steering_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    e.escape_distance_m = declare_ranged_double(
        *this, "escape_distance_m", 0.4, 0.02, 3.0,
        "Reverse escape length (m), integrated from the commanded speed (no odometry; the real "
        "travel is shorter, never longer).");
    e.escape_max_s = declare_ranged_double(*this, "escape_max_s", 2.0, 0.1, 30.0,
                                           "Reverse escape time limit (s).");
    e.escape_retry_after_s = declare_ranged_double(
        *this, "escape_retry_after_s", 3.0, 0.0, 120.0,
        "Wait after an escape safety_node braked (rear corridor) before another may start (s).");
    e.escape_max_attempts = static_cast<int>(declare_ranged_int(
        *this, "escape_max_attempts", 3, 1, 100,
        "Escapes without forward progress (escape_distance_m driven forward) before the node "
        "holds and leaves the car on safety_node's latch."));
    e.block_debounce_s = declare_ranged_double(
        *this, "escape_block_debounce_s", 0.1, 0.02, 2.0,
        "A refused reverse request must persist this long before the escape aborts (s); covers "
        "the cycle or two /drive lags /drive_raw through safety_node.");
    drive_feedback_timeout_s_ = declare_ranged_double(
        *this, "drive_feedback_timeout_s", 0.2, 0.02, 5.0,
        "The gated /drive counts as fresh for this long after its last message (s); stale "
        "/drive never starts an escape and aborts a running one.");
    e.max_acceleration_mps2 = speed_command_.config().max_acceleration_mps2;
    if (!enabled) {
      return;
    }
    escape_.emplace(e);
    const GapFollowConfig& c = follower_.config();
    probe_geometry_.wheelbase_m = c.swept_path.wheelbase_m;
    probe_geometry_.half_width_m = c.half_width_m;
    probe_geometry_.margin_m = c.safety_margin_m;
    probe_geometry_.front_x_m = c.swept_path.body_front_x_m;
    probe_geometry_.rear_x_m = VEHICLE_PARAMS.chassis.rear_overhang_m;
    probe_geometry_.lidar_mount_x_m = c.swept_path.lidar_mount_x_m;
    probe_geometry_.lidar_mount_y_m = c.swept_path.lidar_mount_y_m;
  }

  void on_gated_drive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    gated_speed_mps_ = static_cast<double>(msg->drive.speed);
    // Steady clock, never this->now(): see CLOCK POLICY.
    last_gated_steady_ = steady_clock_.now();
    has_gated_ = true;
  }

  void log_escape_event(EscapeEvent event) {
    const ReverseEscape& e = *escape_;
    const ReverseEscapeConfig& c = e.config();
    switch (event) {
      case EscapeEvent::kStarted:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape %d/%d: forward request refused by "
                    "safety_node for %.2f s and no clear forward arc for %.2f m (corner override "
                    "%s); reversing at %.2f m/s with steering %+.3f rad for up to %.2f m or %.1f s",
                    e.attempts(), c.escape_max_attempts, e.forward_blocked_s(),
                    escape_probe_distance_m_, latest_.corner_blocked ? "on" : "off",
                    c.escape_speed_mps, e.escape_steering_rad(), c.escape_distance_m,
                    c.escape_max_s);
        break;
      case EscapeEvent::kCompleted:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape complete: %.2f m commanded in %.2f s; "
                    "resuming forward",
                    e.escape_distance_m(), e.escape_elapsed_s());
        break;
      case EscapeEvent::kTimedOut:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape timed out after %.2f s (%.2f m commanded); "
                    "resuming forward",
                    e.escape_elapsed_s(), e.escape_distance_m());
        break;
      case EscapeEvent::kAborted:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape aborted after %.2f s (%.2f m commanded): "
                    "safety_node refused the reverse request (rear corridor) or /drive went "
                    "silent; next attempt no sooner than %.1f s",
                    e.escape_elapsed_s(), e.escape_distance_m(), c.escape_retry_after_s);
        break;
      case EscapeEvent::kRetryReady:
        RCLCPP_INFO(this->get_logger(), "gap_follow_node: reverse escape retry wait over");
        break;
      case EscapeEvent::kExhausted:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape: %d attempts without forward progress; "
                    "holding, no more escapes until the car drives %.2f m forward",
                    e.attempts(), c.escape_distance_m);
        break;
      case EscapeEvent::kReset:
        RCLCPP_INFO(this->get_logger(),
                    "gap_follow_node: reverse escape: forward progress, attempt count reset");
        break;
    }
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
    if (result.centering_steering_rad != 0.0) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "gap_follow_node: lane centring %+.3f rad (left wall %.2f m, right "
                            "wall %.2f m)",
                            result.centering_steering_rad, result.lane_walls.left_m.value_or(0.0),
                            result.lane_walls.right_m.value_or(0.0));
    }
    if (result.swept_path_clamped) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "gap_follow_node: swept-path clamp reduced steering %.3f -> %.3f rad "
                            "(target bearing %.3f rad)",
                            result.wanted_steering_rad, result.steering_rad,
                            result.target_vehicle_bearing_rad);
    }
    latest_ = result;
    if (escape_) {
      // The follower's own view of the way forward, for the reverse escape: the corner
      // override fired, or no steering lets the body travel escape_probe_distance_m.
      forward_path_blocked_ =
          result.corner_blocked ||
          !any_forward_arc_clear(scan_, follower_.config().laser_yaw_offset_rad,
                                 VEHICLE_PARAMS.steering.max_angle_rad, escape_probe_distance_m_,
                                 probe_geometry_, probe_points_);
    }
    // Once per scan, not per control cycle: the window spans N scans (window 1 = this scan).
    target_range_m_ = target_range_median_.push(result.target_range_m);
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
      // safety_node brakes the car during the silence, so ramp from rest on resume, and do
      // not let target ranges from before the gap into the median.
      speed_command_.reset();
      target_range_median_.reset();
      // Nothing is being published: no escape survives the gap, and the last request is none.
      if (escape_) {
        escape_->reset();
      }
      last_published_speed_mps_ = 0.0;
      return;
    }
    if (watchdog_active_) {
      RCLCPP_INFO(this->get_logger(), "gap_follow_node: /scan fresh again; resuming /drive_raw.");
      watchdog_active_ = false;
    }

    const double max_angle = VEHICLE_PARAMS.steering.max_angle_rad;
    const double dt_s = has_last_command_time_ ? control_period_s_ : 0.0;
    has_last_command_time_ = true;
    double steering = 0.0;
    double speed = 0.0;
    bool escaping = false;
    if (escape_) {
      EscapeInput in;
      in.dt_s = dt_s;
      in.requested_speed_mps = last_published_speed_mps_;
      if (has_gated_ && (steady_now - last_gated_steady_).seconds() <= drive_feedback_timeout_s_) {
        in.gated_speed_mps = gated_speed_mps_;
      }
      in.forward_path_blocked = forward_path_blocked_;
      in.forward_steering_rad = latest_.wanted_steering_rad;
      const EscapeOutput e = escape_->update(in);
      if (e.event) {
        log_escape_event(*e.event);
      }
      if (e.active) {
        escaping = true;
        steering = steering_filter_.update(e.steering_rad, dt_s);
        speed = e.speed_mps;
        // The forward speed chain restarts from rest when the escape ends.
        speed_command_.reset();
      }
    }
    if (!escaping) {
      steering = steering_filter_.update(latest_.steering_rad, dt_s);
      // Range speed -> steering slowdown -> speed low-pass -> rate limiter (reactive_speed.hpp).
      speed = speed_command_.update(target_range_m_, steering, dt_s);
    }
    if (!std::isfinite(steering) || !std::isfinite(speed)) {
      RCLCPP_ERROR(this->get_logger(),
                   "gap_follow_node: non-finite command computed; not publishing it.");
      return;
    }

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = clamp_for_float32_publish(steering, -max_angle, max_angle);
    drive_msg.drive.speed =
        clamp_for_float32_publish(speed, VEHICLE_PARAMS.limits.min_velocity_mps,
                                  std::min(speed_command_.config().max_speed_mps,
                                           VEHICLE_PARAMS.limits.global_speed_cap_mps));
    drive_pub_->publish(drive_msg);
    last_published_speed_mps_ = static_cast<double>(drive_msg.drive.speed);
  }

  GapFollower follower_;
  FirstOrderLowPass steering_filter_;
  // Placeholders until the constructor body has declared their parameters.
  ReactiveSpeedCommand speed_command_{ReactiveSpeedConfig{}};
  RollingMedian target_range_median_{1};
  double target_range_m_{0.0};
  double control_period_s_{0.02};
  double scan_timeout_s_{0.3};

  ScanInput scan_;
  GapFollowResult latest_;

  // Reverse escape (empty unless reverse_escape is true).
  std::optional<ReverseEscape> escape_;
  ArcProbeGeometry probe_geometry_;
  std::vector<double> probe_points_;
  double escape_probe_distance_m_{0.3};
  double drive_feedback_timeout_s_{0.2};
  bool forward_path_blocked_{false};
  double last_published_speed_mps_{0.0};
  double gated_speed_mps_{0.0};
  bool has_gated_{false};
  rclcpp::Time last_gated_steady_{0, 0, RCL_STEADY_TIME};
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr gated_sub_;

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
