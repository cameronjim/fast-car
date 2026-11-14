// tracker_node: ROS 2 plumbing around racer_control's pure pursuit core
// (claude-docs/01-roadmap.md task S.2, claude-docs/04-architecture.md).
//
// Subscribes /odom (nav_msgs/Odometry, reliable, depth 10) and publishes /drive_raw
// (ackermann_msgs/AckermannDriveStamped, reliable, depth 10) at a fixed rate
// (`control_rate_hz` param, default 50 Hz). Per claude-docs/04-architecture.md's command
// path, `tracker_node` is one of two possible producers of `/drive_raw`;
// `racer_safety/safety_node` (not yet built) is the SOLE publisher of `/drive` and gates
// `/drive_raw` -> `/drive`. This node never publishes `/drive` directly.
//
// Watchdog behavior on /odom silence (claude-docs/12-testing.md L3 checklist item): this
// node STOPS PUBLISHING `/drive_raw` rather than inventing its own brake/zero-speed
// command. Rationale (documented, per task S.2's instruction to pick a degradation
// behavior and state it): claude-docs/04-architecture.md's degradation table assigns the
// staleness watchdog to `safety_node` ("missing /drive_raw for 3 cycles -> brake command"),
// and claude-docs/05-safety.md makes `safety_node` the sole authority for what the vehicle
// does when inputs go bad ("fails CLOSED... brake command, not passthrough"). tracker_node
// is not a safety layer (04-architecture layers 1-4 do not include it); if it kept
// publishing a self-invented "safe" command during odom silence, that command could be
// stale/wrong in a way `safety_node` never sees, and duplicating brake-on-staleness logic
// in two places is exactly the kind of divergence CLAUDE.md invariant 2 warns against for
// physical constants and claude-docs/05-safety.md warns against for safety logic generally.
// Silence in `/drive_raw` is itself the signal `safety_node` acts on.
//
// Speed rate limiting (roadmap milestone 3, found while building
// tests/e2e_sim_safety/test_sim_autopilot_e2e.py): the raceline's target-speed profile
// (tools/raceline) is a function of arc length, not time, so commanding it raw at the
// vehicle's nearest point -- with no ramp of tracker_node's own -- continuously trips
// `racer_safety::SafetyGateLogic`'s rate-limit gate in ordinary driving, not just at
// startup. `speed_limiter_` (racer_control::SpeedRateLimiter) rate-limits the raw command to
// a MARGIN below the `vehicle_params.actuation.max_acceleration_mps2` bound `safety_node`
// independently enforces (`speed_rate_limit_margin_fraction`, see
// `build_speed_rate_limiter()`'s own comment for why a margin -- not the exact bound -- is
// needed against a redundant, independently-clocked safety check) before publishing, so a
// nominal run keeps `safety_node`'s gate a true no-op bystander rather than silently
// absorbing a raw, un-ramped command every cycle. See
// `include/racer_control/speed_rate_limiter.hpp` for why this is a separate class from
// `PurePursuitController::compute_command` rather than built into it (that shared core is
// cross-language-divergence-tested against training/racer_train's Python port).
//
// CLOCK POLICY (read before changing any time arithmetic below). Two clocks, deliberately:
//
//   * `steady_clock_` (RCL_STEADY_TIME, the OS monotonic clock) is the ONLY clock used to
//     MEASURE ELAPSED TIME -- here, the /odom staleness watchdog.
//   * `this->now()` (the node's ROS clock) is used ONLY to STAMP the outgoing /drive_raw
//     header, where a bag-consistent ROS timestamp is what consumers want.
//
// Measuring staleness on the ROS clock is unsafe in both of its modes. With
// `use_sim_time:=false` it is CLOCK_REALTIME, which NTP, a VM resync, or an operator
// running `date` can step backwards -- making the elapsed time negative, hence never
// greater than `odom_timeout_s_`, hence "fresh": this node would keep publishing commands
// computed from a FROZEN, arbitrarily old pose for the whole duration of the step. With
// `use_sim_time:=true` and no /clock publisher, the ROS clock is pinned at zero and the
// measured age is permanently 0.0, disabling the watchdog outright. Same root cause as
// GitHub issue #22 in racer_safety (see racer_safety/src/safety_node.cpp's CLOCK POLICY);
// fixed the same way. See test/test_tracker_node_clock_launch.py.
//
// Float32 wire-precision margin (same milestone, same investigation): steering/speed are
// published as float32 (ackermann_msgs/AckermannDriveStamped), but computed/checked in
// double precision on both ends -- a value sitting exactly at a bound in double precision
// can round the wrong way across that cast, which `safety_node`'s own bounds check then
// (correctly, given the float32 value it actually received) treats as a bounds violation.
// `racer_control::clamp_for_float32_publish` (`include/racer_control/float32_wire_margin.hpp`)
// keeps the PUBLISHED value a small, fixed margin inside the real bound; it does not change
// what `PurePursuitController` computed internally.
//
// POSE SOURCE (2026-11-03 localisation, docs/notes/localization-2026-11-03.md). The raceline is
// written in the map frame. Two parameters, defaults unchanged:
//
//   * `odom_topic` (default /odom): the Odometry this node follows. The sim canaries that drive
//     on ground truth point it at /sim/ground_truth_odom instead of remapping.
//   * `pose_frame` (default empty): empty uses the Odometry's pose as is (ground truth is
//     already in map). Set to "map" when following the EKF's /odom, whose pose is in the odom
//     frame: the node then looks up `pose_frame` -> <the Odometry's header.frame_id> (map ->
//     odom, published by AMCL or slam_toolbox) from TF and composes it with the /odom pose
//     (racer_control::compose). The lookup runs on its own timer and caches plain doubles, so
//     the 50 Hz control callback stays allocation-free (claude-docs/10-conventions.md).
//
// Why compose here rather than a separate node turning /pose + /odom into a map-frame
// Odometry: AMCL's /pose only comes out when the filter updates (every 0.1 m of travel) and
// carries the filter's latency, while its map -> odom transform is republished on every scan
// and the EKF's odom -> base_link runs at 100 Hz. Composing the latest map -> odom with the
// latest /odom is exactly the map-frame pose nav2's own controllers use, at the EKF's rate, with
// no extra process or topic. The /odom watchdog above is unchanged and still the freshness gate;
// on top of it, a map -> odom whose stamp is more than `pose_frame_timeout_s` older than the
// /odom sample it is composed with (both are data stamps on the same ROS clock, so no elapsed
// time is measured on it) counts as stale and stops /drive_raw the same way.
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_control/float32_wire_margin.hpp"
#include "racer_control/pose2d.hpp"
#include "racer_control/pure_pursuit.hpp"
#include "racer_control/raceline.hpp"
#include "racer_control/speed_rate_limiter.hpp"

namespace racer_control {

class TrackerNode : public rclcpp::Node {
 public:
  TrackerNode()
      : Node("tracker_node"),
        raceline_(load_raceline()),
        controller_(build_controller_config()),
        speed_limiter_(build_speed_rate_limiter()),
        has_odom_(false),
        watchdog_active_(false),
        has_last_command_time_(false) {
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    rcl_interfaces::msg::ParameterDescriptor odom_topic_descriptor;
    odom_topic_descriptor.description =
        "Odometry to follow. Default /odom (the EKF). See this file's POSE SOURCE comment.";
    const std::string odom_topic =
        this->declare_parameter<std::string>("odom_topic", "/odom", odom_topic_descriptor);
    rcl_interfaces::msg::ParameterDescriptor pose_frame_descriptor;
    pose_frame_descriptor.description =
        "Empty (default): use the Odometry's pose as is. Otherwise the raceline's frame (map): "
        "the pose is TF pose_frame -> <Odometry header.frame_id> composed with the Odometry "
        "pose. See this file's POSE SOURCE comment.";
    pose_frame_ = this->declare_parameter<std::string>("pose_frame", "", pose_frame_descriptor);
    pose_frame_timeout_s_ = declare_positive_double(
        "pose_frame_timeout_s", 1.0,
        "With pose_frame set: a pose_frame -> odom transform stamped more than this before the "
        "/odom sample counts as stale and stops /drive_raw. AMCL republishes it every scan "
        "(10 Hz) and future-dates it by its transform_tolerance.");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, command_qos, std::bind(&TrackerNode::on_odom, this, std::placeholders::_1));
    rcl_interfaces::msg::ParameterDescriptor drive_topic_descriptor;
    drive_topic_descriptor.description =
        "Where the command goes. /drive_raw (default): straight to safety_node. A policy "
        "deployment (racer_policy/policy_node, roadmap 5.2) sets /tracker/command: this node "
        "then HANDS OFF its command to policy_node, which adds a bounded residual and is the "
        "only publisher of /drive_raw. Nothing else about the command changes.";
    const std::string drive_topic =
        this->declare_parameter<std::string>("drive_topic", "/drive_raw", drive_topic_descriptor);
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,
                                                                                    command_qos);

    const double control_rate_hz =
        declare_positive_double("control_rate_hz", 50.0, "Control loop / /drive_raw publish rate.");
    control_period_s_ = 1.0 / control_rate_hz;
    odom_timeout_s_ = declare_positive_double(
        "odom_timeout_s", 0.3,
        "Watchdog: stop publishing /drive_raw if /odom has been silent this long. Default "
        "is a generous ~3x the nominal >=100 Hz /odom period (claude-docs/04-architecture.md), "
        "erring toward not tripping on ordinary jitter.");

    rcl_interfaces::msg::ParameterDescriptor ros_timers_descriptor;
    ros_timers_descriptor.description =
        "false (default, and always on the car): the control and lookup timers are WALL timers. "
        "true: they tick on this node's ROS clock, so with use_sim_time:=true the control loop "
        "follows /clock (the sim canaries, GitHub issue 156, where sim time must not depend on "
        "CPU scheduling). Elapsed-time measurement stays on the steady clock either way.";
    const bool ros_clock_timers =
        this->declare_parameter<bool>("ros_clock_timers", false, ros_timers_descriptor);
    // One factory for both timers: wall (the car) or ROS clock (sim canaries).
    const auto make_timer = [this, ros_clock_timers](
                                std::chrono::nanoseconds period,
                                std::function<void()> callback) -> rclcpp::TimerBase::SharedPtr {
      if (ros_clock_timers) {
        return rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(period),
                                    std::move(callback));
      }
      return this->create_wall_timer(period, std::move(callback));
    };

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = make_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                        std::bind(&TrackerNode::on_timer, this));

    if (!pose_frame_.empty()) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      const double lookup_rate_hz = declare_positive_double(
          "pose_frame_lookup_rate_hz", 25.0,
          "With pose_frame set: how often the pose_frame -> odom transform is looked up.");
      lookup_timer_ = make_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::duration<double>(1.0 / lookup_rate_hz)),
                                 std::bind(&TrackerNode::on_lookup_timer, this));
    }

    RCLCPP_INFO(this->get_logger(),
                "tracker_node up: %zu raceline point(s), %.1f Hz control rate, following %s%s%s, "
                "publishing %s",
                raceline_.size(), control_rate_hz, odom_topic.c_str(),
                pose_frame_.empty() ? "" : " composed into frame ", pose_frame_.c_str(),
                drive_topic.c_str());
  }

 private:
  double declare_positive_double(const std::string& name, double default_value,
                                 const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 1e-6;
    range.to_value = 1e6;
    descriptor.floating_point_range = {range};
    return this->declare_parameter<double>(name, default_value, descriptor);
  }

  Raceline load_raceline() {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Path to a tools/raceline-generated CSV (claude-docs/02-repo-layout.md: "
        "config/tracks/<venue>_<layout>/raceline.csv). Required; the node refuses to start "
        "without a loadable raceline.";
    const std::string raceline_path =
        this->declare_parameter<std::string>("raceline_path", "", descriptor);
    if (raceline_path.empty()) {
      RCLCPP_FATAL(this->get_logger(),
                   "tracker_node: 'raceline_path' parameter is required and was not set. "
                   "Refusing to start with no raceline (claude-docs/05-safety.md: fail closed).");
      throw std::runtime_error("tracker_node: missing required 'raceline_path' parameter");
    }
    try {
      return Raceline::load_from_csv(raceline_path);
    } catch (const RacelineLoadError& e) {
      RCLCPP_FATAL(this->get_logger(), "tracker_node: failed to load raceline from '%s': %s",
                   raceline_path.c_str(), e.what());
      throw;
    }
  }

  PurePursuitConfig build_controller_config() {
    // Physical constants (wheelbase, steering limit) come ONLY from the generated
    // vehicle_params binding (CLAUDE.md invariant 2, claude-docs/06-vehicle-params.md rule
    // 3) -- never hand-typed here. Lookahead tuning gains are genuine tuning parameters
    // (not physical constants), so they are declared ROS params with documented defaults,
    // per this task's explicit allowance.
    rcl_interfaces::msg::ParameterDescriptor lookahead_min_descriptor;
    lookahead_min_descriptor.description =
        "Curvature-adaptive pure pursuit lookahead lower bound (tight turns).";
    const double lookahead_min_m =
        this->declare_parameter<double>("lookahead_min_m", 0.4, lookahead_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor lookahead_max_descriptor;
    lookahead_max_descriptor.description =
        "Curvature-adaptive pure pursuit lookahead upper bound (straights).";
    const double lookahead_max_m =
        this->declare_parameter<double>("lookahead_max_m", 1.5, lookahead_max_descriptor);

    rcl_interfaces::msg::ParameterDescriptor lookahead_ref_descriptor;
    lookahead_ref_descriptor.description =
        "Curvature magnitude (1/m) at which lookahead reaches lookahead_min_m.";
    const double lookahead_curvature_ref_1pm = this->declare_parameter<double>(
        "lookahead_curvature_ref_1pm", 0.4, lookahead_ref_descriptor);

    PurePursuitConfig config;
    config.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
    config.max_steering_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    config.lookahead_min_m = lookahead_min_m;
    config.lookahead_max_m = lookahead_max_m;
    config.lookahead_curvature_ref_1pm = lookahead_curvature_ref_1pm;
    return config;
  }

  SpeedRateLimiter build_speed_rate_limiter() {
    // The physical bound itself (vehicle_params.actuation.max_acceleration_mps2) comes ONLY
    // from the generated binding, same as build_controller_config() above -- never
    // hand-typed. `speed_rate_limit_margin_fraction` is a genuine tuning parameter, not a
    // second copy of that constant: racer_safety::SafetyGateLogic enforces the exact same
    // bound independently, in its own process, with its own independently-measured dt --
    // two separately-clocked control loops riding the EXACT same boundary will disagree by
    // small amounts purely from clock/scheduling jitter between them (found empirically
    // while building tests/e2e_sim_safety/test_sim_autopilot_e2e.py: with no margin, ordinary
    // driving on this raceline's accel-limited speed profile tripped safety_node's
    // rate-limit gate almost every cycle, not from any real disagreement about the physical
    // limit, but from the two nodes' dt measurements differing by a millisecond or so around
    // a boundary the profile is deliberately riding at all times). Keeping tracker_node's own
    // ramp comfortably under the true bound gives safety_node's independent gate headroom to
    // agree in practice, the same way any layered defense-in-depth control should leave
    // margin against a redundant independent check rather than target it exactly.
    rcl_interfaces::msg::ParameterDescriptor margin_descriptor;
    margin_descriptor.description =
        "Fraction of vehicle_params.actuation.max_acceleration_mps2 tracker_node ramps its "
        "own commanded speed toward (see this function's comment for why this margin exists "
        "-- it is not a second physical constant, just headroom against "
        "racer_safety::SafetyGateLogic's independently-clocked copy of the same bound).";
    rcl_interfaces::msg::FloatingPointRange margin_range;
    margin_range.from_value = 0.01;
    margin_range.to_value = 1.0;
    margin_descriptor.floating_point_range = {margin_range};
    const double margin_fraction =
        this->declare_parameter<double>("speed_rate_limit_margin_fraction", 0.5, margin_descriptor);
    return SpeedRateLimiter(VEHICLE_PARAMS.actuation.max_acceleration_mps2 * margin_fraction);
  }

  void on_lookup_timer() {
    // Off the control path: TF lookups allocate. Only plain doubles cross to on_timer().
    if (!has_odom_) {
      return;
    }
    if (odom_frame_id_ != last_odom_.header.frame_id) {
      odom_frame_id_ = last_odom_.header.frame_id;
    }
    try {
      const geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(pose_frame_, odom_frame_id_, tf2::TimePointZero);
      const auto& q = tf.transform.rotation;
      frame_from_odom_.x_m = tf.transform.translation.x;
      frame_from_odom_.y_m = tf.transform.translation.y;
      frame_from_odom_.yaw_rad = yaw_from_quaternion(q.w, q.x, q.y, q.z);
      frame_from_odom_stamp_s_ = rclcpp::Time(tf.header.stamp).seconds();
      has_frame_from_odom_ = true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tracker_node: no %s -> %s transform yet: %s", pose_frame_.c_str(),
                           odom_frame_id_.c_str(), e.what());
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;
    // Steady clock, NOT this->now() -- see this file's CLOCK POLICY comment. Only ever the
    // left operand of an elapsed-time subtraction, never a published stamp.
    last_odom_steady_ = steady_clock_.now();
    has_odom_ = true;
  }

  void on_timer() {
    // `now` stamps the outgoing /drive_raw header; `steady_now` measures /odom staleness.
    const rclcpp::Time now = this->now();
    const rclcpp::Time steady_now = steady_clock_.now();
    const bool odom_stale =
        !has_odom_ || (steady_now - last_odom_steady_).seconds() > odom_timeout_s_;
    if (odom_stale) {
      if (!watchdog_active_) {
        RCLCPP_WARN(this->get_logger(),
                    "tracker_node: /odom stale (> %.3fs); stopping /drive_raw publication "
                    "until fresh odometry arrives (see this file's header comment for why "
                    "this node does not invent its own brake command).",
                    odom_timeout_s_);
        watchdog_active_ = true;
      }
      // Treat the next resumed cycle as a fresh start for speed rate-limiting too (holds at
      // the last commanded speed rather than allowing a large stale-gap dt to justify a
      // sudden jump -- see SpeedRateLimiter's own "no elapsed time -> no increase" fail-safe
      // convention).
      has_last_command_time_ = false;
      return;  // No heap allocation on this path: just an early return.
    }
    if (watchdog_active_) {
      RCLCPP_INFO(this->get_logger(), "tracker_node: /odom fresh again; resuming /drive_raw.");
      watchdog_active_ = false;
    }

    const auto& pose = last_odom_.pose.pose;
    Pose2D vehicle{pose.position.x, pose.position.y,
                   yaw_from_quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z)};
    if (!pose_frame_.empty()) {
      const double odom_stamp_s = rclcpp::Time(last_odom_.header.stamp).seconds();
      const bool frame_stale =
          !has_frame_from_odom_ || odom_stamp_s - frame_from_odom_stamp_s_ > pose_frame_timeout_s_;
      if (frame_stale) {
        if (!pose_frame_stale_) {
          RCLCPP_WARN(this->get_logger(),
                      "tracker_node: %s -> odom transform missing or stale (> %.3fs behind "
                      "/odom); stopping /drive_raw until localisation is back.",
                      pose_frame_.c_str(), pose_frame_timeout_s_);
          pose_frame_stale_ = true;
        }
        has_last_command_time_ = false;
        return;
      }
      if (pose_frame_stale_) {
        RCLCPP_INFO(this->get_logger(), "tracker_node: %s -> odom transform fresh; resuming.",
                    pose_frame_.c_str());
        pose_frame_stale_ = false;
      }
      vehicle = compose(frame_from_odom_, vehicle);
    }
    const PurePursuitCommand cmd =
        controller_.compute_command(raceline_, vehicle.x_m, vehicle.y_m, vehicle.yaw_rad);

    // Rate-limit the raw commanded speed to the vehicle's own acceleration bound before
    // publishing (see racer_control/include/racer_control/speed_rate_limiter.hpp's header
    // comment for why this lives here rather than in compute_command itself): without this,
    // tracker_node's raw /drive_raw speed continuously trips racer_safety::SafetyGateLogic's
    // rate-limit gate in completely ordinary driving, not just at startup, since the
    // raceline's target-speed profile is a function of arc length, not of time.
    //
    // Uses the CONFIGURED control period (control_period_s_), not a measured elapsed time:
    // safety_node evaluates its own copy of this same rate limit on its OWN independent wall
    // timer, so comparing tracker_node's measured dt (subject to this node's own scheduling
    // jitter) against safety_node's measured dt (subject to ITS scheduling jitter) means two
    // independently-noisy numbers get compared where the true physical bound is meant to be
    // deterministic -- found empirically while building
    // tests/e2e_sim_safety/test_sim_autopilot_e2e.py (real elapsed-time dt here produced
    // noisy, occasionally large safety_node rate-limit event counts on an otherwise nominal
    // run, purely from cross-process jitter, not a real disagreement). Anchoring to the fixed
    // configured period (paired with `speed_rate_limit_margin_fraction`'s headroom below the
    // true accel bound) makes tracker_node's own per-cycle speed increase a known constant
    // instead of a second noisy measurement.
    const double dt_s = has_last_command_time_ ? control_period_s_ : 0.0;
    has_last_command_time_ = true;
    const double rate_limited_speed_mps = speed_limiter_.limit(cmd.speed_mps, dt_s);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle =
        clamp_for_float32_publish(cmd.steering_angle_rad, -VEHICLE_PARAMS.steering.max_angle_rad,
                                  VEHICLE_PARAMS.steering.max_angle_rad);
    drive_msg.drive.speed =
        clamp_for_float32_publish(rate_limited_speed_mps, VEHICLE_PARAMS.limits.min_velocity_mps,
                                  VEHICLE_PARAMS.limits.global_speed_cap_mps);
    drive_pub_->publish(drive_msg);
  }

  Raceline raceline_;
  PurePursuitController controller_;
  SpeedRateLimiter speed_limiter_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry last_odom_;
  // Monotonic clock for elapsed-time measurement only (CLOCK POLICY, top of file).
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_odom_steady_{0, 0, RCL_STEADY_TIME};
  bool has_odom_;
  bool watchdog_active_;
  double odom_timeout_s_{0.3};

  double control_period_s_{0.02};
  bool has_last_command_time_;

  // POSE SOURCE (see the top of this file). Empty pose_frame_ = none of this is used.
  std::string pose_frame_;
  double pose_frame_timeout_s_{1.0};
  std::string odom_frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr lookup_timer_;
  Pose2D frame_from_odom_;
  double frame_from_odom_stamp_s_{0.0};
  bool has_frame_from_odom_{false};
  bool pose_frame_stale_{false};
};

}  // namespace racer_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_control::TrackerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("tracker_node"), "tracker_node: fatal startup error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
