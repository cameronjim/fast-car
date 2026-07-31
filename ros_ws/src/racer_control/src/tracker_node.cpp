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
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include <vehicle_params_generated.hpp>

#include "racer_control/pure_pursuit.hpp"
#include "racer_control/raceline.hpp"

namespace racer_control {

class TrackerNode : public rclcpp::Node {
 public:
  TrackerNode()
      : Node("tracker_node"),
        raceline_(load_raceline()),
        controller_(build_controller_config()),
        has_odom_(false),
        watchdog_active_(false) {
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", command_qos, std::bind(&TrackerNode::on_odom, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive_raw", command_qos);

    const double control_rate_hz = declare_positive_double(
        "control_rate_hz", 50.0, "Control loop / /drive_raw publish rate.");
    odom_timeout_s_ = declare_positive_double(
        "odom_timeout_s", 0.3,
        "Watchdog: stop publishing /drive_raw if /odom has been silent this long. Default "
        "is a generous ~3x the nominal >=100 Hz /odom period (claude-docs/04-architecture.md), "
        "erring toward not tripping on ordinary jitter.");

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TrackerNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "tracker_node up: %zu raceline point(s), %.1f Hz control rate",
                raceline_.size(), control_rate_hz);
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

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;
    last_odom_stamp_ = this->now();
    has_odom_ = true;
  }

  void on_timer() {
    const rclcpp::Time now = this->now();
    const bool odom_stale =
        !has_odom_ || (now - last_odom_stamp_).seconds() > odom_timeout_s_;
    if (odom_stale) {
      if (!watchdog_active_) {
        RCLCPP_WARN(this->get_logger(),
                    "tracker_node: /odom stale (> %.3fs); stopping /drive_raw publication "
                    "until fresh odometry arrives (see this file's header comment for why "
                    "this node does not invent its own brake command).",
                    odom_timeout_s_);
        watchdog_active_ = true;
      }
      return;  // No heap allocation on this path: just an early return.
    }
    if (watchdog_active_) {
      RCLCPP_INFO(this->get_logger(), "tracker_node: /odom fresh again; resuming /drive_raw.");
      watchdog_active_ = false;
    }

    const auto& pose = last_odom_.pose.pose;
    const double yaw = yaw_from_quaternion(pose.orientation.w, pose.orientation.x,
                                           pose.orientation.y, pose.orientation.z);
    const PurePursuitCommand cmd =
        controller_.compute_command(raceline_, pose.position.x, pose.position.y, yaw);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = static_cast<float>(cmd.steering_angle_rad);
    drive_msg.drive.speed = static_cast<float>(cmd.speed_mps);
    drive_pub_->publish(drive_msg);
  }

  Raceline raceline_;
  PurePursuitController controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_odom_stamp_;
  bool has_odom_;
  bool watchdog_active_;
  double odom_timeout_s_{0.3};
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
