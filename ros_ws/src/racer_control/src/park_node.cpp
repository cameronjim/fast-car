// park_node: ROS 2 plumbing around racer_control's low-speed manoeuvre core (roadmap 2.9): a
// LiDAR-detected parallel park and a three-point turn, executed from wheel odometry. The logic
// is ROS-free in include/racer_control/park_{geometry,slot,planner,controller}.hpp; read
// park_controller.hpp's header before changing anything here.
//
// THIS NODE DRIVES THE CAR, INCLUDING IN REVERSE, ON ITS OWN once ~/start or ~/three_point_turn
// is called. It starts in IDLE and publishes nothing until then.
//
// Services (std_srvs/srv/Trigger, under the node's name):
//   ~/start             parallel park: SEARCH along the row on `park_side`, park in the first
//                       feasible slot. `ros2 service call /park_node/start std_srvs/srv/Trigger`
//   ~/three_point_turn  turn round in the lane the car stands in (lane from the current /scan).
//   ~/abort             ABORT whatever is running: zero speed for final_hold_s, then silence.
// The response carries success and the reason; refusals (no odometry with require_odometry,
// already running, lane too narrow, ...) are also logged.
//
// Topics (claude-docs/04-architecture.md; nothing improvised on the command path):
//   * subscribes /scan (sensor_msgs/LaserScan, KeepLast(10) best_effort, like every /scan
//     consumer), /odom/wheel (nav_msgs/Odometry, KeepLast(10) reliable, matching
//     racer_drivers/vesc_odometry_node: pose.position.x is the signed along-track distance,
//     twist.linear.x the wheel speed) and the GATED /drive (ackermann_msgs/AckermannDriveStamped,
//     KeepLast(10) reliable, read only: the speed safety_node actually let through, park_controller
//     "BLOCKED").
//   * publishes /drive_raw (ackermann_msgs/AckermannDriveStamped, KeepLast(10) reliable) at
//     control_rate_hz while a manoeuvre runs, then zero for final_hold_s after DONE or ABORT, then
//     nothing (safety_node brakes on the silence). NEVER /drive: safety_node is its sole publisher
//     (CLAUDE.md invariant 1).
//   * publishes ~/status (std_msgs/String, KeepLast(10) reliable): one line of key=value pairs
//     (phase, stage, mode, side, odom_fallback, dead-reckoned pose, request) at status_rate_hz and
//     on every transition. A diagnostic, not a command interface.
//
// Planning runs OFF the control thread: a confirmed slot's plan (park_planner.hpp, tens of
// milliseconds on the Jetson) is computed with std::async while the timer keeps publishing the
// SEARCH command at 50 Hz, so safety_node's 3-cycle /drive_raw watchdog never sees a gap. The
// three-point turn plans inside its service call, while nothing is being published.
//
// CLOCK POLICY (same as gap_follow_node.cpp / tracker_node.cpp): the steady clock measures every
// staleness (/scan, /odom/wheel, /drive); this->now() only stamps /drive_raw; the controller
// advances by the configured control period per cycle.
//
// Float32 wire margin: published values go through clamp_for_float32_publish, as in the other
// controllers. Physical constants come only from the generated vehicle_params binding (CLAUDE.md
// invariant 2): wheelbase, width, length, both overhangs, steering limit and rate, acceleration,
// speed limits, LiDAR mount and yaw, and the safety gate's corridor, floor, TTC and horizon for
// the plan's gate prediction. laser_yaw_from_vehicle_params false is for synthetic-scan tests
// only, exactly as in gap_follow_node.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_control/float32_wire_margin.hpp"
#include "racer_control/laser_scan.hpp"
#include "racer_control/park_controller.hpp"
#include "reactive_node_params.hpp"

namespace racer_control {

namespace {

std::string declare_described_string(rclcpp::Node& node, const std::string& name,
                                     const std::string& default_value,
                                     const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return node.declare_parameter<std::string>(name, default_value, descriptor);
}

}  // namespace

class ParkNode : public rclcpp::Node {
 public:
  ParkNode() : Node("park_node"), controller_(build_config()) {
    const rclcpp::QoS scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&ParkNode::on_scan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom/wheel", reliable_qos, std::bind(&ParkNode::on_odom, this, std::placeholders::_1));
    // The GATED command, read only. Never published here (invariant 1).
    gated_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", reliable_qos, std::bind(&ParkNode::on_gated, this, std::placeholders::_1));
    drive_pub_ =
        create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_raw", reliable_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", reliable_qos);
    using Trigger = std_srvs::srv::Trigger;
    start_srv_ =
        create_service<Trigger>("~/start", [this](const std::shared_ptr<Trigger::Request>,
                                                  std::shared_ptr<Trigger::Response> response) {
          respond(controller_.start_parallel_park(odometry_fresh()), *response, "start");
        });
    turn_srv_ = create_service<Trigger>(
        "~/three_point_turn", [this](const std::shared_ptr<Trigger::Request>,
                                     std::shared_ptr<Trigger::Response> response) {
          respond(controller_.start_three_point_turn(scan_, scan_fresh(), odometry_fresh()),
                  *response, "three_point_turn");
        });
    abort_srv_ =
        create_service<Trigger>("~/abort", [this](const std::shared_ptr<Trigger::Request>,
                                                  std::shared_ptr<Trigger::Response> response) {
          respond(controller_.abort("~/abort service"), *response, "abort");
        });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(control_period_s_)),
                               std::bind(&ParkNode::on_timer, this));
    status_period_cycles_ = std::max<long>(1, std::lround(control_rate_hz_ / status_rate_hz_));

    const ParkControllerConfig& c = controller_.config();
    const ParkPlannerConfig& p = c.planner;
    const double delta = p.steering_fraction * p.max_steering_rad;
    RCLCPP_INFO(get_logger(),
                "park_node up: %.1f Hz, side %s, search %.2f m/s, park %.2f m/s, steering %.4f "
                "rad (%.2f of %.4f; R %.4f m), margin %.3f m, slot at least %.3f m long and %.3f "
                "m deep, settle %.2f s, blocked abort %.2f s, final forward up to %.3f m, "
                "require_odometry %s, gate prediction %s (margin %.3f m), laser yaw %.4f rad, "
                "LiDAR at x %.3f y %.3f m, three-point turn to the %s",
                control_rate_hz_, park_side_name(c.detector.side), c.search_speed_mps,
                c.park_speed_mps, delta, p.steering_fraction, p.max_steering_rad,
                p.body.wheelbase_m / std::tan(delta), p.margin_m, c.detector.length_min_m,
                c.detector.depth_min_m, c.settle_s, c.blocked_abort_s, p.final_forward_max_m,
                c.require_odometry ? "true" : "false", p.gate.enabled ? "on" : "off",
                p.gate.margin_m, c.detector.laser_yaw_offset_rad, c.detector.lidar_mount_x_m,
                c.detector.lidar_mount_y_m, turn_direction_name(c.turn_direction));
    RCLCPP_WARN(get_logger(),
                "park_node: THIS NODE DRIVES THE CAR, INCLUDING IN REVERSE, ON ITS OWN once "
                "~/start or ~/three_point_turn is called. Clear the space around the car and keep "
                "the kill switch in a second person's hand. Idle until then.");
  }

  ~ParkNode() override {
    if (plan_future_.valid()) {
      plan_future_.wait();
    }
  }

 private:
  ParkControllerConfig build_config() {
    ParkControllerConfig c;
    const double control_rate_hz = declare_ranged_double(
        *this, "control_rate_hz", 50.0, 1.0, 1000.0,
        "/drive_raw publish rate (Hz); claude-docs/04-architecture.md specifies 50 Hz.");
    control_rate_hz_ = control_rate_hz;
    control_period_s_ = 1.0 / control_rate_hz;
    c.control_period_s = control_period_s_;
    status_rate_hz_ = declare_ranged_double(*this, "status_rate_hz", 2.0, 0.1, 50.0,
                                            "~/status publish rate (Hz), plus every transition.");
    scan_timeout_s_ = declare_ranged_double(
        *this, "scan_timeout_s", 0.3, 1e-3, 10.0,
        "/scan counts as fresh for this long (s). SEARCH aborts on a stale scan.");
    odom_timeout_s_ = declare_ranged_double(
        *this, "odom_timeout_s", 0.2, 1e-3, 10.0,
        "/odom/wheel counts as fresh for this long (s); 10 cycles of the 50 Hz driver.");
    drive_feedback_timeout_s_ = declare_ranged_double(
        *this, "drive_feedback_timeout_s", 0.2, 1e-3, 10.0,
        "The gated /drive counts as fresh for this long (s); stale counts as blocked.");

    const auto& vp = VEHICLE_PARAMS;
    if (!vp.sensors.lidar.mount_x_m.has_value() || !vp.sensors.lidar.mount_y_m.has_value()) {
      throw std::invalid_argument(
          "park_node: vehicle_params sensors.lidar.mount_x_m / mount_y_m is null; slot detection "
          "needs the LiDAR position relative to the rear axle. Refusing to start");
    }
    const double yaw_param = declare_ranged_double(
        *this, "laser_yaw_offset_rad", 0.0, -M_PI, M_PI,
        "LiDAR mounting yaw (rad), only used as in gap_follow_node: while vehicle_params "
        "sensors.lidar.mount_yaw_rad is null, or with laser_yaw_from_vehicle_params false.");
    const bool yaw_from_params = declare_described_bool(
        *this, "laser_yaw_from_vehicle_params", true,
        "true (default, the real car): LiDAR yaw from vehicle_params. false: SYNTHETIC-SCAN TESTS "
        "ONLY, use laser_yaw_offset_rad as given.");
    const auto yaw =
        resolve_laser_yaw_offset(vp.sensors.lidar.mount_yaw_rad, yaw_param, yaw_from_params);
    if (!yaw) {
      throw std::invalid_argument(
          "park_node: laser_yaw_offset_rad disagrees with vehicle_params "
          "sensors.lidar.mount_yaw_rad with laser_yaw_from_vehicle_params true (or is not "
          "finite); refusing to start");
    }
    if (!yaw_from_params) {
      RCLCPP_INFO(get_logger(),
                  "park_node: laser_yaw_from_vehicle_params is false, using laser_yaw_offset_rad "
                  "= %.6f rad (synthetic-scan fixture only)",
                  yaw_param);
    }

    // The car (CLAUDE.md invariant 2: the binding only).
    ParkBody body;
    body.wheelbase_m = vp.chassis.wheelbase_m;
    body.half_width_m = vp.chassis.width_m / 2.0;
    body.front_x_m = vp.chassis.wheelbase_m + vp.chassis.front_overhang_m;
    body.rear_x_m = vp.chassis.rear_overhang_m;
    body.lidar_x_m = *vp.sensors.lidar.mount_x_m;
    body.lidar_y_m = *vp.sensors.lidar.mount_y_m;

    const std::string side = declare_described_string(
        *this, "park_side", "right", "Side of the row to park on: right (default) or left.");
    if (side != "right" && side != "left") {
      throw std::invalid_argument("park_node: park_side must be right or left, got " + side);
    }
    const std::string turn = declare_described_string(
        *this, "turn_direction", "left", "Three-point turn direction: left (default) or right.");
    if (turn != "right" && turn != "left") {
      throw std::invalid_argument("park_node: turn_direction must be right or left, got " + turn);
    }
    c.turn_direction = turn == "left" ? TurnDirection::kLeft : TurnDirection::kRight;

    c.search_speed_mps = declare_ranged_double(
        *this, "search_speed_mps", 0.5, 0.05, std::min(2.0, vp.limits.global_speed_cap_mps),
        "SEARCH and every forward move (m/s). Keep it at or above about 0.44 m/s on the car: the "
        "VESC speed loop does nothing below its s_pid_min_erpm.");
    c.park_speed_mps = declare_ranged_double(*this, "park_speed_mps", -0.5,
                                             std::max(-2.0, vp.limits.min_velocity_mps), -0.05,
                                             "Every reverse move (m/s, negative).");
    const double fraction = declare_ranged_double(
        *this, "park_steering_fraction", 0.95, 0.1, 1.0,
        "Arc steering as a fraction of steering.max_angle_rad (margin short of the stop).");
    const double margin = declare_ranged_double(
        *this, "margin_m", 0.1, 0.0, 0.5,
        "Clearance the planned swept body keeps from every obstacle segment, and the parked car's "
        "near side from the row line (m).");
    const double length_margin =
        declare_ranged_double(*this, "slot_length_margin_m", 0.35, 0.0, 3.0,
                              "Default slot_length_min_m is chassis.length_m + this (m).");
    const double length_min = declare_ranged_double(
        *this, "slot_length_min_m", vp.chassis.length_m + length_margin, 0.1, 10.0,
        "A pocket shorter than this is not a slot (m). The plan decides whether it is long "
        "enough; this is the detector's floor.");
    const double depth_min = declare_ranged_double(
        *this, "slot_depth_min_m", vp.chassis.width_m + 2.0 * margin, 0.05, 5.0,
        "A pocket shallower than this (beyond the row face) is not a slot (m). Default "
        "chassis.width_m + 2 * margin_m.");
    c.settle_s = declare_ranged_double(
        *this, "settle_s", 0.3, 0.0, 5.0,
        "Standstill after every steering change before moving, so the servo finishes first (s).");
    c.blocked_abort_s = declare_ranged_double(
        *this, "blocked_abort_s", 2.0, 0.1, 30.0,
        "ABORT when safety_node has held a nonzero request at zero (or /drive is stale) this long "
        "(s).");
    const double final_forward =
        declare_ranged_double(*this, "final_forward_m", 0.15, 0.0, 1.0,
                              "Largest forward centring move after the arcs (m); 0 disables it.");
    c.require_odometry = declare_described_bool(
        *this, "require_odometry", true,
        "true (default; the real car): refuse to start, and abort a run, without fresh "
        "/odom/wheel. false (sim fixtures): fall back to integrating the gated /drive speed, WARN "
        "once per second, odom_fallback=true in ~/status.");
    // Both ramps leave room for TWO of this node's 50 Hz steps landing in one of safety_node's
    // independently clocked cycles (seen in the L5 canary at 0.5: rate_limit interventions).
    const double accel_fraction = declare_ranged_double(
        *this, "speed_rate_limit_margin_fraction", 0.25, 0.01, 1.0,
        "Fraction of actuation.max_acceleration_mps2 the speed request ramps at (headroom against "
        "safety_node's own rate limit).");
    c.acceleration_mps2 = accel_fraction * vp.actuation.max_acceleration_mps2;
    const double steer_fraction = declare_ranged_double(
        *this, "steering_rate_margin_fraction", 0.35, 0.01, 1.0,
        "Fraction of steering.max_rate_rad_per_s the steering request ramps at while stopped.");
    c.steering_rate_rad_per_s = steer_fraction * vp.steering.max_rate_rad_per_s;
    c.stopped_speed_mps = declare_ranged_double(
        *this, "stopped_speed_mps", 0.05, 0.001, 1.0,
        "The car counts as stopped below this wheel speed on two consecutive cycles (m/s).");
    c.stop_timeout_s = declare_ranged_double(*this, "stop_timeout_s", 3.0, 0.1, 30.0,
                                             "ABORT if the car has not stopped after this (s).");
    c.max_stop_lead_m = declare_ranged_double(
        *this, "max_stop_lead_m", 0.15, 0.0, 1.0,
        "Upper bound on the learnt stop lead (m), see park_controller.hpp STOP LEAD.");
    c.initial_stop_lead_m =
        declare_ranged_double(*this, "initial_stop_lead_m", 0.0, 0.0, 1.0,
                              "Stop lead before any stop has been measured (m).");
    c.search_max_distance_m =
        declare_ranged_double(*this, "search_max_distance_m", 6.0, 0.1, 100.0,
                              "ABORT when SEARCH has driven this far without a feasible slot (m).");
    c.slot_confirm_scans = static_cast<int>(
        declare_ranged_int(*this, "slot_confirm_scans", 3, 1, 20,
                           "Consecutive scans that must agree on a slot before it is planned."));
    c.slot_confirm_tolerance_m =
        declare_ranged_double(*this, "slot_confirm_tolerance_m", 0.1, 0.005, 1.0,
                              "How far the confirming scans' edges may disagree (m).");
    c.final_hold_s = declare_ranged_double(
        *this, "final_hold_s", 1.0, 0.0, 10.0,
        "Zero speed is published this long after DONE or ABORT, then nothing (s).");
    c.lane_lateral_max_m = declare_ranged_double(
        *this, "lane_lateral_max_m", 3.0, 0.3, 12.0,
        "Three-point turn: lane walls are looked for this far either side (m).");

    SlotDetectorConfig& d = c.detector;
    d.side = side == "left" ? ParkSide::kLeft : ParkSide::kRight;
    d.laser_yaw_offset_rad = *yaw;
    d.lidar_mount_x_m = body.lidar_x_m;
    d.lidar_mount_y_m = body.lidar_y_m;
    d.lateral_min_m = body.half_width_m;
    d.lateral_max_m =
        declare_ranged_double(*this, "lateral_max_m", 2.0, 0.3, 12.0,
                              "Slot detection band: returns up to this far to the side count (m).");
    d.window_back_m = declare_ranged_double(*this, "window_back_m", 1.5, 0.0, 10.0,
                                            "Slot detection window behind the rear axle (m).");
    d.window_ahead_m = declare_ranged_double(*this, "window_ahead_m", 2.5, 0.1, 10.0,
                                             "Slot detection window ahead of the rear axle (m).");
    d.bin_m = declare_ranged_double(*this, "bin_m", 0.05, 0.005, 0.5,
                                    "Slot detection bin length along the row (m).");
    d.jump_min_m = declare_ranged_double(
        *this, "jump_min_m", 0.1, 0.01, 2.0,
        "A pocket candidate is at least this far beyond the row face (m); a shallower one is "
        "not considered at all, a deeper one shallower than slot_depth_min_m is 'too shallow'.");
    d.max_row_angle_rad = declare_ranged_double(
        *this, "max_row_angle_rad", 0.087, 0.001, 0.5,
        "Refuse a scan whose row is more than this off the car's heading (rad).");
    d.depth_min_m = depth_min;
    d.length_min_m = length_min;

    ParkPlannerConfig& p = c.planner;
    p.body = body;
    p.max_steering_rad = vp.steering.max_angle_rad;
    p.steering_fraction = fraction;
    p.margin_m = margin;
    p.final_forward_max_m = final_forward;
    p.forward_speed_mps = c.search_speed_mps;
    p.reverse_speed_mps = -c.park_speed_mps;
    p.obstacle_extent_m = declare_ranged_double(
        *this, "obstacle_extent_m", 2.0, 0.1, 20.0,
        "The planner extends the row faces and walls this far beyond what it checks (m).");
    p.gate.enabled = declare_described_bool(
        *this, "gate_prediction", true,
        "Reject plans where safety_node's obstacle gate would brake the car mid-manoeuvre "
        "(park_geometry.hpp GATE PREDICTION).");
    p.gate.margin_m =
        declare_ranged_double(*this, "gate_margin_m", 0.03, 0.0, 1.0,
                              "Headroom over the gate's brake distance in that prediction (m).");
    p.gate.corridor_half_width_m = vp.chassis.width_m / 2.0 + vp.limits.obstacle_corridor_margin_m;
    p.gate.sector_half_angle_rad = vp.limits.ttc_forward_sector_half_angle_rad;
    p.gate.outer_corner_horizon_m = vp.limits.outer_corner_horizon_m;
    p.gate.max_steering_rad = vp.steering.max_angle_rad;
    p.gate.floor_m = vp.limits.min_forward_clearance_m;
    p.gate.ttc_brake_s = vp.limits.ttc_brake_s.has_value() ? *vp.limits.ttc_brake_s : 0.0;
    c.synchronous_planning = false;
    return c;
  }

  void respond(const StartResult& result, std_srvs::srv::Trigger::Response& response,
               const char* service) {
    response.success = result.ok;
    response.message = result.message;
    if (result.ok) {
      RCLCPP_INFO(get_logger(), "park_node: ~/%s: %s", service, result.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "park_node: ~/%s refused: %s", service, result.message.c_str());
    }
    flush_logs();
  }

  bool fresh(bool has, const rclcpp::Time& last, double timeout_s) {
    return has && (steady_clock_.now() - last).seconds() <= timeout_s;
  }
  bool scan_fresh() { return fresh(has_scan_, last_scan_steady_, scan_timeout_s_); }
  bool odometry_fresh() { return fresh(has_odom_, last_odom_steady_, odom_timeout_s_); }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_.ranges.assign(msg->ranges.begin(), msg->ranges.end());
    scan_.angle_min = msg->angle_min;
    scan_.angle_increment = msg->angle_increment;
    scan_.range_min = msg->range_min;
    scan_.range_max = msg->range_max;
    // Steady clock, never this->now(): see CLOCK POLICY.
    last_scan_steady_ = steady_clock_.now();
    has_scan_ = true;
    controller_.on_scan(scan_);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double distance = msg->pose.pose.position.x;
    const double speed = msg->twist.twist.linear.x;
    if (!std::isfinite(distance) || !std::isfinite(speed)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "park_node: non-finite /odom/wheel ignored");
      return;
    }
    odometry_ = ParkOdometry{distance, speed};
    last_odom_steady_ = steady_clock_.now();
    has_odom_ = true;
  }

  void on_gated(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    gated_speed_mps_ = static_cast<double>(msg->drive.speed);
    last_gated_steady_ = steady_clock_.now();
    has_gated_ = true;
  }

  void flush_logs() {
    bool any = false;
    for (const ParkLogLine& line : controller_.take_logs()) {
      any = true;
      if (line.warn) {
        RCLCPP_WARN(get_logger(), "park_node: %s", line.text.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "park_node: %s", line.text.c_str());
      }
    }
    if (any) {
      publish_status();
    }
  }

  void publish_status() {
    std_msgs::msg::String msg;
    msg.data = controller_.status_text();
    status_pub_->publish(msg);
  }

  void poll_planning() {
    if (plan_future_.valid()) {
      if (plan_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const ParallelParkPlan plan = plan_future_.get();
        controller_.deliver_plan(plan_request_id_, plan);
      }
      return;
    }
    if (auto request = controller_.take_plan_request()) {
      plan_request_id_ = request->id;
      const ParkPlannerConfig config = controller_.config().planner;
      plan_future_ = std::async(std::launch::async, [request, config]() {
        return plan_parallel_park(request->slot, request->car, config);
      });
    }
  }

  void on_timer() {
    poll_planning();
    ParkStepInput in;
    if (odometry_fresh()) {
      in.odometry = odometry_;
    }
    if (fresh(has_gated_, last_gated_steady_, drive_feedback_timeout_s_)) {
      in.gated_speed_mps = gated_speed_mps_;
    }
    in.scan_fresh = scan_fresh();
    const ParkOutput out = controller_.step(in);
    flush_logs();
    if (controller_.odom_fallback() && controller_.active()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "park_node: ODOMETRY FALLBACK: no fresh /odom/wheel, integrating the "
                           "commanded speed (require_odometry false). Distances WILL be wrong on "
                           "the car.");
    }
    if (++cycles_ % status_period_cycles_ == 0) {
      publish_status();
    }
    if (!out.publish) {
      return;
    }
    if (!std::isfinite(out.speed_mps) || !std::isfinite(out.steering_rad)) {
      RCLCPP_ERROR(get_logger(), "park_node: non-finite command computed; not publishing it.");
      return;
    }
    const auto& vp = VEHICLE_PARAMS;
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.drive.steering_angle = clamp_for_float32_publish(
        out.steering_rad, -vp.steering.max_angle_rad, vp.steering.max_angle_rad);
    msg.drive.speed = clamp_for_float32_publish(out.speed_mps, vp.limits.min_velocity_mps,
                                                vp.limits.global_speed_cap_mps);
    drive_pub_->publish(msg);
  }

  // Declared before controller_ so build_config() can fill them.
  double control_rate_hz_ = 50.0;
  double control_period_s_ = 0.02;
  double status_rate_hz_ = 2.0;
  double scan_timeout_s_ = 0.3;
  double odom_timeout_s_ = 0.2;
  double drive_feedback_timeout_s_ = 0.2;

  ParkController controller_;
  ScanInput scan_;
  ParkOdometry odometry_;
  double gated_speed_mps_ = 0.0;
  bool has_scan_ = false;
  bool has_odom_ = false;
  bool has_gated_ = false;
  long cycles_ = 0;
  long status_period_cycles_ = 25;
  std::future<ParallelParkPlan> plan_future_;
  std::uint64_t plan_request_id_ = 0;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_scan_steady_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_odom_steady_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_gated_steady_{0, 0, RCL_STEADY_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr gated_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racer_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_control::ParkNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("park_node"), "park_node: fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
