// safety_node: ROS 2 plumbing around racer_safety's gate logic core (roadmap milestone 1,
// claude-docs/04-architecture.md, claude-docs/05-safety.md).
//
// Sole publisher of /drive (claude-docs/05-safety.md layer 3). Subscribes /drive_raw
// (command path, reliable) and /scan (sensor data, best_effort) per claude-docs/10-
// conventions.md's QoS rule. Every physical bound fed into `racer_safety::SafetyLimits`
// comes ONLY from the generated vehicle_params binding (CLAUDE.md invariant 2); tuning
// knobs that are not physical constants (control rate, watchdog cadence) are declared ROS
// parameters instead.
//
// Fail-closed (claude-docs/05-safety.md: "any internal error -> brake command, not
// passthrough"): the entire per-cycle body runs inside a try/catch, and ANY exception --
// including the test-only `inject_fault` parameter below -- results in publishing a hard
// brake and an `internal_fault` /safety/events record, never a silent crash or passthrough.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <racer_msgs/msg/safety_event.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {

namespace {

// Nearest usable /scan return this cycle: finite, strictly positive, and within the
// message's own declared [range_min, range_max] -- claude-docs/05-safety.md's TTC gate
// operates on real obstacle distance, not on a sensor's own "no return"/error encodings
// (many LiDAR drivers report those as 0.0, negative, or +inf, all already excluded by
// gate_logic's `is_valid_range`, but filtering to the message's own valid band here as well
// keeps this ROS-message-specific parsing step honest about what "valid" means for THIS
// message type). Returns +infinity ("no valid range this cycle") if nothing qualifies --
// gate_logic.hpp documents that as a safe no-op for the TTC gate, not a fabricated obstacle.
double compute_min_scan_range_m(const sensor_msgs::msg::LaserScan& msg) {
  double min_range = std::numeric_limits<double>::infinity();
  for (const float r : msg.ranges) {
    const double range = static_cast<double>(r);
    if (!std::isfinite(range)) {
      continue;
    }
    if (range <= 0.0) {
      continue;
    }
    if (range < static_cast<double>(msg.range_min) || range > static_cast<double>(msg.range_max)) {
      continue;
    }
    if (range < min_range) {
      min_range = range;
    }
  }
  return min_range;
}

}  // namespace

class SafetyNode : public rclcpp::Node {
 public:
  SafetyNode()
      : Node("safety_node"),
        previous_output_(DriveCommand{0.0, 0.0}),
        min_scan_range_m_(std::numeric_limits<double>::infinity()),
        has_received_command_(false),
        has_evaluated_before_(false) {
    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const rclcpp::QoS scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS events_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

    drive_raw_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive_raw", command_qos,
        std::bind(&SafetyNode::on_drive_raw, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&SafetyNode::on_scan, this, std::placeholders::_1));

    drive_pub_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", command_qos);
    events_pub_ =
        this->create_publisher<racer_msgs::msg::SafetyEvent>("/safety/events", events_qos);

    // control_period_s_ MUST be set before build_limits() runs (it feeds
    // SafetyLimits::control_period_s, which the watchdog timeout is computed from) --
    // this is why `gate_` is a std::optional constructed here in the body rather than in the
    // member-initializer list, where this parameter would not exist yet.
    const double control_rate_hz =
        declare_positive_double("control_rate_hz", 50.0,
                                "Watchdog/gate evaluation rate (claude-docs/04-architecture.md: "
                                "the command path runs at 50 Hz).");
    control_period_s_ = 1.0 / control_rate_hz;
    gate_.emplace(build_limits());

    rcl_interfaces::msg::ParameterDescriptor inject_fault_descriptor;
    inject_fault_descriptor.description =
        "TEST-ONLY. When true, every cycle raises an internal fault before evaluating any "
        "gate, to exercise the fail-closed path (claude-docs/12-testing.md L3: 'publishes "
        "brake on internal exception'). Never set outside a test.";
    this->declare_parameter<bool>("inject_fault", false, inject_fault_descriptor);

    const auto period = std::chrono::duration<double>(control_period_s_);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&SafetyNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "safety_node up: %.1f Hz, watchdog=%d missed cycles, ttc_brake_s=%s, "
                "ttc_warning_s=%s",
                control_rate_hz, gate_limits_.watchdog_missed_cycles,
                gate_limits_.ttc_brake_s.has_value()
                    ? std::to_string(*gate_limits_.ttc_brake_s).c_str()
                    : "unset (untuned; TTC gate is a no-op -- claude-docs/06-vehicle-params.md)",
                gate_limits_.ttc_warning_s.has_value()
                    ? std::to_string(*gate_limits_.ttc_warning_s).c_str()
                    : "unset");
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

  SafetyLimits build_limits() {
    // Every physical bound here comes ONLY from the generated vehicle_params binding
    // (CLAUDE.md invariant 2, claude-docs/06-vehicle-params.md rule 3) -- never hand-typed.
    // watchdog_missed_cycles is node-level tuning (the watchdog cadence), not a physical
    // constant, so it is a declared ROS parameter instead.
    rcl_interfaces::msg::ParameterDescriptor watchdog_descriptor;
    watchdog_descriptor.description =
        "Missed /drive_raw cycles before the watchdog brakes (claude-docs/04-architecture.md: "
        "'missing /drive_raw for 3 cycles -> brake command').";
    rcl_interfaces::msg::IntegerRange watchdog_range;
    watchdog_range.from_value = 1;
    watchdog_range.to_value = 100;
    watchdog_descriptor.integer_range = {watchdog_range};
    const int watchdog_missed_cycles = static_cast<int>(
        this->declare_parameter<int>("watchdog_missed_cycles", 3, watchdog_descriptor));

    // ttc_warning_s / ttc_brake_s are vehicle_params.yaml's limits.ttc_warning_s/ttc_brake_s
    // (CLAUDE.md invariant 2: this is their ONE source of truth) -- currently `null` in the
    // committed file ("needs Phase 1/2 tuning", see that file's comments), which
    // SafetyLimits/evaluate() treat as a documented no-op (the TTC gate does not brake).
    // These are declared as ROS parameters ANYWAY, defaulting to whatever vehicle_params
    // currently holds (a negative default when it is null, meaning "unconfigured"), so that:
    // (a) once a real sysid-tuned value is committed to vehicle_params.yaml, this node picks
    // it up automatically with no launch file needing to change, and (b) a launch file can
    // override the default in the meantime -- test/test_safety_node_launch.py's TTC-brake
    // case is the only thing in this repo that does, to prove the gate itself works ahead of
    // real tuning data existing. A value <= 0 means "unconfigured" (TTC gate disabled).
    rcl_interfaces::msg::ParameterDescriptor ttc_warning_descriptor;
    ttc_warning_descriptor.description =
        "TTC warning threshold, seconds (<=0 means unconfigured/disabled). Defaults to "
        "vehicle_params.yaml's limits.ttc_warning_s.";
    rcl_interfaces::msg::FloatingPointRange ttc_range;
    ttc_range.from_value = -1.0;
    ttc_range.to_value = 30.0;
    ttc_warning_descriptor.floating_point_range = {ttc_range};
    const double ttc_warning_default = VEHICLE_PARAMS.limits.ttc_warning_s.has_value()
                                           ? *VEHICLE_PARAMS.limits.ttc_warning_s
                                           : -1.0;
    const double ttc_warning_param = this->declare_parameter<double>(
        "ttc_warning_s", ttc_warning_default, ttc_warning_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ttc_brake_descriptor;
    ttc_brake_descriptor.description =
        "TTC brake threshold, seconds (<=0 means unconfigured/disabled). Defaults to "
        "vehicle_params.yaml's limits.ttc_brake_s.";
    ttc_brake_descriptor.floating_point_range = {ttc_range};
    const double ttc_brake_default =
        VEHICLE_PARAMS.limits.ttc_brake_s.has_value() ? *VEHICLE_PARAMS.limits.ttc_brake_s : -1.0;
    const double ttc_brake_param =
        this->declare_parameter<double>("ttc_brake_s", ttc_brake_default, ttc_brake_descriptor);

    SafetyLimits limits;
    limits.steering_min_rad = VEHICLE_PARAMS.steering.min_angle_rad;
    limits.steering_max_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    limits.steering_rate_min_rad_per_s = VEHICLE_PARAMS.steering.min_rate_rad_per_s;
    limits.steering_rate_max_rad_per_s = VEHICLE_PARAMS.steering.max_rate_rad_per_s;
    limits.speed_min_mps = VEHICLE_PARAMS.limits.min_velocity_mps;
    limits.speed_max_mps = VEHICLE_PARAMS.limits.global_speed_cap_mps;
    limits.max_acceleration_mps2 = VEHICLE_PARAMS.actuation.max_acceleration_mps2;
    limits.ttc_warning_s =
        ttc_warning_param > 0.0 ? std::optional<double>(ttc_warning_param) : std::nullopt;
    limits.ttc_brake_s =
        ttc_brake_param > 0.0 ? std::optional<double>(ttc_brake_param) : std::nullopt;
    limits.watchdog_missed_cycles = watchdog_missed_cycles;
    limits.control_period_s = control_period_s_;
    gate_limits_ = limits;
    return limits;
  }

  void on_drive_raw(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    last_command_.steering_angle_rad = static_cast<double>(msg->drive.steering_angle);
    last_command_.speed_mps = static_cast<double>(msg->drive.speed);
    last_drive_raw_stamp_ = this->now();
    has_received_command_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    min_scan_range_m_ = compute_min_scan_range_m(*msg);
  }

  void publish_event(GateSource source, EventSeverity severity, const std::string& detail,
                     const rclcpp::Time& stamp) {
    racer_msgs::msg::SafetyEvent event_msg;
    event_msg.stamp = stamp;
    switch (severity) {
      case EventSeverity::kInfo:
        event_msg.severity = racer_msgs::msg::SafetyEvent::SEVERITY_INFO;
        break;
      case EventSeverity::kWarning:
        event_msg.severity = racer_msgs::msg::SafetyEvent::SEVERITY_WARNING;
        break;
      case EventSeverity::kBrake:
        event_msg.severity = racer_msgs::msg::SafetyEvent::SEVERITY_BRAKE;
        break;
    }
    event_msg.source = gate_source_to_string(source);
    event_msg.detail = detail;
    events_pub_->publish(event_msg);
  }

  void publish_drive(const DriveCommand& cmd, const rclcpp::Time& stamp) {
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = stamp;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = static_cast<float>(cmd.steering_angle_rad);
    drive_msg.drive.speed = static_cast<float>(cmd.speed_mps);
    drive_pub_->publish(drive_msg);
  }

  void on_timer() {
    const rclcpp::Time now = this->now();
    try {
      // Read the LIVE parameter value every cycle, not a construction-time snapshot: a test
      // (or an operator) flips this at runtime via the standard ROS 2 set_parameters service,
      // and a cached member would never see that change (claude-docs/12-testing.md L3: "fail-
      // closed on injected internal fault" needs this to actually take effect mid-run).
      if (this->get_parameter("inject_fault").as_bool()) {
        throw std::runtime_error(
            "safety_node: test-only injected fault (inject_fault parameter is true)");
      }

      GateInput input;
      input.command = has_received_command_ ? last_command_ : DriveCommand{0.0, 0.0};
      input.drive_raw_age_s = has_received_command_ ? (now - last_drive_raw_stamp_).seconds()
                                                    : std::numeric_limits<double>::infinity();
      input.dt_s = has_evaluated_before_ ? (now - last_eval_time_).seconds() : control_period_s_;
      input.min_scan_range_m = min_scan_range_m_;
      input.has_pose_input = false;  // TODO(roadmap 2.6): wire from /pose once it exists.
      input.pose_covariance_trace = 0.0;

      const GateResult result = gate_->evaluate(input, previous_output_);
      publish_drive(result.output, now);
      for (const SafetyEvent& event : result.events) {
        publish_event(event.source, event.severity, event.detail, now);
      }
      previous_output_ = result.output;
      last_eval_time_ = now;
      has_evaluated_before_ = true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "safety_node: internal fault, braking: %s", e.what());
      const DriveCommand brake{0.0, 0.0};
      publish_drive(brake, now);
      publish_event(GateSource::kInternalFault, EventSeverity::kBrake,
                    std::string("internal fault: ") + e.what(), now);
      previous_output_ = brake;
      last_eval_time_ = now;
      has_evaluated_before_ = true;
    }
  }

  std::optional<SafetyGateLogic> gate_;
  SafetyLimits gate_limits_;
  double control_period_s_{0.02};

  DriveCommand previous_output_;
  DriveCommand last_command_;
  rclcpp::Time last_drive_raw_stamp_;
  rclcpp::Time last_eval_time_;
  double min_scan_range_m_;
  bool has_received_command_;
  bool has_evaluated_before_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<racer_msgs::msg::SafetyEvent>::SharedPtr events_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racer_safety

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_safety::SafetyNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_node"), "safety_node: fatal startup error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
