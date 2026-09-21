// pwm_output_node: the tail of the command path (claude-docs/04-architecture.md).
//
//   safety_node --/drive--> pwm_output_node --PWM pulses--> layer-1 mux board --> servo/VESC
//
// SAFETY (claude-docs/05-safety.md, CLAUDE.md invariant 1). This node subscribes to `/drive`
// and to nothing else on the command path. It NEVER subscribes to `/drive_raw`: doing so
// would put an ungated command straight onto the wire and route around safety_node, which is
// the exact bypass invariant 1 forbids. The layer-1 RC mux stays physically downstream of
// these two pulses and this node has no way to reach it, reconfigure it, or ask it for
// anything; a cut is still a cut regardless of what is written here.
//
// This is a risk-reduction layer, not a guarantee (claude-docs/05-safety.md): the only
// guarantee is layer 1.
//
// FAIL CLOSED. Both channels come up at the calibrated neutral before any /drive can arrive;
// they return to neutral when /drive goes silent past `drive_timeout_s`; they return to
// neutral on any exception in the cycle; and on shutdown (SIGINT/SIGTERM, which rclcpp
// installs handlers for) they are written neutral and then disabled. There is no code path
// that leaves a stale non-neutral pulse on either channel.
//
// COMMAND PATH DECISION (docs/notes/build-log.md, 2026-09-12). The Jetson commands the motor
// by PWM through the mux board into the VESC's PPM input. The VESC's USB link is telemetry
// and configuration only. A USB current-control path would run around the mux entirely.
//
// CLOCK POLICY. The /drive staleness measurement -- the thing that decides whether both
// channels fall back to neutral -- is taken on RCL_STEADY_TIME (the OS monotonic clock),
// never on this->now(). This node originally used the node clock, which is the exact defect
// docs/notes/first-boot-audit-2026-09-13.md findings #1-#3 removed from safety_node,
// tracker_node and twist_teleop_adapter_node (GitHub issue #22). Two ways it bites here:
// with use_sim_time:=false the node clock is CLOCK_REALTIME, and a backwards NTP/VM step
// makes the measured age NEGATIVE, which `is_stale`'s `age_s >= timeout_s` reads as FRESH --
// so a dead /drive publisher leaves the last commanded throttle pulse on the wire for the
// length of the step; with use_sim_time:=true and no /clock the node clock is pinned at 0,
// every age is exactly 0.0, and the timeout NEVER fires. A monotonic clock has neither
// failure mode. This node publishes no messages, so it needs no ROS stamp at all.
//
// PHYSICAL CONSTANTS. Every pulse-width bound, angle limit and speed reference below comes
// from the GENERATED vehicle_params binding and nowhere else (CLAUDE.md invariant 2). If a
// required field is null the node refuses to start and names the field, exactly like
// firmware/safety_mux's logic/mux_params.c -- it never substitutes a default.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_drivers/pwm_mapping.hpp"
#include "racer_drivers/pwm_output_driver.hpp"
#include "racer_drivers/pwm_sink.hpp"

namespace racer_drivers {

class PwmOutputNode : public rclcpp::Node {
 public:
  PwmOutputNode() : Node("pwm_output_node") {
    const double output_rate_hz = declare_double(
        "output_rate_hz", 50.0, 1.0, 400.0,
        "Rate at which this node REWRITES the two duty cycles, Hz -- the command update "
        "cadence, i.e. how often a fresh /drive can reach the wire. It is no longer the PWM "
        "carrier period: that is now "
        "config/vehicle_params.yaml's actuation.steering_pwm_period_us / "
        "throttle_pwm_period_us (GitHub issue #66), because the frame period decides the "
        "achievable pulse grid and a physical constant does not belong in a node default "
        "(CLAUDE.md invariant 2). 50 Hz matches the command path in "
        "claude-docs/04-architecture.md and the mux loop; raising it does not make the "
        "actuators move sooner, because the mux regenerates its own 50 Hz outputs.");

    const double drive_timeout_s = declare_double(
        "drive_timeout_s", 0.1, 1e-3, 5.0,
        "Seconds since the last /drive message after which both channels output neutral. "
        "Node tuning (a cadence), NOT a physical constant, so it is a declared parameter "
        "rather than a vehicle_params field. Deliberately NOT limits.mux_watchdog_timeout_s: "
        "that is the layer-1 mux MCU's own heartbeat window (claude-docs/05-safety.md layer "
        "1), a different mechanism on a different device, and conflating the two would make "
        "one number silently retune the other. The 0.1 s default is conservative: five "
        "missed frames at 50 Hz.");

    const std::string sysfs_root = declare_string(
        "sysfs_root", "/sys/class/pwm",
        "Root of the Linux sysfs PWM interface. Only ever changed by tests, which point it "
        "at a temp directory holding the same file layout (see "
        "test/test_pwm_output_node_launch.py). On the car this stays /sys/class/pwm.");

    const int steering_chip = declare_int(
        "steering_pwmchip", 0, 0, 15,
        "pwmchip index for the steering channel: the N in /sys/class/pwm/pwmchipN. VERIFIED "
        "on the Jetson Orin Nano Super Dev Kit, JetPack 6.2 / L4T R36.4.4, on the actual "
        "device on 2026-09-20: header pin 15 is pwmchip0 (3280000.pwm), driven at 50 Hz / 50 "
        "percent duty and measured ~1.64 V DC pin 15 to pin 14 after the "
        "/opt/nvidia/jetson-io pinmux change and a reboot. That numbering can still differ "
        "on another unit or kernel build -- read it off the device with 'ls -l "
        "/sys/class/pwm' before trusting this default there. See README.md, 'Enabling PWM "
        "pins on the Jetson'.");
    const int steering_channel = declare_int(
        "steering_pwm_channel", 0, 0, 7,
        "Channel index within the steering pwmchip: the M in pwmchipN/pwmM. VERIFIED, see "
        "steering_pwmchip (npwm=1, so channel 0 is the only channel on this chip).");
    const int throttle_chip = declare_int(
        "throttle_pwmchip", 2, 0, 15,
        "pwmchip index for the throttle channel. VERIFIED, see steering_pwmchip: header pin "
        "33 is pwmchip2 (32c0000.pwm), measured ~1.64 V DC pin 33 to pin 34 at 50 Hz / 50 "
        "percent duty on the same device and date. Pins 15 and 33 are different chips on "
        "this board.");
    const int throttle_channel = declare_int(
        "throttle_pwm_channel", 0, 0, 7,
        "Channel index within the throttle pwmchip. VERIFIED, see steering_pwmchip (npwm=1). "
        "Different pwmchip from steering (0 vs 2), so the all-defaults configuration is "
        "still two distinct channels even though both channel indices are 0 -- "
        "validate_channel_assignment below only refuses an identical (chip, channel) pair. "
        "Matches racer_bringup/launch/car_teleop.launch.py's default.");

    // Refuse before anything is exported: two pulses on one channel is not recoverable at
    // runtime and looks like a wiring fault on the car (pwm_mapping.hpp).
    const std::optional<std::string> bad_channels = validate_channel_assignment(
        steering_chip, steering_channel, throttle_chip, throttle_channel);
    if (bad_channels.has_value()) {
      throw std::runtime_error(*bad_channels);
    }

    config_ = build_config(drive_timeout_s);

    steering_sink_ = std::make_unique<SysfsPwmChannel>(sysfs_root, steering_chip, steering_channel);
    throttle_sink_ = std::make_unique<SysfsPwmChannel>(sysfs_root, throttle_chip, throttle_channel);
    // The frame periods are inside config_, straight from the generated vehicle_params
    // binding; the driver derives the sysfs `period` values from them. There is no
    // hand-written 20000000 anywhere on this path any more (GitHub issue #66).
    driver_ = std::make_unique<PwmOutputDriver>(config_, *steering_sink_, *throttle_sink_);

    // Neutral on BOTH channels before the subscription exists, let alone before any /drive
    // message can be delivered. Any failure here throws out of the constructor and main()
    // exits nonzero -- refuse to start, never run half-configured on the actuator path.
    driver_->start();

    const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", command_qos, std::bind(&PwmOutputNode::on_drive, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / output_rate_hz);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&PwmOutputNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "pwm_output_node up at %.1f Hz on %s: steering pwmchip%d/pwm%d "
                "[%.0f/%.0f/%.0f us, left=%s], throttle pwmchip%d/pwm%d [%.0f/%.0f/%.0f us, "
                "full scale %.2f m/s, cap %.2f m/s, OPEN LOOP PROVISIONAL], drive timeout "
                "%.3f s. Both channels are at neutral until /drive arrives.",
                output_rate_hz, sysfs_root.c_str(), steering_chip, steering_channel,
                config_.steering.min_us, config_.steering.neutral_us, config_.steering.max_us,
                config_.left_is_pwm_max ? "pwm_max_us" : "pwm_min_us", throttle_chip,
                throttle_channel, config_.throttle.min_us, config_.throttle.neutral_us,
                config_.throttle.max_us, config_.speed_full_scale_mps, config_.speed_cap_mps,
                config_.drive_timeout_s);
    // The grid is logged, not merely the period, because the period alone does not tell an
    // operator at the bench what to expect: a commanded pulse lands on a multiple of
    // period/256 (GitHub issue #66), which is what the mux diagnostic will be reporting back.
    RCLCPP_INFO(this->get_logger(),
                "pwm frame: steering %.0f us (%.1f Hz, %.3f us pulse grid), throttle %.0f us "
                "(%.1f Hz, %.3f us pulse grid), from config/vehicle_params.yaml. The servo "
                "and the ESC do not see these frames: the layer-1 mux regenerates its own 50 "
                "Hz outputs (firmware/safety_mux).",
                config_.steering_pwm_period_us, 1.0e6 / config_.steering_pwm_period_us,
                pulse_grid_step_us(config_.steering_pwm_period_us), config_.throttle_pwm_period_us,
                1.0e6 / config_.throttle_pwm_period_us,
                pulse_grid_step_us(config_.throttle_pwm_period_us));
  }

  ~PwmOutputNode() override { shutdown(); }

  PwmOutputNode(const PwmOutputNode&) = delete;
  PwmOutputNode& operator=(const PwmOutputNode&) = delete;

  /// Neutral, then disable, on both channels. Idempotent; called from main() after spin
  /// returns (the SIGINT/SIGTERM path) and again from the destructor.
  void shutdown() noexcept {
    if (driver_) {
      driver_->stop();
    }
  }

 private:
  double declare_double(const std::string& name, double default_value, double min_value,
                        double max_value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min_value;
    range.to_value = max_value;
    descriptor.floating_point_range = {range};
    return this->declare_parameter<double>(name, default_value, descriptor);
  }

  int declare_int(const std::string& name, int default_value, int min_value, int max_value,
                  const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = min_value;
    range.to_value = max_value;
    descriptor.integer_range = {range};
    return static_cast<int>(this->declare_parameter<int>(name, default_value, descriptor));
  }

  std::string declare_string(const std::string& name, const std::string& default_value,
                             const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return this->declare_parameter<std::string>(name, default_value, descriptor);
  }

  /// Assemble the mapping config from the generated vehicle_params binding. Refuses (throws)
  /// on any null required field, naming it. CLAUDE.md invariant 2 and 3.
  MappingConfig build_config(double drive_timeout_s) {
    const std::vector<RequiredField> required = {
        {"steering.pwm_min_us", VEHICLE_PARAMS.steering.pwm_min_us},
        {"steering.pwm_neutral_us", VEHICLE_PARAMS.steering.pwm_neutral_us},
        {"steering.pwm_max_us", VEHICLE_PARAMS.steering.pwm_max_us},
        {"actuation.throttle_pwm_min_us", VEHICLE_PARAMS.actuation.throttle_pwm_min_us},
        {"actuation.throttle_pwm_neutral_us", VEHICLE_PARAMS.actuation.throttle_pwm_neutral_us},
        {"actuation.throttle_pwm_max_us", VEHICLE_PARAMS.actuation.throttle_pwm_max_us},
        // steering.min_angle_rad / max_angle_rad and limits.global_speed_cap_mps are
        // non-nullable in config/vehicle_params.schema.json, so the generated binding types
        // them as plain doubles and they cannot be null here. They are wrapped in
        // std::optional anyway so that this list stays the single place a required field is
        // declared: if the schema ever makes one of them nullable, the generated type
        // changes, this line still compiles, and the refusal starts firing on its own.
        {"steering.min_angle_rad", std::optional<double>(VEHICLE_PARAMS.steering.min_angle_rad)},
        {"steering.max_angle_rad", std::optional<double>(VEHICLE_PARAMS.steering.max_angle_rad)},
        {"limits.global_speed_cap_mps",
         std::optional<double>(VEHICLE_PARAMS.limits.global_speed_cap_mps)},
        // Nullable in the schema, so the generated binding types it as an optional and the
        // refusal above is live: with this field null the node names it and does not start.
        {"actuation.throttle_full_scale_mps", VEHICLE_PARAMS.actuation.throttle_full_scale_mps},
        // Non-nullable in the schema (GitHub issue #66: a frame period that decides actuator
        // resolution has no defensible default), so these are plain doubles in the generated
        // binding and cannot be null here. Listed for the same reason as the two angle limits
        // above: if the schema ever makes them nullable, the refusal starts firing by itself.
        {"actuation.steering_pwm_period_us",
         std::optional<double>(VEHICLE_PARAMS.actuation.steering_pwm_period_us)},
        {"actuation.throttle_pwm_period_us",
         std::optional<double>(VEHICLE_PARAMS.actuation.throttle_pwm_period_us)},
    };
    const std::optional<std::string> missing = find_missing_fields(required);
    if (missing.has_value()) {
      throw std::runtime_error(*missing);
    }

    // steering.pwm_left_bound is non-nullable in the schema (enum of exactly "pwm_min_us" /
    // "pwm_max_us"), so the generated binding types it as a plain std::string and
    // find_missing_fields above does not cover it. It is still checked here, explicitly,
    // rather than trusting the string and defaulting a typo to "not left_is_pwm_max": this
    // is the sign of the steering channel, the exact thing the 2026-09-21 bench session found
    // backwards (docs/notes/build-log.md), and CLAUDE.md invariant 3 refuses on mismatch
    // rather than downgrading to a warning.
    const std::string& left_bound = VEHICLE_PARAMS.steering.pwm_left_bound;
    if (left_bound != "pwm_min_us" && left_bound != "pwm_max_us") {
      throw std::runtime_error(
          "config/vehicle_params.yaml steering.pwm_left_bound is '" + left_bound +
          "', not one of the two values the schema allows ('pwm_min_us', 'pwm_max_us'); "
          "refusing to guess the steering sign.");
    }

    MappingConfig config;
    config.steering.min_us = *VEHICLE_PARAMS.steering.pwm_min_us;
    config.steering.neutral_us = *VEHICLE_PARAMS.steering.pwm_neutral_us;
    config.steering.max_us = *VEHICLE_PARAMS.steering.pwm_max_us;
    config.throttle.min_us = *VEHICLE_PARAMS.actuation.throttle_pwm_min_us;
    config.throttle.neutral_us = *VEHICLE_PARAMS.actuation.throttle_pwm_neutral_us;
    config.throttle.max_us = *VEHICLE_PARAMS.actuation.throttle_pwm_max_us;
    config.steering_min_angle_rad = VEHICLE_PARAMS.steering.min_angle_rad;
    config.steering_max_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
    config.speed_full_scale_mps = *VEHICLE_PARAMS.actuation.throttle_full_scale_mps;
    config.speed_cap_mps = VEHICLE_PARAMS.limits.global_speed_cap_mps;
    config.left_is_pwm_max = (left_bound == "pwm_max_us");
    config.drive_timeout_s = drive_timeout_s;
    config.steering_pwm_period_us = VEHICLE_PARAMS.actuation.steering_pwm_period_us;
    config.throttle_pwm_period_us = VEHICLE_PARAMS.actuation.throttle_pwm_period_us;

    const std::optional<std::string> invalid = validate_config(config);
    if (invalid.has_value()) {
      throw std::runtime_error("config/vehicle_params.yaml is present but unusable: " + *invalid);
    }
    return config;
  }

  void on_drive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    // Deliberately timestamped on ARRIVAL rather than trusting msg->header.stamp: the
    // staleness decision here is "did a command reach me recently", which a
    // stale-but-well-formed header would answer wrongly.
    //
    // And deliberately on the STEADY clock, not this->now() -- see this file's CLOCK POLICY.
    last_command_.steering_angle_rad = static_cast<double>(msg->drive.steering_angle);
    last_command_.speed_mps = static_cast<double>(msg->drive.speed);
    last_command_steady_ = steady_clock_.now();
    last_command_.has_command = true;
  }

  void on_timer() {
    try {
      CommandState state = last_command_;
      state.age_s =
          state.has_command ? (steady_clock_.now() - last_command_steady_).seconds() : 0.0;
      driver_->update(state);
    } catch (const std::exception& e) {
      // Fail closed, then keep running: the mux keeps seeing valid neutral pulses rather
      // than nothing, and an operator sees a throttled error instead of a silent stop.
      driver_->force_neutral();
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "pwm_output_node: cycle failed, both channels forced to neutral: %s",
                            e.what());
    }
  }

  MappingConfig config_;
  CommandState last_command_;
  // Monotonic-clock receipt time, used ONLY as the left operand of an elapsed-time
  // subtraction, never as a stamp (CLOCK POLICY, top of file). Default-constructed with the
  // steady clock's own time source so the subtraction can never mix time sources.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_command_steady_{0, 0, RCL_STEADY_TIME};

  std::unique_ptr<SysfsPwmChannel> steering_sink_;
  std::unique_ptr<SysfsPwmChannel> throttle_sink_;
  std::unique_ptr<PwmOutputDriver> driver_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racer_drivers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<racer_drivers::PwmOutputNode> node;
  try {
    node = std::make_shared<racer_drivers::PwmOutputNode>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("pwm_output_node"), "pwm_output_node: refusing to start: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("pwm_output_node"), "pwm_output_node: spin aborted: %s",
                 e.what());
    node->shutdown();
    rclcpp::shutdown();
    return 1;
  }
  // SIGINT/SIGTERM land here (rclcpp's installed handlers stop the spin): neutral, then
  // disable, before the process exits.
  node->shutdown();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
