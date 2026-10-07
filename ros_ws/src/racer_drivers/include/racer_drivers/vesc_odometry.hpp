// vesc_odometry.hpp -- ROS-free half of vesc_odometry_node (claude-docs/10-conventions.md:
// decision logic separated from node plumbing; claude-docs/12-testing.md L1).
//
// Turns the VESC's electrical RPM into wheel speed and an integrated along-track distance.
// The three drivetrain numbers come from config/vehicle_params.yaml through the generated
// binding (CLAUDE.md invariant 2): drivetrain.pole_pairs, drivetrain.gear_ratio and
// tires.nominal_radius_m. This header never sees the binding; the node fills a
// DrivetrainConfig from it, and test_vesc_odometry_binding.cpp pins that the two agree.
//
// SIGN. The VESC reports ERPM in its own motor frame, where positive is the direction a
// positive duty / current command turns the motor. claude-docs/06-vehicle-params.md fixes
// "motor current: positive = drive torque forward" (drivetrain.current_limit_a's schema sign
// convention), so positive ERPM is forward travel and maps to a positive wheel speed. No new
// sign convention is introduced here; if the bench shows a forward roll reading negative, the
// VESC's motor direction (or the PPM mapping) disagrees with that convention and is the thing
// to fix, not this file.
//
// UNITS. ERPM is the non-SI value at the driver boundary (CLAUDE.md invariant 4); everything
// this header returns is SI (m/s, rad/s, m, s). The only literals are the minute-to-second and
// revolution-to-radian definitions, which are not vehicle properties.
#ifndef RACER_DRIVERS__VESC_ODOMETRY_HPP_
#define RACER_DRIVERS__VESC_ODOMETRY_HPP_

#include <cmath>
#include <optional>
#include <string>

namespace racer_drivers {
namespace vesc_odometry {

/// Seconds per minute: ERPM is revolutions per MINUTE.
inline constexpr double kSecondsPerMinute = 60.0;
/// Radians per revolution.
inline constexpr double kRadiansPerRevolution = 2.0 * M_PI;

/// The drivetrain numbers the conversion needs, all from vehicle_params.
struct DrivetrainConfig {
  double pole_pairs = 0.0;      // drivetrain.pole_pairs (count)
  double gear_ratio = 0.0;      // drivetrain.gear_ratio, motor turns per wheel turn
  double wheel_radius_m = 0.0;  // tires.nominal_radius_m
};

/// Why a DrivetrainConfig is unusable, or nullopt if it is fine. Every field must be finite
/// and strictly positive: a zero or negative value would silently zero or flip the speed.
inline std::optional<std::string> validate(const DrivetrainConfig& config) {
  const auto bad = [](double value) { return !std::isfinite(value) || value <= 0.0; };
  if (bad(config.pole_pairs)) {
    return "drivetrain.pole_pairs must be finite and > 0";
  }
  if (bad(config.gear_ratio)) {
    return "drivetrain.gear_ratio must be finite and > 0";
  }
  if (bad(config.wheel_radius_m)) {
    return "tires.nominal_radius_m must be finite and > 0";
  }
  return std::nullopt;
}

/// Motor shaft angular speed, rad/s, from electrical RPM. Mechanical RPM = ERPM / pole pairs.
inline double erpm_to_motor_rad_per_s(double erpm, const DrivetrainConfig& config) {
  return erpm / config.pole_pairs * kRadiansPerRevolution / kSecondsPerMinute;
}

/// Wheel angular speed, rad/s: the motor speed divided by the overall gear ratio.
inline double erpm_to_wheel_rad_per_s(double erpm, const DrivetrainConfig& config) {
  return erpm_to_motor_rad_per_s(erpm, config) / config.gear_ratio;
}

/// Wheel surface speed, m/s, positive forward (see SIGN above). No-slip: this is the speed of
/// the driven wheels' rim, which equals ground speed only while the tyres are not slipping.
inline double erpm_to_wheel_speed_mps(double erpm, const DrivetrainConfig& config) {
  return erpm_to_wheel_rad_per_s(erpm, config) * config.wheel_radius_m;
}

/// Metres per second of wheel speed per ERPM. The single scale factor the conversion reduces
/// to, exposed so tests and the node's startup log can state it.
inline double mps_per_erpm(const DrivetrainConfig& config) {
  return erpm_to_wheel_speed_mps(1.0, config);
}

/// Inverse of erpm_to_wheel_speed_mps (used by the property tests and for reporting what ERPM a
/// speed corresponds to).
inline double wheel_speed_mps_to_erpm(double speed_mps, const DrivetrainConfig& config) {
  return speed_mps / mps_per_erpm(config);
}

/// Integrates wheel speed over the samples' own timestamps into a signed along-track distance:
/// forward adds, reverse subtracts, so after a three-point turn it is the NET distance along the
/// path, not the odometer total. Trapezoidal rule between consecutive accepted samples.
///
/// It refuses to integrate across anything it would have to guess at, and says which:
///   * the first sample (nothing to integrate from yet);
///   * a non-finite stamp or speed (the sample is dropped, the previous one is kept);
///   * a stamp that does not move forward (duplicate or out of order: dropped);
///   * a gap longer than max_gap_s (the distance over the gap is unknown, so it is NOT added;
///     the sample becomes the new starting point).
/// Silence therefore never turns into invented distance.
class AlongTrackIntegrator {
 public:
  enum class Result {
    kFirstSample,
    kIntegrated,
    kRejectedNonFinite,
    kRejectedNonMonotonic,
    kRestartedAfterGap,
  };

  explicit AlongTrackIntegrator(double max_gap_s) : max_gap_s_(max_gap_s) {}

  Result add(double stamp_s, double speed_mps) {
    if (!std::isfinite(stamp_s) || !std::isfinite(speed_mps)) {
      return Result::kRejectedNonFinite;
    }
    if (!has_previous_) {
      remember(stamp_s, speed_mps);
      return Result::kFirstSample;
    }
    const double dt = stamp_s - previous_stamp_s_;
    if (!(dt > 0.0)) {
      return Result::kRejectedNonMonotonic;
    }
    if (dt > max_gap_s_) {
      remember(stamp_s, speed_mps);
      return Result::kRestartedAfterGap;
    }
    distance_m_ += 0.5 * (previous_speed_mps_ + speed_mps) * dt;
    remember(stamp_s, speed_mps);
    return Result::kIntegrated;
  }

  /// Signed along-track distance since construction, metres, positive forward.
  double distance_m() const { return distance_m_; }
  double max_gap_s() const { return max_gap_s_; }

 private:
  void remember(double stamp_s, double speed_mps) {
    has_previous_ = true;
    previous_stamp_s_ = stamp_s;
    previous_speed_mps_ = speed_mps;
  }

  double max_gap_s_;
  bool has_previous_ = false;
  double previous_stamp_s_ = 0.0;
  double previous_speed_mps_ = 0.0;
  double distance_m_ = 0.0;
};

/// The VESC firmware's mc_fault_code as a name, for /telemetry/vesc/fault. Codes from the
/// bldc firmware's datatypes.h (FW 5.x / 6.x); the first seven match vesc_msgs/VescState's own
/// FAULT_CODE_* constants. Anything unlisted comes back as "UNKNOWN_<code>" rather than being
/// guessed at or dropped.
inline std::string fault_name(int code) {
  switch (code) {
    case 0:
      return "NONE";
    case 1:
      return "OVER_VOLTAGE";
    case 2:
      return "UNDER_VOLTAGE";
    case 3:
      return "DRV";
    case 4:
      return "ABS_OVER_CURRENT";
    case 5:
      return "OVER_TEMP_FET";
    case 6:
      return "OVER_TEMP_MOTOR";
    case 7:
      return "GATE_DRIVER_OVER_VOLTAGE";
    case 8:
      return "GATE_DRIVER_UNDER_VOLTAGE";
    case 9:
      return "MCU_UNDER_VOLTAGE";
    case 10:
      return "BOOTING_FROM_WATCHDOG_RESET";
    case 11:
      return "ENCODER_SPI";
    case 12:
      return "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
    case 13:
      return "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
    case 14:
      return "FLASH_CORRUPTION";
    case 15:
      return "HIGH_OFFSET_CURRENT_SENSOR_1";
    case 16:
      return "HIGH_OFFSET_CURRENT_SENSOR_2";
    case 17:
      return "HIGH_OFFSET_CURRENT_SENSOR_3";
    case 18:
      return "UNBALANCED_CURRENTS";
    default:
      return "UNKNOWN_" + std::to_string(code);
  }
}

/// What /telemetry/vesc/fault carries while no VescStateStamped has arrived for the node's
/// stale timeout (or none has arrived at all yet).
inline constexpr const char* kFaultStale = "STALE_NO_VESC_DATA";

}  // namespace vesc_odometry
}  // namespace racer_drivers

#endif  // RACER_DRIVERS__VESC_ODOMETRY_HPP_
