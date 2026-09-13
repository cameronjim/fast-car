#include "racer_drivers/pwm_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace racer_drivers {

namespace {

/// Linear interpolation from `from_us` at 0 to `to_us` at `span` magnitude, evaluated at
/// `magnitude`. `span` is guaranteed positive by validate_config().
double interpolate(double from_us, double to_us, double magnitude, double span) {
  return from_us + (to_us - from_us) * (magnitude / span);
}

}  // namespace

std::optional<std::string> find_missing_fields(const std::vector<RequiredField>& fields) {
  std::vector<std::string> missing;
  for (const RequiredField& field : fields) {
    if (!field.value.has_value()) {
      missing.push_back(field.name);
    }
  }
  if (missing.empty()) {
    return std::nullopt;
  }
  std::ostringstream message;
  message << "config/vehicle_params.yaml is missing " << missing.size()
          << " field(s) this node requires, still null: ";
  for (std::size_t i = 0; i < missing.size(); ++i) {
    if (i > 0) {
      message << ", ";
    }
    message << missing[i];
  }
  message << ". Measure them on the bench and fill them in "
             "(docs/notes/hardware-arrival-checklist.md section 3); this node refuses to "
             "start rather than invent a value (CLAUDE.md invariant 2).";
  return message.str();
}

std::optional<std::string> validate_config(const MappingConfig& config) {
  const auto finite = [](double v) { return std::isfinite(v); };

  if (!finite(config.steering.min_us) || !finite(config.steering.neutral_us) ||
      !finite(config.steering.max_us)) {
    return std::string("steering pwm calibration contains a non-finite value");
  }
  if (!finite(config.throttle.min_us) || !finite(config.throttle.neutral_us) ||
      !finite(config.throttle.max_us)) {
    return std::string("throttle pwm calibration contains a non-finite value");
  }
  if (!(config.steering.min_us < config.steering.neutral_us &&
        config.steering.neutral_us < config.steering.max_us)) {
    return std::string(
        "steering pwm calibration must satisfy pwm_min_us < pwm_neutral_us < pwm_max_us");
  }
  if (!(config.throttle.min_us < config.throttle.neutral_us &&
        config.throttle.neutral_us < config.throttle.max_us)) {
    return std::string(
        "throttle pwm calibration must satisfy throttle_pwm_min_us < throttle_pwm_neutral_us "
        "< throttle_pwm_max_us");
  }
  if (!finite(config.steering_min_angle_rad) || !finite(config.steering_max_angle_rad)) {
    return std::string("steering angle limits contain a non-finite value");
  }
  if (!(config.steering_min_angle_rad < 0.0 && config.steering_max_angle_rad > 0.0)) {
    return std::string(
        "steering angle limits must straddle zero (steering.min_angle_rad < 0 < "
        "steering.max_angle_rad); this mapping interpolates each side out from neutral");
  }
  if (!finite(config.speed_full_scale_mps) || config.speed_full_scale_mps <= 0.0) {
    return std::string("speed full scale (limits.global_speed_cap_mps) must be finite and > 0");
  }
  if (!finite(config.drive_timeout_s) || config.drive_timeout_s <= 0.0) {
    return std::string("drive_timeout_s must be finite and > 0");
  }
  return std::nullopt;
}

bool is_stale(const CommandState& state, double timeout_s) {
  if (!state.has_command) {
    return true;
  }
  if (!std::isfinite(state.age_s)) {
    return true;
  }
  return state.age_s >= timeout_s;
}

double steering_angle_to_pulse_us(const MappingConfig& config, double steering_angle_rad) {
  if (!std::isfinite(steering_angle_rad)) {
    return config.steering.neutral_us;
  }
  const double left_end_us =
      config.left_is_pwm_max ? config.steering.max_us : config.steering.min_us;
  const double right_end_us =
      config.left_is_pwm_max ? config.steering.min_us : config.steering.max_us;

  double pulse_us = 0.0;
  if (steering_angle_rad >= 0.0) {
    const double angle = std::min(steering_angle_rad, config.steering_max_angle_rad);
    pulse_us =
        interpolate(config.steering.neutral_us, left_end_us, angle, config.steering_max_angle_rad);
  } else {
    const double angle = std::min(-steering_angle_rad, -config.steering_min_angle_rad);
    pulse_us = interpolate(config.steering.neutral_us, right_end_us, angle,
                           -config.steering_min_angle_rad);
  }
  return std::clamp(pulse_us, config.steering.min_us, config.steering.max_us);
}

double speed_to_pulse_us(const MappingConfig& config, double speed_mps) {
  if (!std::isfinite(speed_mps)) {
    return config.throttle.neutral_us;
  }
  double pulse_us = 0.0;
  if (speed_mps >= 0.0) {
    const double speed = std::min(speed_mps, config.speed_full_scale_mps);
    pulse_us = interpolate(config.throttle.neutral_us, config.throttle.max_us, speed,
                           config.speed_full_scale_mps);
  } else {
    const double speed = std::min(-speed_mps, config.speed_full_scale_mps);
    pulse_us = interpolate(config.throttle.neutral_us, config.throttle.min_us, speed,
                           config.speed_full_scale_mps);
  }
  return std::clamp(pulse_us, config.throttle.min_us, config.throttle.max_us);
}

PulsePair neutral_outputs(const MappingConfig& config) {
  return PulsePair{config.steering.neutral_us, config.throttle.neutral_us};
}

PulsePair compute_outputs(const MappingConfig& config, const CommandState& state) {
  if (is_stale(state, config.drive_timeout_s)) {
    return neutral_outputs(config);
  }
  return PulsePair{steering_angle_to_pulse_us(config, state.steering_angle_rad),
                   speed_to_pulse_us(config, state.speed_mps)};
}

unsigned long long pulse_us_to_duty_ns(double pulse_us, unsigned long long period_ns) {
  if (!std::isfinite(pulse_us) || pulse_us <= 0.0) {
    return 0ULL;
  }
  const double duty_ns = std::round(pulse_us * 1000.0);
  if (duty_ns >= static_cast<double>(period_ns)) {
    return period_ns;
  }
  return static_cast<unsigned long long>(duty_ns);
}

}  // namespace racer_drivers
