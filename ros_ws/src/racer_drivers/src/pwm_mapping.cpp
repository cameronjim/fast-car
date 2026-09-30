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
    return std::string(
        "speed full scale (actuation.throttle_full_scale_mps) must be finite and > 0");
  }
  if (!finite(config.speed_cap_mps) || config.speed_cap_mps <= 0.0) {
    return std::string("speed cap (limits.global_speed_cap_mps) must be finite and > 0");
  }
  if (!finite(config.drive_timeout_s) || config.drive_timeout_s <= 0.0) {
    return std::string("drive_timeout_s must be finite and > 0");
  }
  // The deadband offset must leave a non-empty linear span on BOTH sides of neutral: the
  // forward side because that is what the map interpolates over, the reverse side because the
  // mapping stays defined for negative speed even while every teleop source clamps at zero.
  if (!finite(config.throttle_deadband_us) || config.throttle_deadband_us < 0.0) {
    return std::string("actuation.throttle_deadband_us must be finite and >= 0");
  }
  if (!(config.throttle.neutral_us + config.throttle_deadband_us < config.throttle.max_us) ||
      !(config.throttle.neutral_us - config.throttle_deadband_us > config.throttle.min_us)) {
    return std::string(
        "actuation.throttle_deadband_us leaves no linear span between neutral + deadband and a "
        "channel end: it must satisfy throttle_pwm_min_us < neutral - deadband and "
        "neutral + deadband < throttle_pwm_max_us");
  }
  // The frame period has to be long enough to CONTAIN the longest pulse the channel can be
  // commanded to, with a low gap after it: a duty equal to or longer than the period is
  // EINVAL from the kernel, and a pulse that fills most of its frame leaves the mux's edge
  // capture almost no low time to see a falling edge in. The margin is expressed as "at
  // least twice the channel's own maximum pulse" rather than a microsecond constant, so it
  // scales with the calibration instead of being another number to keep in sync: at the
  // committed 4000 us frame and a 2000 us maximum that is exactly the boundary, and a
  // calibration whose maximum grows has to grow the frame with it.
  const auto check_period = [&finite](double period_us, double max_pulse_us, const char* field,
                                      const char* pulse_field) -> std::optional<std::string> {
    if (!finite(period_us) || period_us <= 0.0) {
      return std::string("actuation.") + field + " must be finite and > 0";
    }
    if (period_us < 2.0 * max_pulse_us) {
      return std::string("actuation.") + field + " is shorter than twice " + pulse_field +
             ", so the longest commandable pulse would fill at least half its frame and "
             "leave the safety mux's edge capture no reliable low gap; lengthen the frame "
             "or lower the pulse maximum";
    }
    return std::nullopt;
  };
  if (const std::optional<std::string> bad =
          check_period(config.steering_pwm_period_us, config.steering.max_us,
                       "steering_pwm_period_us", "steering.pwm_max_us")) {
    return bad;
  }
  if (const std::optional<std::string> bad =
          check_period(config.throttle_pwm_period_us, config.throttle.max_us,
                       "throttle_pwm_period_us", "actuation.throttle_pwm_max_us")) {
    return bad;
  }
  // The achieved period scales every duty this node writes (GitHub issue #77), so a value
  // that is not plausibly a measurement of the requested frame refuses rather than drives.
  // The band is relative, so it scales with whatever period is requested.
  const auto check_achieved = [&finite](double achieved_us, double requested_us,
                                        const char* field) -> std::optional<std::string> {
    if (!finite(achieved_us) || achieved_us <= 0.0) {
      return std::string("actuation.") + field + " must be finite and > 0";
    }
    if (std::fabs(achieved_us / requested_us - 1.0) > kMaxAchievedPeriodDeviation) {
      std::ostringstream message;
      message << "actuation." << field << " (" << achieved_us << " us) is more than "
              << kMaxAchievedPeriodDeviation * 100.0 << " percent from the requested period ("
              << requested_us
              << " us). It is a bench measurement of the frame the Jetson actually emits for "
                 "that request and scales every pulse on the channel, so a value this far off "
                 "is treated as a typo; re-measure it (docs/notes/first-boot-runbook.md)";
      return message.str();
    }
    return std::nullopt;
  };
  if (const std::optional<std::string> bad =
          check_achieved(config.steering_pwm_achieved_period_us, config.steering_pwm_period_us,
                         "steering_pwm_achieved_period_us")) {
    return bad;
  }
  if (const std::optional<std::string> bad =
          check_achieved(config.throttle_pwm_achieved_period_us, config.throttle_pwm_period_us,
                         "throttle_pwm_achieved_period_us")) {
    return bad;
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
  if (state.age_s < 0.0) {
    return true;
  }
  return state.age_s >= timeout_s;
}

std::optional<std::string> validate_channel_assignment(int steering_chip, int steering_channel,
                                                       int throttle_chip, int throttle_channel) {
  if (steering_chip == throttle_chip && steering_channel == throttle_channel) {
    std::ostringstream message;
    message << "steering and throttle are both configured on pwmchip" << steering_chip << "/pwm"
            << steering_channel
            << ". They must be two different PWM channels: one drives the steering servo "
               "(Jetson header pin 15, verified pwmchip0) and the other the VESC PPM input "
               "(pin 33, verified pwmchip2). If you overrode the defaults, pass all four "
               "chip/channel values and make sure they name two distinct channels "
               "(docs/notes/first-boot-runbook.md step 4).";
    return message.str();
  }
  return std::nullopt;
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
  // Clamp to the safety cap FIRST (limits.global_speed_cap_mps), then scale by the map's own
  // full scale (actuation.throttle_full_scale_mps). The two are separate numbers: the full
  // scale is normally the smaller of them, so a command between full scale and the cap
  // saturates at the channel end. The final std::clamp guarantees that whatever the two
  // values are, the pulse never leaves the calibrated channel range.
  //
  // Zero is EXACTLY neutral, not neutral + deadband: it is the rest command and the fail-closed
  // output, and it must equal the pulse the mux itself emits on a cut. Any non-zero command
  // starts at neutral +/- throttle_deadband_us (actuation.throttle_deadband_us) so that it
  // clears the ESC's PPM deadband, then interpolates over what is left of the range, so full
  // scale still lands exactly on the channel end.
  const double capped_mps = std::clamp(speed_mps, -config.speed_cap_mps, config.speed_cap_mps);
  if (capped_mps == 0.0) {
    return config.throttle.neutral_us;
  }
  double pulse_us = 0.0;
  if (capped_mps > 0.0) {
    const double speed = std::min(capped_mps, config.speed_full_scale_mps);
    pulse_us = interpolate(config.throttle.neutral_us + config.throttle_deadband_us,
                           config.throttle.max_us, speed, config.speed_full_scale_mps);
  } else {
    const double speed = std::min(-capped_mps, config.speed_full_scale_mps);
    pulse_us = interpolate(config.throttle.neutral_us - config.throttle_deadband_us,
                           config.throttle.min_us, speed, config.speed_full_scale_mps);
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

unsigned long long frame_compensated_duty_ns(double pulse_us,
                                             unsigned long long requested_period_ns,
                                             double achieved_period_us) {
  if (!std::isfinite(achieved_period_us) || achieved_period_us <= 0.0) {
    return 0ULL;
  }
  // requested_period_ns / 1000 is the requested period in us; the ratio requested / achieved
  // is the inverse of the scaling the Tegra driver applies (GitHub issue #77).
  const double requested_period_us = static_cast<double>(requested_period_ns) / 1000.0;
  return pulse_us_to_duty_ns(pulse_us * (requested_period_us / achieved_period_us),
                             requested_period_ns);
}

double tegra_emitted_pulse_us(unsigned long long duty_ns, unsigned long long requested_period_ns,
                              double achieved_period_us) {
  if (requested_period_ns == 0ULL || !std::isfinite(achieved_period_us) ||
      achieved_period_us <= 0.0) {
    return 0.0;
  }
  // pwm-tegra.c: c = DIV_ROUND_CLOSEST_ULL(duty_ns << PWM_DUTY_WIDTH, period_ns). Integer
  // arithmetic, as in the kernel, so the rounding matches it exactly (round half up).
  constexpr unsigned long long kDutySteps = 256ULL;  // 1 << PWM_DUTY_WIDTH, see pulse_grid_step_us
  const unsigned long long count =
      (duty_ns * kDutySteps + requested_period_ns / 2ULL) / requested_period_ns;
  return static_cast<double>(count) * achieved_period_us / static_cast<double>(kDutySteps);
}

unsigned long long period_us_to_ns(double period_us) {
  if (!std::isfinite(period_us) || period_us <= 0.0) {
    return 0ULL;
  }
  return static_cast<unsigned long long>(std::round(period_us * 1000.0));
}

double pulse_grid_step_us(double period_us) {
  if (!std::isfinite(period_us) || period_us <= 0.0) {
    return 0.0;
  }
  // 256 = 2^PWM_DUTY_WIDTH from the Tegra PWM driver; see the header for the measurements
  // this matches. Deliberately a plain divisor and not a vehicle_params field: it is a
  // property of the SoC's PWM peripheral, not of the vehicle (CLAUDE.md invariant 2 covers
  // masses, geometry, ratios and conversions, not a register width).
  return period_us / 256.0;
}

}  // namespace racer_drivers
