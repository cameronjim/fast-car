// See gate_logic_formatting.hpp: deliberately excluded from gate_logic.cpp/.hpp's 100%
// branch-coverage gate (claude-docs/12-testing.md) -- these are /safety/events `detail`
// message-formatting helpers and gate_source_to_string's implementation, not decision logic.
#include "racer_safety/gate_logic_formatting.hpp"

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {

std::string gate_source_to_string(GateSource source) {
  switch (source) {
    case GateSource::kWatchdog:
      return "watchdog";
    case GateSource::kCommandSanity:
      return "command_sanity";
    case GateSource::kBoundsClamp:
      return "bounds_clamp";
    case GateSource::kRateLimit:
      return "rate_limit";
    case GateSource::kTtc:
      return "ttc";
    case GateSource::kCovariance:
      return "covariance";
    case GateSource::kInternalFault:
      return "internal_fault";
  }
  // Defensive only: every GateSource enumerator is handled above, and no code anywhere in
  // this repo constructs a GateSource outside that set.
  return "unknown";
}

namespace formatting {

std::string watchdog_detail(double timeout_s, double age_s) {
  return "/drive_raw stale or age invalid (timeout=" + std::to_string(timeout_s) +
         "s, age=" + std::to_string(age_s) + "s); braking";
}

std::string command_sanity_detail() {
  return "non-finite steering_angle_rad or speed_mps in /drive_raw; braking";
}

std::string bounds_clamp_detail(double steering_min_rad, double steering_max_rad,
                                double speed_min_mps, double speed_max_mps) {
  return "command clamped to vehicle_params bounds (steering [" + std::to_string(steering_min_rad) +
         ", " + std::to_string(steering_max_rad) + "] rad, speed [" +
         std::to_string(speed_min_mps) + ", " + std::to_string(speed_max_mps) + "] m/s)";
}

std::string rate_limit_detail(double dt_s) {
  return "command rate-limited relative to previous output (dt=" + std::to_string(dt_s) + "s)";
}

std::string ttc_brake_detail(double ttc_s, double brake_threshold_s) {
  return "time-to-collision " + std::to_string(ttc_s) + "s <= brake threshold " +
         std::to_string(brake_threshold_s) + "s; braking";
}

std::string ttc_warning_detail(double ttc_s, double warning_threshold_s) {
  return "time-to-collision " + std::to_string(ttc_s) + "s <= warning threshold " +
         std::to_string(warning_threshold_s) + "s (advisory only, no command change)";
}

std::string covariance_detail(double speed_fraction) {
  return "covariance gate engaged; speed fraction " + std::to_string(speed_fraction);
}

}  // namespace formatting

}  // namespace racer_safety
