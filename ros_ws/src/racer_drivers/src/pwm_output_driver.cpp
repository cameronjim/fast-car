#include "racer_drivers/pwm_output_driver.hpp"

namespace racer_drivers {

PwmOutputDriver::PwmOutputDriver(const MappingConfig& config, PwmChannelSink& steering,
                                 PwmChannelSink& throttle, unsigned long long period_ns)
    : config_(config), steering_(steering), throttle_(throttle), period_ns_(period_ns) {}

void PwmOutputDriver::start() {
  const PulsePair neutral = neutral_outputs(config_);
  steering_.start(period_ns_, pulse_us_to_duty_ns(neutral.steering_us, period_ns_));
  throttle_.start(period_ns_, pulse_us_to_duty_ns(neutral.throttle_us, period_ns_));
}

PulsePair PwmOutputDriver::update(const CommandState& state) {
  const PulsePair pulses = compute_outputs(config_, state);
  steering_.write_duty_ns(pulse_us_to_duty_ns(pulses.steering_us, period_ns_));
  throttle_.write_duty_ns(pulse_us_to_duty_ns(pulses.throttle_us, period_ns_));
  return pulses;
}

void PwmOutputDriver::force_neutral() noexcept {
  const PulsePair neutral = neutral_outputs(config_);
  // write_duty_ns can throw (a sysfs write can fail); this is the fail-closed path, so a
  // failure here is swallowed rather than allowed to escape into an exception loop. The
  // channels are disabled by stop() on the way out either way, and layer 1 (the mux) sees
  // invalid/absent pulses and cuts -- which is the point of layer 1.
  try {
    steering_.write_duty_ns(pulse_us_to_duty_ns(neutral.steering_us, period_ns_));
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
  try {
    throttle_.write_duty_ns(pulse_us_to_duty_ns(neutral.throttle_us, period_ns_));
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
}

void PwmOutputDriver::stop() noexcept {
  if (stopped_) {
    return;
  }
  stopped_ = true;
  const PulsePair neutral = neutral_outputs(config_);
  steering_.stop(pulse_us_to_duty_ns(neutral.steering_us, period_ns_));
  throttle_.stop(pulse_us_to_duty_ns(neutral.throttle_us, period_ns_));
}

}  // namespace racer_drivers
