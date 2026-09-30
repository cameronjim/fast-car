#include "racer_drivers/pwm_output_driver.hpp"

namespace racer_drivers {

PwmOutputDriver::PwmOutputDriver(const MappingConfig& config, PwmChannelSink& steering,
                                 PwmChannelSink& throttle)
    : config_(config),
      steering_(steering),
      throttle_(throttle),
      steering_period_ns_(period_us_to_ns(config.steering_pwm_period_us)),
      throttle_period_ns_(period_us_to_ns(config.throttle_pwm_period_us)) {}

// Every duty this driver writes goes through these two, so the frame compensation (GitHub
// issue #77) applies identically to the startup, update, fail-closed and shutdown writes.
unsigned long long PwmOutputDriver::steering_duty_ns(double pulse_us) const {
  return frame_compensated_duty_ns(pulse_us, steering_period_ns_,
                                   config_.steering_pwm_achieved_period_us);
}

unsigned long long PwmOutputDriver::throttle_duty_ns(double pulse_us) const {
  return frame_compensated_duty_ns(pulse_us, throttle_period_ns_,
                                   config_.throttle_pwm_achieved_period_us);
}

void PwmOutputDriver::start() {
  const PulsePair neutral = neutral_outputs(config_);
  steering_.start(steering_period_ns_, steering_duty_ns(neutral.steering_us));
  throttle_.start(throttle_period_ns_, throttle_duty_ns(neutral.throttle_us));
}

PulsePair PwmOutputDriver::update(const CommandState& state) {
  const PulsePair pulses = compute_outputs(config_, state);
  steering_.write_duty_ns(steering_duty_ns(pulses.steering_us));
  throttle_.write_duty_ns(throttle_duty_ns(pulses.throttle_us));
  return pulses;
}

void PwmOutputDriver::force_neutral() noexcept {
  const PulsePair neutral = neutral_outputs(config_);
  // write_duty_ns can throw (a sysfs write can fail); this is the fail-closed path, so a
  // failure here is swallowed rather than allowed to escape into an exception loop. The
  // channels are disabled by stop() on the way out either way, and layer 1 (the mux) sees
  // invalid/absent pulses and cuts -- which is the point of layer 1.
  try {
    steering_.write_duty_ns(steering_duty_ns(neutral.steering_us));
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
  try {
    throttle_.write_duty_ns(throttle_duty_ns(neutral.throttle_us));
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
}

void PwmOutputDriver::stop() noexcept {
  if (stopped_) {
    return;
  }
  stopped_ = true;
  const PulsePair neutral = neutral_outputs(config_);
  steering_.stop(steering_duty_ns(neutral.steering_us));
  throttle_.stop(throttle_duty_ns(neutral.throttle_us));
}

}  // namespace racer_drivers
