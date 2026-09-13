// racer_drivers/pwm_output_driver.hpp -- fail-closed sequencing over two PwmChannelSinks.
//
// ROS-free on purpose (claude-docs/10-conventions.md), so the whole "never leave a stale
// non-neutral pulse" claim is gtest-able against InMemoryPwmChannel with no kernel involved.
// pwm_output_node.cpp owns parameters, the /drive subscription and the timer; this owns what
// actually reaches the wire.
#ifndef RACER_DRIVERS_PWM_OUTPUT_DRIVER_HPP_
#define RACER_DRIVERS_PWM_OUTPUT_DRIVER_HPP_

#include "racer_drivers/pwm_mapping.hpp"
#include "racer_drivers/pwm_sink.hpp"

namespace racer_drivers {

class PwmOutputDriver {
 public:
  /// Non-owning references to the two channels; they must outlive the driver.
  PwmOutputDriver(const MappingConfig& config, PwmChannelSink& steering, PwmChannelSink& throttle,
                  unsigned long long period_ns);

  /// Bring both channels up AT NEUTRAL. Nothing has arrived on /drive yet at this point and
  /// nothing may, so the very first electrical state of both outputs is neutral.
  void start();

  /// One 50 Hz cycle: map (or fall back to neutral) and write both channels.
  /// Returns what was written, for logging/tests. No allocation.
  PulsePair update(const CommandState& state);

  /// Write neutral to both channels without disabling them: the fail-closed response to an
  /// exception anywhere in the node, where the process keeps running and the mux should keep
  /// seeing valid neutral pulses rather than nothing.
  void force_neutral() noexcept;

  /// Neutral on both channels, then disable both. The shutdown path. Safe to call twice.
  void stop() noexcept;

  const MappingConfig& config() const { return config_; }
  unsigned long long period_ns() const { return period_ns_; }

 private:
  MappingConfig config_;
  PwmChannelSink& steering_;
  PwmChannelSink& throttle_;
  unsigned long long period_ns_;
  bool stopped_{false};
};

}  // namespace racer_drivers

#endif  // RACER_DRIVERS_PWM_OUTPUT_DRIVER_HPP_
