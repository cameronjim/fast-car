// racer_drivers/pwm_sink.hpp -- the one narrow seam between the command-path logic and the
// Linux sysfs PWM interface.
//
// Everything above this interface (pwm_mapping.hpp, pwm_output_driver.hpp) is pure and
// host-testable; everything below it is a write() to a file under /sys/class/pwm. The
// interface exists so the driver's sequencing (neutral first, neutral on stop, never a stale
// non-neutral pulse) can be gtest-ed against an in-memory fake with no kernel, no PWM chip
// and no root (claude-docs/12-testing.md L1).
//
// The sysfs PWM ABI this implements (Documentation/ABI/testing/sysfs-class-pwm):
//   <root>/pwmchip<N>/export        <- write the channel index to create pwm<M>/
//   <root>/pwmchip<N>/pwm<M>/period      <- period, nanoseconds
//   <root>/pwmchip<N>/pwm<M>/duty_cycle  <- pulse width, nanoseconds (<= period)
//   <root>/pwmchip<N>/pwm<M>/enable      <- "1" to run, "0" to stop
// `root` is a constructor argument (default /sys/class/pwm) purely so a launch test can
// point the node at a temp directory holding those same files -- see
// test/test_pwm_output_node_launch.py.
#ifndef RACER_DRIVERS_PWM_SINK_HPP_
#define RACER_DRIVERS_PWM_SINK_HPP_

#include <string>
#include <vector>

namespace racer_drivers {

/// One PWM output channel. Implementations must be safe to `stop()` more than once.
class PwmChannelSink {
 public:
  virtual ~PwmChannelSink() = default;

  /// Export the channel if needed, set the period, write `initial_duty_ns` (always the
  /// calibrated neutral -- the node never opens a channel at a driving value), then enable.
  /// Throws std::runtime_error on any failure; the node treats that as refuse-to-start.
  virtual void start(unsigned long long period_ns, unsigned long long initial_duty_ns) = 0;

  /// Write one duty cycle. Called from the 50 Hz timer: implementations must not allocate.
  virtual void write_duty_ns(unsigned long long duty_ns) = 0;

  /// Write `neutral_duty_ns`, then disable the channel, in that order. Never leaves a stale
  /// non-neutral pulse behind. Must not throw.
  virtual void stop(unsigned long long neutral_duty_ns) noexcept = 0;
};

/// Test double: records everything, touches nothing.
class InMemoryPwmChannel : public PwmChannelSink {
 public:
  void start(unsigned long long period, unsigned long long initial_duty_ns) override;
  void write_duty_ns(unsigned long long duty_ns) override;
  void stop(unsigned long long neutral_duty_ns) noexcept override;

  bool started{false};
  bool enabled{false};
  unsigned long long period_ns{0};
  std::vector<unsigned long long> duty_writes;  ///< every duty ever written, in order
};

/// The real thing: /sys/class/pwm.
class SysfsPwmChannel : public PwmChannelSink {
 public:
  SysfsPwmChannel(std::string root, int chip, int channel);
  ~SysfsPwmChannel() override;

  SysfsPwmChannel(const SysfsPwmChannel&) = delete;
  SysfsPwmChannel& operator=(const SysfsPwmChannel&) = delete;

  void start(unsigned long long period_ns, unsigned long long initial_duty_ns) override;
  void write_duty_ns(unsigned long long duty_ns) override;
  void stop(unsigned long long neutral_duty_ns) noexcept override;

  /// The pwm<M> directory this channel writes into (for log messages).
  const std::string& channel_dir() const { return channel_dir_; }

 private:
  std::string chip_dir_;
  std::string channel_dir_;
  int channel_;
  /// duty_cycle is held open across the run so the 50 Hz path is one write(2) with no
  /// open/close and no allocation (claude-docs/10-conventions.md: "No heap allocation in the
  /// 50 Hz control path after init").
  int duty_fd_{-1};
};

}  // namespace racer_drivers

#endif  // RACER_DRIVERS_PWM_SINK_HPP_
