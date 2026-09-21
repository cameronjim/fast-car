#include "racer_drivers/pwm_sink.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

namespace racer_drivers {

namespace {

bool directory_exists(const std::string& path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

/// The attribute files SysfsPwmChannel::start() writes, relative to the channel directory.
/// `polarity` and `capture` are deliberately absent: this driver never writes them, and the
/// Jetson's own udev rule does not grant them either (see the comment in start()).
const char* const kWrittenAttributes[] = {"enable", "period", "duty_cycle"};

/// Read a sysfs attribute as an unsigned integer. Returns nullopt if it cannot be read or
/// parsed -- callers treat that as "unknown", never as zero.
std::optional<unsigned long long> read_attribute_ull(const std::string& path) {
  std::FILE* file = std::fopen(path.c_str(), "re");
  if (file == nullptr) {
    return std::nullopt;
  }
  unsigned long long value = 0;
  const int scanned = std::fscanf(file, "%llu", &value);
  std::fclose(file);
  if (scanned != 1) {
    return std::nullopt;
  }
  return value;
}

/// Is every attribute this driver writes present AND writable by this process right now?
bool channel_attributes_writable(const std::string& channel_dir) {
  for (const char* name : kWrittenAttributes) {
    if (::access((channel_dir + "/" + name).c_str(), W_OK) != 0) {
      return false;
    }
  }
  return true;
}

/// One-shot open/write/close of a sysfs attribute. Init-path only -- the 50 Hz path uses the
/// long-lived duty_cycle fd instead.
void write_attribute(const std::string& path, const std::string& value) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("racer_drivers: cannot open " + path +
                             " for writing: " + std::strerror(errno));
  }
  const ssize_t written = ::write(fd, value.c_str(), value.size());
  const int write_errno = errno;
  ::close(fd);
  if (written < 0 || static_cast<std::size_t>(written) != value.size()) {
    throw std::runtime_error("racer_drivers: failed writing '" + value + "' to " + path + ": " +
                             std::strerror(write_errno));
  }
}

}  // namespace

bool wait_until(const std::function<bool()>& predicate, int attempts, unsigned int interval_us) {
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (predicate()) {
      return true;
    }
    ::usleep(interval_us);
  }
  return predicate();
}

// -- InMemoryPwmChannel --------------------------------------------------------------------

void InMemoryPwmChannel::start(unsigned long long period, unsigned long long initial_duty_ns) {
  started = true;
  enabled = true;
  period_ns = period;
  duty_writes.push_back(initial_duty_ns);
}

void InMemoryPwmChannel::write_duty_ns(unsigned long long duty_ns) {
  duty_writes.push_back(duty_ns);
}

void InMemoryPwmChannel::stop(unsigned long long neutral_duty_ns) noexcept {
  duty_writes.push_back(neutral_duty_ns);
  enabled = false;
}

// -- SysfsPwmChannel -----------------------------------------------------------------------

SysfsPwmChannel::SysfsPwmChannel(std::string root, int chip, int channel)
    : chip_dir_(root + "/pwmchip" + std::to_string(chip)),
      channel_dir_(root + "/pwmchip" + std::to_string(chip) + "/pwm" + std::to_string(channel)),
      channel_(channel) {}

SysfsPwmChannel::~SysfsPwmChannel() {
  if (duty_fd_ >= 0) {
    ::close(duty_fd_);
    duty_fd_ = -1;
  }
}

void SysfsPwmChannel::start(unsigned long long period_ns, unsigned long long initial_duty_ns) {
  if (!directory_exists(chip_dir_)) {
    throw std::runtime_error(
        "racer_drivers: " + chip_dir_ +
        " does not exist. On the Jetson this means the pinmux has not been switched to PWM "
        "for this pin, or the chip index is wrong -- see ros_ws/src/racer_drivers/README.md, "
        "'Enabling PWM pins on the Jetson'. Refusing to start (a missing PWM chip is not "
        "something to retry blindly on the actuator path).");
  }
  if (!directory_exists(channel_dir_)) {
    write_attribute(chip_dir_ + "/export", std::to_string(channel_));
    // The kernel creates pwm<M>/ synchronously on the export write, but udev applies the
    // group and mode to the new attribute files AFTERWARDS, asynchronously. Waiting only for
    // the directory therefore proves nothing: it is already there on the first check, and the
    // very next write below raced udev and lost.
    //
    // Observed on the Jetson on 2026-09-21 (docs/notes/build-log.md), running unprivileged in
    // the car container: the export succeeded, pwm0/ appeared immediately, and start() died
    // on `cannot open /sys/class/pwm/pwmchip0/pwm0/enable for writing: Permission denied`
    // shortly before udev's own rule (/lib/udev/rules.d/60-jetson-gpio-common.rules, which
    // chgrps period/duty_cycle/enable to the `gpio` group on the pwmchip `change` event) got
    // to them. The previous loop's own comment already said "udev may still be adjusting
    // permissions" -- it just waited for the wrong thing.
    //
    // So wait for what is actually needed: those attributes present AND writable by this
    // process. Same bounded budget as before (50 x 10 ms = 500 ms) and the same
    // refuse-rather-than-retry-forever stance on the actuator path. Running as root this is
    // true on the first check, so the --privileged fallback is unaffected.
    const bool ready = wait_until(
        [this] {
          return directory_exists(channel_dir_) && channel_attributes_writable(channel_dir_);
        },
        50, 10000);
    if (!ready) {
      if (!directory_exists(channel_dir_)) {
        throw std::runtime_error("racer_drivers: exported channel " + std::to_string(channel_) +
                                 " but " + channel_dir_ + " never appeared");
      }
      throw std::runtime_error(
          "racer_drivers: exported channel " + std::to_string(channel_) + " and " + channel_dir_ +
          " appeared, but its enable/period/duty_cycle attributes are still not writable by "
          "this process after 500 ms. In the car container this means the process is not in "
          "the host group the Jetson's udev rule grants PWM write access to -- see "
          "docs/notes/first-boot-runbook.md, 'Launch and drive'.");
    }
  }

  // Order matters: a duty longer than the period is EINVAL, so disable, then set the period,
  // then the (neutral) duty, then enable. The channel is never enabled at anything but
  // neutral.
  //
  // The pre-emptive disable is SKIPPED when the channel's period currently reads 0, because
  // the kernel rejects an `enable` write on a channel with no period: found on the Jetson on
  // 2026-09-21 (docs/notes/build-log.md), where pwmchip2 (header pin 33, throttle) exports
  // with period=0 and start() died on `failed writing '0' to
  // /sys/class/pwm/pwmchip2/pwm0/enable: Invalid argument`. (pwmchip0, the steering pin,
  // happens to export with its 20 ms period already set, which is why only one of the two
  // channels failed and why this went unnoticed until both were driven.)
  //
  // Skipping it there is safe, not a weakening: a period of 0 means the channel is emitting
  // nothing at all, so there is no stale driving pulse for the disable to protect against --
  // which is the only reason it is here. Whenever the period is non-zero (a channel left
  // configured, or enabled, by an earlier run) the disable still happens exactly as before.
  const std::optional<unsigned long long> existing_period_ns =
      read_attribute_ull(channel_dir_ + "/period");
  if (existing_period_ns.value_or(1) != 0) {
    write_attribute(channel_dir_ + "/enable", "0");
  }
  write_attribute(channel_dir_ + "/period", std::to_string(period_ns));
  write_attribute(channel_dir_ + "/duty_cycle", std::to_string(initial_duty_ns));
  write_attribute(channel_dir_ + "/enable", "1");

  const std::string duty_path = channel_dir_ + "/duty_cycle";
  duty_fd_ = ::open(duty_path.c_str(), O_WRONLY | O_CLOEXEC);
  if (duty_fd_ < 0) {
    throw std::runtime_error("racer_drivers: cannot hold " + duty_path +
                             " open: " + std::strerror(errno));
  }
}

void SysfsPwmChannel::write_duty_ns(unsigned long long duty_ns) {
  if (duty_fd_ < 0) {
    throw std::runtime_error("racer_drivers: write_duty_ns before start() on " + channel_dir_);
  }
  // Stack buffer + snprintf: no allocation in the 50 Hz path.
  char buffer[32];
  const int length = std::snprintf(buffer, sizeof(buffer), "%llu", duty_ns);
  if (length <= 0) {
    throw std::runtime_error("racer_drivers: could not format duty cycle");
  }
  if (::lseek(duty_fd_, 0, SEEK_SET) < 0) {
    throw std::runtime_error("racer_drivers: lseek on " + channel_dir_ +
                             "/duty_cycle failed: " + std::strerror(errno));
  }
  const ssize_t written = ::write(duty_fd_, buffer, static_cast<std::size_t>(length));
  if (written != length) {
    throw std::runtime_error("racer_drivers: short write to " + channel_dir_ +
                             "/duty_cycle: " + std::strerror(errno));
  }
  // Harmless no-op on real sysfs (which parses each write independently); required when
  // `root` points at a temp directory of ordinary files, as the L3 launch test's fake sysfs
  // does, so a shorter value cannot leave a longer previous value's tail behind.
  static_cast<void>(::ftruncate(duty_fd_, written));
}

void SysfsPwmChannel::stop(unsigned long long neutral_duty_ns) noexcept {
  // noexcept and best-effort by contract: this runs on the shutdown path and on the
  // fail-closed path, where throwing would be worse than a logged failure. Neutral FIRST,
  // then disable, so the channel is never left holding a driving pulse.
  if (duty_fd_ >= 0) {
    char buffer[32];
    const int length = std::snprintf(buffer, sizeof(buffer), "%llu", neutral_duty_ns);
    if (length > 0 && ::lseek(duty_fd_, 0, SEEK_SET) == 0) {
      const ssize_t written = ::write(duty_fd_, buffer, static_cast<std::size_t>(length));
      if (written > 0) {
        static_cast<void>(::ftruncate(duty_fd_, written));
      }
    }
    ::close(duty_fd_);
    duty_fd_ = -1;
  }
  const std::string enable_path = channel_dir_ + "/enable";
  const int fd = ::open(enable_path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd >= 0) {
    const ssize_t written = ::write(fd, "0", 1);
    static_cast<void>(written);
    ::close(fd);
  }
}

}  // namespace racer_drivers
