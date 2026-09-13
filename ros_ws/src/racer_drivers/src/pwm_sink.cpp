#include "racer_drivers/pwm_sink.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace racer_drivers {

namespace {

bool directory_exists(const std::string& path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
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
    // The kernel creates pwm<M>/ synchronously on the export write, but udev may still be
    // adjusting permissions; a short bounded wait rather than an unbounded retry loop.
    for (int attempt = 0; attempt < 50 && !directory_exists(channel_dir_); ++attempt) {
      ::usleep(10000);
    }
    if (!directory_exists(channel_dir_)) {
      throw std::runtime_error("racer_drivers: exported channel " + std::to_string(channel_) +
                               " but " + channel_dir_ + " never appeared");
    }
  }

  // Order matters: a duty longer than the period is EINVAL, so disable, then set the period,
  // then the (neutral) duty, then enable. The channel is never enabled at anything but
  // neutral.
  write_attribute(channel_dir_ + "/enable", "0");
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
