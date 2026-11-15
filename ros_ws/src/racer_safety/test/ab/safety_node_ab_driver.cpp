// Deterministic driver for safety_node (brief 45, claude-docs/12-testing.md L3).
//
// WHY THIS EXISTS. The shipped node runs its gate on a wall timer and measures elapsed time on
// the OS monotonic clock, so a launched node can never be fed one identical input sequence twice.
// The brief's proof of zero behaviour change ("/drive and /safety/events bit-identical across
// main, this branch with publish_cycle_record false, and with it true") needs exactly that. This
// file compiles a safety_node source file TEXTUALLY into a test executable and replaces three
// things at the seam, none of which is node logic:
//
//   1. `main` is renamed, so the driver has its own entry point.
//   2. `private` is public while the node source is included, so the driver calls the same
//      callbacks the executor would (on_drive_raw, on_scan, on_odom, on_timer) and reads the
//      same members.
//   3. RCL_STEADY_TIME becomes RCL_ROS_TIME in the node source, and the driver switches that
//      clock to ROS-time override, so the node's "steady" clock is a manual clock the driver
//      sets before every call. Every elapsed-time value the gate sees (the /drive_raw age, dt,
//      the wheel-odometry age, event durations) is then a pure function of the script.
//
// With AB_CAPTURE defined, `rclcpp::Publisher` is also swapped for a recorder, so /drive,
// /safety/events and (when it exists) /safety/cycle are captured in order with no DDS in the
// loop. Without it the node keeps its real publishers; that build measures the cycle's duration.
//
// The same driver source is compiled against the frozen copy of main's safety_node.cpp
// (test/ab/reference/) and against the branch's src/safety_node.cpp. AB_HAVE_CYCLE is defined for
// a source that has the publish_cycle_record parameter.
#include <rcl/time.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <racer_msgs/msg/safety_event.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

#ifdef AB_HAVE_CYCLE
#include <racer_msgs/msg/safety_cycle.hpp>
#endif

namespace rclcpp {

// Parameter overrides handed to the node at construction (the node builds its own NodeOptions).
inline std::vector<rclcpp::Parameter>& ab_overrides() {
  static std::vector<rclcpp::Parameter> overrides;
  return overrides;
}

// Stand-in for rclcpp::Node that only adds the parameter overrides.
class ParamNode : public Node {
 public:
  explicit ParamNode(const std::string& name)
      : Node(name, NodeOptions().parameter_overrides(ab_overrides())) {}
};

#ifdef AB_CAPTURE
// Records every message the node publishes, in order. A static flag makes publish throw, which
// is how the failure-isolation test breaks the record publisher without touching the node.
template <typename M>
class CapturePublisher {
 public:
  using SharedPtr = std::shared_ptr<CapturePublisher<M>>;
  void publish(const M& message) {
    if (throw_on_publish()) {
      throw std::runtime_error("ab driver: injected publish failure");
    }
    log().push_back(message);
  }
  static std::vector<M>& log() {
    static std::vector<M> messages;
    return messages;
  }
  static bool& throw_on_publish() {
    static bool flag = false;
    return flag;
  }
};

class CaptureNode : public ParamNode {
 public:
  using ParamNode::ParamNode;
  template <typename M>
  typename CapturePublisher<M>::SharedPtr create_publisher(const std::string& /*topic*/,
                                                           const rclcpp::QoS& /*qos*/) {
    return std::make_shared<CapturePublisher<M>>();
  }
};
#endif

}  // namespace rclcpp

// ---- the seam: include the node's source file under the three substitutions ------------------
#define main safety_node_original_main
#define private public
#define RCL_STEADY_TIME RCL_ROS_TIME
#ifdef AB_CAPTURE
#define Node CaptureNode
#define Publisher CapturePublisher
#else
#define Node ParamNode
#endif
#include AB_NODE_SOURCE
#ifdef AB_CAPTURE
#undef Publisher
#endif
#undef Node
#undef RCL_STEADY_TIME
#undef private
#undef main

namespace {

using ackermann_msgs::msg::AckermannDriveStamped;
using racer_msgs::msg::SafetyEvent;
using racer_safety::SafetyNode;

constexpr int64_t kStartNs = 5'000'000'000;  // the manual clock's origin, 5 s
constexpr int64_t kCyclePeriodNs = 20'000'000;
constexpr int64_t kDeliveryLeadNs = 8'000'000;  // inputs arrive 8 ms before the cycle
constexpr int kTotalCycles = 2400;

// Manual "steady" time of cycle k: a 20 ms grid plus a small deterministic jitter, so dt is not
// a constant (the rate limiter and the event durations see varying values).
int64_t cycle_time_ns(int k) {
  return kStartNs + static_cast<int64_t>(k) * kCyclePeriodNs +
         static_cast<int64_t>((k * 37) % 7) * 100'000;
}

uint32_t float_bits(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

uint64_t double_bits(double value) {
  uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

// Deterministic generator for the mixed stretch of the script.
class Lcg {
 public:
  explicit Lcg(uint64_t seed) : state_(seed) {}
  uint32_t next() {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<uint32_t>(state_ >> 33);
  }
  double unit() { return static_cast<double>(next() % 100000) / 100000.0; }

 private:
  uint64_t state_;
};

// One scripted cycle: what arrives before the timer fires and what the node is told.
struct CycleScript {
  bool deliver_command{false};
  double speed_mps{0.0};
  double steering_rad{0.0};
  bool deliver_scan{false};
  std::optional<double> wall_ahead_m;  // along-track distance of a wall ahead, none for clear
  std::optional<double> wall_rear_m;   // along-track distance of a wall behind
  bool deliver_odom{false};
  double odom_speed_mps{0.0};
  bool inject_fault{false};
};

std::vector<CycleScript> build_script() {
  std::vector<CycleScript> script(kTotalCycles);
  std::optional<double> wall_ahead;
  std::optional<double> wall_rear;
  Lcg lcg(20261114);
  // Held values for the mixed stretch.
  double mix_speed = 0.0;
  double mix_steer = 0.0;
  for (int k = 0; k < kTotalCycles; ++k) {
    CycleScript& c = script[static_cast<size_t>(k)];
    // The scan is re-delivered at 10 Hz (every fifth cycle) with the wall as the phase sets it.
    c.deliver_scan = (k % 5 == 0);
    if (k < 40) {
      // No command yet: watchdog on the "never received" path.
    } else if (k < 300) {
      // Pass-through: a gentle speed ramp and a slow steering sweep, inside every limit.
      c.deliver_command = true;
      c.speed_mps = std::min(1.0, 0.02 * (k - 40));
      c.steering_rad = 0.1 * std::sin(2.0 * 3.14159265358979 * k / 100.0);
    } else if (k < 340) {
      // rate_limit: a speed step far above max_acceleration * dt, a steering flip above the
      // steering rate limit.
      c.deliver_command = true;
      c.speed_mps = (k < 320) ? 4.0 : 1.0;
      c.steering_rad = (k < 320) ? 0.3 : -0.3;
    } else if (k < 420) {
      c.deliver_command = true;
      c.speed_mps = 1.0;
      c.steering_rad = 0.0;
    } else if (k < 460) {
      // bounds_clamp: speed above the global cap, steering beyond the servo limit.
      c.deliver_command = true;
      c.speed_mps = (k < 440) ? 25.0 : 1.0;
      c.steering_rad = (k < 440) ? 0.9 : 0.0;
    } else if (k < 520) {
      c.deliver_command = true;
      c.speed_mps = 1.0;
    } else if (k < 560) {
      // watchdog: /drive_raw falls silent.
    } else if (k < 620) {
      c.deliver_command = true;
      c.speed_mps = 1.0;
    } else if (k < 760) {
      // ttc: a wall 0.5 m ahead against a 2 m/s request, then the wall goes away and the latch
      // releases. The wheel speed is reported fresh while the wall is there.
      c.deliver_command = true;
      c.speed_mps = 2.0;
      if (k < 700) {
        wall_ahead = 0.5;
        c.deliver_odom = true;
        c.odom_speed_mps = 1.5;
      } else {
        wall_ahead = std::nullopt;
      }
    } else if (k < 880) {
      c.deliver_command = true;
      c.speed_mps = 1.0;
    } else if (k < 1000) {
      // A stale odometry stretch: wheel speed fresh at 2.5 m/s to cycle 940, then no odometry
      // at all (stale after wheel_odom_timeout_s) while a wall sits at 0.9 m and the request
      // is 1.0 m/s.
      c.deliver_command = true;
      c.speed_mps = 1.0;
      wall_ahead = (k >= 900) ? std::optional<double>(0.9) : std::nullopt;
      if (k < 940) {
        c.deliver_odom = true;
        c.odom_speed_mps = 2.5;
      }
      if (k >= 990) {
        wall_ahead = std::nullopt;
      }
    } else if (k < 1100) {
      c.deliver_command = true;
      c.speed_mps = 0.5;
    } else if (k < 1160) {
      // ttc_reverse: a reverse request toward a wall 0.4 m behind the car.
      c.deliver_command = true;
      c.speed_mps = -0.8;
      wall_rear = 0.4;
    } else if (k < 1280) {
      wall_rear = std::nullopt;
      c.deliver_command = true;
      c.speed_mps = 0.5;
    } else if (k < 1340) {
      // The injected internal fault with the obstacle latch engaged (the fault path holds it).
      c.deliver_command = true;
      c.speed_mps = 1.0;
      wall_ahead = (k < 1330) ? std::optional<double>(0.5) : std::nullopt;
      c.inject_fault = (k >= 1300 && k < 1320);
    } else if (k < 1500) {
      c.deliver_command = true;
      c.speed_mps = 0.5;
    } else if (k < 1530) {
      // A plain internal fault, no latch.
      c.deliver_command = true;
      c.speed_mps = 0.5;
      c.inject_fault = (k >= 1500 && k < 1520);
    } else if (k < 1600) {
      c.deliver_command = true;
      c.speed_mps = 0.5;
    } else {
      // The mixed stretch: random commands that change every 3 cycles, delivered about 85 per
      // cent of the time (gaps of 3 or more starve the watchdog now and then), a wall that
      // comes and goes, an occasional odometry message.
      if (k % 3 == 0) {
        mix_speed = -1.5 + 4.5 * lcg.unit();
        mix_steer = -0.5 + 1.0 * lcg.unit();
      }
      c.deliver_command = lcg.unit() < 0.85;
      c.speed_mps = mix_speed;
      c.steering_rad = mix_steer;
      if (k % 25 == 0) {
        wall_ahead =
            (lcg.unit() < 0.5) ? std::optional<double>(0.3 + 2.0 * lcg.unit()) : std::nullopt;
        wall_rear =
            (lcg.unit() < 0.3) ? std::optional<double>(0.3 + 1.0 * lcg.unit()) : std::nullopt;
      }
      if (lcg.unit() < 0.4) {
        c.deliver_odom = true;
        c.odom_speed_mps = -1.0 + 4.0 * lcg.unit();
      }
    }
    c.wall_ahead_m = wall_ahead;
    c.wall_rear_m = wall_rear;
  }
  return script;
}

// A 360 beam scan aligned to the vehicle (laser_yaw_from_vehicle_params is false here), clear
// except for a flat wall ahead and/or behind at the given along-track distance.
sensor_msgs::msg::LaserScan make_scan(const CycleScript& c) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
  scan.angle_max = scan.angle_min + scan.angle_increment * 359.0F;
  scan.range_min = 0.05F;
  scan.range_max = 12.0F;
  scan.ranges.assign(360, std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double bearing = static_cast<double>(scan.angle_min) +
                           static_cast<double>(scan.angle_increment) * static_cast<double>(i);
    const double cosine = std::cos(bearing);
    if (c.wall_ahead_m.has_value() && std::fabs(bearing) < 0.6) {
      scan.ranges[i] = static_cast<float>(*c.wall_ahead_m / cosine);
    }
    if (c.wall_rear_m.has_value() && std::fabs(bearing) > M_PI - 0.6) {
      scan.ranges[i] = static_cast<float>(*c.wall_rear_m / -cosine);
    }
  }
  return scan;
}

builtin_interfaces::msg::Time command_stamp(int k) {
  builtin_interfaces::msg::Time stamp;
  const int64_t ns = 1'700'000'000'000'000'000LL + static_cast<int64_t>(k) * 20'000'123;
  stamp.sec = static_cast<int32_t>(ns / 1'000'000'000);
  stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000);
  return stamp;
}

struct Options {
  std::string out;
  bool cycle_record{false};
  bool throw_record{false};
  int timing_cycles{0};
};

Options parse_args(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--out" && i + 1 < argc) {
      options.out = argv[++i];
    } else if (arg == "--cycle-record") {
      options.cycle_record = true;
    } else if (arg == "--throw-record") {
      options.throw_record = true;
    } else if (arg == "--timing" && i + 1 < argc) {
      options.timing_cycles = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      std::exit(2);
    }
  }
  return options;
}

void make_overrides(const Options& options) {
  // The simulator setting: the scan is aligned to the vehicle, so the scripted walls need no
  // mount yaw. Identical for every variant.
  rclcpp::ab_overrides().emplace_back("laser_yaw_from_vehicle_params", false);
  rclcpp::ab_overrides().emplace_back("laser_yaw_offset_rad", 0.0);
#ifdef AB_HAVE_CYCLE
  if (options.cycle_record) {
    rclcpp::ab_overrides().emplace_back("publish_cycle_record", true);
  }
#else
  (void)options;
#endif
}

struct ManualClock {
  rcl_clock_t* handle;
  void set(int64_t ns) const {
    if (rcl_set_ros_time_override(handle, ns) != RCL_RET_OK) {
      throw std::runtime_error("ab driver: cannot set the manual clock");
    }
  }
};

// Feeds one scripted cycle to the node, in the order the callbacks would run: inputs 8 ms before
// the gate cycle, then the gate cycle itself.
void run_cycle(SafetyNode& node, const ManualClock& clock, const CycleScript& c, int k) {
  clock.set(cycle_time_ns(k) - kDeliveryLeadNs);
  if (c.deliver_command) {
    auto message = std::make_shared<AckermannDriveStamped>();
    message->header.stamp = command_stamp(k);
    message->drive.speed = static_cast<float>(c.speed_mps);
    message->drive.steering_angle = static_cast<float>(c.steering_rad);
    node.on_drive_raw(message);
  }
  if (c.deliver_odom) {
    auto message = std::make_shared<nav_msgs::msg::Odometry>();
    message->twist.twist.linear.x = c.odom_speed_mps;
    node.on_odom(message);
  }
  if (c.deliver_scan) {
    node.on_scan(std::make_shared<sensor_msgs::msg::LaserScan>(make_scan(c)));
  }
  clock.set(cycle_time_ns(k));
  node.on_timer();
}

#ifdef AB_CAPTURE

std::string event_line(int k, const SafetyEvent& event) {
  std::ostringstream line;
  line << "E " << k << ' ' << event.source << '|' << static_cast<int>(event.phase) << '|'
       << static_cast<int>(event.severity) << '|' << std::hex << double_bits(event.duration_s)
       << std::dec << '|' << event.detail;
  return line.str();
}

int run_trace(const Options& options) {
  make_overrides(options);
  auto node = std::make_shared<SafetyNode>();
  rcl_clock_t* handle = node->steady_clock_.get_clock_handle();
  if (rcl_enable_ros_time_override(handle) != RCL_RET_OK) {
    throw std::runtime_error("ab driver: cannot enable the manual clock");
  }
  const ManualClock clock{handle};
  clock.set(kStartNs);
  const std::vector<CycleScript> script = build_script();

  auto& drive_log = rclcpp::CapturePublisher<AckermannDriveStamped>::log();
  auto& event_log = rclcpp::CapturePublisher<SafetyEvent>::log();
#ifdef AB_HAVE_CYCLE
  auto& cycle_log = rclcpp::CapturePublisher<racer_msgs::msg::SafetyCycle>::log();
  rclcpp::CapturePublisher<racer_msgs::msg::SafetyCycle>::throw_on_publish() = options.throw_record;
#endif

  std::ostringstream trace;
  int record_mismatches = 0;
  size_t drive_seen = 0;
  size_t event_seen = 0;
#ifdef AB_HAVE_CYCLE
  size_t cycle_seen = 0;
  uint32_t delivered = 0;
  builtin_interfaces::msg::Time last_stamp;
#endif

  for (int k = 0; k < kTotalCycles; ++k) {
    const CycleScript& c = script[static_cast<size_t>(k)];
    node->set_parameter(rclcpp::Parameter("inject_fault", c.inject_fault));
    run_cycle(*node, clock, c, k);

    // One /drive per cycle, always, on the nominal and the fault path alike.
    if (drive_log.size() != drive_seen + 1) {
      std::printf("FAIL cycle %d published %zu /drive messages\n", k,
                  drive_log.size() - drive_seen);
      return 1;
    }
    const AckermannDriveStamped& drive = drive_log.back();
    trace << "D " << k << ' ' << std::hex << float_bits(drive.drive.speed) << ' '
          << float_bits(drive.drive.steering_angle) << std::dec << '\n';
    drive_seen = drive_log.size();
    for (; event_seen < event_log.size(); ++event_seen) {
      trace << event_line(k, event_log[event_seen]) << '\n';
    }

#ifdef AB_HAVE_CYCLE
    if (c.deliver_command) {
      ++delivered;
      last_stamp = command_stamp(k);
    }
    if (options.cycle_record && !options.throw_record) {
      if (cycle_log.size() != cycle_seen + 1) {
        std::printf("FAIL cycle %d published %zu /safety/cycle records\n", k,
                    cycle_log.size() - cycle_seen);
        return 1;
      }
      const racer_msgs::msg::SafetyCycle& record = cycle_log.back();
      cycle_seen = cycle_log.size();
      const bool has_command = delivered > 0;
      // The expectations come from the script, not from the node: the manual clock makes the
      // age and the dt exact (the same int64 -> double conversion as rclcpp::Duration::seconds).
      double expected_age = -1.0;
      if (has_command) {
        int last_delivery = k;
        while (!script[static_cast<size_t>(last_delivery)].deliver_command) {
          --last_delivery;
        }
        const int64_t arrival_ns = cycle_time_ns(last_delivery) - kDeliveryLeadNs;
        expected_age =
            rclcpp::Duration(std::chrono::nanoseconds(cycle_time_ns(k) - arrival_ns)).seconds();
      }
      const double expected_dt =
          (k == 0)
              ? (1.0 / 50.0)
              : rclcpp::Duration(std::chrono::nanoseconds(cycle_time_ns(k) - cycle_time_ns(k - 1)))
                    .seconds();
      auto check = [&](bool ok, const char* field) {
        if (!ok) {
          ++record_mismatches;
          trace << "RECORD_MISMATCH " << k << ' ' << field << '\n';
        }
      };
      check(record.stamp == drive.header.stamp, "stamp");
      check(record.has_command == has_command, "has_command");
      const builtin_interfaces::msg::Time zero_stamp;
      check(record.drive_raw_stamp == (has_command ? last_stamp : zero_stamp), "drive_raw_stamp");
      check(record.drive_raw_count == delivered, "drive_raw_count");
      check(double_bits(record.drive_raw_age_s) == double_bits(expected_age), "drive_raw_age_s");
      check(double_bits(record.cycle_dt_s) == double_bits(expected_dt), "cycle_dt_s");
      trace << "R " << k << ' ' << record.has_command << ' ' << record.drive_raw_stamp.sec << '.'
            << record.drive_raw_stamp.nanosec << ' ' << record.drive_raw_count << ' ' << std::hex
            << double_bits(record.drive_raw_age_s) << ' ' << double_bits(record.cycle_dt_s)
            << std::dec << '\n';
    } else if (options.cycle_record && options.throw_record) {
      if (!cycle_log.empty()) {
        std::printf("FAIL a record was logged while its publish was made to throw\n");
        return 1;
      }
    }
#endif
  }
#ifdef AB_HAVE_CYCLE
  if (options.cycle_record && options.throw_record) {
    // Every cycle's record failed; the node counted each one and carried on.
    const uint64_t failures = node->cycle_record_failures_;
    trace << "FAILURES " << failures << '\n';
  }
#endif
  trace << "CYCLES " << kTotalCycles << '\n';
  trace << "RECORD_MISMATCHES " << record_mismatches << '\n';

  std::ofstream out(options.out);
  out << trace.str();
  return out.good() ? 0 : 1;
}

#else  // timing build: real publishers, real subscribers on the same topics

int run_timing(const Options& options) {
  make_overrides(options);
  auto node = std::make_shared<SafetyNode>();
  rcl_clock_t* handle = node->steady_clock_.get_clock_handle();
  if (rcl_enable_ros_time_override(handle) != RCL_RET_OK) {
    throw std::runtime_error("ab driver: cannot enable the manual clock");
  }
  const ManualClock clock{handle};
  clock.set(kStartNs);

  // A recorder-like consumer of every topic the node publishes, on its own thread, so the
  // measured cycle includes real publishes with real subscribers.
  auto sink = std::make_shared<rclcpp::Node>("ab_timing_sink");
  std::atomic<uint64_t> received{0};
  const auto count = [&received](auto) { received.fetch_add(1); };
  auto drive_sub = sink->create_subscription<AckermannDriveStamped>(
      "/drive", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), count);
  auto event_sub = sink->create_subscription<SafetyEvent>(
      "/safety/events", rclcpp::QoS(rclcpp::KeepLast(50)).reliable(), count);
#ifdef AB_HAVE_CYCLE
  auto cycle_sub = sink->create_subscription<racer_msgs::msg::SafetyCycle>(
      "/safety/cycle", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), count);
#endif
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sink);
  std::atomic<bool> stop{false};
  std::thread spinner([&]() {
    while (!stop.load()) {
      executor.spin_once(std::chrono::milliseconds(5));
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // discovery

  // Steady input: a command every cycle, a scan every fifth, a gentle steering sweep, a wall far
  // enough away that nothing engages (the common case on a drive).
  std::vector<double> durations_ns;
  durations_ns.reserve(static_cast<size_t>(options.timing_cycles));
  for (int k = 0; k < options.timing_cycles; ++k) {
    CycleScript c;
    c.deliver_command = true;
    c.speed_mps = std::min(1.5, 0.02 * k);
    c.steering_rad = 0.1 * std::sin(2.0 * M_PI * k / 100.0);
    c.deliver_scan = (k % 5 == 0);
    c.wall_ahead_m = 3.0;
    c.deliver_odom = (k % 2 == 0);
    c.odom_speed_mps = c.speed_mps;
    // Inputs are delivered outside the timed region; only on_timer is measured.
    clock.set(cycle_time_ns(k) - kDeliveryLeadNs);
    if (c.deliver_command) {
      auto message = std::make_shared<AckermannDriveStamped>();
      message->header.stamp = command_stamp(k);
      message->drive.speed = static_cast<float>(c.speed_mps);
      message->drive.steering_angle = static_cast<float>(c.steering_rad);
      node->on_drive_raw(message);
    }
    if (c.deliver_odom) {
      auto message = std::make_shared<nav_msgs::msg::Odometry>();
      message->twist.twist.linear.x = c.odom_speed_mps;
      node->on_odom(message);
    }
    if (c.deliver_scan) {
      node->on_scan(std::make_shared<sensor_msgs::msg::LaserScan>(make_scan(c)));
    }
    clock.set(cycle_time_ns(k));
    const auto begin = std::chrono::steady_clock::now();
    node->on_timer();
    const auto end = std::chrono::steady_clock::now();
    durations_ns.push_back(static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()));
  }
  stop.store(true);
  spinner.join();

  std::sort(durations_ns.begin(), durations_ns.end());
  const auto percentile = [&](double q) {
    return durations_ns[static_cast<size_t>(q * static_cast<double>(durations_ns.size() - 1))];
  };
  std::printf("TIMING cycles=%d p50_ns=%.0f p99_ns=%.0f max_ns=%.0f received=%llu\n",
              options.timing_cycles, percentile(0.50), percentile(0.99), durations_ns.back(),
              static_cast<unsigned long long>(received.load()));
  return 0;
}

#endif

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const Options options = parse_args(argc, argv);
  int status = 0;
  try {
#ifdef AB_CAPTURE
    status = run_trace(options);
#else
    status = run_timing(options);
#endif
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ab driver: %s\n", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
