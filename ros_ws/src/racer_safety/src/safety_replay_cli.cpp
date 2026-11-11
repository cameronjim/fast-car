// safety_replay_cli: offline replay of a recorded /scan + /drive_raw (+ optional /odom/wheel)
// stream through racer_safety's REAL gate: the same SafetyGateLogic, min_path_distance_m and
// min_rear_path_distance_m safety_node runs, on a 50 Hz gate clock, with the latch, the steering
// hold, the watchdog and the speed bound threaded from cycle to cycle exactly as safety_node
// threads them. tools/replay_safety.py drives it from an mcap bag (docs/notes/
// safety-floor-review-2026-11-02.md). No ROS dependency, nothing here drives a car.
//
// usage: safety_replay_cli [--stream <file>] [--set key=value]... [--cycles] [--describe]
//
// Stream: a plain text file (or stdin), one message per line, in time order:
//     S stamp_s angle_min_rad angle_increment_rad range_min_m range_max_m n r0 ... r(n-1)
//     D stamp_s steering_rad speed_mps          (a /drive_raw)
//     O stamp_s speed_mps                       (an /odom/wheel twist.linear.x)
// Numbers are anything strtod reads (inf, nan). Blank lines and # comments are skipped.
//
// The gate clock starts at the first message's stamp and ticks every 1 / control_rate_hz.
// Before each tick every message stamped at or before it is applied the way safety_node's
// callbacks apply them (the last scan and the last command are kept; the command's age is
// measured from its stamp), then the gate runs once: the last scan is reduced along the cached
// request's steering, forward and rear, and evaluate() is called with the threaded state. An
// odometry speed counts while it is at most odom_timeout_s old, as in safety_node.
//
// Parameters (--set, all optional): the defaults are what safety_node runs with, from the
// generated vehicle_params binding (CLAUDE.md invariant 2); --describe prints them as JSON.
//   ttc_brake_s, ttc_warning_s               s; <= 0 disables, as the node's parameters do
//   min_forward_clearance_m                  m; <= 0 disables the floor
//   distance_from_body                       true/false (forward_sector.hpp "DISTANCE FROM
//                                            THE BODY"; false is the pre-0.13.0 head reference)
//   speed_bound_deceleration_mps2            m/s^2; <= 0 turns the output bound off
//   use_odometry, odom_timeout_s             O lines used or ignored; their freshness
//   outer_corner_horizon_m, obstacle_corridor_margin_m, front_overhang_m, rear_overhang_m,
//   ttc_forward_sector_half_angle_rad, obstacle_steering_hold_after_s, laser_yaw_rad,
//   watchdog_missed_cycles, control_rate_hz  as in vehicle_params / safety_node
//   stop_latency_s, stop_deceleration_mps2   the stopping model for predicted_clearance_m
//   max_steering_rad                         the upper steering clamp and the arc corridor's lock
//
// Output, one JSON object per line on stdout:
//   {"type":"event", ...} every /safety/events record the node would publish (phase engage or
//     release, source, severity, detail, duration_s) at gate time t.
//   {"type":"trip", ...} on the cycle an obstacle gate trips (its latch goes from clear to
//     set): gate (ttc / ttc_reverse), cause (ttc / floor), distance_m (the in-path distance the
//     gate judged), request (steering_rad, speed_mps), ttc_speed_mps (the speed the TTC used),
//     stop_speed_mps (the measured speed while an O line is fresh, else ttc_speed_mps),
//     predicted_stop_m = stop_speed * stop_latency_s + stop_speed^2 / (2 stop_deceleration_mps2)
//     and predicted_clearance_m = distance_m - predicted_stop_m: how far from the obstacle, in
//     the same reference as distance_m, the car is predicted to come to rest. Without odometry
//     the TTC speed is the request, which overstates the stop whenever the car is slower than
//     asked (tools/replay_safety.py measures the real travel from the scans instead).
//   {"type":"cycle", ...} with --cycles: every gate cycle (request, output, both distances,
//     latches, the speed bound).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

namespace {

using racer_safety::DriveCommand;
using racer_safety::EventPhase;
using racer_safety::EventSeverity;
using racer_safety::GateResult;
using racer_safety::GateSource;
using racer_safety::SafetyEventRecord;

constexpr double kInf = std::numeric_limits<double>::infinity();

[[noreturn]] void die(int code, const std::string& message) {
  std::cerr << "safety_replay_cli: " << message << "\n";
  std::exit(code);
}

std::string trim(const std::string& s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::optional<double> parse_number(const std::string& text) {
  const std::string t = trim(text);
  if (t == "true") {
    return 1.0;
  }
  if (t == "false") {
    return 0.0;
  }
  if (t.empty()) {
    return std::nullopt;
  }
  char* end = nullptr;
  const double v = std::strtod(t.c_str(), &end);
  if (end != t.c_str() + t.size()) {
    return std::nullopt;
  }
  return v;
}

// key -> (value, min, max). Defaults from the binding, as safety_node takes them.
struct Param {
  double value;
  double min_value;
  double max_value;
};

std::map<std::string, Param> default_params() {
  const auto& vp = VEHICLE_PARAMS;
  std::map<std::string, Param> p;
  p["ttc_brake_s"] = {vp.limits.ttc_brake_s.value_or(-1.0), -1.0, 30.0};
  p["ttc_warning_s"] = {vp.limits.ttc_warning_s.value_or(-1.0), -1.0, 30.0};
  p["min_forward_clearance_m"] = {vp.limits.min_forward_clearance_m, -1.0, 10.0};
  p["distance_from_body"] = {1.0, 0.0, 1.0};
  p["speed_bound_deceleration_mps2"] = {vp.actuation.stop_deceleration_mps2, -1.0, 100.0};
  p["use_odometry"] = {1.0, 0.0, 1.0};
  p["odom_timeout_s"] = {0.1, 1e-3, 10.0};
  p["outer_corner_horizon_m"] = {vp.limits.outer_corner_horizon_m, 1e-3, 100.0};
  p["obstacle_corridor_margin_m"] = {vp.limits.obstacle_corridor_margin_m, 0.0, 2.0};
  p["front_overhang_m"] = {vp.chassis.front_overhang_m, 0.0, 2.0};
  p["rear_overhang_m"] = {vp.chassis.rear_overhang_m, 0.0, 2.0};
  p["ttc_forward_sector_half_angle_rad"] = {vp.limits.ttc_forward_sector_half_angle_rad, 1e-3, 3.2};
  p["obstacle_steering_hold_after_s"] = {vp.limits.obstacle_steering_hold_after_s, 0.0, 1e6};
  p["laser_yaw_rad"] = {vp.sensors.lidar.mount_yaw_rad.value_or(0.0), -4.0, 4.0};
  p["watchdog_missed_cycles"] = {3.0, 1.0, 100.0};
  p["control_rate_hz"] = {50.0, 1.0, 1000.0};
  p["stop_latency_s"] = {vp.actuation.stop_latency_s, 0.0, 10.0};
  p["stop_deceleration_mps2"] = {vp.actuation.stop_deceleration_mps2, 1e-3, 100.0};
  // The steering range the gate clamps a request to and bends the arc corridor with. The replay
  // of a bag recorded before the 2026-11-10 calibration needs the wider real lock of its
  // recorded commands (tools/safety_replay/calibration.py), not the planning limit.
  p["max_steering_rad"] = {vp.steering.max_angle_rad, 1e-3, 1.5708};
  return p;
}

void set_param(std::map<std::string, Param>& params, const std::string& assignment) {
  const auto eq = assignment.find('=');
  if (eq == std::string::npos) {
    die(2, "--set wants key=value, got '" + assignment + "'");
  }
  const std::string key = trim(assignment.substr(0, eq));
  const auto it = params.find(key);
  if (it == params.end()) {
    die(2, "unknown parameter '" + key + "'");
  }
  const auto value = parse_number(assignment.substr(eq + 1));
  if (!value || !std::isfinite(*value)) {
    die(2, "parameter '" + key + "' has a non-numeric value");
  }
  if (*value < it->second.min_value || *value > it->second.max_value) {
    std::ostringstream m;
    m << "parameter '" << key << "' = " << *value << " is outside [" << it->second.min_value << ", "
      << it->second.max_value << "]";
    die(2, m.str());
  }
  it->second.value = *value;
}

std::string json_number(double v) {
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

// Stamps are epoch seconds: %.6f keeps the microseconds that %g would drop.
std::string json_time(double t) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6f", t);
  return buf;
}

std::string json_string(const std::string& s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + "\"";
}

struct Scan {
  racer_safety::ScanGeometry geometry;
  std::vector<float> ranges;
};

struct Message {
  char kind = 0;
  double stamp_s = 0.0;
  double a = 0.0;  // D: steering, O: speed
  double b = 0.0;  // D: speed
  Scan scan;
};

std::optional<Message> parse_line(const std::string& raw, std::size_t line_number) {
  const std::string line = trim(raw);
  if (line.empty() || line[0] == '#') {
    return std::nullopt;
  }
  std::istringstream tokens(line);
  std::vector<std::string> w;
  for (std::string t; tokens >> t;) {
    w.push_back(t);
  }
  auto fail = [&](const std::string& why) {
    die(3, "stream line " + std::to_string(line_number) + ": " + why);
  };
  auto num = [&](std::size_t i) {
    const auto v = parse_number(w[i]);
    if (!v) {
      fail("bad number '" + w[i] + "'");
    }
    return *v;
  };
  Message m;
  m.kind = w[0].size() == 1 ? w[0][0] : '?';
  if (m.kind == 'D') {
    if (w.size() != 4) {
      fail("D wants stamp steering speed");
    }
    m.stamp_s = num(1);
    m.a = num(2);
    m.b = num(3);
  } else if (m.kind == 'O') {
    if (w.size() != 3) {
      fail("O wants stamp speed");
    }
    m.stamp_s = num(1);
    m.a = num(2);
  } else if (m.kind == 'S') {
    if (w.size() < 7) {
      fail("S wants stamp angle_min increment range_min range_max n ranges...");
    }
    m.stamp_s = num(1);
    m.scan.geometry.angle_min_rad = num(2);
    m.scan.geometry.angle_increment_rad = num(3);
    m.scan.geometry.range_min_m = num(4);
    m.scan.geometry.range_max_m = num(5);
    const double n = num(6);
    if (n < 0 || n != std::floor(n) || w.size() != 7 + static_cast<std::size_t>(n)) {
      fail("n does not match the number of ranges");
    }
    m.scan.ranges.reserve(static_cast<std::size_t>(n));
    for (std::size_t k = 7; k < w.size(); ++k) {
      m.scan.ranges.push_back(static_cast<float>(num(k)));
    }
  } else {
    fail("unknown message kind '" + w[0] + "' (S, D or O)");
  }
  if (!std::isfinite(m.stamp_s)) {
    fail("non-finite stamp");
  }
  return m;
}

}  // namespace

int main(int argc, char** argv) {
  auto params = default_params();
  std::string stream_path;
  bool cycles = false;
  bool describe = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--stream" && i + 1 < argc) {
      stream_path = argv[++i];
    } else if (arg == "--set" && i + 1 < argc) {
      set_param(params, argv[++i]);
    } else if (arg == "--cycles") {
      cycles = true;
    } else if (arg == "--describe") {
      describe = true;
    } else {
      die(2,
          "usage: safety_replay_cli [--stream file] [--set key=value]... [--cycles] "
          "[--describe]");
    }
  }
  auto value = [&](const char* key) { return params.at(key).value; };
  if (describe) {
    std::cout << "{";
    bool first = true;
    for (const auto& [key, p] : params) {
      std::cout << (first ? "" : ", ") << json_string(key) << ": " << json_number(p.value);
      first = false;
    }
    std::cout << ", \"wheelbase_m\": " << json_number(VEHICLE_PARAMS.chassis.wheelbase_m)
              << ", \"width_m\": " << json_number(VEHICLE_PARAMS.chassis.width_m)
              << ", \"lidar_mount_x_m\": "
              << json_number(VEHICLE_PARAMS.sensors.lidar.mount_x_m.value_or(0.0))
              << ", \"lidar_mount_y_m\": "
              << json_number(VEHICLE_PARAMS.sensors.lidar.mount_y_m.value_or(0.0))
              << ", \"schema_version\": " << json_string(VEHICLE_PARAMS.meta.schema_version)
              << "}\n";
    return 0;
  }

  // The gate exactly as safety_node builds it (build_limits / configure_scan_sector), with the
  // overrides on top.
  racer_safety::SafetyLimits limits;
  limits.steering_min_rad = VEHICLE_PARAMS.steering.min_angle_rad;
  limits.steering_max_rad = value("max_steering_rad");
  limits.steering_rate_min_rad_per_s = VEHICLE_PARAMS.steering.min_rate_rad_per_s;
  limits.steering_rate_max_rad_per_s = VEHICLE_PARAMS.steering.max_rate_rad_per_s;
  limits.speed_min_mps = VEHICLE_PARAMS.limits.min_velocity_mps;
  limits.speed_max_mps = VEHICLE_PARAMS.limits.global_speed_cap_mps;
  limits.max_acceleration_mps2 = VEHICLE_PARAMS.actuation.max_acceleration_mps2;
  if (value("ttc_brake_s") > 0.0) {
    limits.ttc_brake_s = value("ttc_brake_s");
  }
  if (value("ttc_warning_s") > 0.0) {
    limits.ttc_warning_s = value("ttc_warning_s");
  }
  if (value("min_forward_clearance_m") > 0.0) {
    limits.min_forward_clearance_m = value("min_forward_clearance_m");
  }
  if (value("speed_bound_deceleration_mps2") > 0.0) {
    limits.speed_bound_deceleration_mps2 = value("speed_bound_deceleration_mps2");
  }
  limits.obstacle_steering_hold_after_s = value("obstacle_steering_hold_after_s");
  const double control_period_s = 1.0 / value("control_rate_hz");
  limits.watchdog_missed_cycles = static_cast<int>(value("watchdog_missed_cycles"));
  limits.control_period_s = control_period_s;
  const racer_safety::SafetyGateLogic gate(limits);

  racer_safety::PathGeometry path;
  path.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
  path.max_steering_angle_rad = value("max_steering_rad");
  path.lidar_mount_x_m = VEHICLE_PARAMS.sensors.lidar.mount_x_m.value_or(0.0);
  path.lidar_mount_y_m = VEHICLE_PARAMS.sensors.lidar.mount_y_m.value_or(0.0);
  path.body_front_x_m = VEHICLE_PARAMS.chassis.wheelbase_m + value("front_overhang_m");
  path.outer_corner_horizon_m = value("outer_corner_horizon_m");
  path.distance_from_body = value("distance_from_body") > 0.5;
  const double rear_overhang_m = value("rear_overhang_m");
  const double half_width_m = racer_safety::corridor_half_width_m(
      VEHICLE_PARAMS.chassis.width_m, value("obstacle_corridor_margin_m"));
  const double half_angle_rad = value("ttc_forward_sector_half_angle_rad");
  const double laser_yaw_rad = value("laser_yaw_rad");
  const bool use_odometry = value("use_odometry") > 0.5;
  const double odom_timeout_s = value("odom_timeout_s");
  const double stop_latency_s = value("stop_latency_s");
  const double stop_decel_mps2 = value("stop_deceleration_mps2");

  std::ifstream file;
  std::istream* in = &std::cin;
  if (!stream_path.empty()) {
    file.open(stream_path);
    if (!file) {
      die(2, "cannot open stream file " + stream_path);
    }
    in = &file;
  }
  std::vector<Message> messages;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(*in, line)) {
    ++line_number;
    if (auto m = parse_line(line, line_number)) {
      if (!messages.empty() && m->stamp_s < messages.back().stamp_s) {
        die(3, "stream line " + std::to_string(line_number) + ": stamps go backwards");
      }
      messages.push_back(std::move(*m));
    }
  }
  if (messages.empty()) {
    return 0;
  }

  // safety_node's threaded state.
  DriveCommand previous{0.0, 0.0};
  bool latched = false;
  bool reverse_latched = false;
  std::optional<double> hold_timer_s;
  double bound_forward = 0.0;
  double bound_reverse = 0.0;
  racer_safety::GateEventTracker tracker;
  std::optional<DriveCommand> command;
  double command_stamp_s = 0.0;
  const Scan* scan = nullptr;
  std::optional<double> odom_mps;
  double odom_stamp_s = 0.0;

  const double start_s = messages.front().stamp_s;
  const double end_s = messages.back().stamp_s;
  std::size_t next = 0;
  for (long tick = 0;; ++tick) {
    const double now_s = start_s + static_cast<double>(tick) * control_period_s;
    if (now_s > end_s + control_period_s) {
      break;
    }
    while (next < messages.size() && messages[next].stamp_s <= now_s) {
      const Message& m = messages[next];
      if (m.kind == 'D') {
        command = DriveCommand{m.a, m.b};
        command_stamp_s = m.stamp_s;
      } else if (m.kind == 'O') {
        odom_mps = m.a;
        odom_stamp_s = m.stamp_s;
      } else {
        scan = &m.scan;
      }
      ++next;
    }
    racer_safety::GateInput input;
    input.command = command.value_or(DriveCommand{0.0, 0.0});
    input.drive_raw_age_s = command ? now_s - command_stamp_s : kInf;
    input.dt_s = control_period_s;
    double d_fwd = kInf;
    double d_rear = kInf;
    if (scan != nullptr) {
      d_fwd = racer_safety::min_path_distance_m(scan->geometry, scan->ranges, laser_yaw_rad,
                                                half_angle_rad, half_width_m, path,
                                                input.command.steering_angle_rad);
      d_rear = racer_safety::min_rear_path_distance_m(
          scan->geometry, scan->ranges, laser_yaw_rad, half_angle_rad, half_width_m, path,
          rear_overhang_m, input.command.steering_angle_rad);
    }
    input.min_scan_range_m = d_fwd;
    input.min_rear_scan_range_m = d_rear;
    input.ttc_brake_latched = latched;
    input.reverse_brake_latched = reverse_latched;
    input.obstacle_hold_timer_s = hold_timer_s;
    input.speed_bound_forward_mps = bound_forward;
    input.speed_bound_reverse_mps = bound_reverse;
    if (use_odometry && odom_mps && now_s - odom_stamp_s <= odom_timeout_s) {
      input.measured_speed_mps = odom_mps;
    }
    const GateResult result = gate.evaluate(input, previous);

    // Trips: a latch going from clear to set.
    for (int side = 0; side < 2; ++side) {
      const bool was = side == 0 ? latched : reverse_latched;
      const bool is = side == 0 ? result.ttc_brake_latched : result.reverse_brake_latched;
      if (was || !is) {
        continue;
      }
      const double direction = side == 0 ? 1.0 : -1.0;
      const double d = side == 0 ? d_fwd : d_rear;
      const GateSource source = side == 0 ? GateSource::kTtc : GateSource::kTtcReverse;
      std::string cause = "held";
      for (const auto& a : result.activations) {
        if (a.source == source && a.severity == EventSeverity::kBrake) {
          cause = a.detail.find("clearance") != std::string::npos ? "floor" : "ttc";
        }
      }
      // The speed the TTC used: the request toward this side raised to the bound and the
      // measured speed (gate_logic.hpp "SPEED FOR THE TTC").
      double ttc_speed = direction * input.command.speed_mps;
      const double bound =
          side == 0 ? result.speed_bound_forward_mps : result.speed_bound_reverse_mps;
      if (bound > ttc_speed) {
        ttc_speed = bound;
      }
      if (input.measured_speed_mps && direction * *input.measured_speed_mps > ttc_speed) {
        ttc_speed = direction * *input.measured_speed_mps;
      }
      // The stopping model wants the car's real speed: the measured one while fresh, else the
      // TTC speed (an overestimate whenever the car is slower than it was asked to be, which on
      // the October floor bags was most of the time: about 0.55 of the request in corners).
      double stop_speed = ttc_speed;
      if (input.measured_speed_mps) {
        stop_speed = direction * *input.measured_speed_mps;
        if (stop_speed < 0.0) {
          stop_speed = 0.0;
        }
      }
      const double stop_m =
          stop_speed * stop_latency_s + stop_speed * stop_speed / (2.0 * stop_decel_mps2);
      std::cout << "{\"type\": \"trip\", \"t\": " << json_time(now_s)
                << ", \"gate\": " << json_string(racer_safety::gate_source_to_string(source))
                << ", \"cause\": " << json_string(cause) << ", \"distance_m\": " << json_number(d)
                << ", \"steering_rad\": " << json_number(input.command.steering_angle_rad)
                << ", \"speed_mps\": " << json_number(input.command.speed_mps)
                << ", \"ttc_speed_mps\": " << json_number(ttc_speed)
                << ", \"stop_speed_mps\": " << json_number(stop_speed)
                << ", \"previous_output_mps\": " << json_number(previous.speed_mps)
                << ", \"predicted_stop_m\": " << json_number(stop_m)
                << ", \"predicted_clearance_m\": " << json_number(d - stop_m) << "}\n";
    }
    for (const SafetyEventRecord& r : tracker.update(result.activations, now_s, result.releases)) {
      const char* severity = r.severity == EventSeverity::kBrake     ? "brake"
                             : r.severity == EventSeverity::kWarning ? "warning"
                                                                     : "info";
      std::cout << "{\"type\": \"event\", \"t\": " << json_time(now_s) << ", \"phase\": "
                << json_string(r.phase == EventPhase::kEngage ? "engage" : "release")
                << ", \"source\": " << json_string(racer_safety::gate_source_to_string(r.source))
                << ", \"severity\": " << json_string(severity)
                << ", \"duration_s\": " << json_number(r.duration_s)
                << ", \"detail\": " << json_string(r.detail) << "}\n";
    }
    if (cycles) {
      std::cout << "{\"type\": \"cycle\", \"t\": " << json_time(now_s)
                << ", \"steering_rad\": " << json_number(input.command.steering_angle_rad)
                << ", \"speed_mps\": " << json_number(input.command.speed_mps)
                << ", \"out_steering_rad\": " << json_number(result.output.steering_angle_rad)
                << ", \"out_speed_mps\": " << json_number(result.output.speed_mps)
                << ", \"distance_m\": " << json_number(d_fwd)
                << ", \"rear_distance_m\": " << json_number(d_rear)
                << ", \"latched\": " << (result.ttc_brake_latched ? "true" : "false")
                << ", \"reverse_latched\": " << (result.reverse_brake_latched ? "true" : "false")
                << ", \"speed_bound_mps\": " << json_number(result.speed_bound_forward_mps)
                << "}\n";
    }
    previous = result.output;
    latched = result.ttc_brake_latched;
    reverse_latched = result.reverse_brake_latched;
    hold_timer_s = result.obstacle_hold_timer_s;
    bound_forward = result.speed_bound_forward_mps;
    bound_reverse = result.speed_bound_reverse_mps;
  }
  return 0;
}
