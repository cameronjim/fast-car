// limiter_whatif_cli: a recorded /drive_raw stream through racer_safety's UNCHANGED gate, once
// with the dt of every cycle as recorded and once with the nominal period (brief 34,
// pre-registration PnP v0.3 section 5, "rate limiter what-if"). tools/pnp_session_report.py
// builds the stream from a bag and runs this twice. No ROS dependency, nothing here drives a car.
//
// usage: limiter_whatif_cli [--stream <file>] [--dt-mode recorded|nominal] [--cycles] [--describe]
//
// Stream, one cycle per line in time order (numbers are anything strtod reads, inf included):
//     C stamp_s dt_s steering_rad speed_mps age_s [prev_steering_rad prev_speed_mps]
//   With --teacher every C line carries the previous output (the recorded /drive of the cycle
//   before) and the gate is judged from it: the limiter's decision per cycle, with no drift of a
//   free-running chain.
//     R stamp_s steering_rad speed_mps       start a new segment: the previous output is seeded
//                                            and the next cycle takes the nominal period
// stamp_s: the cycle's time (labels the output only). dt_s: the time since the previous cycle as
// recorded (the gap of /drive header stamps). steering_rad, speed_mps: the /drive_raw command in
// force at that cycle. age_s: the seconds since it arrived. Blank lines and # comments are
// skipped. The first cycle is given the nominal period whatever dt_s says, as safety_node does.
//
// Output, one JSON object per line on stdout:
//   {"type":"event", t, phase, source, severity, duration_s, detail}  every record safety_node
//     would publish on /safety/events (the detail string is the gate's own).
//   {"type":"clip", t, dt_s}   every cycle in which the rate limiter clipped the command.
//   {"type":"cycle", t, dt_s, out_steering_rad, out_speed_mps}   every cycle, with --cycles only
//   {"type":"summary", dt_mode, cycles, clip_cycles, engage_rate_limit, engage_watchdog}
//
// Not modelled, which bounds what a count means: no scan, so the obstacle gate never trips;
// no wheel odometry. See include/racer_session_tools/whatif_core.hpp.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "racer_session_tools/whatif_core.hpp"

namespace {

using racer_session_tools::CycleInput;
using racer_session_tools::EventOut;
using racer_session_tools::Replayer;

[[noreturn]] void die(int code, const std::string& message) {
  std::cerr << "limiter_whatif_cli: " << message << "\n";
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

std::string json_number(double v) {
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  return buf;
}

// Stamps are epoch or bag seconds: %.6f keeps the microseconds that %g would drop.
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

}  // namespace

int main(int argc, char** argv) {
  std::string stream_path;
  std::string dt_mode = "recorded";
  bool describe = false;
  bool cycles_out = false;
  bool teacher = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--stream" && i + 1 < argc) {
      stream_path = argv[++i];
    } else if (arg == "--dt-mode" && i + 1 < argc) {
      dt_mode = argv[++i];
    } else if (arg == "--teacher") {
      teacher = true;
    } else if (arg == "--cycles") {
      cycles_out = true;
    } else if (arg == "--describe") {
      describe = true;
    } else {
      die(2,
          "usage: limiter_whatif_cli [--stream file] [--dt-mode recorded|nominal] [--teacher] "
          "[--cycles] [--describe]");
    }
  }
  if (dt_mode != "recorded" && dt_mode != "nominal") {
    die(2, "--dt-mode must be recorded or nominal");
  }
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  const double nominal_s = racer_session_tools::nominal_period_s(limits);
  if (describe) {
    std::cout << "{\"nominal_period_s\": " << json_number(nominal_s)
              << ", \"steering_rate_min_rad_per_s\": "
              << json_number(limits.steering_rate_min_rad_per_s)
              << ", \"steering_rate_max_rad_per_s\": "
              << json_number(limits.steering_rate_max_rad_per_s)
              << ", \"max_acceleration_mps2\": " << json_number(limits.max_acceleration_mps2)
              << ", \"watchdog_timeout_s\": "
              << json_number(limits.watchdog_missed_cycles * limits.control_period_s) << "}\n";
    return 0;
  }

  std::ifstream file;
  std::istream* in = &std::cin;
  if (!stream_path.empty()) {
    file.open(stream_path);
    if (!file) {
      die(2, "cannot open stream file " + stream_path);
    }
    in = &file;
  }

  Replayer replayer(limits);
  long cycles = 0;
  long clip_cycles = 0;
  long engage_rate_limit = 0;
  long engage_watchdog = 0;
  std::string line;
  std::size_t line_number = 0;
  double previous_stamp = 0.0;
  bool segment_start = true;
  bool seeded = false;
  while (std::getline(*in, line)) {
    ++line_number;
    const std::string text = trim(line);
    if (text.empty() || text[0] == '#') {
      continue;
    }
    std::istringstream tokens(text);
    std::vector<std::string> w;
    for (std::string t; tokens >> t;) {
      w.push_back(t);
    }
    auto fail = [&](const std::string& why) {
      die(3, "stream line " + std::to_string(line_number) + ": " + why);
    };
    if (w[0] == "R") {
      if (w.size() != 4) {
        fail("expected: R stamp_s steering_rad speed_mps");
      }
      const auto a = parse_number(w[2]);
      const auto b = parse_number(w[3]);
      if (!a || !b) {
        fail("bad number in R line");
      }
      replayer.seed(racer_safety::DriveCommand{*a, *b});
      seeded = true;
      segment_start = true;
      continue;
    }
    if (w[0] != "C" || (w.size() != 6 && w.size() != 8)) {
      fail("expected: C stamp_s dt_s steering_rad speed_mps age_s [prev_steering prev_speed]");
    }
    if (teacher && w.size() != 8) {
      fail("--teacher needs the two previous-output fields on every C line");
    }
    double v[7] = {0, 0, 0, 0, 0, 0, 0};
    for (std::size_t k = 0; k + 1 < w.size(); ++k) {
      const auto n = parse_number(w[static_cast<std::size_t>(k) + 1]);
      if (!n) {
        fail("bad number '" + w[static_cast<std::size_t>(k) + 1] + "'");
      }
      v[k] = *n;
    }
    if (!std::isfinite(v[0])) {
      fail("non-finite stamp");
    }
    if (cycles > 0 && !segment_start && v[0] < previous_stamp) {
      fail("stamps go backwards");
    }
    previous_stamp = v[0];
    CycleInput cycle;
    cycle.stamp_s = v[0];
    // The very first cycle of an unseeded stream has no previous cycle: the nominal period, as in
    // safety_node. A segment (R line) starts mid-run, where the recorded dt is known.
    cycle.dt_s = (dt_mode == "nominal" || (cycles == 0 && !seeded)) ? nominal_s : v[1];
    segment_start = false;
    cycle.command = racer_safety::DriveCommand{v[2], v[3]};
    cycle.age_s = v[4];
    if (teacher) {
      replayer.force_previous(racer_safety::DriveCommand{v[5], v[6]});
    }
    const racer_session_tools::CycleOutput out = replayer.step(cycle);
    ++cycles;
    if (cycles_out) {
      std::cout << "{\"type\": \"cycle\", \"t\": " << json_time(cycle.stamp_s)
                << ", \"dt_s\": " << json_number(cycle.dt_s)
                << ", \"out_steering_rad\": " << json_number(out.output.steering_angle_rad)
                << ", \"out_speed_mps\": " << json_number(out.output.speed_mps) << "}\n";
    }
    if (out.rate_limit_clipped) {
      ++clip_cycles;
      std::cout << "{\"type\": \"clip\", \"t\": " << json_time(cycle.stamp_s)
                << ", \"dt_s\": " << json_number(cycle.dt_s) << "}\n";
    }
    for (const EventOut& e : out.events) {
      if (e.engage && e.source == "rate_limit") {
        ++engage_rate_limit;
      }
      if (e.engage && e.source == "watchdog") {
        ++engage_watchdog;
      }
      std::cout << "{\"type\": \"event\", \"t\": " << json_time(e.t_s) << ", \"phase\": "
                << json_string(e.engage ? "engage" : "release")
                << ", \"source\": " << json_string(e.source)
                << ", \"severity\": " << json_string(e.severity)
                << ", \"duration_s\": " << json_number(e.duration_s)
                << ", \"detail\": " << json_string(e.detail) << "}\n";
    }
  }
  std::cout << "{\"type\": \"summary\", \"dt_mode\": " << json_string(dt_mode)
            << ", \"cycles\": " << cycles << ", \"clip_cycles\": " << clip_cycles
            << ", \"engage_rate_limit\": " << engage_rate_limit
            << ", \"engage_watchdog\": " << engage_watchdog << "}\n";
  return 0;
}
