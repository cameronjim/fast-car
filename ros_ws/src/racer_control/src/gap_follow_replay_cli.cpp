// gap_follow_replay_cli: offline replay of recorded /scan messages through the follow-the-gap
// controller, the same code gap_follow_node runs (tools/replay_follower.py drives it from an
// mcap bag; docs/notes/replay-follower.md). No ROS dependency.
//
// usage: gap_follow_replay_cli --params <file> [--scans <file>] [--set key=value]... [--describe]
//
// Scans: a plain text file (or stdin), one scan per line,
//     stamp_s angle_min_rad angle_increment_rad n r0 r1 ... r(n-1)
// Numbers are anything strtod reads, so `inf` and `nan` are fine (an RPLIDAR no-return is
// +inf). Blank lines and lines starting with # are skipped. range_min / range_max are not in
// the line; they are the params range_min_m / range_max_m (defaults: the RPLIDAR C1).
//
// Params: a flat JSON object ({"steering_gain": 1.6, "swept_path_clamp": true}) when the file
// starts with `{`, otherwise key=value lines (# comments). The keys are gap_follow_node's
// ROS parameter names with its defaults and ranges (kParams below; a pytest compares that
// table with the node source so the two cannot drift). The speed and reverse-escape keys the
// node also takes are accepted and ignored: they do not change the steering. An unknown key
// or an out-of-range value is an error (exit 2), as the node refuses to start on one.
// --set overrides a key after the file. --describe prints the resolved vehicle geometry (LiDAR
// yaw and mount, half width, wheelbase, steering limit) as one JSON line and exits.
//
// What runs per scan is GapFollower::process, which is the node's whole perception half:
// sanitize, extend disparities, select the gap (forward preference, switch margin), steering
// gain with lane centring, swept-path clamp, corner override. The node's other steering step
// is the first-order low-pass its 50 Hz timer applies to the latest result; it is reproduced
// here with the same FirstOrderLowPass, stepped control_period_s per tick, as many ticks per
// scan as fit between this scan and the next (the first tick after start or after a /scan
// stale gap steps by 0, as the node does). A scan the core rejects does not replace the latest
// result, as in the node.
//
// Physical constants come from the generated vehicle_params binding only (CLAUDE.md invariant
// 2), exactly as in the node; the LiDAR yaw goes through the same resolve_laser_yaw_offset.
//
// Output, one JSON object per input scan on stdout:
//   stamp_s, valid, gap_found, target_bearing_rad, target_range_m, wanted_steering_rad
//   (gain * bearing + centring, before the clamp), steering_rad (after the swept-path clamp
//   and the corner override, before the low-pass), filtered_steering_rad (the low-pass output
//   after this scan's interval of control ticks, what the node would be publishing just before
//   the next scan), clamped, corner_blocked, centering_rad. Numbers that do not exist for an
//   invalid scan are null.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_control/gap_follow.hpp"
#include "racer_control/laser_scan.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct ParamSpec {
  const char* name;
  double default_value;
  double min_value;
  double max_value;
  bool is_bool;
};

// One row per gap_follow_node parameter the steering depends on: name, default, min, max.
// Keep this in step with src/gap_follow_node.cpp; test_replay_follower.py fails when it is not.
// The last two rows are CLI-only (the line format carries no range limits).
// clang-format off
const ParamSpec kParams[] = {
    {"safety_margin_m", 0.1, 0.0, 2.0, false},
    {"clip_max_range_m", 5.0, 0.1, 100.0, false},
    {"disparity_threshold_m", 0.5, 0.01, 100.0, false},
    {"free_space_threshold_m", 1.5, 0.0, 100.0, false},
    {"cone_half_angle_rad", 1.57, 0.0, kPi, false},
    {"target_deepest_ray", 0.0, 0.0, 1.0, true},
    {"forward_preference", 0.0, 0.0, 1.0, false},
    {"gap_switch_margin", 0.0, 0.0, 1.0, false},
    {"steering_gain", 1.0, 0.0, 10.0, false},
    {"centering_gain", 0.0, 0.0, 5.0, false},
    {"centering_sector_half_angle_rad", 1.0, 0.01, kPi / 2.0, false},
    {"centering_max_range_m", 1.5, 0.05, 20.0, false},
    {"corner_sector_inner_rad", kPi / 2.0, 0.0, kPi, false},
    {"corner_sector_outer_rad", 3.0 * kPi / 4.0, 0.0, kPi, false},
    {"corner_min_clearance_m", 0.2, 0.0, 10.0, false},
    {"laser_yaw_offset_rad", 0.0, -kPi, kPi, false},
    {"laser_yaw_from_vehicle_params", 1.0, 0.0, 1.0, true},
    {"swept_path_clamp", 1.0, 0.0, 1.0, true},
    {"swept_path_lookahead_m", 1.0, 0.01, 10.0, false},
    {"steering_time_constant_s", 0.1, 0.0, 10.0, false},
    {"control_rate_hz", 50.0, 1.0, 1000.0, false},
    {"scan_timeout_s", 0.3, 1e-3, 10.0, false},
    // CLI only:
    {"range_min_m", 0.05, 0.0, 1000.0, false},
    {"range_max_m", 12.0, 0.01, 1000.0, false},
};
// clang-format on

// Node parameters that do not change the steering: accepted and ignored.
bool is_ignored_key(const std::string& key) {
  static const char* const kIgnored[] = {
      "min_speed_mps",
      "max_speed_mps",
      "k_steer",
      "k_speed_per_s",
      "speed_rate_limit_margin_fraction",
      "speed_time_constant_s",
      "target_range_median_scans",
      "reverse_escape",
      "drive_feedback_timeout_s",
  };
  if (key.rfind("escape_", 0) == 0) {
    return true;
  }
  return std::any_of(std::begin(kIgnored), std::end(kIgnored),
                     [&](const char* name) { return key == name; });
}

using ParamMap = std::map<std::string, double>;

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

[[noreturn]] void die(int code, const std::string& message) {
  std::cerr << "gap_follow_replay_cli: " << message << "\n";
  std::exit(code);
}

void set_param(ParamMap& params, const std::string& key, const std::string& value) {
  if (is_ignored_key(key)) {
    return;
  }
  const auto spec = std::find_if(std::begin(kParams), std::end(kParams),
                                 [&](const ParamSpec& p) { return key == p.name; });
  if (spec == std::end(kParams)) {
    die(2, "unknown parameter '" + key + "'");
  }
  const auto number = parse_number(value);
  if (!number || !std::isfinite(*number)) {
    die(2, "parameter '" + key + "' has a non-numeric value '" + value + "'");
  }
  if (*number < spec->min_value || *number > spec->max_value) {
    std::ostringstream message;
    message << "parameter '" << key << "' = " << *number << " is outside [" << spec->min_value
            << ", " << spec->max_value << "] (the node refuses to start on this)";
    die(2, message.str());
  }
  params[key] = *number;
}

// Flat JSON object of string keys to number / true / false values.
void parse_json_params(const std::string& text, ParamMap& params) {
  std::size_t i = 0;
  auto skip_space = [&] {
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
      ++i;
    }
  };
  skip_space();
  if (i >= text.size() || text[i] != '{') {
    die(2, "params JSON must be an object");
  }
  ++i;
  skip_space();
  if (i < text.size() && text[i] == '}') {
    return;
  }
  while (i < text.size()) {
    skip_space();
    if (i >= text.size() || text[i] != '"') {
      die(2, "params JSON: expected a quoted key");
    }
    const auto key_end = text.find('"', i + 1);
    if (key_end == std::string::npos) {
      die(2, "params JSON: unterminated key");
    }
    const std::string key = text.substr(i + 1, key_end - i - 1);
    i = key_end + 1;
    skip_space();
    if (i >= text.size() || text[i] != ':') {
      die(2, "params JSON: expected ':' after '" + key + "'");
    }
    ++i;
    const auto value_end = text.find_first_of(",}", i);
    if (value_end == std::string::npos) {
      die(2, "params JSON: unterminated object");
    }
    set_param(params, key, text.substr(i, value_end - i));
    i = value_end;
    if (text[i] == '}') {
      return;
    }
    ++i;
  }
  die(2, "params JSON: unterminated object");
}

void load_params_file(const std::string& path, ParamMap& params) {
  std::ifstream in(path);
  if (!in) {
    die(2, "cannot open params file " + path);
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  const std::string text = buffer.str();
  if (trim(text).rfind('{', 0) == 0) {
    parse_json_params(text, params);
    return;
  }
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line.erase(hash);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      die(2, "params line without '=': " + line);
    }
    set_param(params, trim(line.substr(0, eq)), line.substr(eq + 1));
  }
}

struct Scan {
  double stamp_s = 0.0;
  racer_control::ScanInput input;
};

std::optional<Scan> parse_scan_line(const std::string& line, double range_min, double range_max,
                                    std::size_t line_number) {
  const std::string trimmed = trim(line);
  if (trimmed.empty() || trimmed[0] == '#') {
    return std::nullopt;
  }
  std::istringstream tokens(trimmed);
  std::vector<std::string> words;
  for (std::string w; tokens >> w;) {
    words.push_back(w);
  }
  auto fail = [&](const std::string& why) {
    die(3, "scans line " + std::to_string(line_number) + ": " + why);
  };
  if (words.size() < 4) {
    fail("expected stamp angle_min angle_increment n ranges...");
  }
  const auto stamp = parse_number(words[0]);
  const auto angle_min = parse_number(words[1]);
  const auto increment = parse_number(words[2]);
  const auto n = parse_number(words[3]);
  if (!stamp || !angle_min || !increment || !n || *n < 0 || *n != std::floor(*n)) {
    fail("bad header numbers");
  }
  if (words.size() != 4 + static_cast<std::size_t>(*n)) {
    fail("n is " + words[3] + " but the line has " + std::to_string(words.size() - 4) + " ranges");
  }
  Scan scan;
  scan.stamp_s = *stamp;
  scan.input.angle_min = *angle_min;
  scan.input.angle_increment = *increment;
  scan.input.range_min = range_min;
  scan.input.range_max = range_max;
  scan.input.ranges.reserve(words.size() - 4);
  for (std::size_t k = 4; k < words.size(); ++k) {
    const auto r = parse_number(words[k]);
    if (!r) {
      fail("bad range '" + words[k] + "'");
    }
    scan.input.ranges.push_back(static_cast<float>(*r));
  }
  return scan;
}

void print_number(const char* key, double value, bool valid) {
  if (valid && std::isfinite(value)) {
    std::printf(",\"%s\":%.9g", key, value);
  } else {
    std::printf(",\"%s\":null", key);
  }
}

const char* json_bool(bool value) { return value ? "true" : "false"; }

}  // namespace

int main(int argc, char** argv) {
  std::string scans_path;
  std::string params_path;
  std::vector<std::pair<std::string, std::string>> overrides;
  bool describe = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        die(2, "missing value after " + arg);
      }
      return argv[++i];
    };
    if (arg == "--scans") {
      scans_path = next();
    } else if (arg == "--params") {
      params_path = next();
    } else if (arg == "--describe") {
      describe = true;
    } else if (arg == "--set") {
      const std::string kv = next();
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {
        die(2, "--set wants key=value, got " + kv);
      }
      overrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
    } else {
      std::cerr << "usage: gap_follow_replay_cli [--params <file>] [--scans <file>] "
                   "[--set key=value]...\nScans on stdin when --scans is omitted. See the "
                   "header comment of src/gap_follow_replay_cli.cpp.\n";
      return arg == "--help" ? 0 : 2;
    }
  }

  ParamMap params;
  for (const ParamSpec& p : kParams) {
    params[p.name] = p.default_value;
  }
  if (!params_path.empty()) {
    load_params_file(params_path, params);
  }
  for (const auto& [key, value] : overrides) {
    set_param(params, key, value);
  }
  const auto get = [&](const char* key) { return params.at(key); };
  if (get("corner_sector_inner_rad") > get("corner_sector_outer_rad")) {
    die(2, "corner_sector_inner_rad must not exceed corner_sector_outer_rad");
  }

  // The same GapFollowConfig gap_follow_node::build_config() fills.
  racer_control::GapFollowConfig c;
  c.half_width_m = VEHICLE_PARAMS.chassis.width_m / 2.0;
  c.max_steering_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
  c.safety_margin_m = get("safety_margin_m");
  c.clip_max_range_m = get("clip_max_range_m");
  c.disparity_threshold_m = get("disparity_threshold_m");
  c.free_space_threshold_m = get("free_space_threshold_m");
  c.cone_half_angle_rad = get("cone_half_angle_rad");
  c.target = get("target_deepest_ray") != 0.0 ? racer_control::GapTarget::kDeepest
                                              : racer_control::GapTarget::kCentre;
  c.forward_preference = get("forward_preference");
  c.gap_switch_margin = get("gap_switch_margin");
  c.steering_gain = get("steering_gain");
  c.centering_gain = get("centering_gain");
  c.centering_sector_half_angle_rad = get("centering_sector_half_angle_rad");
  c.centering_max_range_m = get("centering_max_range_m");
  c.corner_sector_inner_rad = get("corner_sector_inner_rad");
  c.corner_sector_outer_rad = get("corner_sector_outer_rad");
  c.corner_min_clearance_m = get("corner_min_clearance_m");
  const auto yaw = racer_control::resolve_laser_yaw_offset(
      VEHICLE_PARAMS.sensors.lidar.mount_yaw_rad, get("laser_yaw_offset_rad"),
      get("laser_yaw_from_vehicle_params") != 0.0);
  if (!yaw) {
    die(2,
        "laser_yaw_offset_rad disagrees with vehicle_params sensors.lidar.mount_yaw_rad with "
        "laser_yaw_from_vehicle_params true (the node refuses to start on this)");
  }
  c.laser_yaw_offset_rad = *yaw;
  c.swept_path_clamp = get("swept_path_clamp") != 0.0;
  c.swept_path.lookahead_m = get("swept_path_lookahead_m");
  if (!VEHICLE_PARAMS.sensors.lidar.mount_x_m.has_value() ||
      !VEHICLE_PARAMS.sensors.lidar.mount_y_m.has_value()) {
    die(2, "vehicle_params sensors.lidar.mount_x_m / mount_y_m is null");
  }
  c.swept_path.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
  c.swept_path.lidar_mount_x_m = *VEHICLE_PARAMS.sensors.lidar.mount_x_m;
  c.swept_path.lidar_mount_y_m = *VEHICLE_PARAMS.sensors.lidar.mount_y_m;
  c.swept_path.body_front_x_m =
      VEHICLE_PARAMS.chassis.wheelbase_m + VEHICLE_PARAMS.chassis.front_overhang_m;

  if (describe) {
    // What tools/replay_follower.py needs to draw a scan in the vehicle frame, resolved the way
    // the run resolves it (so the plots never carry a hand-typed vehicle constant).
    std::printf(
        "{\"laser_yaw_rad\":%.9g,\"lidar_mount_x_m\":%.9g,\"lidar_mount_y_m\":%.9g,"
        "\"half_width_m\":%.9g,\"wheelbase_m\":%.9g,\"body_front_x_m\":%.9g,"
        "\"max_steering_rad\":%.9g}\n",
        c.laser_yaw_offset_rad, c.swept_path.lidar_mount_x_m, c.swept_path.lidar_mount_y_m,
        c.half_width_m, c.swept_path.wheelbase_m, c.swept_path.body_front_x_m,
        c.max_steering_angle_rad);
    return 0;
  }

  racer_control::GapFollower follower(c);
  racer_control::FirstOrderLowPass steering_filter(get("steering_time_constant_s"));
  const double control_rate_hz = get("control_rate_hz");
  const double control_period_s = 1.0 / control_rate_hz;
  const double scan_timeout_s = get("scan_timeout_s");

  std::ifstream scans_file;
  if (!scans_path.empty()) {
    scans_file.open(scans_path);
    if (!scans_file) {
      die(2, "cannot open scans file " + scans_path);
    }
  }
  std::istream& in = scans_path.empty() ? std::cin : scans_file;
  std::vector<Scan> scans;
  std::string line;
  for (std::size_t number = 1; std::getline(in, line); ++number) {
    if (auto scan = parse_scan_line(line, get("range_min_m"), get("range_max_m"), number)) {
      scans.push_back(std::move(*scan));
    }
  }

  racer_control::GapFollowResult latest;
  bool has_scan = false;
  double last_usable_stamp_s = 0.0;
  bool has_last_command_time = false;
  for (std::size_t k = 0; k < scans.size(); ++k) {
    const Scan& scan = scans[k];
    const racer_control::GapFollowResult result = follower.process(scan.input);
    if (result.valid) {
      latest = result;
      has_scan = true;
      last_usable_stamp_s = scan.stamp_s;
    }
    // The node's timer ticks that fall before the next scan (the last scan gets as many as the
    // one before it). Each tick is skipped while the latest usable scan is older than
    // scan_timeout_s, and the first tick after such a gap steps by 0, as in on_timer().
    double interval_s = control_period_s;
    if (k + 1 < scans.size()) {
      interval_s = scans[k + 1].stamp_s - scan.stamp_s;
    } else if (k > 0) {
      interval_s = scan.stamp_s - scans[k - 1].stamp_s;
    }
    interval_s = std::max(interval_s, 0.0);
    const long ticks = std::max(1L, std::lround(interval_s * control_rate_hz));
    for (long t = 0; t < ticks; ++t) {
      const double tick_stamp_s = scan.stamp_s + static_cast<double>(t) * control_period_s;
      if (!has_scan || tick_stamp_s - last_usable_stamp_s > scan_timeout_s) {
        has_last_command_time = false;
        continue;
      }
      const double dt_s = has_last_command_time ? control_period_s : 0.0;
      has_last_command_time = true;
      steering_filter.update(latest.steering_rad, dt_s);
    }

    std::printf("{\"stamp_s\":%.9f,\"valid\":%s,\"gap_found\":%s", scan.stamp_s,
                json_bool(result.valid), json_bool(result.valid && result.gap_found));
    print_number("target_bearing_rad", result.target_vehicle_bearing_rad, result.valid);
    print_number("target_range_m", result.target_range_m, result.valid);
    print_number("wanted_steering_rad", result.wanted_steering_rad, result.valid);
    print_number("steering_rad", result.steering_rad, result.valid);
    print_number("filtered_steering_rad", steering_filter.output(), true);
    std::printf(",\"clamped\":%s,\"corner_blocked\":%s", json_bool(result.swept_path_clamped),
                json_bool(result.corner_blocked));
    print_number("centering_rad", result.centering_steering_rad, result.valid);
    std::printf("}\n");
  }
  return 0;
}
