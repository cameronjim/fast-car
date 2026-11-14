// planner_replay_cli: offline replay of a recorded /scan + /odom/wheel + /pnp/local_track stream
// through the REAL planner pipeline (racer_planner::Planner, the class mppi_planner_node ticks),
// one planner tick per K line. No ROS dependency, nothing here drives a car. tools/pnp_replay.py
// writes the stream from an mcap bag and reads the CSV back (docs/notes/pnp-replay.md).
//
// usage: planner_replay_cli --bundle DIR [--v-max M_S] [--set name=value]... [--stream FILE]
//                           [--out CSV] [--describe]
//
// --set takes the planner node's parameter names (v_max_mps, max_intrusion_fraction,
// scan_timeout_s, samples, seed, w_progress, ...) with the node's defaults and ranges; --describe
// prints the effective configuration as JSON and exits. A bundle that fails any row of the
// refusal table exits 2 with "<RefusalName>: <message>" on stderr, as the node does.
//
// Stream, one event per line, in time order of ARRIVAL (stdin when --stream is absent). Numbers
// are anything strtod reads (inf, nan). Blank lines and # comments are skipped.
//   S arrival_s stamp_s angle_min angle_increment range_min range_max n r0 ... r(n-1)   a scan
//   O arrival_s wheel_speed_mps                                                        odometry
//   T arrival_s stamp_s valid_count boundary_residual intrusion x0..x15 y0..y15 hl0..hl15
//     hr0..hr15 sigma0..sigma15                a local track, frame base_link, spacing 0.20
//   K now_s                                    run one planner tick at this time
//
// A K line builds a TickInput the way the node's on_timer does: the latest distance field with
// its header stamp, the latest track with its header stamp, the latest wheel speed stamped with
// its ARRIVAL time (the node stamps odometry with now()), no lap guidance. One CSV row per K:
// every TickOutput field plus now_s. NaN is written "nan".
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "binding_geometry.hpp"
#include "gate_binding.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/gate_predictor.hpp"
#include "racer_planner/planner.hpp"

using namespace racer_planner;

namespace {

constexpr double kBodyMarginM = 0.02;  // as in the node
constexpr std::size_t kMaxScanPoints = 8192;

[[noreturn]] void die(int code, const std::string& message) {
  std::cerr << "planner_replay_cli: " << message << "\n";
  std::exit(code);
}

// One planner node parameter that --set may change: the node's default and range.
struct Param {
  double value;
  double lo;
  double hi;
  bool integer;
};

// The node's parameters (mppi_planner_node.cpp), same names, defaults and ranges. Defaults that
// the node takes from PlannerConfig / MppiParams / LogicParams are read from those structs, so
// this table cannot drift from the code on a changed default.
std::map<std::string, Param> default_params() {
  const PlannerConfig cfg;
  const LogicParams& lg = cfg.logic;
  const MppiParams& mp = cfg.mppi;
  const SampleBudgetParams& bg = cfg.budget;
  std::map<std::string, Param> p;
  auto d = [&](const char* n, double v, double lo, double hi) { p[n] = {v, lo, hi, false}; };
  auto i = [&](const char* n, double v, double lo, double hi) { p[n] = {v, lo, hi, true}; };
  d("v_max_mps", 0.5, 0.1, VEHICLE_PARAMS.limits.global_speed_cap_mps);
  d("scan_timeout_s", lg.scan_timeout_s, 0.05, 5.0);
  d("scan_hard_timeout_s", lg.scan_hard_timeout_s, 0.05, 0.60);
  d("odom_timeout_s", lg.odom_timeout_s, 0.02, 5.0);
  d("odom_hard_timeout_s", lg.odom_hard_timeout_s, 0.02, 0.20);
  d("plan_timeout_s", lg.plan_timeout_s, 0.05, 0.12);
  i("start_ticks", lg.start_ticks, 1, 200);
  d("start_tick_gap_s", lg.start_tick_gap_s, 0.01, 0.2);
  d("start_timeout_s", lg.start_timeout_s, 0.5, 120.0);
  d("start_stop_hold_s", lg.start_stop_hold_s, 0.2, 60.0);
  d("require_gated_for_start", 1.0, 0.0, 1.0);
  d("start_gated_grace_s", lg.start_gated_grace_s, 0.0, 5.0);
  i("min_scan_points", lg.min_scan_points, 0, 1000);
  d("v_no_guidance_mps", 0.5, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps);
  d("conf_intrusion_full", lg.conf_intrusion_full, 0.0, 1.0);
  d("conf_intrusion_zero", lg.conf_intrusion_zero, 0.0, 1.0);
  d("conf_residual_full_m", lg.conf_residual_full_m, 0.0, 5.0);
  d("conf_residual_zero_m", lg.conf_residual_zero_m, 0.0, 5.0);
  d("conf_valid_zero", lg.conf_valid_zero, 0.0, 16.0);
  d("conf_valid_full", lg.conf_valid_full, 0.0, 16.0);
  d("conf_age_full_s", lg.conf_age_full_s, 0.0, 5.0);
  d("conf_age_zero_s", lg.conf_age_zero_s, 0.0, 5.0);
  d("conf_rise_per_s", lg.conf_rise_per_s, 0.01, 100.0);
  d("freespace_speed_fraction", lg.freespace_speed_fraction, 0.3, 1.0);
  d("control_rate_hz", 50.0, 20.0, 100.0);
  i("horizon_steps", mp.horizon_steps, 5, 100);
  i("samples", mp.samples, kMinSamples, kMaxSamples);
  i("max_samples", cfg.max_samples, kMinSamples, kMaxSamples);
  d("sigma_speed_mps", mp.sigma_speed_mps, 0.0, 2.0);
  d("sigma_steering_rad", mp.sigma_steering_rad, 0.0, 1.0);
  d("noise_beta", mp.beta, 0.0, 0.99);
  d("lambda", mp.lambda, 1e-3, 1e3);
  d("cost_scale_floor", mp.cost_scale_floor, 1e-3, 1e3);
  i("seed", 1, 0, 2147483647);
  d("w_progress", mp.w_progress, 0.0, 1e3);
  d("c_collision", mp.c_collision, 0.0, 1e9);
  d("d_hard_m", mp.d_hard_m, 0.0, 1.0);
  d("w_clear", mp.w_clear, 0.0, 1e5);
  d("d_soft_base_m", mp.d_soft_base_m, 0.0, 2.0);
  d("t_headway_s", mp.t_headway_s, 0.0, 5.0);
  d("w_bound", mp.w_bound, 0.0, 1e5);
  d("m_bound_m", mp.m_bound_m, 0.0, 1.0);
  d("w_lat", mp.w_lat, 0.0, 1e4);
  d("a_lat_max_mps2", mp.a_lat_max_mps2, 0.1, 50.0);
  d("w_du", mp.w_du, 0.0, 1e4);
  d("w_steer_rate", mp.w_steer_rate, 0.0, 1e4);
  d("corner_radius_m", mp.corner_radius_m, 0.005, 0.2);
  d("steer_output_tau_s", lg.steer_output_tau_s, 0.0, 0.5);
  d("rate_limit_fraction", lg.rate_limit_fraction, 0.05, 1.0);
  d("w_term", mp.w_term, 0.0, 1e5);
  d("w_gate", mp.w_gate, 0.0, 1e4);
  d("gate_ttc_margin", mp.gate_ttc_margin, 0.5, 5.0);
  d("gate_clear_margin", mp.gate_clear_margin, 0.5, 10.0);
  d("w_trust", mp.w_trust, 0.0, 1e7);
  d("move_margin", mp.move_margin, 0.0, 100.0);
  d("speed_floor_margin_mps", mp.speed_floor_margin_mps, 0.0, 0.5);
  d("steer_output_hysteresis_rad", lg.steer_output_hysteresis_rad, 0.0, 0.02);
  d("gap_reset_s", lg.gap_reset_s, 0.0, 0.058);
  d("stall_hold_s", cfg.drivability.stall_hold_s, 0.5, 600.0);
  d("gate_hold_s", cfg.drivability.gate_hold_s, 0.2, 600.0);
  d("recover_distance_m", cfg.recovery.recover_distance_m, 0.1, 1.0);
  d("recover_speed_mps", cfg.recovery.recover_speed_mps, 0.1, 0.6);
  d("recover_max_s", cfg.recovery.recover_max_s, 0.5, 10.0);
  d("recover_retry_s", cfg.recovery.recover_retry_s, 0.5, 30.0);
  d("recover_margin_m", cfg.recovery.recover_margin_m, 0.0, 0.3);
  d("recover_exhausted_hold_s", cfg.recovery.recover_exhausted_hold_s, 0.0, 600.0);
  d("recover_gate_debounce_s", cfg.recovery.recover_gate_debounce_s, 0.02, 1.0);
  d("recover_progress_m", cfg.recovery.recover_progress_m, 0.1, 5.0);
  i("recover_max_attempts", cfg.recovery.recover_max_attempts, 1, 20);
  d("solve_halve_over_ms", 15.0, 1.0, 200.0);
  d("solve_restore_under_ms", 6.0, 0.5, 200.0);
  d("laser_yaw_from_vehicle_params", 1.0, 0.0, 1.0);
  d("laser_yaw_offset_rad", 0.0, -M_PI, M_PI);
  (void)bg;
  return p;
}

bool parse_number(const std::string& text, double* out) {
  if (text == "true") {
    *out = 1.0;
    return true;
  }
  if (text == "false") {
    *out = 0.0;
    return true;
  }
  if (text.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size()) return false;
  *out = v;
  return true;
}

void set_param(std::map<std::string, Param>* params, const std::string& name, double value) {
  const auto it = params->find(name);
  if (it == params->end()) die(2, "unknown parameter '" + name + "' (the planner node's names)");
  Param& p = it->second;
  if (!std::isfinite(value) || value < p.lo || value > p.hi) {
    std::ostringstream m;
    m << "parameter " << name << "=" << value << " is outside the node's range [" << p.lo << ", "
      << p.hi << "]";
    die(2, m.str());
  }
  if (p.integer && value != std::floor(value)) die(2, "parameter " + name + " must be an integer");
  p.value = value;
}

PlannerConfig config_from(const std::map<std::string, Param>& params) {
  auto v = [&](const char* n) { return params.at(n).value; };
  PlannerConfig cfg;
  LogicParams& lg = cfg.logic;
  lg.v_max_mps = v("v_max_mps");
  lg.scan_timeout_s = v("scan_timeout_s");
  lg.scan_hard_timeout_s = v("scan_hard_timeout_s");
  lg.odom_timeout_s = v("odom_timeout_s");
  lg.odom_hard_timeout_s = v("odom_hard_timeout_s");
  lg.plan_timeout_s = v("plan_timeout_s");
  lg.start_ticks = static_cast<int>(v("start_ticks"));
  lg.start_tick_gap_s = v("start_tick_gap_s");
  lg.start_timeout_s = v("start_timeout_s");
  lg.start_stop_hold_s = v("start_stop_hold_s");
  lg.require_gated_for_start = v("require_gated_for_start") != 0.0;
  lg.start_gated_grace_s = v("start_gated_grace_s");
  lg.min_scan_points = static_cast<int>(v("min_scan_points"));
  lg.v_no_guidance_mps = v("v_no_guidance_mps");
  lg.conf_intrusion_full = v("conf_intrusion_full");
  lg.conf_intrusion_zero = v("conf_intrusion_zero");
  lg.conf_residual_full_m = v("conf_residual_full_m");
  lg.conf_residual_zero_m = v("conf_residual_zero_m");
  lg.conf_valid_zero = v("conf_valid_zero");
  lg.conf_valid_full = v("conf_valid_full");
  lg.conf_age_full_s = v("conf_age_full_s");
  lg.conf_age_zero_s = v("conf_age_zero_s");
  lg.conf_rise_per_s = v("conf_rise_per_s");
  lg.freespace_speed_fraction = v("freespace_speed_fraction");
  lg.use_lap_guidance = false;
  lg.steer_output_tau_s = v("steer_output_tau_s");
  lg.rate_limit_fraction = v("rate_limit_fraction");
  lg.steer_output_hysteresis_rad = v("steer_output_hysteresis_rad");
  lg.gap_reset_s = v("gap_reset_s");
  cfg.drivability.stall_hold_s = v("stall_hold_s");
  cfg.drivability.gate_hold_s = v("gate_hold_s");
  RecoveryParams& rc = cfg.recovery;
  rc.recover_distance_m = v("recover_distance_m");
  rc.recover_speed_mps = v("recover_speed_mps");
  rc.recover_max_s = v("recover_max_s");
  rc.recover_retry_s = v("recover_retry_s");
  rc.recover_margin_m = v("recover_margin_m");
  rc.recover_exhausted_hold_s = v("recover_exhausted_hold_s");
  rc.recover_gate_debounce_s = v("recover_gate_debounce_s");
  rc.recover_progress_m = v("recover_progress_m");
  rc.recover_max_attempts = static_cast<int>(v("recover_max_attempts"));
  cfg.tick_s = 1.0 / v("control_rate_hz");
  MppiParams& mp = cfg.mppi;
  mp.horizon_steps = static_cast<int>(v("horizon_steps"));
  mp.samples = static_cast<int>(v("samples"));
  cfg.max_samples = static_cast<int>(v("max_samples"));
  mp.sigma_speed_mps = v("sigma_speed_mps");
  mp.sigma_steering_rad = v("sigma_steering_rad");
  mp.beta = v("noise_beta");
  mp.lambda = v("lambda");
  mp.cost_scale_floor = v("cost_scale_floor");
  mp.seed = static_cast<std::uint64_t>(v("seed"));
  mp.w_progress = v("w_progress");
  mp.c_collision = v("c_collision");
  mp.d_hard_m = v("d_hard_m");
  mp.w_clear = v("w_clear");
  mp.d_soft_base_m = v("d_soft_base_m");
  mp.t_headway_s = v("t_headway_s");
  mp.w_bound = v("w_bound");
  mp.m_bound_m = v("m_bound_m");
  mp.w_lat = v("w_lat");
  mp.a_lat_max_mps2 = v("a_lat_max_mps2");
  mp.w_du = v("w_du");
  mp.w_steer_rate = v("w_steer_rate");
  mp.corner_radius_m = v("corner_radius_m");
  mp.w_term = v("w_term");
  mp.w_gate = v("w_gate");
  mp.gate_ttc_margin = v("gate_ttc_margin");
  mp.gate_clear_margin = v("gate_clear_margin");
  mp.w_trust = v("w_trust");
  mp.move_margin = v("move_margin");
  mp.speed_floor_margin_mps = v("speed_floor_margin_mps");
  cfg.budget.halve_over_s = v("solve_halve_over_ms") * 1e-3;
  cfg.budget.restore_under_s = v("solve_restore_under_ms") * 1e-3;
  return cfg;
}

std::string json_number(double v) {
  if (std::isnan(v)) return "null";
  if (std::isinf(v)) return v > 0 ? "1e999" : "-1e999";
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}

std::string csv_number(double v) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return v > 0 ? "inf" : "-inf";
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}

// The tick time with the microsecond resolution of the stream's K lines (a bag time of a few
// thousand seconds does not survive %.9g at that resolution).
std::string time_number(double v) {
  if (!std::isfinite(v)) return csv_number(v);
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.6f", v);
  return buf;
}

const char* state_name(PlannerState s) {
  switch (s) {
    case PlannerState::Starting:
      return "STARTING";
    case PlannerState::Driving:
      return "DRIVING";
    case PlannerState::NoTrack:
      return "NO_TRACK";
    case PlannerState::NoScan:
      return "NO_SCAN";
    case PlannerState::NoOdom:
      return "NO_ODOM";
    case PlannerState::Blocked:
      return "BLOCKED";
    case PlannerState::Fault:
      return "FAULT";
    case PlannerState::Recovering:
      return "RECOVERING";
  }
  return "UNKNOWN";
}

// Whitespace separated numbers of one line, read with strtod (so inf and nan work).
class Fields {
 public:
  explicit Fields(const std::string& line) : s_(line) {}
  bool next(double* out) {
    const char* p = s_.c_str() + pos_;
    while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
    if (*p == '\0') return false;
    char* end = nullptr;
    const double v = std::strtod(p, &end);
    if (end == p) return false;
    pos_ = static_cast<std::size_t>(end - s_.c_str());
    *out = v;
    return true;
  }

 private:
  const std::string& s_;
  std::size_t pos_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  std::string bundle, stream_path, out_path;
  bool describe = false;
  std::map<std::string, Param> params = default_params();
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) die(2, "missing value after " + a);
      return argv[++i];
    };
    if (a == "--bundle") {
      bundle = next();
    } else if (a == "--stream") {
      stream_path = next();
    } else if (a == "--out") {
      out_path = next();
    } else if (a == "--describe") {
      describe = true;
    } else if (a == "--v-max") {
      double v = 0.0;
      if (!parse_number(next(), &v)) die(2, "--v-max wants a number");
      set_param(&params, "v_max_mps", v);
    } else if (a == "--set") {
      const std::string kv = next();
      const auto eq = kv.find('=');
      double v = 0.0;
      if (eq == std::string::npos || !parse_number(kv.substr(eq + 1), &v)) {
        die(2, "--set wants name=value, got '" + kv + "'");
      }
      set_param(&params, kv.substr(0, eq), v);
    } else {
      die(2,
          "usage: planner_replay_cli --bundle DIR [--v-max M_S] [--set name=value]... "
          "[--stream FILE] [--out CSV] [--describe]");
    }
  }
  if (bundle.empty()) die(2, "--bundle is required");

  std::string why;
  const auto geometry = geometry_from_binding(VEHICLE_PARAMS, &why);
  if (!geometry) die(2, why);
  const PlannerConfig cfg = config_from(params);
  const LoadResult loaded = load_dynamics_bundle(bundle, *geometry, cfg.logic.v_max_mps);
  if (!loaded.ok()) {
    std::cerr << refusal_code_name(loaded.refusal->code) << ": " << loaded.refusal->message << "\n";
    return 2;
  }
  const bool yaw_from_params = params.at("laser_yaw_from_vehicle_params").value != 0.0;
  const double laser_yaw_rad =
      yaw_from_params ? geometry->lidar_mount_yaw_rad : params.at("laser_yaw_offset_rad").value;
  Planner planner(cfg, loaded.bundle->params, *geometry);
  // The gate's view of each scan, as the node builds it (same binding fields, same LiDAR yaw).
  const auto gate_cfg = gate_from_binding(VEHICLE_PARAMS, &why);
  if (!gate_cfg) die(2, why);
  GateConfig gate_config = *gate_cfg;
  gate_config.geometry.lidar_mount_yaw_rad = laser_yaw_rad;
  GateTable gate_table(gate_config.geometry, gate_config.limits);

  if (describe) {
    std::cout << "{\"dynamics_model_id\":\"" << loaded.bundle->model_id << "\",\"parameters\":{";
    bool first = true;
    for (const auto& [name, p] : params) {
      std::cout << (first ? "" : ",") << "\"" << name << "\":" << json_number(p.value);
      first = false;
    }
    std::cout << "},\"laser_yaw_rad\":" << json_number(laser_yaw_rad)
              << ",\"horizon_steps\":" << planner.horizon_steps()
              << ",\"delay_ticks\":" << planner.delay_ticks()
              << ",\"tick_s\":" << json_number(cfg.tick_s) << ",\"steering_max_rate_rad_per_s\":"
              << json_number(geometry->steering_max_rate_rad_per_s)
              << ",\"max_acceleration_mps2\":" << json_number(geometry->max_acceleration_mps2)
              << "}\n";
    return 0;
  }

  std::ifstream file;
  if (!stream_path.empty()) {
    file.open(stream_path);
    if (!file) die(2, "cannot open " + stream_path);
  }
  std::istream& in = stream_path.empty() ? std::cin : file;
  std::ofstream csv_file;
  if (!out_path.empty()) {
    csv_file.open(out_path);
    if (!csv_file) die(2, "cannot write " + out_path);
  }
  std::ostream& csv = out_path.empty() ? std::cout : csv_file;
  csv << "now_s,state,state_name,cmd_speed_mps,cmd_steering_rad,solve_time_s,est_speed_mps,"
         "est_steering_rad,v_cap_mps,best_cost,plan_progress_m,plan_min_clearance_m,"
         "collision_fraction,track_age_s,scan_age_s,samples,samples_halved,track_confidence,"
         "freespace_goal_x_m,freespace_goal_y_m,freespace_reach_m,block_reason,recovery_attempts,"
         "elapsed_s,bunched,plan_stale,gap_reset,start_missing,steer_step_rad,steer_allow_rad,"
         "speed_up_step_mps,speed_allow_mps,slew_violation\n";
  // The elapsed-time check (design 14.7): between two consecutive publishes the steering may move
  // by at most f * steering_rate * dt and the speed magnitude grow by at most f * max_accel * dt,
  // dt the real time between the two ticks (not clamped to a tick). One row per tick; the first
  // has no previous publish.
  const double slew_fraction = cfg.logic.rate_limit_fraction;
  bool have_prev_cmd = false;
  double prev_now = 0.0, prev_steer = 0.0, prev_speed = 0.0;

  std::vector<PointXY> points(kMaxScanPoints);
  DistanceField field;
  FreespaceField freespace(*geometry);
  double last_published_speed = 0.0;
  double last_published_steering = 0.0;
  bool have_prev_goal = false;
  PointXY prev_goal{0.0, 0.0};
  bool has_scan = false;
  double scan_stamp_s = 0.0;
  LocalTrackData track;
  bool has_track = false;
  bool has_odom = false;
  double odom_stamp_s = 0.0;
  double wheel_speed_mps = 0.0;
  std::vector<float> ranges;

  std::string line;
  long line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') continue;
    const char kind = line[first];
    const std::string rest = line.substr(first + 1);
    Fields f(rest);
    auto bad = [&](const char* what) {
      die(2, "stream line " + std::to_string(line_no) + ": bad " + what + " line");
    };
    if (kind == 'S') {
      double arrival, stamp, amin, ainc, rmin, rmax, n;
      if (!f.next(&arrival) || !f.next(&stamp) || !f.next(&amin) || !f.next(&ainc) ||
          !f.next(&rmin) || !f.next(&rmax) || !f.next(&n) || n < 0 || n > 1e6) {
        bad("scan");
      }
      ranges.assign(static_cast<std::size_t>(n), 0.0f);
      for (std::size_t k = 0; k < ranges.size(); ++k) {
        double r;
        if (!f.next(&r)) bad("scan (too few ranges)");
        ranges[k] = static_cast<float>(r);
      }
      const std::size_t count =
          scan_to_points(ranges.data(), ranges.size(), amin, ainc, rmin, rmax, laser_yaw_rad,
                         *geometry, kBodyMarginM, points.data(), points.size());
      field.build(points.data(), count);
      GateScan gate_scan;
      gate_scan.ranges = ranges.data();
      gate_scan.count = ranges.size();
      gate_scan.angle_min_rad = amin;
      gate_scan.angle_increment_rad = ainc;
      gate_scan.range_min_m = rmin;
      gate_scan.range_max_m = rmax;
      gate_table.build(gate_scan);
      // As the node's scan callback: the free-space field with the previous scan's goal.
      freespace.build(field, points.data(), count, have_prev_goal ? &prev_goal : nullptr);
      have_prev_goal = freespace.valid();
      if (have_prev_goal) prev_goal = freespace.goal();
      has_scan = true;
      scan_stamp_s = stamp;
    } else if (kind == 'O') {
      double arrival, v;
      if (!f.next(&arrival) || !f.next(&v)) bad("odometry");
      if (!std::isfinite(v)) continue;  // a bad sample does not refresh the staleness clock
      wheel_speed_mps = v;
      odom_stamp_s = arrival;
      has_odom = true;
    } else if (kind == 'T') {
      double arrival, stamp, valid, residual, intrusion;
      if (!f.next(&arrival) || !f.next(&stamp) || !f.next(&valid) || !f.next(&residual) ||
          !f.next(&intrusion)) {
        bad("track");
      }
      track.stamp_s = stamp;
      track.frame_id = "base_link";
      track.station_spacing_m = kTrackStationSpacingM;
      track.valid_count = static_cast<int>(valid);
      track.boundary_residual_m = residual;
      track.intrusion_fraction = intrusion;
      std::vector<double>* arrays[5] = {&track.x_m, &track.y_m, &track.half_width_left_m,
                                        &track.half_width_right_m, &track.sigma_m};
      for (auto* a : arrays) {
        a->assign(kTrackStations, 0.0);
        for (int k = 0; k < kTrackStations; ++k) {
          if (!f.next(&(*a)[static_cast<std::size_t>(k)])) bad("track (too few numbers)");
        }
      }
      has_track = true;
    } else if (kind == 'K') {
      double now;
      if (!f.next(&now)) bad("tick");
      TickInput ti;
      ti.now_s = now;
      ti.has_odom = has_odom;
      ti.odom_stamp_s = odom_stamp_s;
      ti.wheel_speed_mps = wheel_speed_mps;
      ti.has_scan = has_scan;
      ti.scan_stamp_s = scan_stamp_s;
      ti.field = has_scan ? &field : nullptr;
      ti.freespace = has_scan ? &freespace : nullptr;
      ti.gate = has_scan ? &gate_table : nullptr;
      // The replay has no safety node: the gated /drive is taken to pass what was published.
      ti.has_gated = true;
      ti.gated_stamp_s = now;
      ti.gated_speed_mps = last_published_speed;
      ti.gated_steering_rad = last_published_steering;
      ti.track = has_track ? &track : nullptr;
      const TickOutput o = planner.tick(ti);
      double steer_step = 0.0, steer_allow = 0.0, speed_up = 0.0, speed_allow = 0.0;
      bool violation = false;
      if (have_prev_cmd) {
        const double dt = std::max(0.0, now - prev_now);
        steer_step = std::fabs(o.cmd.steering_rad - prev_steer);
        steer_allow = slew_fraction * geometry->steering_max_rate_rad_per_s * dt;
        speed_up = std::max(0.0, std::fabs(o.cmd.speed_mps) - std::fabs(prev_speed));
        speed_allow = slew_fraction * geometry->max_acceleration_mps2 * dt;
        violation = steer_step > steer_allow + 1e-9 || speed_up > speed_allow + 1e-9;
      }
      have_prev_cmd = true;
      prev_now = now;
      prev_steer = o.cmd.steering_rad;
      prev_speed = o.cmd.speed_mps;
      csv << time_number(now) << ',' << static_cast<int>(o.state) << ',' << state_name(o.state)
          << ',' << csv_number(o.cmd.speed_mps) << ',' << csv_number(o.cmd.steering_rad) << ','
          << csv_number(o.solve_time_s) << ',' << csv_number(o.est_speed_mps) << ','
          << csv_number(o.est_steering_rad) << ',' << csv_number(o.v_cap_mps) << ','
          << csv_number(o.best_cost) << ',' << csv_number(o.plan_progress_m) << ','
          << csv_number(o.plan_min_clearance_m) << ',' << csv_number(o.collision_fraction) << ','
          << csv_number(o.track_age_s) << ',' << csv_number(o.scan_age_s) << ',' << o.samples << ','
          << (o.samples_halved ? 1 : 0) << ',' << csv_number(o.track_confidence) << ','
          << csv_number(o.has_freespace_goal ? o.freespace_goal_x_m : NAN) << ','
          << csv_number(o.has_freespace_goal ? o.freespace_goal_y_m : NAN) << ','
          << csv_number(o.freespace_reach_m) << ',' << static_cast<int>(o.block_reason) << ','
          << o.recovery_attempts << ',' << csv_number(o.elapsed_s) << ',' << (o.bunched ? 1 : 0)
          << ',' << (o.plan_stale ? 1 : 0) << ',' << (o.gap_reset ? 1 : 0) << ','
          << static_cast<int>(o.start_missing) << ',' << csv_number(steer_step) << ','
          << csv_number(steer_allow) << ',' << csv_number(speed_up) << ','
          << csv_number(speed_allow) << ',' << (violation ? 1 : 0) << '\n';
      last_published_speed = o.cmd.speed_mps;
      last_published_steering = o.cmd.steering_rad;
    } else {
      die(2, "stream line " + std::to_string(line_no) + ": unknown event '" + std::string(1, kind) +
                 "'");
    }
  }
  return 0;
}
