// mppi_planner_node: ROS plumbing around racer_planner::Planner (claude-docs/13 part C).
//
//   /scan             sensor_msgs/LaserScan, best effort, keep last 10. Builds the distance field
//   in
//                     its own callback group.
//   /pnp/local_track  racer_pnp_msgs/LocalTrack, reliable, keep last 1.
//   /odom/wheel       nav_msgs/Odometry, best effort (as the VESC driver publishes), the wheel
//   speed. /pnp/lap/guidance racer_pnp_msgs/LapGuidance, reliable, keep last 1, used only when
//                     use_lap_guidance is true.
//   -> /drive_raw          AckermannDriveStamped, reliable, keep last 10, EVERY tick (50 Hz) in
//   every
//                          state, so safety_node's watchdog only fires when this node dies.
//   -> /pnp/planner/status PlannerStatus, reliable, keep last 50, every tick.
//   -> /pnp/plan           nav_msgs/Path in base_link, reliable, keep last 5, every fifth solve.
//
// NEVER /drive: racer_safety/safety_node is the sole publisher of /drive (CLAUDE.md invariant 1).
//
// Startup refuses before any publisher exists: a dynamics bundle that fails any row of the refusal
// table, a speed cap above the bundle validity, a null vehicle_params field, or a non-empty
// debug_stall_profile without use_sim_time logs FATAL "<code>: <message>" and the process exits
// non-zero.
//
// Threading: scan processing runs in one callback group; the solve timer and the small callbacks
// (odometry, track, guidance, the gated /drive) in a second; the 50 Hz publish timer in a third,
// on a three thread executor. The publish timer only wakes, copies the latest plan snapshot and
// publishes, so a late solve cannot starve /drive_raw (a plan older than plan_timeout_s stops the
// car instead). It does not help when the whole process is descheduled. The finished distance
// field is handed to the solve by swapping a shared_ptr under a short mutex, from a pool of three
// so the solve never reads a buffer the scan thread is rebuilding.
//
// On SIGINT, before the context shuts down, one speed 0 command is published and the publish
// timer stops (the hardware mux and safety_node's watchdog remain what stops the car).
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <racer_pnp_msgs/msg/lap_guidance.hpp>
#include <racer_pnp_msgs/msg/local_track.hpp>
#include <racer_pnp_msgs/msg/planner_status.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "binding_geometry.hpp"
#include "gate_binding.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/gate_predictor.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/planner.hpp"

namespace racer_planner {

namespace {

using AckermannMsg = ackermann_msgs::msg::AckermannDriveStamped;
using PlannerStatusMsg = racer_pnp_msgs::msg::PlannerStatus;

constexpr double kBodyMarginM = 0.02;  // same rule as the perception input in the contract
constexpr std::size_t kMaxScanPoints = 8192;

// What one scan produces for the tick: the distance field, the free-space progress field and the
// safety gate's view of the scan (in-path distance along each steering angle).
struct ScanProducts {
  DistanceField field;
  FreespaceField freespace;
  GateTable gate;
  ScanProducts(const VehicleGeometry& g, const GateConfig& gc)
      : freespace(g), gate(gc.geometry, gc.limits) {}
};

// The gated /drive as last seen (read only: never published here).
struct GatedInfo {
  bool has = false;
  double stamp_s = 0.0;
  double speed_mps = 0.0;
  double steering_rad = 0.0;
};

// "T:D,T:D,...": T seconds after the first DRIVING solve the solve thread sleeps D seconds.
// Sorted by T. False with a reason on a malformed list, a negative T or a D outside (0, 10].
bool parse_stall_profile(const std::string& text, std::vector<std::pair<double, double>>* out,
                         std::string* why) {
  out->clear();
  std::stringstream all(text);
  std::string item;
  while (std::getline(all, item, ',')) {
    if (item.empty()) continue;
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
      *why = "'" + item + "' is not T:D";
      return false;
    }
    char* end1 = nullptr;
    char* end2 = nullptr;
    const std::string a = item.substr(0, colon), b = item.substr(colon + 1);
    const double t = std::strtod(a.c_str(), &end1);
    const double d = std::strtod(b.c_str(), &end2);
    if (a.empty() || b.empty() || *end1 != '\0' || *end2 != '\0' || !std::isfinite(t) ||
        !std::isfinite(d) || t < 0.0 || !(d > 0.0) || d > 10.0) {
      *why = "'" + item + "' needs T >= 0 and 0 < D <= 10";
      return false;
    }
    out->emplace_back(t, d);
  }
  std::sort(out->begin(), out->end());
  return true;
}

}  // namespace

class MppiPlannerNode : public rclcpp::Node {
 public:
  MppiPlannerNode() : rclcpp::Node("mppi_planner_node") {
    // -- parameters --------------------------------------------------------------------------
    const std::string bundle_dir = declare_string(
        "dynamics_bundle", "",
        "Directory of the dynamics model bundle (manifest.yaml plus payload). Required.");
    PlannerConfig cfg;
    LogicParams& lg = cfg.logic;
    lg.v_max_mps = declare_double(
        "v_max_mps", 0.5, 0.1, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "Speed cap (m/s). Above the bundle's validity v_max_mps the node refuses to start.");
    lg.scan_timeout_s = declare_double("scan_timeout_s", 0.35, 0.05, 5.0,
                                       "Scan up to this old: full speed cap; older: the cap ramps "
                                       "down to 0 at scan_hard_timeout_s (s).");
    lg.scan_hard_timeout_s = declare_double(
        "scan_hard_timeout_s", 0.60, 0.05, 0.60,
        "Scan older than this: NO_SCAN; between scan_timeout_s and this the speed cap ramps 1 to 0 "
        "(s).");
    lg.odom_timeout_s = declare_double(
        "odom_timeout_s", 0.10, 0.02, 5.0,
        "Wheel speed older than this: the planner uses its own speed estimate and the cap may not "
        "exceed the last measured wheel speed plus 0.05 m/s (s).");
    lg.odom_hard_timeout_s = declare_double("odom_hard_timeout_s", 0.20, 0.02, 0.20,
                                            "Wheel speed older than this: NO_ODOM (s).");
    lg.plan_timeout_s = declare_double(
        "plan_timeout_s", 0.12, 0.05, 0.12,
        "A plan snapshot older than this while driving or recovering: speed 0, FAULT (s).");
    lg.start_ticks = static_cast<int>(declare_int(
        "start_ticks", 25, 1, 200,
        "Warm start: consecutive on-time ticks (and solves on the real inputs) before moving."));
    lg.start_tick_gap_s = declare_double("start_tick_gap_s", 0.03, 0.01, 0.2,
                                         "Warm start: largest gap between two counted ticks (s).");
    lg.start_timeout_s = declare_double(
        "start_timeout_s", 5.0, 0.5, 120.0,
        "STARTING longer than this logs the missing condition once a second (s). It never moves "
        "without the conditions.");
    lg.start_stop_hold_s = declare_double(
        "start_stop_hold_s", 1.0, 0.2, 60.0,
        "A return to DRIVING after the published speed was 0 for this long goes through STARTING "
        "again (s).");
    lg.require_gated_for_start = declare_bool(
        "require_gated_for_start", true,
        "Warm start needs the gated /drive to have been seen (safety_node alive). false: sim "
        "tests that run without safety_node.");
    lg.min_scan_points =
        static_cast<int>(declare_int("min_scan_points", lg.min_scan_points, 0, 1000,
                                     "A scan with fewer usable points than this is NO_SCAN (an "
                                     "empty field is not free space)."));
    lg.start_gated_grace_s = declare_double(
        "start_gated_grace_s", 0.5, 0.0, 5.0,
        "While the gated /drive has not been seen, STARTING publishes no /drive_raw for up to "
        "this long, so the steering it starts from is the one safety_node holds (s).");
    lg.conf_intrusion_full = declare_double("conf_intrusion_full", lg.conf_intrusion_full, 0.0, 1.0,
                                            "Track intrusion fraction up to this: full trust.");
    lg.conf_intrusion_zero = declare_double("conf_intrusion_zero", lg.conf_intrusion_zero, 0.0, 1.0,
                                            "Track intrusion fraction from this: no trust.");
    lg.conf_residual_full_m =
        declare_double("conf_residual_full_m", lg.conf_residual_full_m, 0.0, 5.0,
                       "Track boundary residual up to this: full trust (m).");
    lg.conf_residual_zero_m =
        declare_double("conf_residual_zero_m", lg.conf_residual_zero_m, 0.0, 5.0,
                       "Track boundary residual from this: no trust (m).");
    lg.conf_valid_zero = declare_double("conf_valid_zero", lg.conf_valid_zero, 0.0, 16.0,
                                        "Valid station count up to this: no trust.");
    lg.conf_valid_full = declare_double("conf_valid_full", lg.conf_valid_full, 0.0, 16.0,
                                        "Valid station count from this: full trust.");
    lg.conf_age_full_s = declare_double("conf_age_full_s", lg.conf_age_full_s, 0.0, 5.0,
                                        "Track age up to this: full trust (s).");
    lg.conf_age_zero_s = declare_double("conf_age_zero_s", lg.conf_age_zero_s, 0.0, 5.0,
                                        "Track age from this: no trust (s).");
    lg.conf_rise_per_s = declare_double("conf_rise_per_s", lg.conf_rise_per_s, 0.01, 100.0,
                                        "Track confidence rise limit (1/s); it falls at once.");
    lg.freespace_speed_fraction =
        declare_double("freespace_speed_fraction", lg.freespace_speed_fraction, 0.3, 1.0,
                       "Share of the speed cap kept at track confidence 0.");
    lg.use_lap_guidance = declare_bool(
        "use_lap_guidance", false, "Cap the speed with /pnp/lap/guidance v_cap_mps when fresh.");
    lg.v_no_guidance_mps = declare_double(
        "v_no_guidance_mps", lg.v_max_mps, 0.0, VEHICLE_PARAMS.limits.global_speed_cap_mps,
        "With use_lap_guidance and no fresh, well formed guidance the speed cap is "
        "min(v_max_mps, this) (m/s). Default: v_max_mps.");
    const std::string stall_profile = declare_string(
        "debug_stall_profile", "",
        "TEST ONLY: 'T:D,T:D,...' makes the solve thread sleep D s once the planner has been "
        "driving for T s. The node refuses to start with a non-empty profile unless "
        "use_sim_time is true.");
    const double rate_hz =
        declare_double("control_rate_hz", 50.0, 20.0, 100.0, "Control tick rate (Hz).");
    cfg.tick_s = 1.0 / rate_hz;
    MppiParams& mp = cfg.mppi;
    mp.horizon_steps = static_cast<int>(
        declare_int("horizon_steps", 30, 5, 100, "Rollout steps of the bundle's dt."));
    const int samples = static_cast<int>(declare_int(
        "samples", 512, kMinSamples, kMaxSamples, "Samples per solve (the budget may lower it)."));
    mp.samples = samples;
    cfg.max_samples = static_cast<int>(declare_int("max_samples", 768, kMinSamples, kMaxSamples,
                                                   "Solver capacity, at least samples."));
    mp.sigma_speed_mps =
        declare_double("sigma_speed_mps", mp.sigma_speed_mps, 0.0, 2.0, "Speed noise std (m/s).");
    mp.sigma_steering_rad = declare_double("sigma_steering_rad", mp.sigma_steering_rad, 0.0, 1.0,
                                           "Steering noise std (rad).");
    mp.beta =
        declare_double("noise_beta", mp.beta, 0.0, 0.99, "Time correlation of the control noise.");
    mp.lambda = declare_double("lambda", mp.lambda, 1e-3, 1e3,
                               "MPPI temperature, a fraction of (median - minimum) cost.");
    mp.cost_scale_floor = declare_double("cost_scale_floor", mp.cost_scale_floor, 1e-3, 1e3,
                                         "Floor of (median - minimum) cost in the temperature.");
    mp.seed =
        static_cast<std::uint64_t>(declare_int("seed", 1, 0, 2147483647, "Noise generator seed."));
    mp.w_progress = declare_double("w_progress", mp.w_progress, 0.0, 1e3, "Progress weight.");
    mp.c_collision =
        declare_double("c_collision", mp.c_collision, 0.0, 1e9, "Cost of a hard collision.");
    mp.d_hard_m =
        declare_double("d_hard_m", mp.d_hard_m, 0.0, 1.0, "Hard collision clearance (m).");
    mp.w_clear = declare_double("w_clear", mp.w_clear, 0.0, 1e5, "Clearance weight.");
    mp.d_soft_base_m =
        declare_double("d_soft_base_m", mp.d_soft_base_m, 0.0, 2.0, "Soft clearance at rest (m).");
    mp.t_headway_s = declare_double("t_headway_s", mp.t_headway_s, 0.0, 5.0,
                                    "Soft clearance growth per m/s (s).");
    mp.w_bound = declare_double("w_bound", mp.w_bound, 0.0, 1e5, "Track boundary weight.");
    mp.m_bound_m =
        declare_double("m_bound_m", mp.m_bound_m, 0.0, 1.0, "Track boundary margin (m).");
    mp.w_lat = declare_double("w_lat", mp.w_lat, 0.0, 1e4, "Lateral acceleration weight.");
    mp.a_lat_max_mps2 = declare_double("a_lat_max_mps2", mp.a_lat_max_mps2, 0.1, 50.0,
                                       "Lateral acceleration limit (m/s^2).");
    mp.w_du = declare_double("w_du", mp.w_du, 0.0, 1e4, "Speed command change weight.");
    mp.w_steer_rate = declare_double("w_steer_rate", mp.w_steer_rate, 0.0, 1e4,
                                     "Steering command rate weight (rad/s squared).");
    mp.speed_floor_margin_mps =
        declare_double("speed_floor_margin_mps", mp.speed_floor_margin_mps, 0.0, 0.5,
                       "Margin above the bundle's speed dead zone: commands are 0 or at least "
                       "dead zone + margin (m/s).");
    mp.corner_radius_m = declare_double("corner_radius_m", mp.corner_radius_m, 0.005, 0.2,
                                        "Radius of the footprint corner circles (m).");
    lg.steer_output_tau_s = declare_double("steer_output_tau_s", lg.steer_output_tau_s, 0.0, 0.5,
                                           "Steering output low-pass time constant (s), 0 is off.");
    lg.steer_output_hysteresis_rad = declare_double(
        "steer_output_hysteresis_rad", lg.steer_output_hysteresis_rad, 0.0, 0.02,
        "The steering output holds until the target is this far from it (rad), 0 is off.");
    lg.gap_reset_s = declare_double("gap_reset_s", lg.gap_reset_s, 0.0, 0.058,
                                    "Gap between ticks above which the speed ramp restarts from 0 "
                                    "(safety_node's watchdog trips at 60 ms), 0 is off (s).");
    lg.rate_limit_fraction =
        declare_double("rate_limit_fraction", lg.rate_limit_fraction, 0.05, 1.0,
                       "Share of the steering rate and acceleration limits the output may use.");
    mp.move_margin =
        declare_double("move_margin", mp.move_margin, 0.0, 100.0,
                       "Cost the best moving sample must beat the best standing one by.");
    mp.w_term = declare_double("w_term", mp.w_term, 0.0, 1e5, "Stop within sight weight.");
    mp.w_gate = declare_double("w_gate", mp.w_gate, 0.0, 1e4,
                               "Weight of the gate aware cost: how far the first 0.3 s of a plan "
                               "stay from the safety gate's TTC and distance-floor band (0 off).");
    mp.gate_ttc_margin = declare_double(
        "gate_ttc_margin", mp.gate_ttc_margin, 0.5, 5.0,
        "The gate cost starts at this multiple of limits.ttc_warning_s (time to collision).");
    mp.gate_clear_margin =
        declare_double("gate_clear_margin", mp.gate_clear_margin, 0.5, 10.0,
                       "The gate cost starts at this multiple of limits.min_forward_clearance_m.");
    mp.w_trust = declare_double("w_trust", mp.w_trust, 0.0, 1e7,
                                "Weight of the speed trust region around the lap profile's "
                                "per station speed bound (only while a profile is active).");
    cfg.drivability.stall_hold_s = declare_double(
        "stall_hold_s", cfg.drivability.stall_hold_s, 0.5, 600.0,
        "Wheel speed under 0.03 m/s while a speed at least the floor is published for this long: "
        "nothing drivable, reason 3 (s). Open loop replays set it high.");
    cfg.drivability.gate_hold_s =
        declare_double("gate_hold_s", cfg.drivability.gate_hold_s, 0.2, 600.0,
                       "Gated /drive speed 0 for this long while a positive speed is published: "
                       "nothing drivable, reason 4 (s).");
    RecoveryParams& rc = cfg.recovery;
    rc.recover_distance_m = declare_double("recover_distance_m", rc.recover_distance_m, 0.1, 1.0,
                                           "Length of the planned reverse arc (m).");
    rc.recover_speed_mps = declare_double("recover_speed_mps", rc.recover_speed_mps, 0.1, 0.6,
                                          "Reverse speed (m/s), at least the speed floor.");
    rc.recover_margin_m = declare_double("recover_margin_m", rc.recover_margin_m, 0.0, 0.3,
                                         "Clearance added to d_hard_m for the reverse arc (m).");
    rc.recover_exhausted_hold_s =
        declare_double("recover_exhausted_hold_s", rc.recover_exhausted_hold_s, 0.0, 600.0,
                       "Least time BLOCKED with reason 5 after the attempts run out (s).");
    rc.recover_max_s = declare_double("recover_max_s", rc.recover_max_s, 0.5, 10.0,
                                      "A recovery ends after this long (s).");
    rc.recover_retry_s =
        declare_double("recover_retry_s", rc.recover_retry_s, 0.5, 30.0,
                       "Wait before planning again after a failed plan or abort (s).");
    rc.recover_gate_debounce_s =
        declare_double("recover_gate_debounce_s", rc.recover_gate_debounce_s, 0.02, 1.0,
                       "Gated /drive speed not negative for this long: abort (s).");
    rc.recover_progress_m = declare_double("recover_progress_m", rc.recover_progress_m, 0.1, 5.0,
                                           "Forward distance that resets the attempt count (m).");
    rc.recover_max_attempts =
        static_cast<int>(declare_int("recover_max_attempts", rc.recover_max_attempts, 1, 20,
                                     "Recoveries before BLOCKED with reason 5."));
    cfg.budget.halve_over_s =
        declare_double("solve_halve_over_ms", 15.0, 1.0, 200.0,
                       "A solve slower than this halves the sample count (ms).") *
        1e-3;
    cfg.budget.restore_under_s =
        declare_double("solve_restore_under_ms", 6.0, 0.5, 200.0,
                       "Solves faster than this restore the sample count gradually (ms).") *
        1e-3;
    const bool yaw_from_params =
        declare_bool("laser_yaw_from_vehicle_params", true,
                     "true (the car): LiDAR yaw from vehicle_params. false: SIMULATOR AND "
                     "SYNTHETIC SCAN TESTS ONLY, use laser_yaw_offset_rad.");
    const double yaw_offset =
        declare_double("laser_yaw_offset_rad", 0.0, -M_PI, M_PI,
                       "LiDAR yaw in base_link when laser_yaw_from_vehicle_params is false (rad).");

    // -- vehicle geometry and the bundle, before any publisher exists --------------------------
    std::string why;
    const auto geometry = geometry_from_binding(VEHICLE_PARAMS, &why);
    if (!geometry) {
      RCLCPP_FATAL(get_logger(), "mppi_planner_node: %s", why.c_str());
      throw std::runtime_error(why);
    }
    geometry_ = *geometry;
    laser_yaw_rad_ = yaw_from_params ? geometry_.lidar_mount_yaw_rad : yaw_offset;
    // The gate's view of the scan, from the same binding fields safety_node uses, with the LiDAR
    // yaw the node resolved (so a simulator run with an explicit yaw matches safety_node's).
    const auto gate_cfg = gate_from_binding(VEHICLE_PARAMS, &why);
    if (!gate_cfg) {
      RCLCPP_FATAL(get_logger(), "mppi_planner_node: %s", why.c_str());
      throw std::runtime_error(why);
    }
    gate_cfg_ = *gate_cfg;
    gate_cfg_.geometry.lidar_mount_yaw_rad = laser_yaw_rad_;
    if (bundle_dir.empty()) {
      const std::string m = "ManifestMissing: parameter dynamics_bundle is empty";
      RCLCPP_FATAL(get_logger(), "%s", m.c_str());
      throw std::runtime_error(m);
    }
    if (lg.scan_hard_timeout_s < lg.scan_timeout_s || lg.odom_hard_timeout_s < lg.odom_timeout_s) {
      const std::string m =
          "ParameterInvalid: scan_hard_timeout_s must be at least scan_timeout_s and "
          "odom_hard_timeout_s at least odom_timeout_s";
      RCLCPP_FATAL(get_logger(), "%s", m.c_str());
      throw std::runtime_error(m);
    }
    if (!stall_profile.empty()) {
      // A stall injected into the solve thread is for sim acceptance only: it can never be on by
      // accident on the car, where use_sim_time is false.
      if (!get_parameter("use_sim_time").as_bool()) {
        const std::string m =
            "StallProfileRefused: debug_stall_profile is set and use_sim_time is false; the "
            "stall injection is for simulation only";
        RCLCPP_FATAL(get_logger(), "%s", m.c_str());
        throw std::runtime_error(m);
      }
      std::string perr;
      if (!parse_stall_profile(stall_profile, &stalls_, &perr)) {
        const std::string m = "StallProfileInvalid: " + perr;
        RCLCPP_FATAL(get_logger(), "%s", m.c_str());
        throw std::runtime_error(m);
      }
      RCLCPP_WARN(get_logger(), "debug_stall_profile with %zu stalls: SIMULATION ONLY",
                  stalls_.size());
    }
    const LoadResult loaded = load_dynamics_bundle(bundle_dir, geometry_, lg.v_max_mps);
    if (!loaded.ok()) {
      const std::string m =
          std::string(refusal_code_name(loaded.refusal->code)) + ": " + loaded.refusal->message;
      RCLCPP_FATAL(get_logger(), "%s", m.c_str());
      throw std::runtime_error(m);
    }
    model_id_ = loaded.bundle->model_id;
    RCLCPP_INFO(get_logger(),
                "dynamics bundle %s loaded from %s (golden max error %.2e, residual %s)",
                model_id_.c_str(), bundle_dir.c_str(), loaded.bundle->golden_max_abs_error,
                loaded.bundle->params.residual.enabled ? "enabled" : "off");
    for (const auto& p : loaded.bundle->payload) {
      RCLCPP_INFO(get_logger(), "  payload %s sha256 %s", p.file.c_str(), p.sha256.c_str());
    }
    guidance_timeout_s_ = lg.guidance_timeout_s;
    planner_ = std::make_unique<Planner>(cfg, loaded.bundle->params, geometry_);
    tick_s_ = cfg.tick_s;
    const std::size_t horizon = static_cast<std::size_t>(planner_->horizon_steps()) + 1;
    path_.poses.resize(horizon);
    path_.header.frame_id = "base_link";
    for (int i = 0; i < 3; ++i) pool_[i] = std::make_shared<ScanProducts>(geometry_, gate_cfg_);
    points_.resize(kMaxScanPoints);
    status_.dynamics_model_id = model_id_;

    // -- I/O ----------------------------------------------------------------------------------
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = scan_group_;
    rclcpp::SubscriptionOptions command_opts;
    command_opts.callback_group = command_group_;
    const rclcpp::QoS scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const rclcpp::QoS reliable_10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const rclcpp::QoS reliable_1 = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const rclcpp::QoS reliable_5 = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    const rclcpp::QoS reliable_50 = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&MppiPlannerNode::on_scan, this, std::placeholders::_1),
        scan_opts);
    track_sub_ = create_subscription<racer_pnp_msgs::msg::LocalTrack>(
        "/pnp/local_track", reliable_1,
        std::bind(&MppiPlannerNode::on_track, this, std::placeholders::_1), command_opts);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom/wheel", scan_qos, std::bind(&MppiPlannerNode::on_odom, this, std::placeholders::_1),
        command_opts);
    // The gated command, read only (never published here: invariant 1), the QoS gap_follow_node
    // uses for it. Recovery watches it to see whether the safety node's rear gate passes a reverse.
    gated_sub_ = create_subscription<AckermannMsg>(
        "/drive", reliable_10, std::bind(&MppiPlannerNode::on_gated, this, std::placeholders::_1),
        command_opts);
    guidance_sub_ = create_subscription<racer_pnp_msgs::msg::LapGuidance>(
        "/pnp/lap/guidance", reliable_1,
        std::bind(&MppiPlannerNode::on_guidance, this, std::placeholders::_1), command_opts);
    drive_pub_ = create_publisher<AckermannMsg>("/drive_raw", reliable_10);
    status_pub_ = create_publisher<PlannerStatusMsg>("/pnp/planner/status", reliable_50);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/pnp/plan", reliable_5);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(tick_s_));
    solve_timer_ = create_wall_timer(period, std::bind(&MppiPlannerNode::on_solve_timer, this),
                                     command_group_);
    publish_timer_ = create_wall_timer(period, std::bind(&MppiPlannerNode::on_publish_timer, this),
                                       publish_group_);
    RCLCPP_INFO(get_logger(),
                "mppi_planner_node up: %.0f Hz, %d samples, horizon %d x %.3f s, v_max %.2f m/s, "
                "delay %d ticks, laser yaw %.3f rad",
                rate_hz, samples, planner_->horizon_steps(), loaded.bundle->params.dt_s,
                lg.v_max_mps, planner_->delay_ticks(), laser_yaw_rad_);
  }

 private:
  // -- parameter helpers ---------------------------------------------------------------------
  double declare_double(const std::string& name, double def, double lo, double hi,
                        const std::string& desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = lo;
    r.to_value = hi;
    d.floating_point_range = {r};
    return declare_parameter<double>(name, def, d);
  }
  int64_t declare_int(const std::string& name, int64_t def, int64_t lo, int64_t hi,
                      const std::string& desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = lo;
    r.to_value = hi;
    r.step = 1;
    d.integer_range = {r};
    return declare_parameter<int64_t>(name, def, d);
  }
  bool declare_bool(const std::string& name, bool def, const std::string& desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    return declare_parameter<bool>(name, def, d);
  }
  std::string declare_string(const std::string& name, const std::string& def,
                             const std::string& desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    return declare_parameter<std::string>(name, def, d);
  }

  double stamp_or_now(const builtin_interfaces::msg::Time& t) const {
    if (t.sec == 0 && t.nanosec == 0) return now().seconds();
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  }

  // -- callbacks -----------------------------------------------------------------------------
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const std::size_t n =
        scan_to_points(msg->ranges.data(), msg->ranges.size(), msg->angle_min, msg->angle_increment,
                       msg->range_min, msg->range_max, laser_yaw_rad_, geometry_, kBodyMarginM,
                       points_.data(), points_.size());
    std::shared_ptr<ScanProducts> target;
    {
      std::lock_guard<std::mutex> lock(field_mutex_);
      for (auto& p : pool_) {
        if (p != field_ && p.use_count() == 1) {
          target = p;
          break;
        }
      }
    }
    if (!target) return;  // cannot happen with a pool of three, but never write a buffer in use
    target->field.build(points_.data(), n);
    GateScan gate_scan;
    gate_scan.ranges = msg->ranges.data();
    gate_scan.count = msg->ranges.size();
    gate_scan.angle_min_rad = msg->angle_min;
    gate_scan.angle_increment_rad = msg->angle_increment;
    gate_scan.range_min_m = msg->range_min;
    gate_scan.range_max_m = msg->range_max;
    target->gate.build(gate_scan);
    target->freespace.build(target->field, points_.data(), n,
                            have_prev_goal_ ? &prev_goal_ : nullptr);
    have_prev_goal_ = target->freespace.valid();
    if (have_prev_goal_) prev_goal_ = target->freespace.goal();
    const double stamp = stamp_or_now(msg->header.stamp);
    std::lock_guard<std::mutex> lock(field_mutex_);
    field_ = target;
    scan_stamp_s_ = stamp;
  }

  void on_track(const racer_pnp_msgs::msg::LocalTrack::SharedPtr msg) {
    track_.stamp_s = stamp_or_now(msg->header.stamp);
    track_.frame_id = msg->header.frame_id;
    track_.station_spacing_m = msg->station_spacing_m;
    track_.x_m.assign(msg->x_m.begin(), msg->x_m.end());
    track_.y_m.assign(msg->y_m.begin(), msg->y_m.end());
    track_.half_width_left_m.assign(msg->half_width_left_m.begin(), msg->half_width_left_m.end());
    track_.half_width_right_m.assign(msg->half_width_right_m.begin(),
                                     msg->half_width_right_m.end());
    track_.sigma_m.assign(msg->sigma_m.begin(), msg->sigma_m.end());
    track_.valid_count = msg->valid_count;
    track_.boundary_residual_m = msg->boundary_residual_m;
    track_.intrusion_fraction = msg->intrusion_fraction;
    {
      std::lock_guard<std::mutex> lock(meta_mutex_);
      perception_model_id_ = msg->model_id;
    }
    perception_id_version_.fetch_add(1, std::memory_order_release);
    has_track_ = true;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double v = msg->twist.twist.linear.x;
    if (!std::isfinite(v)) return;  // a bad sample does not refresh the staleness clock
    wheel_speed_mps_ = v;
    odom_stamp_s_ = now().seconds();
    has_odom_ = true;
  }

  void on_guidance(const racer_pnp_msgs::msg::LapGuidance::SharedPtr msg) {
    guidance_v_cap_mps_ = msg->v_cap_mps;
    guidance_stamp_s_ = now().seconds();
    has_guidance_ = true;
    // The per station profile (empty in D v0). A refused one is counted and logged at most once
    // per second; the planner then runs without a profile (the speed cap above still holds).
    const double now_s = now().seconds();
    const LapProfileStatus st =
        make_lap_profile(msg->s_m.data(), msg->v_ref_mps.data(), msg->cost_to_go_s.data(),
                         msg->s_m.size(), msg->v_ref_mps.size(), msg->cost_to_go_s.size(),
                         stamp_or_now(msg->header.stamp), now_s, &lap_profile_);
    guidance_profile_bad_ = st == LapProfileStatus::kBad;
    if (st == LapProfileStatus::kBad) {
      ++guidance_profile_bad_count_;
      if (now_s - guidance_bad_log_s_ >= 1.0) {
        guidance_bad_log_s_ = now_s;
        RCLCPP_WARN(get_logger(),
                    "lap guidance profile refused (%u so far): lengths %zu/%zu/%zu, age %.3f s",
                    guidance_profile_bad_count_, msg->s_m.size(), msg->v_ref_mps.size(),
                    msg->cost_to_go_s.size(), now_s - stamp_or_now(msg->header.stamp));
      }
    }
  }

  void on_gated(const AckermannMsg::SharedPtr msg) {
    if (!std::isfinite(msg->drive.speed)) return;
    GatedInfo g;
    g.has = true;
    g.speed_mps = msg->drive.speed;
    g.steering_rad = std::isfinite(msg->drive.steering_angle) ? msg->drive.steering_angle : 0.0;
    g.stamp_s = now().seconds();
    std::lock_guard<std::mutex> lock(gated_mutex_);
    gated_ = g;
  }

  GatedInfo gated_snapshot() {
    std::lock_guard<std::mutex> lock(gated_mutex_);
    return gated_;
  }

  // The solve: inputs in, one plan snapshot out. Runs in the command callback group, so it never
  // overlaps the small callbacks that feed it.
  void on_solve_timer() {
    // TEST ONLY: a stall of the solve thread from debug_stall_profile (the node refused to start
    // with one unless use_sim_time is true).
    if (have_driving_ && next_stall_ < stalls_.size() &&
        now().seconds() - driving_since_s_ >= stalls_[next_stall_].first) {
      const double d = stalls_[next_stall_].second;
      ++next_stall_;
      RCLCPP_WARN(get_logger(), "debug stall: the solve thread sleeps %.3f s", d);
      std::this_thread::sleep_for(std::chrono::duration<double>(d));
    }
    TickInput in;
    in.now_s = now().seconds();
    if (last_solve_tick_s_ > 0.0 && in.now_s - last_solve_tick_s_ > 0.040) {
      solve_gaps_.fetch_add(1, std::memory_order_relaxed);
    }
    last_solve_tick_s_ = in.now_s;
    const GatedInfo g = gated_snapshot();
    in.has_gated = g.has;
    in.gated_stamp_s = g.stamp_s;
    in.gated_speed_mps = g.speed_mps;
    in.gated_steering_rad = g.steering_rad;
    in.has_odom = has_odom_;
    in.odom_stamp_s = odom_stamp_s_;
    in.wheel_speed_mps = wheel_speed_mps_;
    std::shared_ptr<ScanProducts> products;
    {
      std::lock_guard<std::mutex> lock(field_mutex_);
      products = field_;
      in.scan_stamp_s = scan_stamp_s_;
    }
    in.has_scan = products != nullptr;
    in.field = products ? &products->field : nullptr;
    in.freespace = products ? &products->freespace : nullptr;
    in.track = has_track_ ? &track_ : nullptr;
    in.has_guidance = has_guidance_;
    in.guidance_stamp_s = guidance_stamp_s_;
    in.guidance_v_cap_mps = guidance_v_cap_mps_;
    in.gate = products ? &products->gate : nullptr;
    // A refused profile, or one that has gone stale, makes the planner fall back to
    // v_no_guidance_mps (LogicInputs::guidance_profile_bad).
    const bool profile_stale =
        lap_profile_.active && in.now_s - lap_profile_.stamp_s > guidance_timeout_s_;
    in.guidance_profile_bad = guidance_profile_bad_ || profile_stale;
    in.lap = lap_profile_.active && !profile_stale ? &lap_profile_ : nullptr;

    const SolveReport rep = planner_->solve(in);
    if (rep.bunched) return;
    if (rep.state == PlannerState::Driving && !have_driving_) {
      have_driving_ = true;
      driving_since_s_ = in.now_s;
    }
    if (rep.samples_halved && !halved_logged_) {
      halved_logged_ = true;
      RCLCPP_WARN(get_logger(), "a solve took %.1f ms, over the budget; the sample count is halved",
                  rep.solve_time_s * 1e3);
    }
    if (++solve_cycle_ % 5 == 0) publish_plan(now());
  }

  // The 50 Hz output: the latest plan snapshot through the output shaping. No solve here.
  void on_publish_timer() {
    if (stopping_.load(std::memory_order_acquire)) return;
    PublishInput pin;
    pin.now_s = now().seconds();
    const GatedInfo g = gated_snapshot();
    pin.has_gated = g.has;
    pin.gated_stamp_s = g.stamp_s;
    pin.gated_steering_rad = g.steering_rad;
    if (last_pub_tick_s_ > 0.0 && pin.now_s - last_pub_tick_s_ > 0.040) ++pub_gaps_;
    last_pub_tick_s_ = pin.now_s;
    if (pin.now_s - gap_log_s_ >= 10.0) {
      if (gap_log_s_ > 0.0) {
        const std::uint32_t bunched = planner_->bunched_ticks();
        RCLCPP_INFO(
            get_logger(),
            "tick gaps over 40 ms in the last 10 s: publish %u, solve %u; bunched ticks: %u",
            pub_gaps_, solve_gaps_.exchange(0, std::memory_order_relaxed),
            bunched - bunched_logged_);
        bunched_logged_ = bunched;
      }
      pub_gaps_ = 0;
      gap_log_s_ = pin.now_s;
    }

    const TickOutput out = planner_->publish(pin);
    ++cycle_;
    const rclcpp::Time stamp = now();

    AckermannMsg drive;
    drive.header.stamp = stamp;
    // Negative only while recovering, and then no further than the reverse speed.
    const double reverse_limit =
        out.state == PlannerState::Recovering ? planner_->reverse_speed_mps() : 0.0;
    drive.drive.speed = float32_within(out.cmd.speed_mps, -reverse_limit,
                                       VEHICLE_PARAMS.limits.global_speed_cap_mps);
    drive.drive.steering_angle = float32_within(out.cmd.steering_rad, geometry_.steering_min_rad,
                                                geometry_.steering_max_rad);
    {
      std::lock_guard<std::mutex> lock(drive_mutex_);
      if (stopping_.load(std::memory_order_acquire)) return;
      if (!out.drive_suppressed) {
        drive_pub_->publish(drive);
        last_cmd_steering_ = drive.drive.steering_angle;
      }
    }

    status_.header.stamp = stamp;
    status_.state = static_cast<uint8_t>(out.state);
    status_.cycle = cycle_;
    status_.solve_time_s = static_cast<float>(out.solve_time_s);
    status_.cmd_speed_mps = drive.drive.speed;
    status_.cmd_steering_rad = drive.drive.steering_angle;
    status_.est_speed_mps = static_cast<float>(out.est_speed_mps);
    status_.est_steering_rad = static_cast<float>(out.est_steering_rad);
    status_.v_cap_mps = static_cast<float>(out.v_cap_mps);
    status_.best_cost = static_cast<float>(out.best_cost);
    status_.plan_progress_m = static_cast<float>(out.plan_progress_m);
    status_.plan_min_clearance_m = static_cast<float>(out.plan_min_clearance_m);
    status_.collision_fraction = static_cast<float>(out.collision_fraction);
    status_.track_age_s = static_cast<float>(out.track_age_s);
    status_.scan_age_s = static_cast<float>(out.scan_age_s);
    status_.samples = static_cast<uint16_t>(out.samples);
    status_.track_confidence = static_cast<float>(out.track_confidence);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    status_.freespace_goal_x_m =
        out.has_freespace_goal ? static_cast<float>(out.freespace_goal_x_m) : nan;
    status_.freespace_goal_y_m =
        out.has_freespace_goal ? static_cast<float>(out.freespace_goal_y_m) : nan;
    status_.freespace_reach_m =
        out.has_freespace_goal ? static_cast<float>(out.freespace_reach_m) : nan;
    status_.block_reason = static_cast<uint8_t>(out.block_reason);
    status_.recovery_attempts = static_cast<uint8_t>(std::min(out.recovery_attempts, 255));
    const std::uint32_t id_version = perception_id_version_.load(std::memory_order_acquire);
    if (id_version != status_id_version_) {
      status_id_version_ = id_version;
      std::lock_guard<std::mutex> lock(meta_mutex_);
      status_.perception_model_id = perception_model_id_;
    }
    status_pub_->publish(status_);

    if (out.plan_stale && !stale_logged_) {
      RCLCPP_WARN(get_logger(), "plan stale: the newest plan is %.3f s old, publishing speed 0",
                  out.plan_age_s);
    }
    stale_logged_ = out.plan_stale;
    if (out.left_starting) {
      RCLCPP_INFO(get_logger(), "start gate passed after %.2f s", out.starting_s);
    }
    if (out.state == PlannerState::Starting && out.start_timed_out &&
        pin.now_s - start_log_s_ >= 1.0) {
      start_log_s_ = pin.now_s;
      RCLCPP_WARN(get_logger(), "STARTING for %.1f s, still holding speed 0; missing: %s",
                  out.starting_s, start_missing_name(out.start_missing));
    }
    if (out.state != last_state_) {
      RCLCPP_INFO(get_logger(), "state %d -> %d", static_cast<int>(last_state_),
                  static_cast<int>(out.state));
      last_state_ = out.state;
    }
  }

  // Called once by the shutdown path before the context goes away: one speed 0 command, then the
  // publish timer stays quiet. safety_node's watchdog and the hardware mux are what stop the car;
  // this is only the polite end of a clean exit.
  void publish_stop() {
    if (stopping_.exchange(true)) return;
    std::lock_guard<std::mutex> lock(drive_mutex_);
    AckermannMsg m;
    m.header.stamp = now();
    m.drive.speed = 0.0f;
    m.drive.steering_angle = last_cmd_steering_;
    try {
      drive_pub_->publish(m);
    } catch (...) {
    }
  }

 public:
  void install_shutdown_hook() {
    std::weak_ptr<MppiPlannerNode> weak =
        std::static_pointer_cast<MppiPlannerNode>(shared_from_this());
    get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak] {
      if (auto self = weak.lock()) self->publish_stop();
    });
  }

 private:
  void publish_plan(const rclcpp::Time& stamp) {
    path_.header.stamp = stamp;
    if (!planner_->plan_valid()) {
      path_.poses.clear();
      plan_pub_->publish(path_);
      path_.poses.resize(static_cast<std::size_t>(planner_->horizon_steps()) + 1);
      return;
    }
    const auto& plan = planner_->plan_scan_frame();
    const Pose2& ego = planner_->ego_in_scan_frame();
    const double c = std::cos(-ego.yaw);
    const double s = std::sin(-ego.yaw);
    for (std::size_t i = 0; i < plan.size() && i < path_.poses.size(); ++i) {
      const double dx = plan[i].x - ego.x;
      const double dy = plan[i].y - ego.y;
      const double yaw = plan[i].yaw - ego.yaw;
      auto& ps = path_.poses[i];
      ps.header.stamp = stamp;
      ps.header.frame_id = "base_link";
      ps.pose.position.x = c * dx - s * dy;
      ps.pose.position.y = s * dx + c * dy;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = std::sin(0.5 * yaw);
      ps.pose.orientation.w = std::cos(0.5 * yaw);
    }
    plan_pub_->publish(path_);
  }

  VehicleGeometry geometry_;
  GateConfig gate_cfg_;
  double laser_yaw_rad_ = 0.0;
  double tick_s_ = 0.02;
  std::string model_id_;
  std::unique_ptr<Planner> planner_;

  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<racer_pnp_msgs::msg::LocalTrack>::SharedPtr track_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<racer_pnp_msgs::msg::LapGuidance>::SharedPtr guidance_sub_;
  rclcpp::Subscription<AckermannMsg>::SharedPtr gated_sub_;
  rclcpp::Publisher<AckermannMsg>::SharedPtr drive_pub_;
  rclcpp::Publisher<PlannerStatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::TimerBase::SharedPtr solve_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // scan thread -> solve hand-off
  std::mutex field_mutex_;
  std::array<std::shared_ptr<ScanProducts>, 3> pool_;
  std::shared_ptr<ScanProducts> field_;
  bool have_prev_goal_ = false;  // scan thread only
  PointXY prev_goal_{0.0, 0.0};
  double scan_stamp_s_ = 0.0;
  std::vector<PointXY> points_;

  // command group state (the solve timer and the small callbacks never run concurrently)
  LocalTrackData track_;
  bool has_track_ = false;
  double wheel_speed_mps_ = 0.0;
  double odom_stamp_s_ = 0.0;
  bool has_odom_ = false;
  double guidance_v_cap_mps_ = 0.0;
  double guidance_stamp_s_ = 0.0;
  bool has_guidance_ = false;
  LapProfile lap_profile_;  // preallocated, filled by on_guidance
  bool guidance_profile_bad_ = false;
  uint32_t guidance_profile_bad_count_ = 0;
  double guidance_bad_log_s_ = 0.0;
  double guidance_timeout_s_ = 2.0;
  double last_solve_tick_s_ = 0.0;
  std::uint32_t solve_cycle_ = 0;
  bool halved_logged_ = false;
  bool have_driving_ = false;  // solve thread: the first Driving solve has happened
  double driving_since_s_ = 0.0;
  std::vector<std::pair<double, double>> stalls_;  // (time since first Driving, stall) pairs
  std::size_t next_stall_ = 0;
  nav_msgs::msg::Path path_;

  // read by the publish timer, written by the command group
  std::mutex gated_mutex_;
  GatedInfo gated_;
  std::mutex meta_mutex_;
  std::string perception_model_id_;
  std::atomic<std::uint32_t> perception_id_version_{0};

  // publish timer state
  std::uint32_t status_id_version_ = 0;
  double last_pub_tick_s_ = 0.0;
  double gap_log_s_ = 0.0;
  std::uint32_t pub_gaps_ = 0;
  std::atomic<std::uint32_t> solve_gaps_{0};
  std::uint32_t bunched_logged_ = 0;
  std::uint32_t cycle_ = 0;
  bool stale_logged_ = false;
  double start_log_s_ = 0.0;
  PlannerState last_state_ = PlannerState::Starting;
  PlannerStatusMsg status_;

  // shutdown
  std::mutex drive_mutex_;
  std::atomic<bool> stopping_{false};
  float last_cmd_steering_ = 0.0f;
};

}  // namespace racer_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_planner::MppiPlannerNode>();
    node->install_shutdown_hook();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mppi_planner_node"), "mppi_planner_node: refusing to run: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
