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
//   -> /pnp/plan           nav_msgs/Path in base_link, reliable, keep last 5, every fifth tick.
//
// NEVER /drive: racer_safety/safety_node is the sole publisher of /drive (CLAUDE.md invariant 1).
//
// Startup refuses before any publisher exists: a dynamics bundle that fails any row of the refusal
// table, a speed cap above the bundle validity, or a null vehicle_params field logs FATAL
// "<code>: <message>" and the process exits non-zero.
//
// Threading, as gap_follow_node does: scan processing runs in one callback group, the 50 Hz timer
// and the small callbacks in the other, on a two thread executor. The finished distance field is
// handed to the tick by swapping a shared_ptr under a short mutex, from a pool of three so the tick
// never reads a buffer the scan thread is rebuilding.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <array>
#include <chrono>
#include <cmath>
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
#include <stdexcept>
#include <string>
#include <vector>

#include "binding_geometry.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/planner.hpp"

namespace racer_planner {

namespace {

using AckermannMsg = ackermann_msgs::msg::AckermannDriveStamped;
using PlannerStatusMsg = racer_pnp_msgs::msg::PlannerStatus;

constexpr double kBodyMarginM = 0.02;  // same rule as the perception input in the contract
constexpr std::size_t kMaxScanPoints = 8192;

// What one scan produces for the tick: the distance field and the free-space progress field.
struct ScanProducts {
  DistanceField field;
  FreespaceField freespace;
  explicit ScanProducts(const VehicleGeometry& g) : freespace(g) {}
};

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
    lg.scan_timeout_s =
        declare_double("scan_timeout_s", 0.35, 0.05, 5.0, "Scan older than this: NO_SCAN (s).");
    lg.odom_timeout_s = declare_double("odom_timeout_s", 0.10, 0.02, 5.0,
                                       "Wheel speed older than this: NO_ODOM (s).");
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
    if (bundle_dir.empty()) {
      const std::string m = "ManifestMissing: parameter dynamics_bundle is empty";
      RCLCPP_FATAL(get_logger(), "%s", m.c_str());
      throw std::runtime_error(m);
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
    planner_ = std::make_unique<Planner>(cfg, loaded.bundle->params, geometry_);
    tick_s_ = cfg.tick_s;
    const std::size_t horizon = static_cast<std::size_t>(planner_->horizon_steps()) + 1;
    path_.poses.resize(horizon);
    path_.header.frame_id = "base_link";
    for (int i = 0; i < 3; ++i) pool_[i] = std::make_shared<ScanProducts>(geometry_);
    points_.resize(kMaxScanPoints);
    status_.dynamics_model_id = model_id_;

    // -- I/O ----------------------------------------------------------------------------------
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(tick_s_)),
                               std::bind(&MppiPlannerNode::on_timer, this), command_group_);
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
    perception_model_id_ = msg->model_id;
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
  }

  void on_gated(const AckermannMsg::SharedPtr msg) {
    if (!std::isfinite(msg->drive.speed)) return;
    gated_speed_mps_ = msg->drive.speed;
    gated_stamp_s_ = now().seconds();
    has_gated_ = true;
  }

  void on_timer() {
    TickInput in;
    in.now_s = now().seconds();
    if (last_tick_s_ > 0.0 && in.now_s - last_tick_s_ > 0.040) ++tick_gaps_;
    last_tick_s_ = in.now_s;
    if (in.now_s - gap_log_s_ >= 10.0) {
      if (gap_log_s_ > 0.0) {
        RCLCPP_INFO(get_logger(), "tick gaps over 40 ms in the last 10 s: %u", tick_gaps_);
      }
      tick_gaps_ = 0;
      gap_log_s_ = in.now_s;
    }
    in.has_gated = has_gated_;
    in.gated_stamp_s = gated_stamp_s_;
    in.gated_speed_mps = gated_speed_mps_;
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

    const TickOutput out = planner_->tick(in);
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
    drive_pub_->publish(drive);

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
    status_.perception_model_id = perception_model_id_;
    status_pub_->publish(status_);

    if (out.samples_halved && !halved_logged_) {
      halved_logged_ = true;
      RCLCPP_WARN(get_logger(), "a solve took %.1f ms, over the budget; the sample count is halved",
                  out.solve_time_s * 1e3);
    }
    if (out.state != last_state_) {
      RCLCPP_INFO(get_logger(), "state %d -> %d", static_cast<int>(last_state_),
                  static_cast<int>(out.state));
      last_state_ = out.state;
    }
    if (cycle_ % 5 == 0) publish_plan(stamp);
  }

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
  double laser_yaw_rad_ = 0.0;
  double tick_s_ = 0.02;
  std::string model_id_;
  std::unique_ptr<Planner> planner_;

  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<racer_pnp_msgs::msg::LocalTrack>::SharedPtr track_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<racer_pnp_msgs::msg::LapGuidance>::SharedPtr guidance_sub_;
  rclcpp::Subscription<AckermannMsg>::SharedPtr gated_sub_;
  rclcpp::Publisher<AckermannMsg>::SharedPtr drive_pub_;
  rclcpp::Publisher<PlannerStatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // scan thread -> tick hand-off
  std::mutex field_mutex_;
  std::array<std::shared_ptr<ScanProducts>, 3> pool_;
  std::shared_ptr<ScanProducts> field_;
  bool have_prev_goal_ = false;  // scan thread only
  PointXY prev_goal_{0.0, 0.0};
  double scan_stamp_s_ = 0.0;
  std::vector<PointXY> points_;

  // command group state (timer and small callbacks never run concurrently)
  LocalTrackData track_;
  bool has_track_ = false;
  std::string perception_model_id_;
  double wheel_speed_mps_ = 0.0;
  double odom_stamp_s_ = 0.0;
  bool has_odom_ = false;
  double guidance_v_cap_mps_ = 0.0;
  double guidance_stamp_s_ = 0.0;
  bool has_guidance_ = false;
  double gated_speed_mps_ = 0.0;
  double gated_stamp_s_ = 0.0;
  bool has_gated_ = false;
  double last_tick_s_ = 0.0;
  double gap_log_s_ = 0.0;
  uint32_t tick_gaps_ = 0;
  uint32_t cycle_ = 0;
  bool halved_logged_ = false;
  PlannerState last_state_ = PlannerState::Starting;
  PlannerStatusMsg status_;
  nav_msgs::msg::Path path_;
};

}  // namespace racer_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<racer_planner::MppiPlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
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
