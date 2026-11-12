// The per tick pipeline of the planner without any ROS: staleness gate, ego motion bookkeeping,
// delay roll-forward, solver, rate limits, state update. mppi_planner_node is plumbing around
// this class, and the closed loop tests drive it directly.
#ifndef RACER_PLANNER_PLANNER_HPP_
#define RACER_PLANNER_PLANNER_HPP_

#include <array>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/ego_motion.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"
#include "racer_planner/planner_logic.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct PlannerConfig {
  LogicParams logic;
  MppiParams mppi;
  SampleBudgetParams budget;
  int max_samples = 768;  // solver capacity, also the budget's upper bound
  double tick_s = 0.02;
};

struct TickInput {
  double now_s = 0.0;
  bool has_odom = false;
  double odom_stamp_s = 0.0;
  double wheel_speed_mps = 0.0;
  bool has_scan = false;
  double scan_stamp_s = 0.0;
  const DistanceField* field = nullptr;   // built from the scan stamped scan_stamp_s
  const LocalTrackData* track = nullptr;  // latest track, null when none yet
  bool has_guidance = false;
  double guidance_stamp_s = 0.0;
  double guidance_v_cap_mps = 0.0;
};

struct TickOutput {
  PlannerState state = PlannerState::Starting;
  Command cmd;
  double solve_time_s = 0.0;
  double est_speed_mps = 0.0;
  double est_steering_rad = 0.0;
  double v_cap_mps = 0.0;
  double best_cost = 0.0;
  double plan_progress_m = 0.0;
  double plan_min_clearance_m = 0.0;
  double collision_fraction = 0.0;
  double track_age_s = 0.0;
  double scan_age_s = 0.0;
  int samples = 0;
  bool samples_halved = false;  // the budget halved the sample count after this tick's solve
};

class Planner {
 public:
  Planner(const PlannerConfig& config, const DynamicsParams& dynamics,
          const VehicleGeometry& geometry);

  // Never throws: an internal error gives Fault with speed 0 and steering 0.
  TickOutput tick(const TickInput& in);

  // Weighted mean plan of the last Driving tick, in the planning frame (the scan frame).
  const std::vector<Pose2>& plan_scan_frame() const { return solver_.plan(); }
  // Car pose now in the planning frame, valid after a Driving or Blocked tick.
  const Pose2& ego_in_scan_frame() const { return ego_pose_; }
  bool plan_valid() const { return plan_valid_; }
  int horizon_steps() const { return horizon_; }
  int delay_ticks() const { return delay_ticks_; }
  const PlannerConfig& config() const { return cfg_; }
  void set_v_max(double v_max_mps) { cfg_.logic.v_max_mps = v_max_mps; }

 private:
  TickOutput run(const TickInput& in);
  void finish_tick(const Command& published);

  PlannerConfig cfg_;
  VehicleGeometry geometry_;
  DynamicsModel model_tick_;
  EgoMotion ego_;
  MppiSolver solver_;
  SampleBudget budget_;
  SelfCheckHold hold_;
  TrackProjector projector_;
  int horizon_;
  int delay_ticks_;
  State est_;                       // v from the wheel, delta from the model
  std::array<Control, 32> ring_{};  // commands published but not yet acting
  int ring_head_ = 0;
  Command last_published_;
  double steer_lp_ = 0.0;  // output low-pass state, valid while driving
  bool steer_lp_valid_ = false;
  Pose2 ego_pose_;
  bool plan_valid_ = false;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_PLANNER_HPP_
