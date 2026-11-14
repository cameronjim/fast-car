// The per tick pipeline of the planner without any ROS, in two pieces that the node runs on two
// threads and that tick() runs one after the other:
//
//   solve(in)    staleness gate, ego motion bookkeeping, delay roll-forward, the MPPI solve, the
//                state machine. Makes a plan snapshot (the filtered nominal control sequence, the
//                time it was made for, the cap and the status numbers). Slow, can be late.
//   publish(pin) the 50 Hz output: takes the latest snapshot, indexes the control sequence by the
//                time since it was made, shapes the command (state table, speed floor snap,
//                steering low-pass, slew limit by the measured elapsed time) and does the model
//                side bookkeeping in publish order. A few hundred microseconds, never waits for a
//                solve.
//
// mppi_planner_node is plumbing around this class (solve on one timer, publish on another, in
// their own callback groups); the closed loop tests and planner_replay_cli call tick(). Nothing
// here is a safety function: safety_node gates /drive_raw.
//
// Threads: solve() from one thread, publish() from another, the accessors marked solve thread
// only from the solve thread. The two share a plan snapshot and the published state under one
// mutex that is held only to copy fixed size data (never across a solve, never allocating).
#ifndef RACER_PLANNER_PLANNER_HPP_
#define RACER_PLANNER_PLANNER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/ego_motion.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/gate_predictor.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"
#include "racer_planner/planner_logic.hpp"
#include "racer_planner/recovery.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

constexpr int kMaxPlanSteps = 128;  // horizon_steps upper bound (the node allows 100)

struct PlannerConfig {
  LogicParams logic;
  MppiParams mppi;
  SampleBudgetParams budget;
  RecoveryParams recovery;
  DrivabilityParams drivability;
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
  const DistanceField* field = nullptr;       // built from the scan stamped scan_stamp_s
  const FreespaceField* freespace = nullptr;  // built from the same scan, null or invalid: none
  const LocalTrackData* track = nullptr;      // latest track, null when none yet
  bool has_guidance = false;
  double guidance_stamp_s = 0.0;
  double guidance_v_cap_mps = 0.0;
  bool guidance_profile_bad = false;  // see LogicInputs
  // The gated /drive as seen by the node, read only. A stale or missing one never counts as the
  // gate refusing.
  bool has_gated = false;
  double gated_stamp_s = 0.0;
  double gated_speed_mps = 0.0;
  // The in-path distances of the scan stamped scan_stamp_s as the safety gate judges them
  // (gate_predictor.hpp), null or not valid(): no gate aware cost.
  const GateTable* gate = nullptr;
  // The lap profile (lap_profile.hpp), null or not active: no profile.
  const LapProfile* lap = nullptr;
  double gated_steering_rad = 0.0;  // where STARTING slews the steering from
};

// What the publish side needs from outside.
struct PublishInput {
  double now_s = 0.0;
  bool has_gated = false;  // the gated /drive has been seen at least once
  double gated_stamp_s = 0.0;
  double gated_steering_rad = 0.0;
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
  double track_confidence = 0.0;
  bool has_freespace_goal = false;
  double freespace_goal_x_m = 0.0;  // base_link now, valid when has_freespace_goal
  double freespace_goal_y_m = 0.0;
  double freespace_reach_m = 0.0;
  BlockReason block_reason = BlockReason::None;
  int recovery_attempts = 0;
  bool gap_reset = false;  // the tick after a stall: the speed ramp restarted from zero
  // Timing of the publish side.
  bool bunched = false;     // arrived under half a tick after the last publish: republished it
  bool plan_stale = false;  // the snapshot was too old for a moving state: speed 0, FAULT
  double plan_age_s = 0.0;  // now minus the plan time of the snapshot used (0 when none)
  double elapsed_s = 0.0;   // the elapsed time the slew limits used
  StartMissing start_missing = StartMissing::None;  // while STARTING: the first missing condition
  bool start_timed_out = false;                     // STARTING for longer than start_timeout_s
  double starting_s = 0.0;                          // how long STARTING has lasted
  bool left_starting = false;                       // this publish ended STARTING
  bool drive_suppressed =
      false;  // STARTING before the gated /drive is known: publish no /drive_raw
};

struct SolveReport {
  bool bunched = false;  // arrived under half a tick after the last solve: nothing was solved
  PlannerState state = PlannerState::Starting;
  double solve_time_s = 0.0;
  bool solver_ran = false;
  bool samples_halved = false;
};

// What publish() works from. Fixed size, copied under the mutex.
struct PlanSnapshot {
  bool valid = false;
  PlannerState state = PlannerState::Starting;  // the state the solve side decided
  bool internal_fault = false;                  // Fault from an exception: 0, 0
  bool use_sequence = false;                    // seq is the plan; otherwise hold is
  bool plan_moves = false;
  int n = 0;
  double dt_s = 0.05;
  double plan_time_s = 0.0;  // the time of the inputs the plan was made for
  Command hold;              // a constant command (recovery, or no plan)
  std::array<Control, kMaxPlanSteps> seq{};
  TickOutput status;  // the status numbers of the solve (the publish side fills the rest)
  // Facts for start_gate, from the solve side.
  bool scan_fresh = false;
  bool odom_fresh = false;
  bool track_ever = false;
  bool track_fresh = false;
  int solve_good_ticks = 0;
};

class Planner {
 public:
  Planner(const PlannerConfig& config, const DynamicsParams& dynamics,
          const VehicleGeometry& geometry);

  // Solve side. Never throws: an internal error makes a Fault snapshot.
  SolveReport solve(const TickInput& in);

  // Publish side. Never throws: an internal error gives Fault with speed 0 and steering 0.
  TickOutput publish(const PublishInput& in);

  // solve(in) then publish(in): what one tick of the old planner was, built from the same two
  // pieces the node runs on two timers.
  TickOutput tick(const TickInput& in);

  // Weighted mean plan of the last Driving solve, in the planning frame (the scan frame). Solve
  // thread only.
  const std::vector<Pose2>& plan_scan_frame() const { return solver_.plan(); }
  // Car pose now in the planning frame, valid after a solve that was not Fault, NoOdom or NoScan.
  // Solve thread only.
  const Pose2& ego_in_scan_frame() const { return ego_pose_; }
  bool plan_valid() const { return plan_valid_; }  // solve thread only
  int horizon_steps() const { return horizon_; }
  int delay_ticks() const { return delay_ticks_; }
  // The magnitude limit of a reverse command: max(recover_speed_mps, the speed floor).
  double reverse_speed_mps() const { return reverse_limit_mps_; }
  const PlannerConfig& config() const { return cfg_; }
  void set_v_max(double v_max_mps) { cfg_.logic.v_max_mps = v_max_mps; }
  // Timer callbacks (solve or publish) that arrived under half a tick after the previous one.
  std::uint32_t bunched_ticks() const { return bunched_ticks_.load(std::memory_order_relaxed); }

 private:
  // Published state shared with the solve side: what the commands in flight and the model say.
  struct Shared {
    std::array<Control, 32> ring{};  // commands published but not yet acting
    int ring_head = 0;
    State est;  // v from the wheel (solve side) or the model (publish side), delta from the model
    Command last_published;
    bool starting = true;
  };

  SolveReport solve_impl(const TickInput& in);
  SolveReport solve_fault(const TickInput& in);
  TickOutput publish_impl(const PublishInput& in);
  void advance_model(const Command& published, int steps, const Command& before);
  void store_snapshot(const PlanSnapshot& s);
  static bool is_moving_state(PlannerState s) {
    return s == PlannerState::Driving || s == PlannerState::Recovering;
  }

  PlannerConfig cfg_;
  VehicleGeometry geometry_;
  SlewLimits slew_;
  DynamicsModel model_pub_;  // publish side (the model has scratch space: one instance per thread)
  double plan_dt_s_;
  double reverse_limit_mps_;

  // -- solve side only ----------------------------------------------------------------------
  DynamicsModel model_roll_;  // the delay roll-forward
  EgoMotion ego_;
  MppiSolver solver_;
  SampleBudget budget_;
  DrivabilityMonitor drivability_;
  Recovery recovery_;
  bool recovery_was_active_ = false;
  TrackProjector projector_;
  double confidence_ = 0.0;
  int horizon_;
  int delay_ticks_;
  double last_solve_s_ = 0.0;     // the last real solve
  double last_solve_cb_s_ = 0.0;  // the last solve callback of any kind (a bunched one counts)
  bool have_last_solve_ = false;
  PlannerState last_solve_state_ = PlannerState::Starting;
  int solve_good_ticks_ = 0;
  Pose2 ego_pose_;
  bool plan_valid_ = false;

  // -- publish side only --------------------------------------------------------------------
  Command last_published_;
  double steer_lp_ = 0.0;  // output low-pass state, valid while driving
  bool steer_lp_valid_ = false;
  double last_publish_s_ = 0.0;  // the last message of any kind (a republish counts)
  double last_real_s_ = 0.0;     // the last real publish (a plan snapshot through the shaping)
  bool have_last_publish_ = false;
  int publish_good_ticks_ = 0;
  bool starting_ = true;
  bool start_seeded_ = false;
  double starting_since_s_ = 0.0;
  bool have_starting_since_ = false;
  double last_nonzero_s_ = 0.0;
  TickOutput last_out_;

  // -- shared, under mu_ --------------------------------------------------------------------
  std::mutex mu_;
  PlanSnapshot snapshot_;
  Shared shared_;

  std::atomic<std::uint32_t> bunched_ticks_{0};
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_PLANNER_HPP_
