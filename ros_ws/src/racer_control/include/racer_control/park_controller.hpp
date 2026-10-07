// park_node's state machine (roadmap 2.9): slot search, the parallel park and the three-point
// turn, executed from wheel odometry. ROS-free and gtest-covered (test/test_park_controller.cpp
// drives it through a kinematic bicycle simulation with synthetic scans); park_node owns the ROS
// plumbing and calls it once per control cycle.
//
// THIS DRIVES THE CAR, INCLUDING IN REVERSE, ON ITS OWN once started. It only ever produces a
// request: park_node publishes it on /drive_raw and racer_safety's safety_node gates it into
// /drive (CLAUDE.md invariant 1); this class never sees /drive except as the gated speed it reads
// back (BLOCKED below).
//
// PHASES (ParkPhase), each transition logged at INFO:
//   IDLE            nothing published.
//   SEARCH          straight ahead (steering 0) at search_speed_mps, looking for a slot in every
//                   scan (park_slot.hpp). A pocket must be found in slot_confirm_scans
//                   consecutive scans with both edges within slot_confirm_tolerance_m (in the
//                   odometry frame) before it counts; the confirmed slot is their average.
//   SLOT_FOUND      still driving straight while the plan is computed (park_planner.hpp; park_node
//                   runs it off the control thread). An infeasible plan is logged with its reason
//                   and the slot is ignored from then on: SEARCH continues past it. After
//                   search_max_distance_m without a feasible slot: ABORT.
//   DRIVE_TO_START  straight to the planned start x (forward; backward if already past it).
//   ARC_1, ARC_2    reverse at park_speed_mps on the two arcs (park_planner.hpp).
//   STRAIGHTEN      steering to 0, then the forward centring move (may be 0).
//   TURN_1..TURN_3  the three-point turn's arcs.
//   DONE            zero speed for final_hold_s, then nothing published.
//   ABORT           the same, entered from any active phase with the reason logged (WARN).
//
// STEPS. Each phase after SLOT_FOUND is one PlanStep with one steering. A step runs in stages:
//   STOPPING  zero speed until the car has stopped (wheel speed below stopped_speed_mps on two
//             consecutive cycles; without odometry, after settle_s), at most stop_timeout_s.
//             Skipped when the car is already stopped, or when the new step continues in the same
//             direction with the same steering (SEARCH -> DRIVE_TO_START forward).
//   STEERING  the steering request ramps to the step's steering at steering_rate_rad_per_s, the
//             car standing still: steering only ever changes while stopped.
//   SETTLING  settle_s with the new steering held and zero speed, so the servo finishes moving
//             before the car does. Skipped when the steering did not change.
//   MOVING    the step's speed (forward steps search_speed_mps, reverse steps park_speed_mps),
//             ramped at acceleration_mps2 (braking is a step to zero), until the step's distance
//             is covered.
// At the start of MOVING the step's target (park_planner.hpp "STEPS") is resolved against the
// dead-reckoned pose, so ARC_2 ends parallel even if ARC_1 overshot, and STRAIGHTEN reaches the
// planned final x; then the swept body of that resolved motion is checked again against the
// plan's obstacles and must keep recheck_clearance_fraction * margin, or ABORT.
//
// STOP LEAD. The distance the car rolls after the zero command (from the stop command to
// stopped) is measured at every stop. The next step in the same direction sends its zero command
// that much early (at most max_stop_lead_m), so a car that overshoots by a repeatable amount
// stops where it should from the second stop on. Until a direction has had a stop of its own,
// it borrows the other direction's measurement (DRIVE_TO_START's forward stop sets ARC_1's
// lead), and before any stop at all initial_stop_lead_m applies. An overshoot that is not taken
// out this way still ends parallel (the heading targets), but ARC_1 overshooting by e turns
// theta into theta + e / R and parks the car about 2 R sin(theta) e / R deeper.
//
// ODOMETRY AND DEAD RECKONING. The pose (rear axle, plan frame: the car's pose when the run
// started) advances every cycle by the signed distance from /odom/wheel (pose.position.x, the
// VESC's along-track distance) on the arc of the current steering request: the steering only
// changes while the car stands, so while it moves the request is the steering. ASSUMPTION,
// stated: the odometry distance is the rear axle's arc length. On this 4WD car it is the motor's
// distance through the gear ratio and the tyre radius (both PROVISIONAL), which reads a little
// more than the rear axle travels on a tight arc; the turning radius is the kinematic
// L / tan(delta) with the steering map assumed linear (vehicle_params steering.pwm_to_angle_table
// is null). Both must be measured before the arcs are trusted (docs/notes/parking-2026-10-07.md).
// Without fresh odometry:
//   * require_odometry true (the real car's profile): a run refuses to start, and a run already
//     going ABORTs.
//   * require_odometry false (sim fixtures): the distance is integrated from the gated /drive
//     speed (or, while that is stale, this controller's own request), odom_fallback() turns true
//     and park_node WARNs once per second. The real car moves less than commanded (it lags, and
//     the VESC does nothing below about 0.44 m/s), so a fallback run comes up SHORT.
//
// BLOCKED. While a nonzero speed is requested, a gated /drive speed of zero (safety_node braking
// the request) or no fresh gated /drive at all accumulates time; after blocked_abort_s: ABORT,
// with the phase in the log line. Without safety_node in the loop there is no gated /drive, so
// the run aborts: that is deliberate (fail closed).
//
// Time is the configured control period per call (the "no measured dt" rule of the other
// racer_control nodes).
#ifndef RACER_CONTROL_PARK_CONTROLLER_HPP_
#define RACER_CONTROL_PARK_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "racer_control/laser_scan.hpp"
#include "racer_control/park_geometry.hpp"
#include "racer_control/park_planner.hpp"
#include "racer_control/park_slot.hpp"

namespace racer_control {

enum class ParkMode { kNone, kParallel, kThreePointTurn };
enum class MotionStage { kStopping, kSteering, kSettling, kMoving };
const char* park_mode_name(ParkMode mode);
const char* motion_stage_name(MotionStage stage);

struct ParkControllerConfig {
  SlotDetectorConfig detector;
  ParkPlannerConfig planner;
  double control_period_s = 0.02;
  double search_speed_mps = 0.5;  // > 0; also every forward step
  double park_speed_mps = -0.5;   // < 0; every reverse step
  double settle_s = 0.3;
  // Both set by park_node from the binding (a fraction of steering.max_rate_rad_per_s and of
  // actuation.max_acceleration_mps2); a run refuses to start while either is not positive.
  double steering_rate_rad_per_s = 0.0;
  double acceleration_mps2 = 0.0;
  double stopped_speed_mps = 0.05;
  double stop_timeout_s = 3.0;
  double blocked_abort_s = 2.0;
  double search_max_distance_m = 6.0;
  int slot_confirm_scans = 3;
  double slot_confirm_tolerance_m = 0.1;
  double max_stop_lead_m = 0.15;
  // The stop lead before any stop has been measured (both directions).
  double initial_stop_lead_m = 0.0;
  double recheck_clearance_fraction = 0.5;
  bool require_odometry = true;
  double odom_jump_max_m = 0.5;
  double final_hold_s = 1.0;
  double lane_window_m = 0.3;  // lane measured over the body length plus this each end
  double lane_lateral_max_m = 3.0;
  TurnDirection turn_direction = TurnDirection::kLeft;
  // true: plan inside on_scan/step (tests). false: park_node takes the request with
  // take_plan_request(), plans off the control thread and hands the result to deliver_plan().
  bool synchronous_planning = true;
};

struct ParkOdometry {
  double distance_m = 0.0;  // /odom/wheel pose.position.x, signed along-track distance
  double speed_mps = 0.0;   // /odom/wheel twist.linear.x
};

struct ParkStepInput {
  std::optional<ParkOdometry> odometry;   // only while fresh
  std::optional<double> gated_speed_mps;  // the gated /drive speed, only while fresh
  bool scan_fresh = false;
};

struct ParkOutput {
  bool publish = false;
  double speed_mps = 0.0;
  double steering_rad = 0.0;
};

struct ParkLogLine {
  bool warn = false;
  std::string text;
};

struct StartResult {
  bool ok = false;
  std::string message;
};

struct PlanRequest {
  std::uint64_t id = 0;
  ParkSlot slot;
  Pose2 car;
};

class ParkController {
 public:
  explicit ParkController(const ParkControllerConfig& config);

  StartResult start_parallel_park(bool odometry_fresh);
  StartResult start_three_point_turn(const ScanInput& scan, bool scan_fresh, bool odometry_fresh);
  // Any active phase -> ABORT (logged with the reason). No effect when idle, done or aborted.
  StartResult abort(const std::string& reason);

  // Every /scan; only SEARCH uses it.
  void on_scan(const ScanInput& scan);
  // Once per control cycle.
  ParkOutput step(const ParkStepInput& input);

  // Asynchronous planning (synchronous_planning false).
  std::optional<PlanRequest> take_plan_request();
  void deliver_plan(std::uint64_t request_id, const ParallelParkPlan& plan);

  std::vector<ParkLogLine> take_logs();

  ParkPhase phase() const { return phase_; }
  ParkMode mode() const { return mode_; }
  MotionStage stage() const { return stage_; }
  bool active() const;
  bool odom_fallback() const { return odom_fallback_; }
  const Pose2& pose() const { return pose_; }
  const ParallelParkPlan& parallel_plan() const { return parallel_plan_; }
  const ThreePointTurnPlan& turn_plan() const { return turn_plan_; }
  std::string status_text() const;
  const ParkControllerConfig& config() const { return config_; }

 private:
  void log(bool warn, const std::string& text);
  void set_phase(ParkPhase phase, const std::string& detail);
  void enter_terminal(ParkPhase phase, const std::string& reason);
  void begin_step(std::size_t index);
  void start_moving();
  void finish_step();
  void handle_detection(const SlotDetection& detection);
  void accept_plan(const ParallelParkPlan& plan);
  double step_speed(int direction) const;
  bool usable_run_config() const;
  void reset_run();

  ParkControllerConfig config_;
  SlotDetector detector_;
  ParkMode mode_ = ParkMode::kNone;
  ParkPhase phase_ = ParkPhase::kIdle;
  MotionStage stage_ = MotionStage::kMoving;
  std::vector<ParkLogLine> logs_;

  // Dead reckoning and odometry.
  Pose2 pose_;
  bool has_odom_baseline_ = false;
  double last_odom_distance_m_ = 0.0;
  bool odom_fallback_ = false;
  std::optional<double> measured_speed_mps_;

  // Request.
  double speed_cmd_mps_ = 0.0;
  double steering_cmd_rad_ = 0.0;

  // Steps.
  std::vector<PlanStep> steps_;
  std::size_t step_index_ = 0;
  int direction_ = 1;
  double step_distance_m_ = 0.0;
  double travelled_m_ = 0.0;
  double stage_time_s_ = 0.0;
  bool steering_changed_ = false;
  int stopped_cycles_ = 0;
  std::array<double, 2> stop_lead_m_{{0.0, 0.0}};  // [0] reverse, [1] forward
  std::array<bool, 2> lead_measured_{{false, false}};
  int stop_direction_ = 0;
  double overshoot_m_ = 0.0;
  double blocked_s_ = 0.0;
  double terminal_time_s_ = 0.0;
  std::vector<Segment2> obstacles_;

  // Search.
  std::vector<ParkSlot> confirm_;
  std::vector<std::pair<double, double>> rejected_;  // x intervals of slots found infeasible
  SlotReject last_search_reject_ = SlotReject::kNone;
  std::optional<PlanRequest> pending_request_;
  std::uint64_t next_request_id_ = 1;
  std::uint64_t awaited_request_id_ = 0;
  ParkSlot awaited_slot_;

  ParallelParkPlan parallel_plan_;
  ThreePointTurnPlan turn_plan_;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_PARK_CONTROLLER_HPP_
