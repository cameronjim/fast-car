// Recovery by reversing: when nothing is drivable for a while (nose-in to a wall, wedged in a
// corner, wheels not turning, the safety gate refusing forward speed) the planner plans one short
// reverse arc from the scan, backs along it and returns to normal driving. ROS-free.
//
// The reasoning follows racer_control's reverse_escape (vocabulary and the safety argument): the
// reverse speed is small, the arc is checked against the whole body (all seven circles) with the
// hard collision clearance, the gated /drive is watched and the attempt is abandoned when the
// safety node's rear gate does not pass it, and a bounded number of attempts is made before the
// planner reports BLOCKED (reason 5) and waits to be moved. Nothing here bypasses safety_node:
// the planner publishes /drive_raw only and safety_node gates it.
//
//   Start      reasons 1 and 2 held 0.6 s, reason 3 held 1.5 s, reason 4 held 1.0 s (the
//              DrivabilityMonitor reports 3 and 4 only after their holds), attempts under the
//              limit.
//   Plan       once, at the start: recover_arcs reverse arcs of recover_distance_m with steering
//              evenly across the range; each must keep the hard clearance along the way; from its
//              end pose 9 constant forward arcs at the floor speed are scored by the free-space
//              progress (Phi drop of the front axle point) among those that stay clear; the reverse
//              arc with the best score wins. None: wait recover_retry_s and plan again.
//   Execute    Stopping (forward speed command 0 until the wheel speed is under 0.05 m/s), then
//              Reversing at -max(recover_speed_mps, floor) with the arc's steering. Ends when the
//              signed wheel odometry integral reaches the arc length or recover_max_s passes.
//   Abort      the gated /drive speed is not negative for recover_gate_debounce_s while the request
//              is negative, or /drive is stale: wait recover_retry_s.
//   Attempts   counted since the car last covered recover_progress_m forward. After
//              recover_max_attempts the recovery is exhausted: BLOCKED, reason 5, for at least
//              recover_exhausted_hold_s and until the car is moved (a drivable forward plan for
//              recover_release_s).
#ifndef RACER_PLANNER_RECOVERY_HPP_
#define RACER_PLANNER_RECOVERY_HPP_

#include "racer_planner/distance_field.hpp"
#include "racer_planner/footprint.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/planner_logic.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct RecoveryParams {
  double recover_distance_m = 0.35;
  double recover_speed_mps = 0.30;
  double recover_max_s = 2.5;
  double recover_retry_s = 3.0;
  double recover_gate_debounce_s = 0.1;
  double recover_progress_m = 0.5;
  int recover_max_attempts = 3;
  double recover_release_s = 1.0;
  // Once exhausted the planner stays BLOCKED at least this long: backing out of a wall makes a
  // forward plan possible again at once, and without a hold it would drive straight back at it.
  double recover_exhausted_hold_s = 30.0;
  double hold_collide_s = 0.6;  // reasons 1 and 2
  double hold_stall_s = 1.5;    // reason 3
  double hold_gate_s = 1.0;     // reason 4
  double stop_wheel_speed_mps = 0.05;
  double forward_distance_m = 0.35;  // the forward arcs scored from the end of a reverse arc
  double d_hard_m = 0.03;            // the planner passes mppi.d_hard_m + recover_margin_m
  double recover_margin_m = 0.03;    // extra clearance a reversing arc keeps: it runs open loop
  double corner_radius_m = Footprint::kDefaultCornerRadiusM;
  int reverse_arcs = 7;
  int forward_arcs = 9;
};

struct RecoveryPlan {
  bool ok = false;
  double steering_rad = 0.0;  // of the chosen reverse arc
  double length_m = 0.0;
  double score = 0.0;  // the forward progress (or clearance without a field) from its end pose
  int arc = -1;        // index among the reverse arcs, 0 = lowest steering
  int clear_arcs = 0;  // reverse arcs that stayed clear
};

// One plan from the car pose `ego` in the frame of `field`. `freespace` may be null or not valid.
RecoveryPlan plan_recovery(const DistanceField& field, const FreespaceField* freespace,
                           const Pose2& ego, const VehicleGeometry& geometry,
                           const RecoveryParams& p);

enum class RecoveryPhase { Idle, Stopping, Reversing, Exhausted };

struct RecoveryInput {
  double now_s = 0.0;
  double tick_s = 0.02;
  BlockReason reason = BlockReason::None;  // from the DrivabilityMonitor this tick
  double reason_held_s = 0.0;              // how long that reason has held continuously
  double wheel_speed_mps = 0.0;            // signed
  double published_speed_mps = 0.0;        // last published, before this tick
  bool gate_fresh = false;
  double gated_speed_mps = 0.0;
  const DistanceField* field = nullptr;
  const FreespaceField* freespace = nullptr;
  Pose2 ego;  // the car now, in the frame of `field`
};

struct RecoveryOutput {
  bool active = false;  // RECOVERING: publish `cmd`
  Command cmd;          // speed 0 while stopping, negative while reversing
  RecoveryPhase phase = RecoveryPhase::Idle;
  bool exhausted = false;  // report BLOCKED with reason 5
  int attempts = 0;
};

class Recovery {
 public:
  Recovery(const RecoveryParams& p, const VehicleGeometry& g, double speed_floor_mps);

  // One call per tick the planner is not in FAULT / NO_ODOM / NO_SCAN.
  RecoveryOutput update(const RecoveryInput& in);
  // Forget everything (a state that is not a normal solving tick: the car is not being driven).
  void reset();

  RecoveryPhase phase() const { return phase_; }
  int attempts() const { return attempts_; }
  const RecoveryPlan& plan() const { return plan_; }
  double reverse_speed_mps() const { return reverse_speed_; }

 private:
  bool start_due(const RecoveryInput& in) const;
  void begin(const RecoveryInput& in);
  void abort(const RecoveryInput& in);
  void finish();
  RecoveryOutput out(const RecoveryInput& in, bool active, double speed) const;

  RecoveryParams p_;
  VehicleGeometry g_;
  double reverse_speed_;
  RecoveryPhase phase_ = RecoveryPhase::Idle;
  int attempts_ = 0;
  RecoveryPlan plan_;
  double started_s_ = 0.0;
  double reversed_m_ = 0.0;   // signed wheel odometry integral while reversing (positive = back)
  double forward_m_ = 0.0;    // forward distance since the last attempt began
  double next_plan_s_ = 0.0;  // earliest time of the next plan after a failed plan or an abort
  bool have_next_plan_ = false;
  bool gate_bad_ = false;
  double gate_bad_since_s_ = 0.0;
  double exhausted_since_s_ = 0.0;
  bool release_ = false;
  double release_since_s_ = 0.0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_RECOVERY_HPP_
