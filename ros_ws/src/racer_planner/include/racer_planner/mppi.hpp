// MPPI solver. Samples control sequences around a warm started nominal, rolls each through the
// dynamics model, scores it and returns the weighted average. No allocation after construction,
// and no random number is drawn in solve: the noise is a table generated once from the seed.
#ifndef RACER_PLANNER_MPPI_HPP_
#define RACER_PLANNER_MPPI_HPP_

#include <cstdint>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/footprint.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/gate_predictor.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/rng.hpp"
#include "racer_planner/speed_set.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct MppiParams {
  int horizon_steps = 30;
  int samples = 512;
  double sigma_speed_mps = 0.15;
  double sigma_steering_rad = 0.12;
  double beta = 0.8;
  double lambda = 0.15;
  double cost_scale_floor = 0.2;  // floor of (J_median - J_min) in the weight temperature
  std::uint64_t seed = 1;
  // cost weights and margins (see claude-docs/13 section 5.C and the commit that tuned them)
  double w_progress = 10.0;
  double c_collision = 1e4;
  double d_hard_m = 0.03;
  double w_clear = 200.0;
  double d_soft_base_m = 0.15;
  double t_headway_s = 0.25;
  double w_bound = 200.0;
  double m_bound_m = 0.05;
  double w_lat = 5.0;
  double a_lat_max_mps2 = 2.0;
  double w_du = 0.5;          // speed command differences
  double w_steer_rate = 0.1;  // steering command rate squared, first step against prev_cmd
  double w_term = 50.0;
  // The plan moves only if the best moving sample beats the best standing one by this much cost.
  double move_margin = 0.3;
  double corner_radius_m = Footprint::kDefaultCornerRadiusM;
  double plan_window_s = 0.5;  // window of SolveOutput::plan_speed_mps
  double tick_s = 0.02;        // control tick: the warm start advances the nominal by tick_s / dt
  // Sampled and output speeds are 0 or at least this (0 turns the rule off). The Planner sets it
  // to the bundle's speed dead zone plus speed_floor_margin_mps; the solver takes it as given.
  double speed_floor_mps = 0.0;
  double speed_floor_margin_mps = 0.03;  // read by the Planner only
  // Gate aware cost (gate_predictor.hpp): over the first kGateCostSteps steps of a rollout, with
  // d_k the table distance along the commanded steering less the path already travelled and
  // ttc_k = d_k / max(v_cmd, v, 1e-3), w_gate * (max(0, 1 - ttc_k / (gate_ttc_margin *
  // ttc_warning_s))^2 + max(0, 1 - d_k / (gate_clear_margin * min_forward_clearance_m))^2).
  // 0 turns the term off. Only a forward request is judged, like the gate. The defaults come
  // from the closed loop sweeps in test_gate_closed_loop.cpp (stadium ovals, both directions, caps
  // 1.2 and 1.5 m/s, planner margins just under the gate's): the starting values 300 / 1.3 made
  // the planner 70 percent slower, 3 / 1.1 / 1.5 keeps the gate from engaging at 3 to 9 percent
  // of lap time. A planner with much thinner own margins needs a larger weight (15 and more) and
  // pays more lap time for it.
  double w_gate = 3.0;
  double gate_ttc_margin = 1.1;
  double gate_clear_margin = 1.5;
  // Lap profile (lap_profile.hpp), used only while SolveInput::lap is active: weight of the
  // trust region on speed, w_trust * max(0, v_k - v_bound(s_k))^2 * dt, every step. There is no
  // setting that turns it off while a profile is active. 2500 keeps the driven speed within 0.04
  // m/s of a bound with 0.8 m ramps in test_gate_closed_loop.cpp (the starting value 400 left it
  // 0.06 over); the quadratic hinge trades against the cost-to-go, so no weight makes a literal
  // step in the bound exact for a car with a speed lag.
  double w_trust = 2500.0;
  // Brief 53 switches (brief 44 section 1). Every default reproduces the behaviour before them.
  // lap_reward_mode: kReplace (default, registered D v1) lets the profile's cost-to-go replace the
  // progress term; kKeepProgress keeps the planner's own progress term, ignores the cost-to-go and
  // keeps the speed trust region.
  enum class LapRewardMode : std::uint8_t { kReplace = 0, kKeepProgress = 1 };
  LapRewardMode lap_reward_mode = LapRewardMode::kReplace;
  // lap_station_mode: kPath (default) advances the lap station inside a rollout by the wheel
  // distance of the rollout; kTrack by the projected track progress while the track is in use
  // (the wheel distance since the last projection is added between projections), by the wheel
  // distance otherwise.
  enum class LapStationMode : std::uint8_t { kPath = 0, kTrack = 1 };
  LapStationMode lap_station_mode = LapStationMode::kPath;
  // terminal_curvature_bound (default off): while the track is in use, a terminal penalty
  // c * w_curvature_bound * max(0, v_end - v_allow(s_end))^2, added to the terminal term, where
  // v_allow(s) is the lowest over the visible stations s_j >= s of
  // sqrt(a / kappa_j + 2 * stop_decel * (s_j - s)) and a is curvature_bound_a_lat_mps2. No memory.
  bool terminal_curvature_bound = false;
  double curvature_bound_a_lat_mps2 = 2.0;
  double w_curvature_bound = 200.0;
  // Check the final sequence (the smoothed weighted mean) for collision and fall back to the best
  // evaluated collision-free sample of the chosen class when it fails (claude-docs/13 17.4.1).
  // false is the behaviour of main f5db7f1 and exists only so planner_replay_cli can measure M-C1
  // (pre-registration addendum 1 section 4) and the L1 tests can show the fault on the old path:
  // no node parameter reads it, it is never false on the car.
  bool check_final_sequence = true;
  // Marginal speed probe of the speed limiter attribution (see SpeedLimiter): the whole final
  // sequence is re-evaluated with every speed command raised by this much.
  double limiter_probe_mps = 0.1;
  double limiter_cap_band_mps = 0.05;  // "at the cap": the plan speed is within this of it
};

// What the final command sequence of a solve is (SolveOutput::final_sequence). The numeric values
// are the PlannerTiming.msg constants.
enum class FinalSequence : std::uint8_t {
  kMean = 0,        // the smoothed weighted mean, collision free and finite
  kFallback = 1,    // the mean collided or was not finite: the best clean sample of its class
  kAllCollide = 2,  // every sample collided: the least bad one with speed 0
};

// Which term limits the speed of the final sequence. The numeric values are the PlannerTiming.msg
// constants. The rule (MppiSolver::solve): re-evaluate the final sequence with every speed command
// raised by MppiParams::limiter_probe_mps (no clamp at the cap). The marginal cost of a term is its
// change under that probe. The terms that can limit speed are clearance, terminal, gate and
// lap_trust. G is the marginal reward, the fall of progress plus lap_terminal. If the sum of the
// positive marginal costs of those four terms is at least G (and above 0) the limiter is the one
// with the largest marginal cost (ties in the order clearance, terminal, gate, lap bound). If not,
// speed is held by something else: kCap when the mean speed command over the plan window reaches
// the cap (within MppiParams::limiter_cap_band_mps), else kSampleChoice (the sampled sequences or
// their average stop short of both the cap and any penalty). kNone: the plan does not move. The
// planner turns kCap into kValidity when the track confidence is what lowered the cap
// (Planner::solve_impl).
enum class SpeedLimiter : std::uint8_t {
  kNone = 0,
  kCap = 1,
  kClearance = 2,
  kTerminal = 3,
  kGate = 4,
  kLapBound = 5,
  kValidity = 6,
  kSampleChoice = 7,
};

// Order of the cost terms of the final sequence in PlannerTiming.cost_terms.
constexpr int kCostTermCount = 11;
// progress, collision, clearance, boundary, lateral, smoothness, steer_rate, terminal, gate,
// lap_trust, lap_terminal

struct CostBreakdown {
  double progress = 0.0;  // cost term, negative when moving forward
  double collision = 0.0;
  double clearance = 0.0;
  double boundary = 0.0;
  double lateral = 0.0;
  double smoothness = 0.0;  // speed command differences
  double steer_rate = 0.0;
  double terminal = 0.0;
  double gate = 0.0;          // gate aware cost, 0 without a gate table
  double lap_trust = 0.0;     // trust region on the lap profile's speed bound
  double lap_terminal = 0.0;  // cost-to-go gain, replaces `progress` while a profile is active
  double total = 0.0;
  double s_end_m = 0.0;
  double progress_m = 0.0;  // blended: c * (s_end - s_start) + (1 - c) * (Phi_start - Phi_end)
  double min_clearance_m = 0.0;
  double v_end_mps = 0.0;
  bool collided = false;
};

struct SolveInput {
  State x0;          // car state in the planning frame, after the delay roll-forward
  Control prev_cmd;  // last published command, for the smoothness terms
  double v_cap_mps = 0.0;
  const TrackProjector* track = nullptr;  // null (or confidence 0): the track is not used
  const DistanceField* field = nullptr;
  // Weight of the track prior, 0 to 1. The free-space progress field weighs (1 - confidence).
  double confidence = 1.0;
  const FreespaceField* freespace = nullptr;  // null or not valid(): no free-space part
  // In-path distances of the latest scan along each steering angle, as the safety gate will judge
  // them. Null or not valid(): no gate term.
  const GateTable* gate = nullptr;
  // Measured wheel speed, m/s: the gate's speed estimate at the first step.
  double wheel_speed_mps = 0.0;
  // Distance the car has travelled since the scan the gate table was built from, m: the table's
  // distances are from the scan pose, the rollout starts at x0 (0 when the table is fresh).
  double gate_travel_since_scan_m = 0.0;
  // Lap profile from lap-over-lap learning. Null or not active: no trust region and the progress
  // term is the one above.
  const LapProfile* lap = nullptr;
  // Wheel distance travelled between the profile's stamp and now, m. The station of x0 is
  // lap->s0_m + lap_station_now_m.
  double lap_station_now_m = 0.0;
};

struct SolveOutput {
  Control cmd;  // first command of the filtered weighted average, before rate limiting
  bool all_collide = false;
  double best_cost = 0.0;
  double plan_progress_m = 0.0;
  double plan_min_clearance_m = 0.0;
  double plan_speed_mps = 0.0;  // mean speed command of the weighted mean over its first window
  double collision_fraction = 0.0;
  int samples = 0;
  // The gate would brake the first command given the table and the wheel speed (diagnostic: the
  // gate cost is what steers away from it). False without a table.
  bool cmd_would_trip = false;
  // The final sequence outcome (FinalSequence), its cost terms and the speed limiter. final_cost
  // is the evaluation of the sequence the command comes from (the mean, or the fallback sample).
  FinalSequence final_sequence = FinalSequence::kMean;
  // True when the unchecked mean collided or was not finite although a clean sample existed
  // (M-C1). Reported whether or not MppiParams::check_final_sequence acted on it.
  bool unchecked_final_collided = false;
  CostBreakdown final_cost;
  SpeedLimiter speed_limiter = SpeedLimiter::kNone;
};

// The terms of a CostBreakdown in the order of PlannerTiming.cost_terms.
inline void cost_terms_array(const CostBreakdown& b, double out[kCostTermCount]) {
  out[0] = b.progress;
  out[1] = b.collision;
  out[2] = b.clearance;
  out[3] = b.boundary;
  out[4] = b.lateral;
  out[5] = b.smoothness;
  out[6] = b.steer_rate;
  out[7] = b.terminal;
  out[8] = b.gate;
  out[9] = b.lap_trust;
  out[10] = b.lap_terminal;
}

constexpr int kMinSamples = 64;
constexpr int kMaxSamples = 4096;
// Sample 0 is the nominal, 1 the nominal steering at speed 0, 2 to 28 the constant command arcs.
constexpr int kArcSteeringValues = 9;
constexpr int kArcSpeedValues = 3;
constexpr int kStructuredSamples = 2 + kArcSteeringValues * kArcSpeedValues;

// 3 tap smoothing (0.25, 0.5, 0.25) of the steering of a sequence, the end values repeated.
void smooth_steering_3tap(Control* seq, int n);

class MppiSolver {
 public:
  MppiSolver(const MppiParams& params, const DynamicsModel& model, const VehicleGeometry& geometry,
             int max_samples);

  // Sample count for the following solves, clamped to [kMinSamples, capacity].
  void set_samples(int n);
  int samples() const { return samples_; }
  int capacity() const { return capacity_; }

  void reset();  // forget the warm start

  // Online speed correction of the rollout model (speed_adaptation.hpp); zero by default.
  void set_speed_correction(const SpeedCorrection& c) { model_.set_speed_correction(c); }

  // Needs a distance field and either a valid track with confidence above 0 or a valid free-space
  // field (the planner decides "nothing drivable" before calling). Never allocates.
  SolveOutput solve(const SolveInput& in);

  // Weighted mean rollout of the last solve, horizon_steps + 1 poses in the planning frame
  // (the first is the initial state).
  const std::vector<Pose2>& plan() const { return plan_; }
  const std::vector<Control>& nominal() const { return u_; }
  // The filtered weighted mean of the last solve (before the warm start shift).
  const std::vector<Control>& mean_sequence() const { return u_new_; }
  // The sampled sequences of the last solve (samples() * horizon_steps controls, sample major),
  // for tests.
  const std::vector<Control>& samples_buffer() const { return v_; }
  int horizon() const { return horizon_; }

  // Cost of one control sequence of horizon_steps controls (for tests and diagnostics).
  double evaluate_sequence(const SolveInput& in, const Control* seq, CostBreakdown* out) const;

 private:
  struct Ctx {
    double s_start = 0.0;
    int hint0 = 0;
    double phi_start = 0.0;
    double c = 0.0;  // effective track weight
    bool use_track = false;
    bool use_free = false;
    bool use_gate = false;
    bool use_lap = false;
    double lap_s_now = 0.0;  // station of x0 on the profile
    double lap_q_now = 0.0;
    double gate_travel_m = 0.0;   // sanitised SolveInput::gate_travel_since_scan_m
    double gate_ttc_ref_s = 0.0;  // gate_ttc_margin * ttc_warning_s
    double gate_clear_ref_m = 0.0;
    bool lap_track_station = false;  // lap_station_mode kTrack with the track in use
    bool use_curv = false;           // terminal_curvature_bound with the track in use
    int curv_n = 0;
    double curv_s[kTrackStations] = {};   // arc length of the visible stations
    double curv_v2[kTrackStations] = {};  // a_lat / kappa at each, m^2/s^2 (large on a straight)
  };
  Ctx make_ctx(const SolveInput& in) const;
  double evaluate(const SolveInput& in, const Ctx& ctx, const Control* u, CostBreakdown* bd,
                  Pose2* poses) const;
  void build_samples(const SolveInput& in, double v_cap);
  SpeedLimiter limiter_of(const SolveInput& in, const Ctx& ctx, const CostBreakdown& base,
                          double plan_speed_mps, double v_cap);

  MppiParams p_;
  DynamicsModel model_;
  Footprint footprint_;
  double half_width_m_;
  double wheelbase_m_;
  double stop_decel_mps2_;
  double steer_min_, steer_max_;
  int horizon_;
  int capacity_;
  int samples_;
  bool first_ = true;
  std::vector<double> noise_;       // capacity * horizon * 2 standard normals, frozen
  std::vector<Control> u_;          // nominal sequence
  std::vector<Control> v_;          // capacity * horizon sampled sequences
  std::vector<double> cost_;        // capacity
  std::vector<double> scratch_;     // capacity, for the median
  std::vector<unsigned char> hit_;  // capacity, 1 when the sample collided
  std::vector<unsigned char> cls_;  // capacity, 1 when the sample moves (see solve)
  std::vector<Control> u_new_;
  std::vector<Control> probe_;  // the limiter probe sequence
  std::vector<Pose2> plan_;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_MPPI_HPP_
