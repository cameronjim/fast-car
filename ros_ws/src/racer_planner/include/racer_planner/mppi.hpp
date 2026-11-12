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
  double corner_radius_m = Footprint::kDefaultCornerRadiusM;
  double plan_window_s = 0.5;  // window of SolveOutput::plan_speed_mps
  double tick_s = 0.02;        // control tick: the warm start advances the nominal by tick_s / dt
  // Sampled and output speeds are 0 or at least this (0 turns the rule off). The Planner fills it
  // from the dynamics bundle; the solver takes it as given.
  double speed_floor_mps = 0.0;
};

struct CostBreakdown {
  double progress = 0.0;  // cost term, negative when moving forward
  double collision = 0.0;
  double clearance = 0.0;
  double boundary = 0.0;
  double lateral = 0.0;
  double smoothness = 0.0;  // speed command differences
  double steer_rate = 0.0;
  double terminal = 0.0;
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
};

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
  };
  Ctx make_ctx(const SolveInput& in) const;
  double evaluate(const SolveInput& in, const Ctx& ctx, const Control* u, CostBreakdown* bd,
                  Pose2* poses) const;
  void build_samples(const SolveInput& in, double v_cap);

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
  std::vector<Control> u_new_;
  std::vector<Pose2> plan_;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_MPPI_HPP_
