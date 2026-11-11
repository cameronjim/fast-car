// MPPI solver. Samples control sequences around a warm started nominal, rolls each through the
// dynamics model, scores it and returns the weighted average. No allocation after construction.
#ifndef RACER_PLANNER_MPPI_HPP_
#define RACER_PLANNER_MPPI_HPP_

#include <cstdint>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/footprint.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/rng.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct MppiParams {
  int horizon_steps = 30;
  int samples = 768;
  double sigma_speed_mps = 0.25;
  double sigma_steering_rad = 0.25;
  double beta = 0.7;
  double lambda = 1.0;
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
  double w_du = 0.5;
  double w_term = 50.0;
};

struct CostBreakdown {
  double progress = 0.0;  // cost term, negative when moving forward
  double collision = 0.0;
  double clearance = 0.0;
  double boundary = 0.0;
  double lateral = 0.0;
  double smoothness = 0.0;
  double terminal = 0.0;
  double total = 0.0;
  double s_end_m = 0.0;
  double progress_m = 0.0;  // s_end - s_start
  double min_clearance_m = 0.0;
  double v_end_mps = 0.0;
  bool collided = false;
};

struct SolveInput {
  State x0;          // car state in the planning frame, after the delay roll-forward
  Control prev_cmd;  // last published command, for the smoothness term
  double v_cap_mps = 0.0;
  const TrackProjector* track = nullptr;
  const DistanceField* field = nullptr;
};

struct SolveOutput {
  Control cmd;  // first command of the weighted average, before rate limiting
  bool all_collide = false;
  double best_cost = 0.0;
  double plan_progress_m = 0.0;
  double plan_min_clearance_m = 0.0;
  double collision_fraction = 0.0;
  int samples = 0;
};

constexpr int kMinSamples = 64;
constexpr int kMaxSamples = 4096;

class MppiSolver {
 public:
  MppiSolver(const MppiParams& params, const DynamicsModel& model, const VehicleGeometry& geometry,
             int max_samples);

  // Sample count for the following solves, clamped to [kMinSamples, capacity].
  void set_samples(int n);
  int samples() const { return samples_; }
  int capacity() const { return capacity_; }

  void reset();  // forget the warm start

  // Needs a valid track (build() true) and a field. Never allocates.
  SolveOutput solve(const SolveInput& in);

  // Weighted mean rollout of the last solve, horizon_steps + 1 poses in the planning frame
  // (the first is the initial state).
  const std::vector<Pose2>& plan() const { return plan_; }
  const std::vector<Control>& nominal() const { return u_; }

  // Cost of one control sequence of horizon_steps controls (for tests and diagnostics).
  double evaluate_sequence(const SolveInput& in, const Control* seq, CostBreakdown* out) const;

 private:
  struct Ctx {
    double s_start = 0.0;
    int hint0 = 0;
  };
  double evaluate(const SolveInput& in, const Ctx& ctx, const Control* u, CostBreakdown* bd,
                  Pose2* poses) const;

  MppiParams p_;
  DynamicsModel model_;
  Footprint footprint_;
  double half_width_m_;
  double stop_decel_mps2_;
  double steer_min_, steer_max_;
  int horizon_;
  int capacity_;
  int samples_;
  bool first_ = true;
  GaussianRng rng_;
  std::vector<Control> u_;          // nominal sequence
  std::vector<Control> v_;          // capacity * horizon sampled sequences
  std::vector<double> cost_;        // capacity
  std::vector<unsigned char> hit_;  // capacity, 1 when the sample collided
  std::vector<Control> u_new_;
  std::vector<Pose2> plan_;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_MPPI_HPP_
