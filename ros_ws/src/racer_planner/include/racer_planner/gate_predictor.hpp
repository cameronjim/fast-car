// Predicts what racer_safety's obstacle gate will judge, so the solver can stay out of its band.
//
// safety_node brakes a forward request when the time to collision along the arc of the REQUESTED
// steering is at or under limits.ttc_brake_s, or when the in-path distance is under
// limits.min_forward_clearance_m. The in-path distance is racer_safety's min_path_distance_m
// (forward_sector.hpp "ARC CORRIDOR", "OUTER BOUNDARY", "DISTANCE FROM THE BODY"): the shortest
// arc the body's leading point sweeps before it reaches a scan return inside the corridor of the
// arc. This header holds a faithful reimplementation of that function for the forward direction
// with the body reference (the way safety_node runs it), and a table of it over steering angles.
//
// The planner is not allowed to link racer_safety (that is an owner decision), so the copy is kept
// honest by test/test_gate_divergence.cpp, which compiles racer_safety's own sources read-only and
// compares the two over 10^5 random scans. Every number here comes from the vehicle_params
// bindings through GateGeometry and GateLimits; the only literals are protocol constants of the
// gate itself (the straight steering epsilon and the quarter turn), mirrored from
// forward_sector.hpp and guarded by that test.
//
// Nothing here is a guarantee: the gate is the safety layer, this only helps the planner avoid
// asking it to brake.
#ifndef RACER_PLANNER_GATE_PREDICTOR_HPP_
#define RACER_PLANNER_GATE_PREDICTOR_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace racer_planner {

// Steering grid of the table, evenly across [steering_min, steering_max] ends included.
constexpr int kGateSteeringValues = 33;
// Rollout steps scored by the gate cost: the first 0.3 s at the solver's 0.05 s step, which covers
// the gate's stop latency.
constexpr int kGateCostSteps = 6;
constexpr std::size_t kGateMaxSteerings = 64;
constexpr std::size_t kGateDefaultMaxBeams = 4096;

// The in-path test's inputs (all from vehicle_params). Field for field what safety_node hands to
// min_path_distance_m, with distance_from_body true.
struct GateGeometry {
  double wheelbase_m = 0.0;
  double steering_min_rad = 0.0;  // table range
  double steering_max_rad = 0.0;
  // The request is clamped to +/- this before the arc is built (steering.max_angle_rad).
  double max_steering_angle_rad = 0.0;
  double lidar_mount_x_m = 0.0;
  double lidar_mount_y_m = 0.0;
  double lidar_mount_yaw_rad = 0.0;
  double body_front_x_m = 0.0;         // rear axle to front bumper line
  double corridor_half_width_m = 0.0;  // chassis.width_m / 2 + limits.obstacle_corridor_margin_m
  double sector_half_angle_rad = 0.0;  // limits.ttc_forward_sector_half_angle_rad
  double outer_corner_horizon_m = std::numeric_limits<double>::infinity();
};

// The thresholds the gate judges against (all from vehicle_params limits).
struct GateLimits {
  double ttc_brake_s = 0.0;              // trips at or under this
  double ttc_warning_s = 0.0;            // the latch's release line
  double min_forward_clearance_m = 0.0;  // trips under this
};

bool gate_geometry_is_usable(const GateGeometry& g);
bool gate_limits_are_usable(const GateLimits& l);

// One LaserScan, by reference. `ranges` is not copied.
struct GateScan {
  const float* ranges = nullptr;
  std::size_t count = 0;
  double angle_min_rad = 0.0;
  double angle_increment_rad = 0.0;
  double range_min_m = 0.0;
  double range_max_m = 0.0;
};

// min_path_distance_m for `count` steering angles at once (count at most kGateMaxSteerings), one
// pass over the scan. out[k] is the in-path distance for steering deltas[k] (clamped to
// +/- max_steering_angle_rad like the gate does), +infinity when nothing is in the path. A scan
// whose geometry the gate would not trust (non finite or non positive angle increment) gives the
// minimum usable range over the whole scan for every angle, the gate's fail closed reading.
// Does not allocate.
void gate_distances(const GateGeometry& g, const GateScan& scan, const double* deltas,
                    std::size_t count, double* out);

// The same for one steering angle.
double gate_distance(const GateGeometry& g, const GateScan& scan, double delta_rad);

// In-path distance for kGateSteeringValues steering angles of one scan.
class GateTable {
 public:
  // `max_beams` is the capacity of the copy of the scan kept for exact_distance; a longer scan
  // still gives the table, only exact_distance falls back to it. Allocates here, never in build.
  GateTable(const GateGeometry& geometry, const GateLimits& limits,
            std::size_t max_beams = kGateDefaultMaxBeams);

  // Rebuilds the table from a scan. Does not allocate. Under 0.5 ms for 720 beams.
  void build(const GateScan& scan);

  // False until the first build.
  bool valid() const { return valid_; }

  const GateGeometry& geometry() const { return geometry_; }
  const GateLimits& limits() const { return limits_; }

  double steering(int j) const;
  double at(int j) const { return d_[static_cast<std::size_t>(j)]; }

  // The smaller of the two neighbouring grid entries (conservative between grid points). The
  // steering is clamped to the table range. NaN gives the smallest entry. +infinity before the
  // first build.
  double distance(double delta_rad) const;

  // safety's own distance for this one steering angle on the stored scan, exactly (one pass over
  // the scan, about 10 us). The table is a 33 point sample of a discontinuous function: between
  // grid points it can read a few millimetres to centimetres farther than the gate will, so the
  // decision about a command that is about to be published uses this. Falls back to distance()
  // for a scan longer than the capacity.
  double exact_distance(double delta_rad) const;

 private:
  GateGeometry geometry_;
  GateLimits limits_;
  bool valid_ = false;
  std::array<double, kGateSteeringValues> deltas_{};
  std::array<double, kGateSteeringValues> d_{};
  std::vector<float> scan_;  // copy of the last scan, capacity fixed at construction
  GateScan scan_view_;
  bool exact_ok_ = false;
};

// True when the gate would brake a forward request (v_cmd_mps, delta_rad) given the measured wheel
// speed. `d_m` is the in-path distance for that steering. The speed the gate divides by is the
// request raised to the measured speed (gate_logic.hpp "SPEED FOR THE TTC" and "JUDGED SPEED FROM
// THE MEASURED WHEEL SPEED"); with a fresh measurement it is at most max(request, measured), which
// is the upper bound taken here (a larger speed only adds trips). A request that is not forward
// is never braked and an infinite or NaN distance never trips, like the gate.
bool gate_would_trip_at(double d_m, double v_cmd_mps, double v_wheel_mps, const GateLimits& limits);

// would_trip with the distance read from the table at the request's steering.
bool would_trip(const GateTable& table, double delta_rad, double v_cmd_mps, double v_wheel_mps,
                const GateLimits& limits);

// The same with the exact distance (GateTable::exact_distance). This is the one with no false
// negative against safety's own decision (test_gate_divergence.cpp); the table version has a
// small share of them between grid points.
bool would_trip_exact(const GateTable& table, double delta_rad, double v_cmd_mps,
                      double v_wheel_mps, const GateLimits& limits);

}  // namespace racer_planner

#endif  // RACER_PLANNER_GATE_PREDICTOR_HPP_
