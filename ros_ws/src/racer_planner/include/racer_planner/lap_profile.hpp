// The lap profile the planner consumes from lap-over-lap learning (racer_pnp_msgs/LapGuidance):
// per station speed bounds and time to finish, from the car's own clean laps. Nothing here is
// learned in the planner. The solver uses the profile as a trust region on speed along every
// rollout and as a terminal cost-to-go (Learning MPC in the style of Rosolia and Borrelli, reduced
// to station and speed).
//
// The producer sends stations s_m[0] = the car's station at header.stamp, then every 0.10 m for
// 6.0 m (61 stations). make_lap_profile validates one message against the rules below and copies
// it into a fixed capacity LapProfile, so the 50 Hz path never allocates.
#ifndef RACER_PLANNER_LAP_PROFILE_HPP_
#define RACER_PLANNER_LAP_PROFILE_HPP_

#include <cstddef>

namespace racer_planner {

constexpr std::size_t kLapProfileCapacity = 128;
constexpr std::size_t kLapProfileMinStations = 2;
// The station spacing the producer promises, and how far a step may be from it.
constexpr double kLapStationSpacingM = 0.10;
constexpr double kLapSpacingToleranceM = 1e-3;
// cost_to_go_s must be non-increasing within this (it is a time to finish).
constexpr double kLapCostToGoToleranceS = 1e-3;
// Age of the profile, now minus header.stamp, must be inside [0, kLapProfileMaxAgeS].
constexpr double kLapProfileMaxAgeS = 0.5;

struct LapProfile {
  bool active = false;
  double stamp_s = 0.0;                    // the time the station s0_m refers to
  double s0_m = 0.0;                       // station of the first entry
  double spacing_m = kLapStationSpacingM;  // measured: (s[n-1] - s[0]) / (n - 1)
  std::size_t n = 0;
  double v_bound[kLapProfileCapacity] = {};  // m/s, per station speed bound
  double q[kLapProfileCapacity] = {};        // s, per station time to finish

  // Linear interpolation at station s (metres). Before s0 the first value is used. Beyond the
  // last station v_bound holds its last value and q continues with its last slope. An inactive
  // profile gives a large bound and 0.
  double v_bound_at(double s_m) const;
  double q_at(double s_m) const;
};

enum class LapProfileStatus {
  kNone,  // all three arrays empty: no guidance profile (D v0), not an error
  kOk,    // `out` holds an active profile
  kBad,  // refused: the planner falls back to v_no_guidance_mps (LogicInputs::guidance_profile_bad)
};

// Validation, in this order, kBad on the first failure:
//   1. the three lengths are equal (all empty is kNone, checked first);
//   2. the length is between 2 and 128 inclusive;
//   3. every value of s, v, q and both times is finite;
//   4. s increases by 0.10 per step, each step within 1e-3 of 0.10;
//   5. v >= 0 everywhere;
//   6. q is non-increasing: q[i+1] <= q[i] + 1e-3 everywhere;
//   7. the age now_s - stamp_s is in [0, 0.5] seconds, both ends inclusive.
// out->active is false unless the result is kOk; out is not otherwise touched on kNone or kBad
// except for the active flag.
LapProfileStatus make_lap_profile(const float* s, const float* v, const float* q, std::size_t n_s,
                                  std::size_t n_v, std::size_t n_q, double stamp_s, double now_s,
                                  LapProfile* out);

}  // namespace racer_planner

#endif  // RACER_PLANNER_LAP_PROFILE_HPP_
