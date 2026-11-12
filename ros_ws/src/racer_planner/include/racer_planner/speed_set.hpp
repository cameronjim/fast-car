// The speeds the planner may command: 0, or at least the speed floor (the drivetrain ignores
// anything in between), never above the effective cap.
#ifndef RACER_PLANNER_SPEED_SET_HPP_
#define RACER_PLANNER_SPEED_SET_HPP_

#include <algorithm>

namespace racer_planner {

// Snaps a forward speed to the allowed set. Below half the floor goes to 0, otherwise below the
// floor goes up to the floor. A cap under the floor leaves only 0. NaN and negative give 0. The
// result is 0 or within [floor, cap].
inline double snap_speed(double v, double floor_mps, double cap_mps) {
  const double floor_m = std::max(floor_mps, 0.0);
  const double cap = std::max(cap_mps, 0.0);
  if (!(v > 0.0)) return 0.0;
  if (cap < floor_m || cap <= 0.0) return 0.0;
  v = std::min(v, cap);
  if (v < 0.5 * floor_m) return 0.0;
  if (v < floor_m) return floor_m;
  return v;
}

}  // namespace racer_planner

#endif  // RACER_PLANNER_SPEED_SET_HPP_
