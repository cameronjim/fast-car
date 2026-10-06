// In-path obstacle distance for racer_safety's obstacle gate (TTC brake + distance floor).
//
// ROS-free, like gate_logic.hpp, so it is gtest-unit-testable with no ROS install and sits
// under the same 100% branch-coverage gate (.github/scripts/racer_safety_coverage.sh).
// safety_node.cpp copies a sensor_msgs/LaserScan's geometry into `ScanGeometry` and calls
// `min_corridor_distance_m` once per scan.
//
// WHY A FORWARD SECTOR (2026-10-06). safety_node used to take the minimum valid range over the
// WHOLE scan. On the car the RPLIDAR C1 sees 360 degrees, so the car's own mount, the person
// holding the kill switch, and a wall beside or behind the car all fed the TTC gate as if they
// were straight ahead. The TTC gate and the distance floor protect FORWARD motion, so they
// first looked only at returns whose VEHICLE bearing is within
// +/- limits.ttc_forward_sector_half_angle_rad of the car's +x axis.
//
// CORRIDOR, NOT WEDGE (2026-10-06, floor test). The wedge alone was still far too aggressive
// on a tight track of backpacks about 1 m wide with gap_follow_node at 0.8 to 1.0 m/s: a bag
// 0.3 m to the side of the car's path, which the car passes cleanly, sat inside the +/- 0.6
// rad wedge and tripped the TTC brake and the clearance floor exactly like a bag straight
// ahead. So the PRIMARY filter is now a corridor the width of the car. Each usable return at
// vehicle bearing theta and range r becomes
//   x = r cos(theta)   (ahead, along the car's +x axis)
//   y = r sin(theta)   (left positive)
// and counts as "in the path" only if x > 0 and |y| <= corridor_half_width_m, where
//   corridor_half_width_m = chassis.width_m / 2 + limits.obstacle_corridor_margin_m
// (corridor_half_width_m() below; safety_node feeds it from the generated binding). The
// distance handed to the gate is x, the along-track distance the car can still travel before
// reaching the return, not the slant range r: TTC is distance along the direction of travel
// divided by forward speed, and r overstates it for an off-axis return.
//
// The sector half angle stays, as an OUTER bound only: a return outside +/-
// half_angle_rad is never considered, even if a huge margin is configured, so nothing behind
// or far to the side of the car can ever enter the corridor test. With the committed values
// (0.6 rad, half width 0.155 + 0.05 = 0.205 m) the sector, not the corridor, is the binding
// limit for x below 0.205 / tan(0.6), about 0.30 m from the head: a return at the edge of the
// corridor that close is at a bearing wider than 0.6 rad and is not seen. That is a property
// of the committed sector value, stated here rather than hidden.
//
// The corridor is straight (along +x). It does not bend with the steering angle, so on a
// curve it is a short-horizon approximation of the swept path. The LiDAR sits on the
// centreline (sensors.lidar.mount_y_m 0.0), so the corridor is centred on the head; x is
// measured from the head, the same origin the clearance floor has always used.
//
// LASER BEARING vs VEHICLE BEARING. A LaserScan's ray i has laser bearing
// angle_min + i * angle_increment in the laser frame. The head is mounted rotated relative to
// base_link by sensors.lidar.mount_yaw_rad (pi on this car: a box straight ahead of the nose
// read at laser bearing 177 deg on the 2026-10-06 bench run), so
//   vehicle bearing = wrap(laser bearing + mount yaw)
// with LEFT positive (REP-103) and no mirroring (verified on the same bench run).
//
// racer_control has the same bearing arithmetic (laser_scan.hpp), but racer_safety does not
// depend on racer_control: claude-docs/02-repo-layout.md lists no such dependency, and layer 3
// should not share a code path with the controllers it gates. This is a small, separately
// tested copy.
#ifndef RACER_SAFETY_FORWARD_SECTOR_HPP_
#define RACER_SAFETY_FORWARD_SECTOR_HPP_

#include <optional>
#include <vector>

namespace racer_safety {

// The parts of a sensor_msgs/LaserScan the in-path computation needs (SI: rad, m).
struct ScanGeometry {
  double angle_min_rad = 0.0;
  double angle_increment_rad = 0.0;
  double range_min_m = 0.0;
  double range_max_m = 0.0;
};

// Wraps an angle to [-pi, pi). A non-finite input comes back non-finite.
double wrap_angle_rad(double angle_rad);

// INVALID-RETURN POLICY. A return counts as an obstacle distance only if it is
//   * finite (NaN and +/-inf are drivers' "no return" encodings, not distances),
//   * strictly positive (0.0 is the RPLIDAR driver's "no return"; a negative is garbage),
//   * not below the message's own range_min (closer than the head can measure, so not a
//     trustworthy distance), and
//   * not above the message's own range_max.
// A bound that is itself non-finite is not applied. Anything else is ignored, never turned
// into an obstacle. KNOWN LIMITATION, stated rather than hidden: an object closer to the head
// than range_min produces no valid return at all, so it reads as "no obstacle". On the C1
// range_min is 0.05 m and the head sits behind the bumper, so such an object is already
// touching the car; the distance floor (limits.min_forward_clearance_m) exists to stop the
// car well before that.
bool is_usable_return(float range_m, const ScanGeometry& geometry);

// Half width of the in-path corridor: chassis_width_m / 2 + margin_m (SI, m). safety_node
// passes chassis.width_m and limits.obstacle_corridor_margin_m from the generated
// vehicle_params binding (CLAUDE.md invariant 2); nothing here holds a vehicle dimension.
double corridor_half_width_m(double chassis_width_m, double margin_m);

// Along-track distance x to the nearest usable return (see is_usable_return) that is in the
// path: vehicle bearing within [-half_angle_rad, +half_angle_rad] (the outer bound), x > 0,
// and |y| <= corridor_half_width_m (the primary filter). See "CORRIDOR, NOT WEDGE" above.
// Returns +infinity if there is none ("nothing in the path this scan", which the gate treats
// as clear, not as garbage).
//
// Fails CONSERVATIVE on garbage geometry: if angle_min/angle_increment are non-finite,
// angle_increment is not positive, laser_yaw_rad/half_angle_rad are non-finite or
// half_angle_rad is not positive, or corridor_half_width_m is non-finite or not positive, the
// in-path test cannot be trusted, so the result is the minimum slant range r over EVERY usable
// return (the pre-2026-10-06 whole-scan behaviour). That sees every return the corridor could
// have counted, never fewer. (Without trustworthy bearings x cannot be computed, so r is the
// only distance available there.)
double min_corridor_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                               double laser_yaw_rad, double half_angle_rad,
                               double corridor_half_width_m);

// LiDAR mounting yaw for safety_node, resolved against the generated vehicle_params binding
// (CLAUDE.md invariant 2: sensors.lidar.mount_yaw_rad is the one source of truth). Same rules
// as racer_control's resolve_laser_yaw_offset (laser_scan.hpp), so both nodes agree:
//   * from_vehicle_params true (default, the real car), binding unset: parameter_rad is used.
//   * true, binding set: the binding wins. parameter_rad must be 0 or equal the binding
//     (within 1e-9 rad); anything else returns nullopt and the node refuses to start.
//   * false: the binding is ignored and parameter_rad is used as given. ONLY for scans that
//     are not the real car's LiDAR: the simulator (racer_gym_bridge's /scan is aligned to the
//     vehicle, yaw 0) and synthetic-scan tests.
// A non-finite parameter, or a non-finite binding that would be used, returns nullopt.
std::optional<double> resolve_laser_yaw_rad(std::optional<double> binding_rad, double parameter_rad,
                                            bool from_vehicle_params);

}  // namespace racer_safety

#endif  // RACER_SAFETY_FORWARD_SECTOR_HPP_
