// Forward-sector minimum range for racer_safety's obstacle gate (TTC brake + distance floor).
//
// ROS-free, like gate_logic.hpp, so it is gtest-unit-testable with no ROS install and sits
// under the same 100% branch-coverage gate (.github/scripts/racer_safety_coverage.sh).
// safety_node.cpp copies a sensor_msgs/LaserScan's geometry into `ScanGeometry` and calls
// `min_forward_range_m` once per scan.
//
// WHY A FORWARD SECTOR (2026-10-06). safety_node used to take the minimum valid range over the
// WHOLE scan. On the car the RPLIDAR C1 sees 360 degrees, so the car's own mount, the person
// holding the kill switch, and a wall beside or behind the car all fed the TTC gate as if they
// were straight ahead. The TTC gate and the distance floor protect FORWARD motion, so they now
// look only at returns whose VEHICLE bearing is within
// +/- limits.ttc_forward_sector_half_angle_rad of the car's +x axis.
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

// The parts of a sensor_msgs/LaserScan the sector computation needs (SI: rad, m).
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

// Nearest usable return (see is_usable_return) whose VEHICLE bearing lies in
// [-half_angle_rad, +half_angle_rad], or +infinity if there is none ("nothing in the sector
// this scan", which the gate treats as clear, not as garbage).
//
// Fails CONSERVATIVE on garbage geometry: if angle_min/angle_increment are non-finite,
// angle_increment is not positive, or laser_yaw_rad/half_angle_rad are non-finite or
// half_angle_rad is not positive, the bearings cannot be trusted, so the minimum is taken over
// EVERY usable return (the pre-2026-10-06 whole-scan behaviour). That can only see more
// obstacles than the sector would, never fewer.
double min_forward_range_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                           double laser_yaw_rad, double half_angle_rad);

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
