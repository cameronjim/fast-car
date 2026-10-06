// Shared, ROS-free LaserScan helpers for the reactive controllers (GitHub issue 26).
//
// ROS-free on purpose (claude-docs/10-conventions.md: decision logic separated from node
// plumbing) so gtest covers it with no ROS install. gap_follow_node and wall_follow_node copy
// the fields of a sensor_msgs/LaserScan into a ScanInput and hand it to the cores in
// gap_follow.hpp / wall_follow.hpp.
//
// Ray ordering and frames: sensor_msgs/LaserScan semantics. Ray i is at bearing
// angle_min + i * angle_increment in the LASER frame, counter-clockwise positive (REP-103: x
// forward, y left), so for a forward-facing LiDAR low indices are on the RIGHT and high
// indices on the LEFT. racer_gym_bridge publishes exactly this (angle_min = -fov/2, ascending
// increment; f1tenth_gym's ScanSimulator2D casts beam k at pose_theta - fov/2 + k * fov/(n-1),
// i.e. counter-clockwise from the right), and so does the RPLIDAR ROS driver.
//
// Two frames appear in this API:
//   * LASER bearing: what angle_min / angle_increment describe.
//   * VEHICLE bearing: REP-103 base_link, 0 = straight ahead, left positive.
// vehicle_bearing = laser_bearing + laser_yaw_offset_rad, where laser_yaw_offset_rad is the
// LiDAR's mounting yaw in base_link (0 for a forward-facing LiDAR, pi for one mounted facing
// backwards). Nothing here assumes the scan is centred on forward or that ray num_rays/2 is
// straight ahead (the old Python code did; that is wrong for a 360 degree RPLIDAR C1 with
// angle_min = -pi, and for any off-centre scan).
#ifndef RACER_CONTROL_LASER_SCAN_HPP_
#define RACER_CONTROL_LASER_SCAN_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace racer_control {

// The subset of sensor_msgs/LaserScan the reactive cores need, as a plain struct.
struct ScanInput {
  std::vector<float> ranges;
  double angle_min = 0.0;        // rad, laser frame
  double angle_increment = 0.0;  // rad per ray, must be > 0 (see is_usable)
  double range_min = 0.0;        // m
  double range_max = 0.0;        // m
};

// Wraps an angle into [-pi, pi).
double wrap_angle(double angle_rad);

// True when the scan has at least one ray, a finite positive angle_increment, and finite
// angle_min / range_min / range_max with range_max > range_min. A scan that fails this is
// rejected outright by the cores (no command is produced from it).
bool is_usable(const ScanInput& scan);

// True when the rays cover a full circle (n * angle_increment within half an increment of
// 2 pi), in which case bearing lookups wrap around the seam between the last and first ray.
bool is_full_circle(const ScanInput& scan);

// Laser-frame bearing of ray `index` (angle_min + index * angle_increment, NOT wrapped).
double laser_bearing_of_index(const ScanInput& scan, std::size_t index);

// Nearest ray to a LASER-frame bearing. Bearings are compared modulo 2 pi, so -pi and pi are
// the same direction. Returns nullopt when the bearing is more than half an increment
// outside the scan's coverage (a partial scan cannot see behind itself); a full-circle scan
// always returns a ray.
std::optional<std::size_t> index_for_laser_bearing(const ScanInput& scan, double laser_bearing_rad);

// Same, for a VEHICLE-frame bearing (laser_bearing = vehicle_bearing - laser_yaw_offset_rad).
std::optional<std::size_t> index_for_vehicle_bearing(const ScanInput& scan,
                                                     double vehicle_bearing_rad,
                                                     double laser_yaw_offset_rad);

// Invalid-return policy (GitHub issue 26 item b). Writes one sanitised range per ray into
// `out` (resized to scan.ranges.size(); reuses its capacity, so no heap allocation once the
// buffer has grown to the scan size):
//
//   * finite r with range_min <= r <= range_max and r > 0: kept, then clipped to
//     clip_max_range_m.
//   * +inf, or finite r > range_max: FREE SPACE, written as clip_max_range_m. The RPLIDAR
//     driver reports "no return" as +inf; a return beyond the rated range is likewise
//     evidence of nothing closer.
//   * NaN, -inf, r <= 0, or r < range_min: INVALID (the old Hokuyo path reported invalid as
//     0, and a zero is not a measured obstacle at the lens). Filled from the nearest valid
//     ray by index distance; on a tie the smaller of the two neighbours (conservative). The
//     fill does not wrap around the seam of a 360 degree scan.
//
// Returns the number of rays that were valid (kept or free space). When it is 0 there is no
// valid neighbour to fill from, every entry of `out` is 0.0, and callers must treat the scan
// as unusable rather than drive on it. No NaN or inf is ever written to `out`.
std::size_t sanitize_ranges(const ScanInput& scan, double clip_max_range_m,
                            std::vector<double>& out);

// LiDAR mounting yaw, resolved against the generated vehicle_params binding (CLAUDE.md
// invariant 2: sensors.lidar.mount_yaw_rad is the one source of truth once it is measured).
// `from_vehicle_params` is the node's laser_yaw_from_vehicle_params parameter (default true):
//   * true, binding unset (null): the node's laser_yaw_offset_rad parameter is used, so a
//     rear-facing bench mount can be driven before the measurement lands.
//   * true, binding set: the binding wins. The parameter must be left at its default of 0 or
//     equal the binding (within 1e-9 rad); anything else returns nullopt and the node refuses
//     to start rather than silently picking one of two disagreeing values.
//   * false: the binding is ignored and the parameter is used as given. This is ONLY for
//     scans that are not the real car's LiDAR: the simulator (racer_gym_bridge publishes
//     /scan aligned to the vehicle, yaw 0) and synthetic-scan tests. The real car always runs
//     with true.
// A non-finite parameter returns nullopt in every mode.
std::optional<double> resolve_laser_yaw_offset(std::optional<double> binding_rad,
                                               double parameter_rad, bool from_vehicle_params);

}  // namespace racer_control

#endif  // RACER_CONTROL_LASER_SCAN_HPP_
