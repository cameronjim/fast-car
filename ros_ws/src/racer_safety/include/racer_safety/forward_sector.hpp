// In-path obstacle distance for racer_safety's obstacle gate (TTC brake + distance floor),
// ahead of the car for a forward request and, since 2026-10-06 night, behind it for a reverse
// request ("REAR CORRIDOR" below; the file keeps its name).
//
// ROS-free, like gate_logic.hpp, so it is gtest-unit-testable with no ROS install and sits
// under the same 100% branch-coverage gate (.github/scripts/racer_safety_coverage.sh).
// safety_node.cpp keeps the last sensor_msgs/LaserScan and calls `min_path_distance_m` with it
// once per GATE CYCLE, with that cycle's requested steering (see "WHICH STEERING" below).
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
// or far to the side of the car can ever enter the corridor test. At 0.6 rad (schema 0.9.0)
// the sector, not the corridor, was the binding limit for x below 0.205 / tan(0.6), about
// 0.30 m from the head; schema 0.9.1 widened it to 1.2 rad, which moves that edge in to
// 0.205 / tan(1.2), about 0.08 m. Close in, a front-corner return wider than the sector is
// still not seen; that is a property of the committed sector value, stated here rather than
// hidden.
//
// ARC CORRIDOR (2026-10-06, late floor test). The straight corridor above ignored the steering,
// so a car stopped against a wall and steering hard away from it still had the wall in its
// corridor and never released (bag 2026-10-06T22-12-40_car_teleop: three correct brakes, each
// latched for 20 to 37 s, the car at full lock 85 percent of the run). The corridor now follows
// the REQUESTED steering arc (min_path_distance_m below):
//   * delta = the request's steering angle clamped to +/- steering.max_angle_rad. If
//     |delta| < kStraightSteeringEpsilonRad the path is straight and the straight corridor
//     above is used unchanged (centred on the car's centreline, which with the head on the
//     centreline is the head's +x axis, exactly as before).
//   * Otherwise the rear axle follows a circle of signed radius R = L / tan(delta), L =
//     chassis.wheelbase_m, centred at (0, R) in the rear-axle frame (left turn R > 0).
//   * A return at head-relative (x, y) moves to the rear-axle frame with the LiDAR mount
//     (sensors.lidar.mount_x_m, mount_y_m): xr = x + mount_x, yr = y + mount_y. Its distance
//     from the turn centre is rho = hypot(xr, yr - R). It is in the swept band if
//     |rho - |R|| <= corridor_half_width_m (the same half width as the straight corridor).
//   * Its arc angle from the car's position, measured around the turn centre in the direction
//     of travel, is phi = atan2(xr, |R| - sign(R) yr). It counts only if 0 < phi < pi/2:
//     nothing behind the rear axle and nothing beyond a quarter turn.
//   * The distance handed to the gate is the ARC LENGTH the rear axle travels until the head
//     reaches the return's angle, |R| (phi - phi_head), phi_head being the head's own arc
//     angle, and it must be > 0 (the return is ahead of the head along the arc). REFERENCE:
//     like the straight corridor's x, and the clearance floor, this is measured from the LiDAR
//     HEAD, not from the bumper; as R grows it tends to the straight corridor's x. It is never
//     the straight-line range.
// The outer sector bound applies before either corridor, unchanged.
//
// WHICH STEERING. The path is the one the car is being ASKED to drive, the bounds-clamped
// /drive_raw steering, not the gate's rate-limited or held output. That is what lets a latched
// car release by steering away from what it braked for, even while the steering hold has
// frozen the output (gate_logic.hpp). safety_node therefore keeps the last /scan and recomputes
// this distance on EVERY gate cycle with the current request's steering, instead of reducing
// each scan once in the scan callback. The output steering lags the request by the steering
// rate limiter (steering.max_rate_rad_per_s, about 0.13 s from centre to full lock); the speed
// ramps up from zero at the same time, so the car is still slow while the wheels catch up.
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

// The swept-path model the arc corridor needs besides the scan (SI). safety_node fills it from
// the generated vehicle_params binding (CLAUDE.md invariant 2); nothing here holds a vehicle
// dimension.
struct PathGeometry {
  double wheelbase_m = 0.0;             // chassis.wheelbase_m
  double max_steering_angle_rad = 0.0;  // steering.max_angle_rad (the request is clamped to +/-)
  double lidar_mount_x_m = 0.0;         // sensors.lidar.mount_x_m: head ahead of the rear axle
  double lidar_mount_y_m = 0.0;         // sensors.lidar.mount_y_m: head left of the centreline
};

// Below this |clamped steering| the path is treated as straight (a numerical guard, not a
// vehicle constant: at 1e-3 rad and a 0.33 m wheelbase R is 330 m, and the arc is within 2 mm
// of the straight line over the first metre).
inline constexpr double kStraightSteeringEpsilonRad = 1e-3;

// Distance to the nearest usable return (see is_usable_return) that is in the path the
// requested steering sweeps: vehicle bearing within [-half_angle_rad, +half_angle_rad] (the
// outer bound), then the straight corridor (|delta| < kStraightSteeringEpsilonRad: x > 0,
// |y + mount_y| <= corridor_half_width_m, distance x) or the arc corridor (see "ARC CORRIDOR"
// above, distance = arc length from the head). Returns +infinity if there is none ("nothing in
// the path this scan", which the gate treats as clear, not as garbage).
//
// Fails CONSERVATIVE on garbage input: if angle_min/angle_increment are non-finite,
// angle_increment is not positive, laser_yaw_rad/half_angle_rad are non-finite or
// half_angle_rad is not positive, corridor_half_width_m is non-finite or not positive, the
// wheelbase is non-finite or not positive, the max steering angle is non-finite, negative or
// not below pi/2, a mount offset is non-finite, or the requested steering is non-finite, the
// in-path test cannot be trusted, so the result is the minimum slant range r over EVERY usable
// return (the pre-2026-10-06 whole-scan behaviour). That sees every return either corridor
// could have counted, never fewer.
double min_path_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                           double laser_yaw_rad, double half_angle_rad,
                           double corridor_half_width_m, const PathGeometry& path,
                           double requested_steering_rad);

// REAR CORRIDOR (2026-10-06 night floor finding). In both lap directions the car ended nose-in
// to a corner tighter than its turning circle (full lock 0.4189 rad on the 0.3302 m wheelbase):
// the arc corridor braked correctly, then no steering gave a clear forward arc and the latch
// held forever. gap_follow_node can now back out (its reverse escape), so safety_node needs a
// rear check: until now nothing judged a reverse request at all. min_rear_path_distance_m is
// the same in-path test mirrored behind the car, for the arc a REVERSE request sweeps:
//   * Mirror the rear-axle frame front to back: a return at head-relative (x, y) becomes
//     xm = -(x + mount_x) (behind the rear axle is positive) and ym = y + mount_y. Backing up
//     with steering delta, the rear axle follows the same circle of signed radius
//     R = L / tan(delta) about (0, R) as driving forward with delta, only in the other
//     direction; in the mirrored frame that is exactly the forward case. So the forward
//     formulas apply unchanged to (xm, ym).
//   * Straight (|delta| < kStraightSteeringEpsilonRad): in the path if |ym| <=
//     corridor_half_width_m, and the distance is xm - rear_overhang_m, which must be > 0.
//   * Arc: in the swept band if |hypot(xm, ym - R) - |R|| <= corridor_half_width_m, arc angle
//     phi = atan2(xm, |R| - sign(R) ym) in (0, pi/2), and the distance is the arc length the
//     rear axle travels until the rear bumper line's centre reaches the return's angle,
//     |R| (phi - atan2(rear_overhang_m, |R|)), which must be > 0.
//   * REFERENCE: the REAR BUMPER LINE, rear_overhang_m (chassis.rear_overhang_m, schema
//     0.10.0) behind the rear axle. The forward corridor is measured from the LiDAR head (about
//     0.15 m behind the front bumper on this car), the rear one from the bumper itself: the
//     head sits far forward, so measuring the rear from it would put 0.4 m of car inside the
//     distance. Returns inside the car's own footprint behind the head (the body, cables) have
//     a distance <= 0 and never count.
//   * Outer bound: the same half angle as the forward sector, centred on the car's -x axis:
//     |wrap(vehicle bearing - pi)| <= half_angle_rad, as seen from the head.
// Same invalid-return policy, same +infinity for "nothing in the path", and the same
// conservative fallback (minimum slant range over every usable return) on garbage input, which
// here also covers a non-finite or negative rear_overhang_m.
double min_rear_path_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                                double laser_yaw_rad, double half_angle_rad,
                                double corridor_half_width_m, const PathGeometry& path,
                                double rear_overhang_m, double requested_steering_rad);

// The straight corridor on its own: min_path_distance_m with a straight request and the head
// on the centreline (mount_y 0). The arc corridor reduces to this as the steering goes to
// zero; kept as the named reference the L1 suite pins the straight case against.
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
