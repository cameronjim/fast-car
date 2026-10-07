// Shared arc primitives for the low-speed manoeuvres of park_node (roadmap 2.9): the parallel
// park (park_planner.hpp plan_parallel_park) and the three-point turn (plan_three_point_turn)
// are both built from the pieces in this one file. ROS-free and gtest-covered
// (test/test_park_geometry.cpp); nothing here includes rclcpp or the generated binding, so every
// vehicle dimension arrives in ParkBody / ParkGateModel, filled by park_node from
// vehicle_params (CLAUDE.md invariant 2).
//
// FRAMES. Everything is planar, SI, REP-103 (x forward, y left, yaw counter-clockwise). A pose
// is the REAR AXLE CENTRE and the heading, the same reference racer_safety's arc corridor and
// gap_follow's swept-path clamp use. Body-local points are relative to the rear axle centre.
//
// KINEMATIC MODEL. A kinematic bicycle about the rear axle: with steering delta the rear axle
// follows a circle of signed curvature k = tan(delta) / L (L = chassis.wheelbase_m), radius
// R = 1 / k, centre (0, R) in the body frame (left turn R > 0). A signed rear-axle travel s
// (positive forward) turns the heading by k * s. Driving backwards with a left steering the
// rear axle runs the same circle the other way, so the heading DEcreases; with a right
// steering it increases. advance_pose is exact for this model (no small-step integration).
//
// BODY. The bounding box about the rear axle: x in [-rear_x_m, front_x_m], |y| <= half_width_m,
// where front_x_m = chassis.wheelbase_m + chassis.front_overhang_m and rear_x_m =
// chassis.rear_overhang_m (both overhangs PROVISIONAL placeholders until measured, see
// vehicle_params.yaml). Obstacles are line segments (the faces of what the LiDAR saw, or a lane
// wall). body_clearance is the smallest distance between the box and any segment, 0 when they
// touch or overlap (a segment wholly inside the box also counts as 0).
//
// SWEPT-BODY CHECK. sweep_body samples the motion every step_m of rear-axle travel, ending
// exactly at the motion's end, and evaluates body_clearance at each sample. Between samples a
// body corner moves at most step_m * hypot(front_x, R + half_width) / R, about 1.3 times the
// step at the manoeuvres' radius, so with the default 0.01 m step a segment can come at most
// about 0.007 m closer between two samples than the check saw; the planning margin (0.1 m in
// the floor profile) is an order of magnitude larger.
//
// GATE PREDICTION. predicted_gate_distance_m computes, for a pose, a steering and a direction,
// the in-path distance racer_safety's obstacle gate would compute from a scan of these
// obstacle points: the forward corridor from the LiDAR head, or the rear corridor from the rear
// bumper line, straight or along the steering arc, with the body band, the outer-corner band
// and its horizon, the quarter-turn limit and the outer sector bound
// (ros_ws/src/racer_safety/include/racer_safety/forward_sector.hpp, "ARC CORRIDOR", "REAR
// CORRIDOR", "OUTER BOUNDARY", "OUTER-CORNER HORIZON"). It is a separate, small re-statement of
// that model for PLANNING, not shared code: racer_safety must not share a code path with the
// controllers it gates (claude-docs/05-safety.md, forward_sector.hpp's last header paragraph),
// and this copy only decides whether a plan is worth starting. It is conservative in one way
// and not in another, both stated: it sees every obstacle point (the real LiDAR sees only the
// first return along each ray, so occluded points make the prediction smaller, never larger),
// and it sees a straight segment as densely sampled points (the real scan samples at the
// LiDAR's angular resolution). If the prediction is wrong the gate still brakes, and park_node
// aborts on a held car (park_controller.hpp "BLOCKED").
#ifndef RACER_CONTROL_PARK_GEOMETRY_HPP_
#define RACER_CONTROL_PARK_GEOMETRY_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace racer_control {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2 {
  double x = 0.0;    // rear axle centre, m
  double y = 0.0;    // m
  double yaw = 0.0;  // rad, counter-clockwise
};

struct Segment2 {
  Point2 a;
  Point2 b;
};

// The car, from vehicle_params (park_node fills it; nothing here holds a dimension).
struct ParkBody {
  double wheelbase_m = 0.0;   // chassis.wheelbase_m
  double half_width_m = 0.0;  // chassis.width_m / 2
  double front_x_m = 0.0;     // chassis.wheelbase_m + chassis.front_overhang_m
  double rear_x_m = 0.0;      // chassis.rear_overhang_m (positive, behind the rear axle)
  double lidar_x_m = 0.0;     // sensors.lidar.mount_x_m (head ahead of the rear axle)
  double lidar_y_m = 0.0;     // sensors.lidar.mount_y_m (head left of the centreline)
};

// True when every dimension is finite, the wheelbase, half width and front_x are positive and
// rear_x is not negative.
bool is_usable_body(const ParkBody& body);

// Wraps an angle into [-pi, pi).
double wrap_pi(double angle_rad);

// Signed curvature tan(steering) / wheelbase (1/m, left positive). 0 for a non-finite input or a
// non-positive wheelbase.
double curvature_for_steering(double steering_rad, double wheelbase_m);

// The pose after the rear axle travels `distance_m` (signed, positive forward) at constant
// `curvature` (1/m). Exact for the kinematic model above; a curvature below 1e-9 in magnitude is
// a straight line.
Pose2 advance_pose(const Pose2& pose, double curvature, double distance_m);

// Body-local point -> world, and back.
Point2 to_world(const Pose2& pose, const Point2& local);
Point2 to_local(const Pose2& pose, const Point2& world);

// Body corners in the world, in order front-left, front-right, rear-right, rear-left.
std::array<Point2, 4> body_corners(const Pose2& pose, const ParkBody& body);

double point_segment_distance(const Point2& p, const Segment2& s);
// 0 when the segments intersect.
double segment_segment_distance(const Segment2& s, const Segment2& t);

// Smallest distance between the body box at `pose` and `segment` (0 when touching, crossing, or
// the segment lies inside the box).
double body_segment_distance(const Pose2& pose, const ParkBody& body, const Segment2& segment);

struct Clearance {
  double distance_m = std::numeric_limits<double>::infinity();
  std::size_t obstacle_index = 0;  // which segment set the distance (meaningless when +inf)
};
Clearance body_clearance(const Pose2& pose, const ParkBody& body,
                         const std::vector<Segment2>& obstacles);

// One motion of a manoeuvre: a constant steering held over a signed rear-axle travel.
struct ParkMotion {
  double steering_rad = 0.0;
  double distance_m = 0.0;  // signed: positive forward, negative backward
};

struct SweepResult {
  bool clear = true;  // min_clearance_m >= the required clearance at every sample
  double min_clearance_m = std::numeric_limits<double>::infinity();
  double worst_travel_m = 0.0;  // |travel| at the sample with the least clearance
  std::size_t worst_obstacle = 0;
  Pose2 end;  // the pose at the end of the motion
};

// Samples the motion every step_m (> 0) of rear-axle travel, including the start and the end.
// With stop_at_violation the sampling ends at the first sample below the required clearance
// (min_clearance_m and worst_* then describe that sample; for planning, where only clear or not
// matters).
SweepResult sweep_body(const Pose2& start, const ParkMotion& motion, const ParkBody& body,
                       const std::vector<Segment2>& obstacles, double required_clearance_m,
                       double step_m, bool stop_at_violation = false);

// racer_safety's obstacle-gate geometry, from vehicle_params (see GATE PREDICTION above).
struct ParkGateModel {
  bool enabled = false;
  double corridor_half_width_m = 0.0;   // chassis.width_m / 2 + limits.obstacle_corridor_margin_m
  double sector_half_angle_rad = 0.0;   // limits.ttc_forward_sector_half_angle_rad
  double outer_corner_horizon_m = 0.0;  // limits.outer_corner_horizon_m
  double max_steering_rad = 0.0;        // steering.max_angle_rad (the request is clamped to it)
  double floor_m = 0.0;                 // limits.min_forward_clearance_m
  double ttc_brake_s = 0.0;             // limits.ttc_brake_s (<= 0: not configured)
  double margin_m = 0.0;                // planning headroom added on top (park_node gate_margin_m)
};

// The distance below which the gate brakes a request of `speed_mps`, plus the model's margin:
// max(floor_m, |speed| * ttc_brake_s) + margin_m.
double gate_threshold_m(const ParkGateModel& gate, double speed_mps);

// The gate's in-path distance for a request with `steering_rad` going forward (`reverse` false,
// from the LiDAR head) or backward (from the rear bumper line), with the car at `pose` and the
// obstacle `points` in the world. +infinity when no point is in the path.
//
// `ignore_beyond_m` (default: none) is a speed-up for threshold tests: points farther than
// gate_cull_radius_m(..., threshold) from the reference point cannot have an in-path distance at
// or below that threshold, so sweep_gate skips them; the result is then exact below the
// threshold and may be +infinity above it.
double predicted_gate_distance_m(const Pose2& pose, double steering_rad, bool reverse,
                                 const std::vector<Point2>& points, const ParkBody& body,
                                 const ParkGateModel& gate,
                                 double ignore_beyond_m = std::numeric_limits<double>::infinity());

// A radius about the gate's reference point (the head going forward, the rear bumper line's
// centre going backward) beyond which no point can have an in-path distance <= threshold_m.
// Straight: distance >= Euclidean - corridor half width - |mount_y|. Arc: a point and the
// reference both lie in the band [|R| - half width, R_out] about the turn centre, so their
// Euclidean distance is at most (R_out - |R| + half width) + R_out * (arc distance) / |R|.
double gate_cull_radius_m(double steering_rad, bool reverse, const ParkBody& body,
                          const ParkGateModel& gate, double threshold_m);

struct GateSweepResult {
  bool clear = true;
  double min_distance_m = std::numeric_limits<double>::infinity();
  double worst_travel_m = 0.0;
};

// predicted_gate_distance_m at every sample of the motion (sampled as in sweep_body) for a
// request of `speed_mps` in the motion's direction; clear when every sample's distance exceeds
// gate_threshold_m. A disabled gate model is always clear. stop_at_violation as for sweep_body.
GateSweepResult sweep_gate(const Pose2& start, const ParkMotion& motion, double speed_mps,
                           const std::vector<Point2>& points, const ParkBody& body,
                           const ParkGateModel& gate, double step_m,
                           bool stop_at_violation = false);

// Points along every segment, at most spacing_m apart, both ends included.
std::vector<Point2> sample_segments(const std::vector<Segment2>& segments, double spacing_m);

}  // namespace racer_control

#endif  // RACER_CONTROL_PARK_GEOMETRY_HPP_
