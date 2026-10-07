// park_controller.hpp through a pure C++ kinematic bicycle simulation (roadmap 2.9).
//
// The simulated car is the planner's own kinematic model (rear axle on circles of
// L / tan(delta)) with what the planner does NOT model: the steering servo slews at
// steering.max_rate_rad_per_s, the speed follows the request with a first-order lag, and the
// wheel odometry is the true rear-axle distance. The LiDAR is park_test_helpers.hpp's 360 degree
// scan (yaw pi, mount applied) of the world's segments, every fifth 50 Hz cycle (10 Hz, like the
// C1). The "gated" speed fed back is the request, one cycle late (safety_node passing it), unless
// a test blocks it. The test drives the whole run: IDLE -> SEARCH -> SLOT_FOUND -> ... -> DONE,
// and checks at EVERY simulated step that the true body never touches the world, then that the
// final pose is parallel within 3 degrees and inside the pocket.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "park_test_helpers.hpp"
#include "racer_control/park_controller.hpp"

namespace racer_control {
namespace {

using namespace park_test;

struct SimOptions {
  double speed_time_constant_s = 0.08;
  // Extra roll after the request drops to zero, as a distance (models a car that coasts).
  bool block_gate = false;
  bool odometry = true;
  double max_time_s = 60.0;
  int substeps = 4;
};

struct SimResult {
  ParkPhase phase = ParkPhase::kIdle;
  Pose2 pose;
  double min_clearance_m = 1e9;
  std::vector<std::string> logs;
  bool saw_reverse = false;
  bool odom_fallback_seen = false;
  double time_s = 0.0;
};

bool contains(const std::vector<std::string>& logs, const std::string& needle) {
  return std::any_of(logs.begin(), logs.end(),
                     [&](const std::string& l) { return l.find(needle) != std::string::npos; });
}

// Runs the controller against the world until it is DONE or ABORT (or max_time_s).
// `start` is called once, after a second of idle cycles.
template <typename StartFn>
SimResult simulate(ParkController& controller, const std::vector<Segment2>& world,
                   const Pose2& initial, const SimOptions& options, StartFn start) {
  SimResult r;
  const ParkBody body = car_body();
  const double dt = controller.config().control_period_s;
  Pose2 pose = initial;
  double speed = 0.0;
  double steering = 0.0;
  double odom = 0.0;
  double gated_prev = 0.0;
  bool started = false;
  int cycle = 0;
  for (double t = 0.0; t < options.max_time_s; t += dt, ++cycle) {
    if (!started && t >= 1.0) {
      started = true;
      start(pose);
    }
    if (cycle % 5 == 0) {
      controller.on_scan(synthetic_scan(pose, world));
    }
    ParkStepInput in;
    if (options.odometry) {
      in.odometry = ParkOdometry{odom, speed};
    }
    in.gated_speed_mps = options.block_gate ? 0.0 : gated_prev;
    in.scan_fresh = true;
    const ParkOutput out = controller.step(in);
    for (const ParkLogLine& l : controller.take_logs()) {
      r.logs.push_back(l.text);
    }
    r.odom_fallback_seen = r.odom_fallback_seen || controller.odom_fallback();
    const double request = out.publish ? out.speed_mps : 0.0;
    const double gated = options.block_gate ? 0.0 : request;
    gated_prev = gated;
    if (out.publish && request < 0.0) {
      r.saw_reverse = true;
    }
    const double steer_target = out.publish ? out.steering_rad : steering;
    const double h = dt / options.substeps;
    for (int k = 0; k < options.substeps; ++k) {
      const double max_step = kMaxSteeringRate * h;
      steering += std::clamp(steer_target - steering, -max_step, max_step);
      speed += (gated - speed) * h / (options.speed_time_constant_s + h);
      const double ds = speed * h;
      pose = advance_pose(pose, curvature_for_steering(steering, kWheelbase), ds);
      odom += ds;
    }
    r.min_clearance_m = std::min(r.min_clearance_m, body_clearance(pose, body, world).distance_m);
    if (started && !controller.active()) {
      // Let it roll to a stop.
      for (int k = 0; k < 100; ++k) {
        speed += (0.0 - speed) * dt / (options.speed_time_constant_s + dt);
        pose = advance_pose(pose, curvature_for_steering(steering, kWheelbase), speed * dt);
        r.min_clearance_m =
            std::min(r.min_clearance_m, body_clearance(pose, body, world).distance_m);
      }
      r.time_s = t;
      break;
    }
  }
  r.phase = controller.phase();
  r.pose = pose;
  return r;
}

void expect_parked(const SimResult& r, ParkSide side, double row_lateral, double near_x,
                   double far_x, double depth) {
  ASSERT_EQ(r.phase, ParkPhase::kDone) << (r.logs.empty() ? "" : r.logs.back());
  EXPECT_TRUE(r.saw_reverse);
  // Parallel within 3 degrees.
  EXPECT_LT(std::abs(wrap_pi(r.pose.yaw)), 3.0 * M_PI / 180.0) << "final yaw " << r.pose.yaw;
  // Inside the pocket: every body corner between the row line and the back, and between the
  // edges.
  const double s = side_sign(side);
  for (const Point2& c : body_corners(r.pose, car_body())) {
    const double lateral = s * c.y;
    EXPECT_GE(lateral, row_lateral - 1e-9) << "corner sticks out of the pocket";
    EXPECT_LE(lateral, row_lateral + depth);
    EXPECT_GT(c.x, near_x);
    EXPECT_LT(c.x, far_x);
  }
  // No swept-body contact at any simulated step.
  EXPECT_GT(r.min_clearance_m, 0.0);
}

// Right-side slot: row 0.5 m from the car's centreline, pocket 1.7 m long and 0.65 m deep, a
// wall 0.9 m away on the left; the car starts 2.5 m before the pocket.
TEST(ParkController, ParksInARightSidePocket) {
  const double row = 0.5;
  const double near_x = 0.0;
  const double far_x = 1.7;
  const double depth = 0.65;
  const auto world = pocket_world(ParkSide::kRight, row, near_x, far_x, depth, 0.9);
  ParkController controller(car_controller(ParkSide::kRight));
  const SimResult r =
      simulate(controller, world, Pose2{-2.5, 0.0, 0.0}, SimOptions{},
               [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(true).ok); });
  for (const auto& l : r.logs) {
    std::printf("  log: %s\n", l.c_str());
  }
  std::printf("final pose (%.3f, %.3f, %.4f), min clearance %.3f m, %.1f s\n", r.pose.x, r.pose.y,
              r.pose.yaw, r.min_clearance_m, r.time_s);
  expect_parked(r, ParkSide::kRight, row, near_x, far_x, depth);
  EXPECT_TRUE(contains(r.logs, "SEARCH -> SLOT_FOUND"));
  EXPECT_TRUE(contains(r.logs, "-> DRIVE_TO_START"));
  EXPECT_TRUE(contains(r.logs, "-> ARC_1"));
  EXPECT_TRUE(contains(r.logs, "-> ARC_2"));
  EXPECT_TRUE(contains(r.logs, "-> STRAIGHTEN"));
  EXPECT_TRUE(contains(r.logs, "-> DONE"));
}

// The mirror image on the left.
TEST(ParkController, ParksInALeftSidePocket) {
  const double row = 0.45;
  const double near_x = 0.5;
  const double far_x = 2.25;
  const double depth = 0.7;
  const auto world = pocket_world(ParkSide::kLeft, row, near_x, far_x, depth, 0.9);
  ParkController controller(car_controller(ParkSide::kLeft));
  const SimResult r =
      simulate(controller, world, Pose2{-2.0, 0.0, 0.0}, SimOptions{},
               [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(true).ok); });
  std::printf("final pose (%.3f, %.3f, %.4f), min clearance %.3f m, %.1f s\n", r.pose.x, r.pose.y,
              r.pose.yaw, r.min_clearance_m, r.time_s);
  expect_parked(r, ParkSide::kLeft, row, near_x, far_x, depth);
}

// A sluggish car (speed time constant 0.25 s) rolls on after every stop command. The heading
// targets take ARC_1's overshoot out in ARC_2, and the stop lead learnt at ARC_1's stop brings
// ARC_2's in: still parallel within 3 degrees, still clear.
TEST(ParkController, OvershootIsTakenOutByTheNextArc) {
  const double row = 0.5;
  const auto world = pocket_world(ParkSide::kRight, row, 0.0, 1.8, 0.7, 0.9);
  ParkController controller(car_controller(ParkSide::kRight));
  SimOptions options;
  options.speed_time_constant_s = 0.25;
  const SimResult r =
      simulate(controller, world, Pose2{-2.5, 0.0, 0.0}, options,
               [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(true).ok); });
  std::printf("final pose (%.3f, %.3f, %.4f), min clearance %.3f m\n", r.pose.x, r.pose.y,
              r.pose.yaw, r.min_clearance_m);
  expect_parked(r, ParkSide::kRight, row, 0.0, 1.8, 0.7);
  EXPECT_TRUE(contains(r.logs, "rolled"));
}

// No pocket along the row: SEARCH runs out after search_max_distance_m and ABORTs.
TEST(ParkController, NoPocketAbortsAfterTheSearchDistance) {
  const auto world = pocket_world(ParkSide::kRight, 0.5, 30.0, 31.0, 0.6, 0.9);
  ParkControllerConfig config = car_controller(ParkSide::kRight);
  config.search_max_distance_m = 1.5;
  ParkController controller(config);
  const SimResult r = simulate(controller, world, Pose2{}, SimOptions{}, [&](const Pose2&) {
    EXPECT_TRUE(controller.start_parallel_park(true).ok);
  });
  EXPECT_EQ(r.phase, ParkPhase::kAbort);
  EXPECT_TRUE(contains(r.logs, "no feasible slot"));
  EXPECT_FALSE(r.saw_reverse);
}

// A pocket long enough for the detector (0.93 m) but too short for the plan: the plan is
// rejected with its reason, the slot is ignored and SEARCH carries on (then runs out).
TEST(ParkController, InfeasibleSlotIsReportedAndSkipped) {
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.0, 1.1, 0.7, 0.9);
  ParkControllerConfig config = car_controller(ParkSide::kRight);
  config.search_max_distance_m = 4.0;
  ParkController controller(config);
  const SimResult r =
      simulate(controller, world, Pose2{-2.0, 0.0, 0.0}, SimOptions{},
               [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(true).ok); });
  EXPECT_EQ(r.phase, ParkPhase::kAbort);
  EXPECT_TRUE(contains(r.logs, "plan rejected"));
  EXPECT_TRUE(contains(r.logs, "SLOT_FOUND -> SEARCH"));
  EXPECT_FALSE(r.saw_reverse);
}

// safety_node refusing every request (gated speed 0): ABORT after blocked_abort_s, naming the
// phase.
TEST(ParkController, HeldBySafetyNodeAborts) {
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.0, 1.7, 0.65, 0.9);
  ParkController controller(car_controller(ParkSide::kRight));
  SimOptions options;
  options.block_gate = true;
  const SimResult r =
      simulate(controller, world, Pose2{-2.5, 0.0, 0.0}, options,
               [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(true).ok); });
  EXPECT_EQ(r.phase, ParkPhase::kAbort);
  EXPECT_TRUE(contains(r.logs, "safety_node held the car"));
  EXPECT_TRUE(contains(r.logs, "in SEARCH"));
  // About settle_s + blocked_abort_s after the start (plus the idle second).
  EXPECT_LT(r.time_s, 1.0 + 0.3 + 2.0 + 0.2);
}

// require_odometry true refuses to start without odometry; false starts and falls back.
TEST(ParkController, OdometryRequiredOrFallback) {
  {
    ParkController controller(car_controller(ParkSide::kRight));
    const StartResult result = controller.start_parallel_park(false);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("require_odometry"), std::string::npos);
    EXPECT_EQ(controller.phase(), ParkPhase::kIdle);
  }
  {
    ParkControllerConfig config = car_controller(ParkSide::kRight);
    config.require_odometry = false;
    const auto world = pocket_world(ParkSide::kRight, 0.5, 0.0, 1.7, 0.65, 0.9);
    ParkController controller(config);
    SimOptions options;
    options.odometry = false;
    options.speed_time_constant_s = 0.02;
    const SimResult r =
        simulate(controller, world, Pose2{-2.5, 0.0, 0.0}, options,
                 [&](const Pose2&) { EXPECT_TRUE(controller.start_parallel_park(false).ok); });
    EXPECT_TRUE(r.odom_fallback_seen);
    EXPECT_TRUE(contains(r.logs, "FALLING BACK"));
    // With a fast car and the request as the gated speed the fallback is nearly exact here.
    EXPECT_EQ(r.phase, ParkPhase::kDone);
  }
  {
    // Odometry lost mid-run with require_odometry true: ABORT.
    ParkController controller(car_controller(ParkSide::kRight));
    ASSERT_TRUE(controller.start_parallel_park(true).ok);
    ParkStepInput in;
    in.odometry = ParkOdometry{0.0, 0.0};
    in.gated_speed_mps = 0.0;
    in.scan_fresh = true;
    controller.step(in);
    in.odometry.reset();
    controller.step(in);
    EXPECT_EQ(controller.phase(), ParkPhase::kAbort);
  }
}

// ~/abort mid-run: ABORT, zero speed published for final_hold_s, then nothing.
TEST(ParkController, AbortPublishesZeroThenGoesQuiet) {
  ParkController controller(car_controller(ParkSide::kRight));
  ASSERT_TRUE(controller.start_parallel_park(true).ok);
  ParkStepInput in;
  in.odometry = ParkOdometry{0.0, 0.0};
  in.gated_speed_mps = 0.0;
  in.scan_fresh = true;
  for (int i = 0; i < 40; ++i) {
    in.gated_speed_mps = controller.step(in).speed_mps;
  }
  EXPECT_TRUE(controller.abort("test").ok);
  EXPECT_FALSE(controller.abort("again").ok);
  int zero = 0;
  int silent = 0;
  for (int i = 0; i < 100; ++i) {
    const ParkOutput out = controller.step(in);
    if (out.publish) {
      EXPECT_EQ(out.speed_mps, 0.0);
      ++zero;
    } else {
      ++silent;
    }
  }
  EXPECT_EQ(zero, 50);  // final_hold_s 1.0 at 50 Hz
  EXPECT_EQ(silent, 50);
  EXPECT_TRUE(controller.start_parallel_park(true).ok);  // a new run may start after ABORT
}

// The three-point turn in a 1.8 m lane (walls 0.9 m either side): heading pi within 3 degrees,
// no contact.
TEST(ParkController, ThreePointTurnInALane) {
  const std::vector<Segment2> world = {{{-5.0, 0.9}, {5.0, 0.9}}, {{-5.0, -0.9}, {5.0, -0.9}}};
  ParkController controller(car_controller(ParkSide::kRight));
  const SimResult r = simulate(controller, world, Pose2{}, SimOptions{}, [&](const Pose2& pose) {
    const StartResult result =
        controller.start_three_point_turn(synthetic_scan(pose, world), true, true);
    EXPECT_TRUE(result.ok) << result.message;
  });
  for (const auto& l : r.logs) {
    std::printf("  log: %s\n", l.c_str());
  }
  std::printf("final pose (%.3f, %.3f, %.4f), min clearance %.3f m\n", r.pose.x, r.pose.y,
              r.pose.yaw, r.min_clearance_m);
  ASSERT_EQ(r.phase, ParkPhase::kDone);
  EXPECT_LT(std::abs(wrap_pi(r.pose.yaw - M_PI)), 3.0 * M_PI / 180.0);
  EXPECT_GT(r.min_clearance_m, 0.0);
  EXPECT_TRUE(r.saw_reverse);
  EXPECT_TRUE(contains(r.logs, "-> TURN_1"));
  EXPECT_TRUE(contains(r.logs, "-> TURN_2"));
  EXPECT_TRUE(contains(r.logs, "-> TURN_3"));
}

// A lane too narrow for the arcs: refused before anything moves.
TEST(ParkController, ThreePointTurnRefusedInANarrowLane) {
  const std::vector<Segment2> world = {{{-5.0, 0.6}, {5.0, 0.6}}, {{-5.0, -0.6}, {5.0, -0.6}}};
  ParkController controller(car_controller(ParkSide::kRight));
  const StartResult result =
      controller.start_three_point_turn(synthetic_scan(Pose2{}, world), true, true);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("lane too narrow"), std::string::npos) << result.message;
  EXPECT_EQ(controller.phase(), ParkPhase::kIdle);
  // No wall on one side: refused too.
  const std::vector<Segment2> open = {{{-5.0, 0.9}, {5.0, 0.9}}};
  EXPECT_FALSE(controller.start_three_point_turn(synthetic_scan(Pose2{}, open), true, true).ok);
}

}  // namespace
}  // namespace racer_control
