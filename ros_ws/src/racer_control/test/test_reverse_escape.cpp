// L1 tests for the reverse escape (2026-10-06 night floor finding (b)): the forward arc probe
// (gap_follow.hpp any_forward_arc_clear) and the escape state machine (reverse_escape.hpp).
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "racer_control/gap_follow.hpp"
#include "racer_control/laser_scan.hpp"
#include "racer_control/reverse_escape.hpp"

namespace racer_control {
namespace {

constexpr double kC1Yaw = M_PI;
constexpr double kMaxSteer = 0.4189;  // steering.max_angle_rad
constexpr double kDt = 0.02;          // 50 Hz control period

// -- Forward arc probe ------------------------------------------------------------------------

// The car's geometry from config/vehicle_params.yaml (schema 0.10.0), typed as a fixture.
ArcProbeGeometry car_geometry(double margin_m = 0.05) {
  ArcProbeGeometry g;
  g.wheelbase_m = 0.3302;
  g.half_width_m = 0.155;
  g.margin_m = margin_m;
  g.front_x_m = 0.17145 + 0.58 / 2.0;  // cg_to_rear_axle_m + length_m / 2, as for the clamp
  g.rear_x_m = 0.12;                   // chassis.rear_overhang_m (PROVISIONAL)
  g.lidar_mount_x_m = 0.285;
  g.lidar_mount_y_m = 0.0;
  return g;
}

struct Segment {
  double x0, y0, x1, y1;  // in the vehicle-axis frame centred on the LiDAR head
};

double ray_cast(double bearing, const std::vector<Segment>& segments) {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);
  double best = std::numeric_limits<double>::infinity();
  for (const Segment& s : segments) {
    const double ex = s.x1 - s.x0;
    const double ey = s.y1 - s.y0;
    const double denom = dx * ey - dy * ex;
    if (std::abs(denom) < 1e-12) {
      continue;
    }
    const double t = (s.x0 * ey - s.y0 * ex) / denom;
    const double u = (s.x0 * dy - s.y0 * dx) / denom;
    if (t > 0.0 && u >= 0.0 && u <= 1.0) {
      best = std::min(best, t);
    }
  }
  return best;
}

// The real C1: 720 rays from angle_min -pi, mounted backwards (or `yaw`).
ScanInput scan_with(const std::vector<Segment>& segments, double yaw = kC1Yaw) {
  ScanInput scan;
  scan.ranges.assign(720, 0.0f);
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 720.0;
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    scan.ranges[i] = static_cast<float>(ray_cast(laser_bearing_of_index(scan, i) + yaw, segments));
  }
  return scan;
}

// Head-frame x of the front of the body: front_x from the rear axle minus the mount.
constexpr double kHeadToFront = 0.17145 + 0.29 - 0.285;

TEST(ForwardArcProbe, AnOpenLaneIsClearStraightAhead) {
  std::vector<double> buf;
  const ScanInput lane = scan_with({{-3.0, 0.55, 5.0, 0.55}, {-3.0, -0.55, 5.0, -0.55}});
  EXPECT_TRUE(forward_arc_clear(lane, kC1Yaw, 0.0, 0.3, car_geometry(), buf));
  EXPECT_TRUE(any_forward_arc_clear(lane, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
}

TEST(ForwardArcProbe, NoseInToAWallAcrossTheLaneIsBlockedAtEveryLock) {
  // The night's trap, simplified: a wall across a 1.1 m lane 0.2 m ahead of the bumper. No
  // steering gives 0.3 m of travel: the wall spans the whole lane.
  std::vector<double> buf;
  const double wall_x = kHeadToFront + 0.2;
  const ScanInput trap = scan_with(
      {{wall_x, -0.55, wall_x, 0.55}, {-3.0, 0.55, wall_x, 0.55}, {-3.0, -0.55, wall_x, -0.55}});
  EXPECT_FALSE(any_forward_arc_clear(trap, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
  for (const double s : {-kMaxSteer, -0.2, 0.0, 0.2, kMaxSteer}) {
    EXPECT_FALSE(forward_arc_clear(trap, kC1Yaw, s, 0.3, car_geometry(), buf)) << s;
  }
  // A shorter probe still fits: 0.1 m of travel stops 0.05 m (the margin) short of the wall.
  EXPECT_TRUE(forward_arc_clear(trap, kC1Yaw, 0.0, 0.1, car_geometry(), buf));
}

TEST(ForwardArcProbe, ACornerTighterThanTheTurningCircleIsBlocked) {
  // A 1.1 m lane turning left through a square corner: the outside wall 0.25 m ahead of the
  // bumper, the left wall ending at the corner. Full lock left (radius 0.74 m at the rear
  // axle) swings the outside front corner into the wall long before 0.3 m of travel.
  std::vector<double> buf;
  const double wall_x = kHeadToFront + 0.25;
  const ScanInput corner = scan_with({{wall_x, -0.55, wall_x, 2.0},
                                      {-3.0, -0.55, wall_x, -0.55},
                                      {-3.0, 0.55, wall_x - 1.1, 0.55},
                                      {wall_x - 1.1, 0.55, wall_x - 1.1, 2.0}});
  EXPECT_FALSE(any_forward_arc_clear(corner, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
}

TEST(ForwardArcProbe, OnlyFullLockAwayFromAPostClearsIt) {
  // A post ahead, reaching from 0.3 m right of the centreline to just inside the right edge
  // of the inflated body (0.18 m < 0.205 m): straight and right are blocked, a hard left arc
  // swings the right front corner inside it (outer corner radius 1.075 m about the turn centre,
  // the post at 1.128 m or more).
  std::vector<double> buf;
  const double post_x = 0.65 - 0.285;  // 0.65 m ahead of the rear axle
  const ScanInput post = scan_with({{post_x, -0.30, post_x, -0.18}});
  EXPECT_FALSE(forward_arc_clear(post, kC1Yaw, 0.0, 0.3, car_geometry(), buf));
  EXPECT_FALSE(forward_arc_clear(post, kC1Yaw, -kMaxSteer, 0.3, car_geometry(), buf));
  EXPECT_TRUE(forward_arc_clear(post, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
  EXPECT_TRUE(any_forward_arc_clear(post, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
}

TEST(ForwardArcProbe, ReturnsInsideTheStartFootprintAreIgnored) {
  std::vector<double> buf;
  // A return 0.02 m off the side of the body (inside the 0.05 m margin) and nothing else.
  const ScanInput touching = scan_with({{-0.2, 0.175, 0.0, 0.175}});
  EXPECT_TRUE(forward_arc_clear(touching, kC1Yaw, 0.0, 0.3, car_geometry(), buf));
}

TEST(ForwardArcProbe, UsesTheVehicleFrameWhateverTheMountYaw) {
  std::vector<double> buf;
  const double post_x = 0.65 - 0.285;
  const std::vector<Segment> post{{post_x, -0.30, post_x, -0.18}};
  for (const double s : {-kMaxSteer, 0.0, kMaxSteer}) {
    EXPECT_EQ(forward_arc_clear(scan_with(post, M_PI), M_PI, s, 0.3, car_geometry(), buf),
              forward_arc_clear(scan_with(post, 0.0), 0.0, s, 0.3, car_geometry(), buf))
        << s;
  }
}

TEST(ForwardArcProbe, GarbageReportsClear) {
  std::vector<double> buf;
  const double wall_x = kHeadToFront + 0.2;
  const ScanInput trap = scan_with({{wall_x, -0.55, wall_x, 0.55}});
  ASSERT_FALSE(any_forward_arc_clear(trap, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<ArcProbeGeometry> bad(6, car_geometry());
  bad[0].wheelbase_m = 0.0;
  bad[1].half_width_m = nan;
  bad[2].margin_m = -0.01;
  bad[3].front_x_m = nan;
  bad[4].rear_x_m = -1.0;
  bad[5].lidar_mount_x_m = nan;
  for (const ArcProbeGeometry& g : bad) {
    EXPECT_TRUE(any_forward_arc_clear(trap, kC1Yaw, kMaxSteer, 0.3, g, buf));
  }
  EXPECT_TRUE(any_forward_arc_clear(trap, kC1Yaw, kMaxSteer, 0.0, car_geometry(), buf));
  EXPECT_TRUE(any_forward_arc_clear(trap, nan, kMaxSteer, 0.3, car_geometry(), buf));
  EXPECT_TRUE(any_forward_arc_clear(trap, kC1Yaw, M_PI_2, 0.3, car_geometry(), buf));
  EXPECT_TRUE(forward_arc_clear(trap, kC1Yaw, nan, 0.3, car_geometry(), buf));
  ScanInput unusable = trap;
  unusable.angle_increment = 0.0;
  EXPECT_TRUE(any_forward_arc_clear(unusable, kC1Yaw, kMaxSteer, 0.3, car_geometry(), buf));
}

// -- Escape state machine --------------------------------------------------------------------

ReverseEscapeConfig config() {
  ReverseEscapeConfig c;
  c.escape_after_s = 1.5;
  c.escape_speed_mps = -0.5;
  c.escape_steering_rad = kMaxSteer;
  c.escape_distance_m = 0.4;
  c.escape_max_s = 2.0;
  c.escape_retry_after_s = 3.0;
  c.escape_max_attempts = 3;
  c.block_debounce_s = 0.1;
  c.max_acceleration_mps2 = 0.951;  // the floor profile: 0.1 x 9.51
  return c;
}

// Runs the machine like gap_follow_node does: the "requested" speed fed back is what it
// commanded last cycle (the follower's forward request when the escape is idle), and the
// gate is modelled by a callback that turns that request into the gated speed.
struct Harness {
  explicit Harness(ReverseEscapeConfig c = config()) : escape(c) {}

  // forward_mps: what the follower asks for when not escaping. gate: request -> gated speed
  // (nullopt = stale /drive).
  template <typename Gate>
  EscapeOutput step(double forward_mps, bool path_blocked, double forward_steering, Gate gate) {
    EscapeInput in;
    in.dt_s = kDt;
    in.requested_speed_mps = last_request;
    in.gated_speed_mps = gate(last_request);
    in.forward_path_blocked = path_blocked;
    in.forward_steering_rad = forward_steering;
    EscapeOutput out = escape.update(in);
    last_request = out.active ? out.speed_mps : forward_mps;
    if (out.event) {
      events.push_back(*out.event);
    }
    return out;
  }

  ReverseEscape escape;
  double last_request = 0.0;
  std::vector<EscapeEvent> events;
};

// Gate models.
std::optional<double> blocks_forward_passes_reverse(double request) {
  return request > 0.0 ? 0.0 : request;
}
std::optional<double> blocks_everything(double) { return 0.0; }
std::optional<double> passes_everything(double request) { return request; }
std::optional<double> silent(double) { return std::nullopt; }

int cycles(double seconds) { return static_cast<int>(std::lround(seconds / kDt)); }

// Drives the harness, forward-blocked with the path blocked, until an escape starts. Returns
// the number of cycles it took (or -1).
int run_until_start(Harness& h, double steering = 0.3) {
  for (int i = 0; i < cycles(10.0); ++i) {
    const EscapeOutput out = h.step(0.5, true, steering, blocks_forward_passes_reverse);
    if (out.event == EscapeEvent::kStarted) {
      return i;
    }
  }
  return -1;
}

TEST(ReverseEscape, StartsAfterEscapeAfterSecondsForwardBlockedWithTheForwardPathBlocked) {
  Harness h;
  const int started = run_until_start(h, 0.3);
  // The first cycle only establishes the forward request (nothing was requested before it).
  EXPECT_NEAR(started * kDt, 1.5 + kDt, 2.0 * kDt);
  EXPECT_EQ(h.escape.state(), EscapeState::kEscaping);
  EXPECT_EQ(h.escape.attempts(), 1);
  // Reversing, opposite to the wanted (left) forward steering.
  EXPECT_LT(h.last_request, 0.0);
  EXPECT_DOUBLE_EQ(h.escape.escape_steering_rad(), -kMaxSteer);
}

TEST(ReverseEscape, NeverStartsWithoutGatedFeedback) {
  Harness h;
  for (int i = 0; i < cycles(10.0); ++i) {
    EXPECT_FALSE(h.step(0.5, true, 0.3, silent).active);
  }
  EXPECT_TRUE(h.events.empty());
}

TEST(ReverseEscape, NeverStartsWhileTheFollowerStillSeesAWayThrough) {
  Harness h;
  for (int i = 0; i < cycles(10.0); ++i) {
    EXPECT_FALSE(h.step(0.5, false, 0.3, blocks_forward_passes_reverse).active);
  }
  EXPECT_TRUE(h.events.empty());
  // Once the follower's path closes it starts on the very next cycle: the gate has been
  // blocking forward for longer than escape_after_s already.
  EXPECT_EQ(h.step(0.5, true, 0.3, blocks_forward_passes_reverse).event, EscapeEvent::kStarted);
}

TEST(ReverseEscape, TheBlockTimerRestartsWhenTheGatePassesForward) {
  Harness h;
  for (int i = 0; i < cycles(1.4); ++i) {
    h.step(0.5, true, 0.3, blocks_forward_passes_reverse);
  }
  h.step(0.5, true, 0.3, passes_everything);  // one cycle through
  for (int i = 0; i < cycles(1.4); ++i) {
    EXPECT_FALSE(h.step(0.5, true, 0.3, blocks_forward_passes_reverse).active);
  }
  EXPECT_TRUE(h.events.empty());
}

TEST(ReverseEscape, SteeringIsOppositeToTheWantedForwardSteering) {
  for (const auto& [forward, expected] :
       std::vector<std::pair<double, double>>{{0.3, -kMaxSteer}, {-0.05, kMaxSteer}, {0.0, 0.0}}) {
    Harness h;
    ASSERT_GE(run_until_start(h, forward), 0);
    EXPECT_DOUBLE_EQ(h.escape.escape_steering_rad(), expected) << forward;
    const EscapeOutput out = h.step(0.5, true, forward, blocks_forward_passes_reverse);
    EXPECT_DOUBLE_EQ(out.steering_rad, expected) << forward;
  }
}

TEST(ReverseEscape, CompletesAfterTheDistanceIntegratedFromTheCommandedSpeed) {
  Harness h;
  ASSERT_GE(run_until_start(h), 0);
  double integrated = -h.last_request * kDt;
  double peak = -h.last_request;
  int n = 0;
  EscapeOutput out;
  while ((out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse)).active) {
    // Ramp at 0.951 m/s^2 up to 0.5 m/s, never faster.
    EXPECT_LE(-out.speed_mps, 0.5 + 1e-12);
    EXPECT_LE(-out.speed_mps - peak, 0.951 * kDt + 1e-12);
    peak = std::max(peak, -out.speed_mps);
    integrated += -out.speed_mps * kDt;
    ASSERT_LT(++n, cycles(5.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kCompleted);
  EXPECT_EQ(h.escape.state(), EscapeState::kIdle);
  EXPECT_GE(h.escape.escape_distance_m(), 0.4);
  EXPECT_LT(h.escape.escape_distance_m(), 0.4 + 0.5 * kDt + 1e-9);
  EXPECT_NEAR(h.escape.escape_distance_m(), integrated, 1e-9);
  EXPECT_DOUBLE_EQ(peak, 0.5);
  // Ramp 0.53 s (0.13 m), then 0.27 m at 0.5 m/s: about 1.06 s in all.
  EXPECT_NEAR(h.escape.escape_elapsed_s(), 1.06, 0.05);
}

TEST(ReverseEscape, AnInstantRampStepsToTheEscapeSpeed) {
  ReverseEscapeConfig c = config();
  c.max_acceleration_mps2 = 0.0;
  Harness h(c);
  ASSERT_GE(run_until_start(h), 0);
  EXPECT_DOUBLE_EQ(h.last_request, -0.5);
}

TEST(ReverseEscape, TimesOutAfterEscapeMaxSeconds) {
  ReverseEscapeConfig c = config();
  c.escape_speed_mps = -0.1;  // 0.2 m in 2 s: the time limit ends it first
  Harness h(c);
  ASSERT_GE(run_until_start(h), 0);
  EscapeOutput out;
  int n = 0;
  while ((out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse)).active) {
    ASSERT_LT(++n, cycles(5.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kTimedOut);
  EXPECT_NEAR(h.escape.escape_elapsed_s(), 2.0, kDt + 1e-9);
  EXPECT_LT(h.escape.escape_distance_m(), 0.4);
}

TEST(ReverseEscape, AbortsWhenSafetyNodeBrakesTheReverseThenWaitsBeforeRetrying) {
  Harness h;
  ASSERT_GE(run_until_start(h), 0);
  // The rear corridor brakes every reverse request.
  EscapeOutput out;
  int n = 0;
  while ((out = h.step(0.5, true, 0.3, blocks_everything)).active) {
    ASSERT_LT(++n, cycles(1.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kAborted);
  EXPECT_EQ(h.escape.state(), EscapeState::kRetryWait);
  // Debounced: about 0.1 s of refusals (plus the cycle of lag) before the abort.
  EXPECT_NEAR(n * kDt, 0.1, 2.5 * kDt);
  // No escape during the retry wait, even though everything still says "escape".
  int waited = 0;
  while (!(out = h.step(0.5, true, 0.3, blocks_everything)).event) {
    EXPECT_FALSE(out.active);
    ASSERT_LT(++waited, cycles(5.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kRetryReady);
  EXPECT_NEAR((waited + 1) * kDt, 3.0, kDt + 1e-9);
  // Still forward-blocked the whole time: the next cycle starts attempt 2.
  out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse);
  EXPECT_EQ(out.event, EscapeEvent::kStarted);
  EXPECT_EQ(h.escape.attempts(), 2);
}

TEST(ReverseEscape, TheFirstCyclesOfLagAreNotARearBlock) {
  Harness h;
  ASSERT_GE(run_until_start(h), 0);
  // The gate passes reverse, but the first 3 cycles of /drive still read 0 (lag).
  int lag = 3;
  const auto lagging = [&lag](double request) -> std::optional<double> {
    if (lag > 0) {
      --lag;
      return 0.0;
    }
    return request;
  };
  EscapeOutput out;
  int n = 0;
  while ((out = h.step(0.5, true, 0.3, lagging)).active) {
    ASSERT_LT(++n, cycles(5.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kCompleted);
}

TEST(ReverseEscape, SilentFeedbackDuringAnEscapeAbortsIt) {
  Harness h;
  ASSERT_GE(run_until_start(h), 0);
  EscapeOutput out;
  int n = 0;
  while ((out = h.step(0.5, true, 0.3, silent)).active) {
    ASSERT_LT(++n, cycles(1.0));
  }
  EXPECT_EQ(out.event, EscapeEvent::kAborted);
}

TEST(ReverseEscape, HoldsAfterMaxAttemptsUntilForwardProgress) {
  Harness h;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    // Wait out the forward block, escape, get braked, wait out the retry.
    int n = 0;
    EscapeOutput out;
    while ((out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse)).event !=
           EscapeEvent::kStarted) {
      ASSERT_LT(++n, cycles(10.0));
    }
    ASSERT_EQ(h.escape.attempts(), attempt);
    while ((out = h.step(0.5, true, 0.3, blocks_everything)).event != EscapeEvent::kRetryReady) {
      ASSERT_LT(++n, cycles(20.0));
    }
  }
  // Attempt 4 is not made: the machine holds, the follower drives (the gate holds the car).
  EscapeOutput out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse);
  EXPECT_EQ(out.event, EscapeEvent::kExhausted);
  EXPECT_EQ(h.escape.state(), EscapeState::kExhausted);
  for (int i = 0; i < cycles(10.0); ++i) {
    EXPECT_FALSE(h.step(0.5, true, 0.3, blocks_forward_passes_reverse).active);
  }
  // Someone moves the obstacle: 0.4 m of forward progress at 0.5 m/s resets the count.
  int n = 0;
  while ((out = h.step(0.5, false, 0.3, passes_everything)).event != EscapeEvent::kReset) {
    ASSERT_LT(++n, cycles(5.0));
  }
  EXPECT_NEAR((n + 1) * kDt, 0.8, 2.0 * kDt);
  EXPECT_EQ(h.escape.state(), EscapeState::kIdle);
  EXPECT_EQ(h.escape.attempts(), 0);
  EXPECT_GE(run_until_start(h), 0);
}

TEST(ReverseEscape, CompletedEscapesWithoutProgressAlsoCountTowardTheLimit) {
  Harness h;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    ASSERT_GE(run_until_start(h), 0);
    EscapeOutput out;
    while ((out = h.step(0.5, true, 0.3, blocks_forward_passes_reverse)).active) {
    }
    ASSERT_EQ(out.event, EscapeEvent::kCompleted);
  }
  EXPECT_EQ(run_until_start(h), -1);
  EXPECT_EQ(h.escape.state(), EscapeState::kExhausted);
}

TEST(ReverseEscape, ResetReturnsToIdleAndKeepsTheAttemptCount) {
  Harness h;
  ASSERT_GE(run_until_start(h), 0);
  h.escape.reset();
  EXPECT_EQ(h.escape.state(), EscapeState::kIdle);
  EXPECT_EQ(h.escape.attempts(), 1);
  h.last_request = 0.0;
  EXPECT_FALSE(h.step(0.5, true, 0.3, blocks_forward_passes_reverse).active);
}

TEST(ReverseEscape, GarbageTimeIsNoTime) {
  ReverseEscape escape(config());
  EscapeInput in;
  in.dt_s = std::numeric_limits<double>::quiet_NaN();
  in.requested_speed_mps = 0.5;
  in.gated_speed_mps = 0.0;
  in.forward_path_blocked = true;
  for (int i = 0; i < 1000; ++i) {
    EXPECT_FALSE(escape.update(in).active);
  }
  EXPECT_EQ(escape.forward_blocked_s(), 0.0);
}

}  // namespace
}  // namespace racer_control
