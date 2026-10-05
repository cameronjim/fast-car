// L1 tests for racer_control's follow-the-gap core (GitHub issue 26). Ported cases from the
// old Python test_gap_logic.py are marked "Ported"; where the expected value changed, the
// comment says why. New cases cover the maths fixes a (forward ray and cone in radians), b
// (invalid returns), c (disparity bubble), d (deepest-ray target), and e (steering law and
// low-pass filter).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_control/gap_follow.hpp"
#include "racer_control/reactive_speed.hpp"

namespace racer_control {
namespace {

constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();
constexpr float kInfF = std::numeric_limits<float>::infinity();
constexpr double kDeg = M_PI / 180.0;

ScanInput make_scan(std::size_t n, double angle_min, double angle_increment, float fill = 5.0f,
                    double range_max = 30.0) {
  ScanInput scan;
  scan.ranges.assign(n, fill);
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.range_min = 0.05;
  scan.range_max = range_max;
  return scan;
}

// racer_gym_bridge's /scan: f1tenth_gym's 1080 beams over a 4.7 rad fov, angle_min = -fov/2,
// counter-clockwise (low index = right, high index = left).
ScanInput bridge_scan(float fill = 5.0f) {
  const double fov = 4.7;
  return make_scan(1080, -fov / 2.0, fov / 1079.0, fill);
}

std::vector<double> to_double(const ScanInput& scan) {
  return std::vector<double>(scan.ranges.begin(), scan.ranges.end());
}

void set_bearings(ScanInput& scan, double lo_rad, double hi_rad, float value) {
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    // 1e-9 rad slack so a ray sitting exactly on a boundary is included despite rounding.
    const double b = wrap_angle(laser_bearing_of_index(scan, i));
    if (b >= lo_rad - 1e-9 && b <= hi_rad + 1e-9) {
      scan.ranges[i] = value;
    }
  }
}

// -- extend_disparities (item c) ---------------------------------------------------------------

constexpr double kInc = 0.1;
constexpr double kHalfWidth = 0.5;

// Ported. atan2(0.5, 1.0) / 0.1 = 4.64 rays: the old int() truncation gave 4 (rays 3..6), the
// new ceil gives 5 (rays 3..7), one ray more conservative.
TEST(ExtendDisparities, InflatesAwayFromTheNearEdgeWithCeil) {
  const std::vector<double> ranges{1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  EXPECT_EQ(out, (std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 5.0}));
}

// Ported, same ceil change (rays 4 down to 0).
TEST(ExtendDisparities, WalksBackwardsWhenTheNearEdgeIsOnTheHighSide) {
  const std::vector<double> ranges{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  EXPECT_EQ(out, (std::vector<double>{5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
}

// Ported.
TEST(ExtendDisparities, NoDisparityLeavesTheScanAlone) {
  const std::vector<double> ranges{2.0, 2.1, 2.2, 2.1, 2.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  EXPECT_EQ(out, ranges);
}

// Ported (the input is a const reference, so this is about the documented contract).
TEST(ExtendDisparities, DoesNotMutateItsInput) {
  const std::vector<double> ranges{1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  const std::vector<double> original = ranges;
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  EXPECT_EQ(ranges, original);
}

TEST(ExtendDisparities, NearZeroIsGuardedAndBlocksAQuarterTurn) {
  std::vector<double> ranges(20, 5.0);
  ranges[0] = 0.0;
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  // ceil((pi/2) / 0.1) = 16 rays: 1..16.
  for (std::size_t k = 0; k <= 16; ++k) {
    EXPECT_DOUBLE_EQ(out[k], 0.0) << k;
  }
  for (std::size_t k = 17; k < 20; ++k) {
    EXPECT_DOUBLE_EQ(out[k], 5.0) << k;
  }
}

TEST(ExtendDisparities, SafetyMarginWidensTheBubble) {
  std::vector<double> ranges(20, 5.0);
  ranges[0] = 1.0;
  std::vector<double> narrow;
  std::vector<double> wide;
  extend_disparities(ranges, 1.0, 0.155, kInc, narrow);       // 0.31 m chassis / 2
  extend_disparities(ranges, 1.0, 0.155 + 0.10, kInc, wide);  // plus a 0.10 m margin
  // ceil(atan2(0.155, 1) / 0.1) = 2, ceil(atan2(0.255, 1) / 0.1) = 3.
  EXPECT_DOUBLE_EQ(narrow[2], 1.0);
  EXPECT_DOUBLE_EQ(narrow[3], 5.0);
  EXPECT_DOUBLE_EQ(wide[3], 1.0);
  EXPECT_DOUBLE_EQ(wide[4], 5.0);
}

TEST(ExtendDisparities, EdgesAreDetectedOnTheInputNotThePartiallyExtendedOutput) {
  // Two near obstacles either side of a deep gap. Each bubble only ever lowers ranges, and
  // the second edge is found even though the first bubble already rewrote ray 3.
  const std::vector<double> ranges{1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 2.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, 0.2, kInc, out);
  // Edge 0|1: near 1.0, ceil(atan2(0.2, 1)/0.1) = 2 rays -> 1, 2.
  // Edge 6|7: near 2.0, ceil(atan2(0.2, 2)/0.1) = 1 ray -> 6.
  EXPECT_EQ(out, (std::vector<double>{1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 2.0, 2.0}));
}

TEST(ExtendDisparities, BubbleIsClippedAtTheScanEnds) {
  const std::vector<double> ranges{5.0, 5.0, 1.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, kInc, out);
  EXPECT_EQ(out, (std::vector<double>{1.0, 1.0, 1.0}));
}

TEST(ExtendDisparities, GarbageGeometryLeavesRangesUnchanged) {
  const std::vector<double> ranges{1.0, 5.0, 5.0};
  std::vector<double> out;
  extend_disparities(ranges, 1.0, kHalfWidth, 0.0, out);
  EXPECT_EQ(out, ranges);
  extend_disparities(ranges, 1.0, 0.0, kInc, out);
  EXPECT_EQ(out, ranges);
}

// -- select_gap (items a and d) -------------------------------------------------------------

// The old tests' 12-ray scan with ray 6 straight ahead. The old cone was the slice [3:9]
// (fractions 0.25 / 0.75); a symmetric half-angle of 0.3 rad at 0.1 rad per ray covers rays
// 3..9.
ScanInput old_twelve_ray_scan() { return make_scan(12, -0.6, 0.1, 0.0f); }

// Ported.
TEST(SelectGap, TargetIsTheCentreOfTheWidestGap) {
  ScanInput scan = old_twelve_ray_scan();
  scan.ranges[3] = 2.0f;
  scan.ranges[5] = scan.ranges[6] = scan.ranges[7] = 2.0f;
  const auto sel = select_gap(scan, to_double(scan), 1.0, 0.3, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_TRUE(sel->gap_found);
  EXPECT_EQ(sel->target_index, 6u);
  EXPECT_EQ(sel->gap_first, 5u);
  EXPECT_EQ(sel->gap_last, 7u);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 0.0, 1e-12);
}

// Ported.
TEST(SelectGap, EqualGapsTieBreakTowardForward) {
  ScanInput scan = old_twelve_ray_scan();
  scan.ranges[3] = scan.ranges[4] = 2.0f;
  scan.ranges[7] = scan.ranges[8] = 2.0f;
  const auto sel = select_gap(scan, to_double(scan), 1.0, 0.3, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_EQ(sel->target_index, 7u);
}

// Ported.
TEST(SelectGap, FullyBlockedConeAimsStraightAhead) {
  const ScanInput scan = old_twelve_ray_scan();
  const auto sel = select_gap(scan, to_double(scan), 1.0, 0.3, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_FALSE(sel->gap_found);
  EXPECT_EQ(sel->target_index, 6u);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 0.0, 1e-12);
}

TEST(SelectGap, GapOnTheLeftGivesPositiveBearingInBridgeRayOrdering) {
  ScanInput scan = bridge_scan(1.0f);
  set_bearings(scan, 30.0 * kDeg, 40.0 * kDeg, 5.0f);  // free only up and to the left
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_TRUE(sel->gap_found);
  EXPECT_GT(sel->target_index, 540u);  // high index = left
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 35.0 * kDeg, 0.5 * kDeg);
}

TEST(SelectGap, GapOnTheRightGivesNegativeBearing) {
  ScanInput scan = bridge_scan(1.0f);
  set_bearings(scan, -40.0 * kDeg, -30.0 * kDeg, 5.0f);
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, -35.0 * kDeg, 0.5 * kDeg);
}

TEST(SelectGap, FullCircleScanFromMinusPi) {
  ScanInput scan = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
  set_bearings(scan, 30.0 * kDeg, 40.0 * kDeg, 5.0f);
  // Also free directly BEHIND the car (wider gap), which the cone must ignore.
  set_bearings(scan, 150.0 * kDeg, M_PI, 5.0f);
  set_bearings(scan, -M_PI, -150.0 * kDeg, 5.0f);
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_TRUE(sel->gap_found);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 35.0 * kDeg, 1.0 * kDeg);
}

TEST(SelectGap, RearFacingLidarConeWrapsAcrossTheSeam) {
  // 360 scan from -pi with the LiDAR yawed pi: vehicle forward is laser ray 0, so the cone
  // spans the seam. A free gap symmetric about the seam is straight ahead for the car.
  ScanInput scan = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
  for (std::size_t i = 355; i < 360; ++i) {
    scan.ranges[i] = 5.0f;
  }
  for (std::size_t i = 0; i <= 5; ++i) {
    scan.ranges[i] = 5.0f;
  }
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, M_PI, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_TRUE(sel->gap_found);
  EXPECT_EQ(sel->gap_first, 355u);
  EXPECT_EQ(sel->gap_last, 5u);
  EXPECT_EQ(sel->target_index, 0u);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 0.0, 1e-9);
}

TEST(SelectGap, RearFacingLidarSignIsVehicleFrame) {
  // Laser bearings just counter-clockwise of -pi are the vehicle's front-left.
  ScanInput scan = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
  for (std::size_t i = 10; i <= 20; ++i) {
    scan.ranges[i] = 5.0f;
  }
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, M_PI, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 15.0 * kDeg, 1e-9);
}

TEST(SelectGap, ScanCentredOnForward) {
  ScanInput scan = make_scan(181, -M_PI / 2.0, kDeg, 1.0f);
  set_bearings(scan, -10.0 * kDeg, 10.0 * kDeg, 5.0f);
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_EQ(sel->target_index, 90u);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 0.0, 1e-9);
}

TEST(SelectGap, OffCentreScanUsesTheRealForwardRay) {
  // Covers [-0.2, 1.79] rad. The old code would have treated ray 100 (bearing 0.8 rad) as
  // straight ahead.
  ScanInput scan = make_scan(200, -0.2, 0.01, 1.0f);
  set_bearings(scan, -0.1, 0.1, 5.0f);
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_EQ(sel->target_index, 20u);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 0.0, 1e-9);
}

TEST(SelectGap, ConeIsAHalfAngleInRadians) {
  ScanInput scan = bridge_scan(1.0f);
  set_bearings(scan, 1.8, 2.0, 5.0f);  // outside a 1.57 rad cone
  auto sel = select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_FALSE(sel->gap_found);
  sel = select_gap(scan, to_double(scan), 1.2, 2.1, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_TRUE(sel->gap_found);
  EXPECT_NEAR(sel->target_vehicle_bearing_rad, 1.9, 0.01);
}

TEST(SelectGap, ForwardOutsideScanIsNullopt) {
  const ScanInput scan = make_scan(100, 0.5, 0.01);  // covers [0.5, 1.49] only
  EXPECT_FALSE(select_gap(scan, to_double(scan), 1.2, 1.57, 0.0, GapTarget::kCentre));
}

TEST(SelectGap, MismatchedRangeLengthIsNullopt) {
  const ScanInput scan = bridge_scan();
  EXPECT_FALSE(select_gap(scan, std::vector<double>(10, 5.0), 1.2, 1.57, 0.0, GapTarget::kCentre));
}

// Item d, tested both ways on the same scan.
TEST(SelectGap, CentreVersusDeepestTarget) {
  ScanInput scan = make_scan(21, -1.0, 0.1, 0.5f);
  // Gap spans rays 8..14 (bearings -0.2 .. 0.4); its deepest ray is 13.
  const float gap[] = {2.0f, 2.0f, 3.0f, 3.0f, 3.0f, 6.0f, 2.0f};
  for (std::size_t k = 0; k < 7; ++k) {
    scan.ranges[8 + k] = gap[k];
  }
  const auto centre = select_gap(scan, to_double(scan), 1.2, 1.0, 0.0, GapTarget::kCentre);
  const auto deepest = select_gap(scan, to_double(scan), 1.2, 1.0, 0.0, GapTarget::kDeepest);
  ASSERT_TRUE(centre && deepest);
  EXPECT_EQ(centre->target_index, 11u);
  EXPECT_EQ(deepest->target_index, 13u);
  EXPECT_EQ(deepest->gap_first, 8u);
  EXPECT_EQ(deepest->gap_last, 14u);
  EXPECT_NEAR(deepest->target_vehicle_bearing_rad, 0.3, 1e-9);
}

TEST(SelectGap, DeepestTargetTieBreaksTowardForward) {
  ScanInput scan = make_scan(21, -1.0, 0.1, 0.5f);
  for (std::size_t i = 6; i <= 14; ++i) {
    scan.ranges[i] = 2.0f;
  }
  scan.ranges[7] = 4.0f;   // bearing -0.3
  scan.ranges[12] = 4.0f;  // bearing +0.2, nearer forward
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.0, 0.0, GapTarget::kDeepest);
  ASSERT_TRUE(sel);
  EXPECT_EQ(sel->target_index, 12u);
}

TEST(SelectGap, WiderGapBeatsNearerNarrowerGap) {
  ScanInput scan = make_scan(21, -1.0, 0.1, 0.5f);
  scan.ranges[10] = 3.0f;  // single free ray dead ahead
  for (std::size_t i = 14; i <= 18; ++i) {
    scan.ranges[i] = 3.0f;
  }
  const auto sel = select_gap(scan, to_double(scan), 1.2, 1.0, 0.0, GapTarget::kCentre);
  ASSERT_TRUE(sel);
  EXPECT_EQ(sel->target_index, 16u);
}

// -- corner_blocked (item e) ------------------------------------------------------------------

constexpr double kSectorInner = M_PI / 2.0;
constexpr double kSectorOuter = 3.0 * M_PI / 4.0;

TEST(CornerBlocked, ChecksTheSideTheCarTurnsInto) {
  ScanInput scan = bridge_scan();
  set_bearings(scan, -M_PI, -M_PI / 2.0, 0.1f);  // right side jammed against a wall
  const auto r = to_double(scan);
  EXPECT_TRUE(corner_blocked(scan, r, -0.3, kSectorInner, kSectorOuter, 0.2, 0.0));
  EXPECT_FALSE(corner_blocked(scan, r, 0.3, kSectorInner, kSectorOuter, 0.2, 0.0));
}

// The old code checked the high-index (LEFT) sector for a right turn. Pin the correction: a
// blocked LEFT side must not veto a RIGHT turn, and must veto a LEFT one.
TEST(CornerBlocked, OldSideBugIsFixed) {
  ScanInput scan = bridge_scan();
  set_bearings(scan, M_PI / 2.0, M_PI, 0.1f);
  const auto r = to_double(scan);
  EXPECT_FALSE(corner_blocked(scan, r, -0.3, kSectorInner, kSectorOuter, 0.2, 0.0));
  EXPECT_TRUE(corner_blocked(scan, r, 0.3, kSectorInner, kSectorOuter, 0.2, 0.0));
}

// Ported (mirrored onto the corrected side).
TEST(CornerBlocked, ClearWhenAnyRayInTheSectorIsOpen) {
  ScanInput scan = bridge_scan(0.1f);
  set_bearings(scan, -2.0, -1.99, 5.0f);  // one open ray inside the right sector
  const auto r = to_double(scan);
  EXPECT_FALSE(corner_blocked(scan, r, -0.3, kSectorInner, kSectorOuter, 0.2, 0.0));
}

TEST(CornerBlocked, ZeroSteeringIsNeverBlocked) {
  const ScanInput scan = bridge_scan(0.1f);
  EXPECT_FALSE(corner_blocked(scan, to_double(scan), 0.0, kSectorInner, kSectorOuter, 0.2, 0.0));
}

TEST(CornerBlocked, SectorOutsideScanCoverageIsNotBlocked) {
  const ScanInput scan = make_scan(181, -M_PI / 2.0, kDeg, 0.1f);  // +/- 90 deg only
  EXPECT_FALSE(
      corner_blocked(scan, to_double(scan), 0.3, kSectorInner + 0.05, kSectorOuter, 0.2, 0.0));
}

TEST(CornerBlocked, RearFacingLidarUsesVehicleBearings) {
  ScanInput scan = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 5.0f, 12.0);
  // Vehicle right side (-135..-90 deg) is laser +45..+90 deg when the LiDAR is yawed pi.
  set_bearings(scan, 44.0 * kDeg, 91.0 * kDeg, 0.1f);
  const auto r = to_double(scan);
  EXPECT_TRUE(corner_blocked(scan, r, -0.3, kSectorInner, kSectorOuter, 0.2, M_PI));
  EXPECT_FALSE(corner_blocked(scan, r, 0.3, kSectorInner, kSectorOuter, 0.2, M_PI));
}

// -- steering law and low-pass (item e) ---------------------------------------------------------

TEST(SteeringFromBearing, GainSignAndClamp) {
  EXPECT_DOUBLE_EQ(steering_from_bearing(0.1, 2.0, 0.4189), 0.2);
  EXPECT_DOUBLE_EQ(steering_from_bearing(-0.1, 2.0, 0.4189), -0.2);
  EXPECT_DOUBLE_EQ(steering_from_bearing(1.0, 1.0, 0.4189), 0.4189);
  EXPECT_DOUBLE_EQ(steering_from_bearing(-1.0, 1.0, 0.4189), -0.4189);
  EXPECT_DOUBLE_EQ(steering_from_bearing(std::nan(""), 1.0, 0.4189), 0.0);
}

TEST(FirstOrderLowPass, FirstSampleWithNoElapsedTimeHoldsAtZero) {
  FirstOrderLowPass f(0.1);
  EXPECT_DOUBLE_EQ(f.update(0.3, 0.0), 0.0);
}

TEST(FirstOrderLowPass, StepResponseMatchesTheDiscreteFormula) {
  FirstOrderLowPass f(0.1);
  const double dt = 0.02;
  const double a = dt / (0.1 + dt);
  EXPECT_NEAR(f.update(1.0, dt), a, 1e-12);
  EXPECT_NEAR(f.update(1.0, dt), a + (1.0 - a) * a, 1e-12);
  for (int i = 0; i < 200; ++i) {
    f.update(1.0, dt);
  }
  EXPECT_NEAR(f.output(), 1.0, 1e-9);
}

TEST(FirstOrderLowPass, ReachesAboutSixtyThreePercentAfterOneTimeConstant) {
  FirstOrderLowPass f(0.1);
  for (int i = 0; i < 100; ++i) {
    f.update(1.0, 0.001);
  }
  EXPECT_NEAR(f.output(), 1.0 - std::exp(-1.0), 0.01);
}

TEST(FirstOrderLowPass, ZeroTimeConstantPassesThrough) {
  FirstOrderLowPass f(0.0);
  EXPECT_DOUBLE_EQ(f.update(0.25, 0.02), 0.25);
}

TEST(FirstOrderLowPass, GarbageInputsHoldTheOutput) {
  FirstOrderLowPass f(0.1);
  f.update(1.0, 0.02);
  const double held = f.output();
  EXPECT_DOUBLE_EQ(f.update(std::nan(""), 0.02), held);
  EXPECT_DOUBLE_EQ(f.update(1.0, -0.02), held);
  EXPECT_DOUBLE_EQ(f.update(1.0, std::numeric_limits<double>::infinity()), held);
  f.reset();
  EXPECT_DOUBLE_EQ(f.output(), 0.0);
}

// -- speed law (item f) -----------------------------------------------------------------------

TEST(ReactiveSpeed, RangeBasedSpeedClamps) {
  EXPECT_DOUBLE_EQ(range_based_speed(2.0, 1.0, 0.5, 3.0), 2.0);
  EXPECT_DOUBLE_EQ(range_based_speed(0.1, 1.0, 0.5, 3.0), 0.5);
  EXPECT_DOUBLE_EQ(range_based_speed(10.0, 1.0, 0.5, 3.0), 3.0);
  EXPECT_DOUBLE_EQ(range_based_speed(std::nan(""), 1.0, 0.5, 3.0), 0.5);
  EXPECT_DOUBLE_EQ(range_based_speed(-1.0, 1.0, 0.5, 3.0), 0.5);
}

TEST(ReactiveSpeed, SteeringSlowdown) {
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, 0.0, 0.5, 0.4), 2.0);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, 0.4, 0.5, 0.4), 1.0);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, -0.2, 0.5, 0.4), 1.5);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, 0.8, 0.5, 0.4), 1.0);  // beyond lock
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, 0.4, 1.0, 0.4), 0.0);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(std::nan(""), 0.0, 0.5, 0.4), 0.0);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, std::nan(""), 0.5, 0.4), 0.0);
  EXPECT_DOUBLE_EQ(apply_steering_slowdown(2.0, 0.1, 0.5, 0.0), 0.0);
}

// -- GapFollower end to end --------------------------------------------------------------------

GapFollowConfig default_config() {
  GapFollowConfig c;
  c.half_width_m = 0.155;
  c.max_steering_angle_rad = 0.4189;
  c.safety_margin_m = 0.05;
  c.clip_max_range_m = 3.5;
  c.disparity_threshold_m = 0.5;
  c.free_space_threshold_m = 1.2;
  c.cone_half_angle_rad = 1.57;
  c.steering_gain = 1.0;
  c.corner_sector_inner_rad = kSectorInner;
  c.corner_sector_outer_rad = kSectorOuter;
  c.corner_min_clearance_m = 0.2;
  return c;
}

TEST(GapFollower, LeftGapSteersLeft) {
  ScanInput scan = bridge_scan(1.0f);
  set_bearings(scan, 10.0 * kDeg, 50.0 * kDeg, 5.0f);
  GapFollower follower(default_config());
  const GapFollowResult r = follower.process(scan);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.gap_found);
  EXPECT_GT(r.target_vehicle_bearing_rad, 0.0);
  EXPECT_GT(r.steering_rad, 0.0);
  EXPECT_LE(r.steering_rad, 0.4189);
  EXPECT_GT(r.target_range_m, 1.2);
}

TEST(GapFollower, CornerOverrideZeroesSteering) {
  ScanInput scan = bridge_scan(1.0f);
  set_bearings(scan, -50.0 * kDeg, -10.0 * kDeg, 5.0f);  // gap to the right
  set_bearings(scan, -M_PI, -M_PI / 2.0, 0.1f);          // right side jammed
  GapFollower follower(default_config());
  const GapFollowResult r = follower.process(scan);
  ASSERT_TRUE(r.valid);
  EXPECT_LT(r.target_vehicle_bearing_rad, 0.0);
  EXPECT_TRUE(r.corner_blocked);
  EXPECT_DOUBLE_EQ(r.steering_rad, 0.0);
}

TEST(GapFollower, NanNeverReachesTheOutput) {
  ScanInput scan = bridge_scan(kNaNF);
  set_bearings(scan, -5.0 * kDeg, 5.0 * kDeg, kInfF);
  set_bearings(scan, 60.0 * kDeg, 61.0 * kDeg, 0.8f);
  GapFollower follower(default_config());
  const GapFollowResult r = follower.process(scan);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(std::isfinite(r.steering_rad));
  EXPECT_TRUE(std::isfinite(r.target_range_m));
  EXPECT_TRUE(std::isfinite(r.target_vehicle_bearing_rad));
}

TEST(GapFollower, AllInvalidScanIsNotValid) {
  GapFollower follower(default_config());
  EXPECT_FALSE(follower.process(bridge_scan(kNaNF)).valid);
  EXPECT_FALSE(follower.process(bridge_scan(0.0f)).valid);
}

TEST(GapFollower, UnusableGeometryIsNotValid) {
  ScanInput scan = bridge_scan();
  scan.angle_increment = 0.0;
  GapFollower follower(default_config());
  EXPECT_FALSE(follower.process(scan).valid);
}

TEST(GapFollower, AllFreeScanDrivesStraight) {
  GapFollower follower(default_config());
  const GapFollowResult r = follower.process(bridge_scan(kInfF));
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.steering_rad, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(r.target_range_m, 3.5);
}

}  // namespace
}  // namespace racer_control
