// L1 tests for racer_control's follow-the-gap core (GitHub issue 26). Ported cases from the
// old Python test_gap_logic.py are marked "Ported"; where the expected value changed, the
// comment says why. New cases cover the maths fixes a (forward ray and cone in radians), b
// (invalid returns), c (disparity bubble), d (deepest-ray target), and e (steering law and
// low-pass filter). The forward-preference and gap-switch-margin cases (2026-10-06 floor
// test) have their own section after select_gap's.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <utility>
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

// -- select_gap forward preference and gap switch margin (2026-10-06 floor test) -----------

// Copy of select_gap as it was before forward_preference existed (main at 26f2503), kept here
// as the reference for the "preference 0 is bit-identical" tests. Do not edit it to match the
// production code; it is the old behaviour.
std::optional<GapSelection> legacy_select_gap(const ScanInput& geometry,
                                              const std::vector<double>& ranges,
                                              double free_space_threshold_m,
                                              double cone_half_angle_rad,
                                              double laser_yaw_offset_rad, GapTarget target) {
  if (!is_usable(geometry) || ranges.size() != geometry.ranges.size() ||
      !std::isfinite(cone_half_angle_rad) || cone_half_angle_rad < 0.0) {
    return std::nullopt;
  }
  const auto forward = index_for_vehicle_bearing(geometry, 0.0, laser_yaw_offset_rad);
  if (!forward) {
    return std::nullopt;
  }
  const std::size_t n = ranges.size();
  const auto nn = static_cast<std::int64_t>(n);
  const auto f = static_cast<std::int64_t>(*forward);
  auto half_rays =
      static_cast<std::int64_t>(std::floor(cone_half_angle_rad / geometry.angle_increment + 1e-9));
  const bool wraps = is_full_circle(geometry);
  std::int64_t lo = 0;
  std::int64_t hi = 0;
  if (wraps) {
    half_rays = std::min(half_rays, (nn - 1) / 2);
    lo = -half_rays;
    hi = half_rays;
  } else {
    lo = std::max(-half_rays, -f);
    hi = std::min(half_rays, nn - 1 - f);
  }
  const auto offset_of = [&](std::size_t j) { return lo + static_cast<std::int64_t>(j); };
  const auto index_of = [&](std::size_t j) {
    std::int64_t idx = f + offset_of(j);
    if (wraps) {
      idx = ((idx % nn) + nn) % nn;
    }
    return static_cast<std::size_t>(idx);
  };
  const auto bearing_of = [&](std::size_t index) {
    return wrap_angle(laser_bearing_of_index(geometry, index) + laser_yaw_offset_rad);
  };
  GapSelection result;
  result.target_index = *forward;
  result.gap_first = *forward;
  result.gap_last = *forward;
  result.target_vehicle_bearing_rad = bearing_of(*forward);
  const auto m = static_cast<std::size_t>(hi - lo + 1);
  bool found = false;
  std::size_t best_start = 0;
  std::size_t best_end = 0;
  std::size_t best_len = 0;
  std::int64_t best_centre_dist = 0;
  std::size_t j = 0;
  while (j < m) {
    if (!(ranges[index_of(j)] > free_space_threshold_m)) {
      ++j;
      continue;
    }
    const std::size_t start = j;
    while (j < m && ranges[index_of(j)] > free_space_threshold_m) {
      ++j;
    }
    const std::size_t end = j - 1;
    const std::size_t len = end - start + 1;
    const std::int64_t centre_dist = std::abs(offset_of((start + end) / 2));
    if (!found || len > best_len || (len == best_len && centre_dist < best_centre_dist)) {
      found = true;
      best_start = start;
      best_end = end;
      best_len = len;
      best_centre_dist = centre_dist;
    }
  }
  if (!found) {
    return result;
  }
  std::size_t target_j = (best_start + best_end) / 2;
  if (target == GapTarget::kDeepest) {
    double best_range = -1.0;
    std::int64_t best_dist = 0;
    for (std::size_t k = best_start; k <= best_end; ++k) {
      const double r = ranges[index_of(k)];
      const std::int64_t dist = std::abs(offset_of(k));
      if (r > best_range || (r == best_range && dist < best_dist)) {
        best_range = r;
        best_dist = dist;
        target_j = k;
      }
    }
  }
  result.gap_found = true;
  result.gap_first = index_of(best_start);
  result.gap_last = index_of(best_end);
  result.target_index = index_of(target_j);
  result.target_vehicle_bearing_rad = bearing_of(result.target_index);
  return result;
}

// Exact equality on every field, bearing included (bit-identical, not "near").
void expect_identical(const std::optional<GapSelection>& a, const std::optional<GapSelection>& b) {
  ASSERT_EQ(a.has_value(), b.has_value());
  if (!a) {
    return;
  }
  EXPECT_EQ(a->gap_found, b->gap_found);
  EXPECT_EQ(a->target_index, b->target_index);
  EXPECT_EQ(a->gap_first, b->gap_first);
  EXPECT_EQ(a->gap_last, b->gap_last);
  EXPECT_EQ(a->target_vehicle_bearing_rad, b->target_vehicle_bearing_rad);
}

struct SelectCase {
  ScanInput scan;
  double free_space_threshold_m;
  double cone_half_angle_rad;
  double laser_yaw_offset_rad;
};

// The scans of the select_gap tests above, rebuilt with the same arguments.
std::vector<SelectCase> existing_select_gap_fixtures() {
  std::vector<SelectCase> cases;
  {
    ScanInput s = old_twelve_ray_scan();
    s.ranges[3] = 2.0f;
    s.ranges[5] = s.ranges[6] = s.ranges[7] = 2.0f;
    cases.push_back({s, 1.0, 0.3, 0.0});
  }
  {
    ScanInput s = old_twelve_ray_scan();
    s.ranges[3] = s.ranges[4] = 2.0f;
    s.ranges[7] = s.ranges[8] = 2.0f;
    cases.push_back({s, 1.0, 0.3, 0.0});
  }
  cases.push_back({old_twelve_ray_scan(), 1.0, 0.3, 0.0});
  {
    ScanInput s = bridge_scan(1.0f);
    set_bearings(s, 30.0 * kDeg, 40.0 * kDeg, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
  }
  {
    ScanInput s = bridge_scan(1.0f);
    set_bearings(s, -40.0 * kDeg, -30.0 * kDeg, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
  }
  {
    ScanInput s = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
    set_bearings(s, 30.0 * kDeg, 40.0 * kDeg, 5.0f);
    set_bearings(s, 150.0 * kDeg, M_PI, 5.0f);
    set_bearings(s, -M_PI, -150.0 * kDeg, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
  }
  {
    ScanInput s = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
    for (std::size_t i = 355; i < 360; ++i) {
      s.ranges[i] = 5.0f;
    }
    for (std::size_t i = 0; i <= 5; ++i) {
      s.ranges[i] = 5.0f;
    }
    cases.push_back({s, 1.2, 1.57, M_PI});
  }
  {
    ScanInput s = make_scan(360, -M_PI, 2.0 * M_PI / 360.0, 1.0f, 12.0);
    for (std::size_t i = 10; i <= 20; ++i) {
      s.ranges[i] = 5.0f;
    }
    cases.push_back({s, 1.2, 1.57, M_PI});
  }
  {
    ScanInput s = make_scan(181, -M_PI / 2.0, kDeg, 1.0f);
    set_bearings(s, -10.0 * kDeg, 10.0 * kDeg, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
  }
  {
    ScanInput s = make_scan(200, -0.2, 0.01, 1.0f);
    set_bearings(s, -0.1, 0.1, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
  }
  {
    ScanInput s = bridge_scan(1.0f);
    set_bearings(s, 1.8, 2.0, 5.0f);
    cases.push_back({s, 1.2, 1.57, 0.0});
    cases.push_back({s, 1.2, 2.1, 0.0});
  }
  cases.push_back({make_scan(100, 0.5, 0.01), 1.2, 1.57, 0.0});
  {
    ScanInput s = make_scan(21, -1.0, 0.1, 0.5f);
    const float gap[] = {2.0f, 2.0f, 3.0f, 3.0f, 3.0f, 6.0f, 2.0f};
    for (std::size_t k = 0; k < 7; ++k) {
      s.ranges[8 + k] = gap[k];
    }
    cases.push_back({s, 1.2, 1.0, 0.0});
  }
  {
    ScanInput s = make_scan(21, -1.0, 0.1, 0.5f);
    for (std::size_t i = 6; i <= 14; ++i) {
      s.ranges[i] = 2.0f;
    }
    s.ranges[7] = 4.0f;
    s.ranges[12] = 4.0f;
    cases.push_back({s, 1.2, 1.0, 0.0});
  }
  {
    ScanInput s = make_scan(21, -1.0, 0.1, 0.5f);
    s.ranges[10] = 3.0f;
    for (std::size_t i = 14; i <= 18; ++i) {
      s.ranges[i] = 3.0f;
    }
    cases.push_back({s, 1.2, 1.0, 0.0});
  }
  return cases;
}

TEST(SelectGapForwardPreference, ZeroIsBitIdenticalToTheOldCodeOnTheExistingFixtures) {
  const auto cases = existing_select_gap_fixtures();
  ASSERT_EQ(cases.size(), 16u);
  for (std::size_t c = 0; c < cases.size(); ++c) {
    const SelectCase& sc = cases[c];
    const auto r = to_double(sc.scan);
    for (const GapTarget target : {GapTarget::kCentre, GapTarget::kDeepest}) {
      SCOPED_TRACE(c);
      const auto legacy =
          legacy_select_gap(sc.scan, r, sc.free_space_threshold_m, sc.cone_half_angle_rad,
                            sc.laser_yaw_offset_rad, target);
      // Default argument, and explicit zeros with a previous bearing set (ignored while the
      // margin is 0): both must be the old behaviour.
      expect_identical(select_gap(sc.scan, r, sc.free_space_threshold_m, sc.cone_half_angle_rad,
                                  sc.laser_yaw_offset_rad, target),
                       legacy);
      GapPreference zero;
      zero.previous_target_vehicle_bearing_rad = 0.7;
      expect_identical(select_gap(sc.scan, r, sc.free_space_threshold_m, sc.cone_half_angle_rad,
                                  sc.laser_yaw_offset_rad, target, zero),
                       legacy);
    }
  }
}

TEST(SelectGapForwardPreference, ZeroIsBitIdenticalToTheOldCodeOnRandomScans) {
  // Seeded, so deterministic. Blocky random scans over the geometries the fixtures use, both
  // yaws, both targets: many gaps per scan, including equal-width ties.
  std::mt19937 rng(20261006u);
  std::uniform_real_distribution<double> range(0.2, 4.0);
  std::uniform_int_distribution<int> block(1, 12);
  struct Geometry {
    std::size_t n;
    double angle_min;
    double inc;
  };
  const Geometry geometries[] = {
      {1080, -2.35, 4.7 / 1079.0}, {360, -M_PI, 2.0 * M_PI / 360.0},
      {720, -M_PI, M_PI / 360.0},  {181, -M_PI / 2.0, kDeg},
      {200, -0.2, 0.01},
  };
  int compared = 0;
  for (const Geometry& g : geometries) {
    for (int trial = 0; trial < 40; ++trial) {
      ScanInput s = make_scan(g.n, g.angle_min, g.inc, 1.0f);
      std::size_t i = 0;
      while (i < g.n) {
        const auto len = static_cast<std::size_t>(block(rng));
        const auto value = static_cast<float>(range(rng));
        for (std::size_t k = i; k < std::min(g.n, i + len); ++k) {
          s.ranges[k] = value;
        }
        i += len;
      }
      const auto r = to_double(s);
      for (const double yaw : {0.0, M_PI}) {
        for (const GapTarget target : {GapTarget::kCentre, GapTarget::kDeepest}) {
          SCOPED_TRACE(::testing::Message() << g.n << " rays, trial " << trial << ", yaw " << yaw);
          expect_identical(select_gap(s, r, 1.5, 1.57, yaw, target),
                           legacy_select_gap(s, r, 1.5, 1.57, yaw, target));
          ++compared;
        }
      }
    }
  }
  EXPECT_EQ(compared, 5 * 40 * 2 * 2);
}

// The floor geometry, idealised: the real C1's 360 degree scan (angle_min -pi, mounted
// backwards, yaw pi). The car sits in a 1.2 m lane (walls at y = +/- 0.6 m in the vehicle
// frame, x forward). One wall is continuous; the other has a 2 m opening whose centre is at
// 70 degrees from the car (x from -0.78 m to 1.22 m on the wall line), through which the room
// is open (no return). Ray-cast against the wall segments; misses read as inf.
constexpr double kLaneHalfWidth = 0.6;
constexpr double kTan70Deg = 2.7474774194546216;
constexpr double kOpeningCentreX = kLaneHalfWidth / kTan70Deg;
constexpr double kOpeningHalfLength = 1.0;

struct Segment {
  double x0, y0, x1, y1;
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
    const double t = (s.x0 * ey - s.y0 * ex) / denom;  // along the ray
    const double u = (s.x0 * dy - s.y0 * dx) / denom;  // along the segment
    if (t > 0.0 && u >= 0.0 && u <= 1.0) {
      best = std::min(best, t);
    }
  }
  return best;
}

// side = +1: opening on the LEFT; side = -1: mirrored, opening on the RIGHT.
ScanInput lane_with_side_opening(double side) {
  const double yaw = M_PI;
  ScanInput scan = make_scan(720, -M_PI, 2.0 * M_PI / 720.0, 0.0f, 12.0);
  const double y_open = side * kLaneHalfWidth;
  const double y_wall = -side * kLaneHalfWidth;
  const std::vector<Segment> segments{
      {-3.0, y_wall, 20.0, y_wall},                                  // continuous wall
      {-3.0, y_open, kOpeningCentreX - kOpeningHalfLength, y_open},  // behind the opening
      {kOpeningCentreX + kOpeningHalfLength, y_open, 20.0, y_open},  // lane ahead
  };
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double vehicle_bearing = laser_bearing_of_index(scan, i) + yaw;
    scan.ranges[i] = static_cast<float>(ray_cast(vehicle_bearing, segments));
  }
  return scan;
}

GapFollowConfig floor_config(double forward_preference) {
  GapFollowConfig c;
  c.half_width_m = 0.155;
  c.max_steering_angle_rad = 0.4189;
  c.safety_margin_m = 0.05;
  c.clip_max_range_m = 3.5;
  c.disparity_threshold_m = 0.5;
  c.free_space_threshold_m = 1.5;  // gap_follow_node's default
  c.cone_half_angle_rad = 1.57;    // gap_follow_node's default
  c.laser_yaw_offset_rad = M_PI;
  c.steering_gain = 1.0;
  c.corner_sector_inner_rad = M_PI / 2.0;
  c.corner_sector_outer_rad = 3.0 * M_PI / 4.0;
  c.corner_min_clearance_m = 0.2;
  c.forward_preference = forward_preference;
  return c;
}

TEST(SelectGapForwardPreference, FloorLaneVersusLeftOpening) {
  const ScanInput scan = lane_with_side_opening(+1.0);
  // Today's behaviour: the opening (about 35 to 90 degrees after the disparity bubble, about
  // 55 degrees wide) beats the lane (about +/- 23.6 degrees, about 47 degrees wide).
  GapFollower widest(floor_config(0.0));
  const GapFollowResult today = widest.process(scan);
  ASSERT_TRUE(today.valid);
  ASSERT_TRUE(today.gap_found);
  EXPECT_GT(today.target_vehicle_bearing_rad, 50.0 * kDeg);
  EXPECT_GT(today.steering_rad, 0.0);
  // With forward_preference 0.6 the opening's centre near 62 degrees costs it about 32
  // percent and the lane ahead wins.
  GapFollower forward(floor_config(0.6));
  const GapFollowResult lane = forward.process(scan);
  ASSERT_TRUE(lane.valid);
  ASSERT_TRUE(lane.gap_found);
  EXPECT_NEAR(lane.target_vehicle_bearing_rad, 0.0, 2.0 * kDeg);
  EXPECT_NEAR(lane.steering_rad, 0.0, 2.0 * kDeg);
}

TEST(SelectGapForwardPreference, FloorLaneVersusRightOpening) {
  const ScanInput scan = lane_with_side_opening(-1.0);
  GapFollower widest(floor_config(0.0));
  const GapFollowResult today = widest.process(scan);
  ASSERT_TRUE(today.valid);
  ASSERT_TRUE(today.gap_found);
  EXPECT_LT(today.target_vehicle_bearing_rad, -50.0 * kDeg);
  EXPECT_LT(today.steering_rad, 0.0);
  GapFollower forward(floor_config(0.6));
  const GapFollowResult lane = forward.process(scan);
  ASSERT_TRUE(lane.valid);
  ASSERT_TRUE(lane.gap_found);
  EXPECT_NEAR(lane.target_vehicle_bearing_rad, 0.0, 2.0 * kDeg);
  EXPECT_NEAR(lane.steering_rad, 0.0, 2.0 * kDeg);
}

TEST(SelectGapForwardPreference, LeftAndRightAreMirrorImages) {
  for (const double p : {0.0, 0.3, 0.6, 1.0}) {
    GapFollower left(floor_config(p));
    GapFollower right(floor_config(p));
    const GapFollowResult l = left.process(lane_with_side_opening(+1.0));
    const GapFollowResult r = right.process(lane_with_side_opening(-1.0));
    ASSERT_TRUE(l.valid && r.valid);
    EXPECT_NEAR(l.target_vehicle_bearing_rad, -r.target_vehicle_bearing_rad, 1.0 * kDeg) << p;
  }
}

// Synthetic ranges straight into select_gap (no disparity extension) so the scores are exact:
// 61 rays at 0.1 rad from -3.0 rad, forward is ray 30 (ray k has bearing (k - 30) / 10), cone
// +/- 3.0 rad.
ScanInput scoring_scan() { return make_scan(61, -3.0, 0.1, 0.5f); }

void free_rays(ScanInput& scan, std::size_t first, std::size_t last) {
  for (std::size_t i = first; i <= last; ++i) {
    scan.ranges[i] = 3.0f;
  }
}

std::optional<GapSelection> select_scoring(const ScanInput& scan, GapTarget target,
                                           const GapPreference& pref) {
  return select_gap(scan, to_double(scan), 1.0, 3.0, 0.0, target, pref);
}

GapPreference with_forward_preference(double p) {
  GapPreference pref;
  pref.forward_preference = p;
  return pref;
}

TEST(SelectGapForwardPreference, ScoreFollowsTheDocumentedFormula) {
  // Gap A: rays 28..32, width 0.5 rad, centred dead ahead, factor 1.
  // Gap B: rays 36..44, width 0.9 rad, centred at 1.0 rad, factor 1 - p (1 - cos 1.0)
  //        = 1 - 0.4597 p. B wins while 0.9 (1 - 0.4597 p) > 0.5, i.e. p < 0.9668.
  ScanInput scan = scoring_scan();
  free_rays(scan, 28, 32);
  free_rays(scan, 36, 44);
  for (const auto& [p, expected] : std::vector<std::pair<double, std::size_t>>{
           {0.0, 40u}, {0.5, 40u}, {0.95, 40u}, {0.98, 30u}, {1.0, 30u}}) {
    const auto sel = select_scoring(scan, GapTarget::kCentre, with_forward_preference(p));
    ASSERT_TRUE(sel && sel->gap_found) << p;
    EXPECT_EQ(sel->target_index, expected) << p;
  }
}

TEST(SelectGapForwardPreference, FullPreferenceAlmostZeroesASideGap) {
  // Rays 41..49 are centred at 1.5 rad (86 degrees): with p = 1 the factor is cos(1.5) =
  // 0.071, so the 0.9 rad gap scores 0.064 and loses to a single free ray dead ahead (0.1).
  ScanInput scan = scoring_scan();
  scan.ranges[30] = 3.0f;
  free_rays(scan, 41, 49);
  const auto sel = select_scoring(scan, GapTarget::kCentre, with_forward_preference(1.0));
  ASSERT_TRUE(sel && sel->gap_found);
  EXPECT_EQ(sel->target_index, 30u);
}

TEST(SelectGapForwardPreference, FactorClampsAtZeroPastNinetyDegrees) {
  // Two gaps behind the car's side, centred at 2.1 rad and 2.75 rad (the second wider). With
  // p = 1 both factors would be negative; clamped, both score 0 and the tie goes toward
  // forward, so the wider, further-back gap does not win.
  ScanInput scan = scoring_scan();
  free_rays(scan, 50, 52);
  free_rays(scan, 55, 60);
  const auto sel = select_scoring(scan, GapTarget::kCentre, with_forward_preference(1.0));
  ASSERT_TRUE(sel && sel->gap_found);
  EXPECT_EQ(sel->target_index, 51u);
}

TEST(SelectGapForwardPreference, TieBreakTowardForwardStillHolds) {
  // Equal widths centred at -0.5 rad (rays 24..26) and +0.3 rad (rays 32..34). The scores tie
  // only at p = 0, where the tie-break must pick the gap nearer forward; with p > 0 that gap
  // also scores higher.
  ScanInput scan = scoring_scan();
  free_rays(scan, 24, 26);
  free_rays(scan, 32, 34);
  for (const double p : {0.0, 0.5, 1.0}) {
    const auto sel = select_scoring(scan, GapTarget::kCentre, with_forward_preference(p));
    ASSERT_TRUE(sel && sel->gap_found) << p;
    EXPECT_EQ(sel->target_index, 33u) << p;
  }
  // Mirror-image gaps of different widths (rays 18..22 at -1.0 rad, rays 37..43 at +1.0 rad)
  // get the same factor for any p, so the wider one wins at every p.
  ScanInput mirrored = scoring_scan();
  free_rays(mirrored, 18, 22);
  free_rays(mirrored, 37, 43);
  for (const double p : {0.0, 0.5, 1.0}) {
    const auto sel = select_scoring(mirrored, GapTarget::kCentre, with_forward_preference(p));
    ASSERT_TRUE(sel && sel->gap_found) << p;
    EXPECT_EQ(sel->target_index, 40u) << p;
  }
}

TEST(SelectGapForwardPreference, DeepestTargetIsChosenInsideThePreferredGap) {
  // With p = 1: forward gap 0.5 vs side gap 0.9 * cos(1.0) = 0.486, forward wins, and the
  // target is the deepest ray of the forward gap, not of the side gap.
  ScanInput scan = scoring_scan();
  free_rays(scan, 28, 32);
  scan.ranges[29] = 4.0f;
  free_rays(scan, 36, 44);
  scan.ranges[43] = 5.0f;
  const auto sel = select_scoring(scan, GapTarget::kDeepest, with_forward_preference(1.0));
  ASSERT_TRUE(sel && sel->gap_found);
  EXPECT_EQ(sel->gap_first, 28u);
  EXPECT_EQ(sel->gap_last, 32u);
  EXPECT_EQ(sel->target_index, 29u);
}

TEST(SelectGapForwardPreference, OutOfRangeFieldsAreRejected) {
  ScanInput scan = scoring_scan();
  free_rays(scan, 28, 32);
  for (const double bad : {-0.01, 1.01, std::nan(""), std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(select_scoring(scan, GapTarget::kCentre, with_forward_preference(bad))) << bad;
    GapPreference margin;
    margin.switch_margin = bad;
    EXPECT_FALSE(select_scoring(scan, GapTarget::kCentre, margin)) << bad;
  }
  for (const double ok : {0.0, 1.0}) {
    GapPreference pref;
    pref.forward_preference = ok;
    pref.switch_margin = ok;
    EXPECT_TRUE(select_scoring(scan, GapTarget::kCentre, pref)) << ok;
  }
  // GapFollower passes the config through, so a bad value gives an invalid result (no
  // command), never a silently clamped one.
  GapFollower bad_preference(floor_config(1.5));
  EXPECT_FALSE(bad_preference.process(lane_with_side_opening(+1.0)).valid);
  GapFollowConfig c = floor_config(0.0);
  c.gap_switch_margin = -0.5;
  GapFollower bad_margin(c);
  EXPECT_FALSE(bad_margin.process(lane_with_side_opening(+1.0)).valid);
}

// Two gaps: A = rays 20..24 (5 rays, centre ray 22 at -0.8 rad), B = rays 36..41 (6 rays,
// centre ray 38 at +0.8 rad). At p = 0, B's score beats A's by 20 percent.
ScanInput near_tie_scan() {
  ScanInput scan = scoring_scan();
  free_rays(scan, 20, 24);
  free_rays(scan, 36, 41);
  return scan;
}

std::optional<GapSelection> select_with_margin(const ScanInput& scan, double margin,
                                               std::optional<double> previous) {
  GapPreference pref;
  pref.switch_margin = margin;
  pref.previous_target_vehicle_bearing_rad = previous;
  return select_scoring(scan, GapTarget::kCentre, pref);
}

TEST(SelectGapSwitchMargin, KeepsThePreviousGapOnANearTie) {
  // Previous target -1.0 rad is ray 20, inside A. B is only 20 percent better, inside a 25
  // percent margin, so A is kept and its own centre is targeted.
  const auto kept = select_with_margin(near_tie_scan(), 0.25, -1.0);
  ASSERT_TRUE(kept && kept->gap_found);
  EXPECT_EQ(kept->gap_first, 20u);
  EXPECT_EQ(kept->gap_last, 24u);
  EXPECT_EQ(kept->target_index, 22u);
}

TEST(SelectGapSwitchMargin, SwitchesOnAClearWin) {
  // A 10 percent margin lets the 20 percent better gap through.
  const auto switched = select_with_margin(near_tie_scan(), 0.1, -1.0);
  ASSERT_TRUE(switched && switched->gap_found);
  EXPECT_EQ(switched->gap_first, 36u);
  // A much wider B (rays 36..47, 2.4x A) beats even the largest margin.
  ScanInput wide = scoring_scan();
  free_rays(wide, 20, 24);
  free_rays(wide, 36, 47);
  const auto clear = select_with_margin(wide, 1.0, -1.0);
  ASSERT_TRUE(clear && clear->gap_found);
  EXPECT_EQ(clear->gap_first, 36u);
}

TEST(SelectGapSwitchMargin, IsOffWithZeroMarginOrNoContainingGap) {
  const ScanInput scan = near_tie_scan();
  // Margin 0: the previous bearing is ignored and the best gap wins.
  const auto off = select_with_margin(scan, 0.0, -1.0);
  ASSERT_TRUE(off && off->gap_found);
  EXPECT_EQ(off->gap_first, 36u);
  // Previous target on a blocked ray (ray 30): no incumbent, plain best score.
  const auto stale = select_with_margin(scan, 0.5, 0.0);
  ASSERT_TRUE(stale && stale->gap_found);
  EXPECT_EQ(stale->gap_first, 36u);
  // No previous target: plain best score.
  const auto none = select_with_margin(scan, 0.5, std::nullopt);
  ASSERT_TRUE(none && none->gap_found);
  EXPECT_EQ(none->gap_first, 36u);
  // A previous target on a gap's edge ray (ray 24) still counts as inside it.
  const auto edge = select_with_margin(scan, 0.25, -0.6);
  ASSERT_TRUE(edge && edge->gap_found);
  EXPECT_EQ(edge->gap_first, 20u);
}

TEST(SelectGapSwitchMargin, WorksWithForwardPreference) {
  // The margin compares preference-weighted scores. Gap A: rays 26..30, midpoint -0.2 rad,
  // width 0.5. Gap B: rays 32..37, midpoint +0.45 rad, width 0.6. With p = 1:
  // A = 0.5 cos(0.2) = 0.490, B = 0.6 cos(0.45) = 0.540, so B is better by 10.2 percent.
  ScanInput scan = scoring_scan();
  free_rays(scan, 26, 30);
  free_rays(scan, 32, 37);
  GapPreference pref = with_forward_preference(1.0);
  pref.previous_target_vehicle_bearing_rad = -0.2;  // ray 28, inside A
  pref.switch_margin = 0.15;
  auto sel = select_scoring(scan, GapTarget::kCentre, pref);
  ASSERT_TRUE(sel && sel->gap_found);
  EXPECT_EQ(sel->gap_first, 26u);
  pref.switch_margin = 0.05;
  sel = select_scoring(scan, GapTarget::kCentre, pref);
  ASSERT_TRUE(sel && sel->gap_found);
  EXPECT_EQ(sel->gap_first, 32u);
}

TEST(SelectGapSwitchMargin, GapFollowerRemembersTheLastTargetAcrossScans) {
  // Scan 1: only A is open, so A is chosen. Scan 2: both open, B 20 percent better. With a
  // 25 percent margin the follower stays in A; without one it moves to B.
  ScanInput first = scoring_scan();
  free_rays(first, 20, 24);
  const ScanInput second = near_tie_scan();
  GapFollowConfig c;
  c.half_width_m = 0.0;  // no disparity extension: the gaps stay exactly as built
  c.max_steering_angle_rad = 0.4189;
  c.clip_max_range_m = 10.0;
  c.disparity_threshold_m = 100.0;
  c.free_space_threshold_m = 1.0;
  c.cone_half_angle_rad = 3.0;
  c.steering_gain = 1.0;
  c.gap_switch_margin = 0.25;
  GapFollower sticky(c);
  ASSERT_TRUE(sticky.process(first).valid);
  const GapFollowResult stayed = sticky.process(second);
  ASSERT_TRUE(stayed.valid);
  EXPECT_NEAR(stayed.target_vehicle_bearing_rad, -0.8, 1e-9);  // ray 22
  c.gap_switch_margin = 0.0;
  GapFollower plain(c);
  ASSERT_TRUE(plain.process(first).valid);
  const GapFollowResult moved = plain.process(second);
  ASSERT_TRUE(moved.valid);
  EXPECT_NEAR(moved.target_vehicle_bearing_rad, 0.8, 1e-9);  // ray 38
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

// -- Swept-path clamp (2026-10-06 floor finding) ----------------------------------------------

// The real C1: 360 degrees from angle_min -pi, mounted backwards (yaw pi), 0.5 degree rays.
// Objects are segments in the vehicle-axis frame centred on the LiDAR HEAD (x forward, y left,
// laser yaw already applied); everything else reads inf (no return).
constexpr double kC1Yaw = M_PI;
constexpr double kWheelbase = 0.3302;  // vehicle_params chassis.wheelbase_m
constexpr double kMountX = 0.285;      // vehicle_params sensors.lidar.mount_x_m
constexpr double kFullLeft = 0.4189;   // vehicle_params steering.max_angle_rad

ScanInput c1_scan_with(const std::vector<Segment>& segments) {
  ScanInput scan = make_scan(720, -M_PI, 2.0 * M_PI / 720.0, 0.0f, 12.0);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double vehicle_bearing = laser_bearing_of_index(scan, i) + kC1Yaw;
    scan.ranges[i] = static_cast<float>(ray_cast(vehicle_bearing, segments));
  }
  return scan;
}

SweptPathGeometry swept_geometry(double lookahead_m = 1.0) {
  SweptPathGeometry g;
  g.wheelbase_m = kWheelbase;
  g.half_width_m = 0.155;
  g.margin_m = 0.05;
  g.body_front_x_m = 0.17145 + 0.58 / 2.0;  // cg_to_rear_axle_m + length_m / 2
  g.lidar_mount_x_m = kMountX;
  g.lidar_mount_y_m = 0.0;
  g.lookahead_m = lookahead_m;
  return g;
}

// Object 0.5 m to the left (side = +1) or right (side = -1) of the head, x 0.1 to 0.4 m.
std::vector<Segment> side_object(double side) { return {{0.1, side * 0.5, 0.4, side * 0.5}}; }

struct RearAxlePoint {
  double x, y;
};

std::vector<RearAxlePoint> rear_axle_returns(const ScanInput& scan) {
  std::vector<RearAxlePoint> points;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r <= 0.0) {
      continue;
    }
    const double b = laser_bearing_of_index(scan, i) + kC1Yaw;
    points.push_back({r * std::cos(b) + kMountX, r * std::sin(b)});
  }
  return points;
}

// Smallest (inside flank radius - margin) - (distance from the turn centre) over the points,
// for a left turn of steering delta. >= 0 means the inside flank misses every point by at
// least the margin.
double inside_clearance_m(const std::vector<RearAxlePoint>& points, double delta) {
  const double radius = kWheelbase / std::tan(delta);
  const double inner = radius - 0.155 - 0.05;
  double worst = std::numeric_limits<double>::infinity();
  for (const RearAxlePoint& p : points) {
    worst = std::min(worst, inner - std::hypot(p.x, radius - p.y));
  }
  return worst;
}

TEST(SweptPathClamp, ObjectBesideTheCarClampsALeftTurnToClearItByTheMargin) {
  const ScanInput scan = c1_scan_with(side_object(+1.0));
  const std::vector<RearAxlePoint> points = rear_axle_returns(scan);
  ASSERT_GT(points.size(), 10u);
  // Full left would sweep the inside flank across the object.
  ASSERT_LT(inside_clearance_m(points, kFullLeft), 0.0);

  const SweptPathClamp clamp =
      clamp_steering_to_swept_path(scan, kC1Yaw, kFullLeft, swept_geometry());
  EXPECT_TRUE(clamp.clamped);
  EXPECT_GT(clamp.steering_rad, 0.1);
  EXPECT_LT(clamp.steering_rad, kFullLeft);
  // The inside flank circle misses every return by the margin, and the binding return by
  // exactly the margin (the clamp is the largest such turn).
  EXPECT_GE(inside_clearance_m(points, clamp.steering_rad), -1e-9);
  EXPECT_NEAR(inside_clearance_m(points, clamp.steering_rad), 0.0, 1e-9);
  EXPECT_LT(inside_clearance_m(points, clamp.steering_rad + 1e-3), 0.0);
  // Closed form against the corner of the object that binds, (0.685, 0.5) in the rear-axle
  // frame: k = 2 (y - c) / (x^2 + y^2 - c^2), delta = atan(k L), about 0.28 rad. The nearest
  // ray to that corner sits a little inside it, hence the tolerance.
  const double c = 0.205;
  const double x = 0.4 + kMountX;
  const double y = 0.5;
  const double k = 2.0 * (y - c) / (x * x + y * y - c * c);
  EXPECT_NEAR(clamp.steering_rad, std::atan(k * kWheelbase), 5e-3);
}

TEST(SweptPathClamp, ObjectOnTheOtherSideDoesNotClamp) {
  const ScanInput right = c1_scan_with(side_object(-1.0));
  const SweptPathClamp left_turn =
      clamp_steering_to_swept_path(right, kC1Yaw, kFullLeft, swept_geometry());
  EXPECT_FALSE(left_turn.clamped);
  EXPECT_EQ(left_turn.steering_rad, kFullLeft);
  // It does clamp a right turn, as the mirror image of the left case.
  const SweptPathClamp right_turn =
      clamp_steering_to_swept_path(right, kC1Yaw, -kFullLeft, swept_geometry());
  const SweptPathClamp mirror = clamp_steering_to_swept_path(c1_scan_with(side_object(+1.0)),
                                                             kC1Yaw, kFullLeft, swept_geometry());
  EXPECT_TRUE(right_turn.clamped);
  EXPECT_NEAR(right_turn.steering_rad, -mirror.steering_rad, 1e-6);
}

TEST(SweptPathClamp, NoObjectLeavesTheSteeringBitForBit) {
  const ScanInput empty = c1_scan_with({});
  for (const double steering : {kFullLeft, 0.1234567, -0.3, -kFullLeft, 0.0}) {
    const SweptPathClamp clamp =
        clamp_steering_to_swept_path(empty, kC1Yaw, steering, swept_geometry());
    EXPECT_FALSE(clamp.clamped);
    EXPECT_EQ(clamp.steering_rad, steering);
  }
}

TEST(SweptPathClamp, ObjectBeyondTheLookaheadIsIgnored) {
  // Wanted 0.2 rad left: R = 1.629 m. An object 0.5 m left of the head at x 0.715 to 1.015 m
  // ahead of the head (rear axle x 1.0 to 1.3 m) is inside the swept annulus, but the rear
  // axle only reaches it after more than 1.0 m of arc (R * atan2(1.0, R - 0.5) = 1.18 m).
  const ScanInput scan = c1_scan_with({{0.715, 0.5, 1.015, 0.5}});
  const SweptPathClamp within_1m =
      clamp_steering_to_swept_path(scan, kC1Yaw, 0.2, swept_geometry());
  EXPECT_FALSE(within_1m.clamped);
  EXPECT_EQ(within_1m.steering_rad, 0.2);
  // With a 2 m lookahead the same object constrains the turn.
  const SweptPathClamp within_2m =
      clamp_steering_to_swept_path(scan, kC1Yaw, 0.2, swept_geometry(2.0));
  EXPECT_TRUE(within_2m.clamped);
  EXPECT_LT(within_2m.steering_rad, 0.2);
}

TEST(SweptPathClamp, ObjectStraightAheadInsideTheCarsWidthIsIgnored) {
  // Straight ahead of the head, |y| <= 0.1 m < half width: the TTC gate's job, not this one.
  const ScanInput scan = c1_scan_with({{0.4, -0.1, 0.4, 0.1}});
  ASSERT_GT(rear_axle_returns(scan).size(), 5u);
  for (const double steering : {0.3, -0.3, kFullLeft}) {
    const SweptPathClamp clamp =
        clamp_steering_to_swept_path(scan, kC1Yaw, steering, swept_geometry());
    EXPECT_FALSE(clamp.clamped);
    EXPECT_EQ(clamp.steering_rad, steering);
  }
}

TEST(SweptPathClamp, ReturnWithinTheMarginBesideTheCarForbidsTheTurn) {
  // 0.18 m left of the centreline: outside the half width (0.155) but inside half width +
  // margin (0.205), alongside the car just behind the head. No left turn clears it.
  const ScanInput scan = c1_scan_with({{-0.1, 0.18, 0.1, 0.18}});
  const SweptPathClamp clamp = clamp_steering_to_swept_path(scan, kC1Yaw, 0.3, swept_geometry());
  EXPECT_TRUE(clamp.clamped);
  EXPECT_EQ(clamp.steering_rad, 0.0);
}

TEST(SweptPathClamp, InvalidReturnsAreIgnored) {
  const ScanInput scan = c1_scan_with(side_object(+1.0));
  // 0.01 is below range_min (0.05), 20 above range_max (12).
  for (const float bad : {kNaNF, -kInfF, 0.0f, -1.0f, 0.01f, 20.0f}) {
    ScanInput s = scan;
    for (float& r : s.ranges) {
      r = std::isfinite(r) ? bad : r;
    }
    const SweptPathClamp clamp =
        clamp_steering_to_swept_path(s, kC1Yaw, kFullLeft, swept_geometry());
    EXPECT_FALSE(clamp.clamped) << bad;
    EXPECT_EQ(clamp.steering_rad, kFullLeft) << bad;
  }
  // Invalid rays between valid ones are not filled from their neighbours: knocking out every
  // other ray of the object leaves a clamp set only by the remaining rays, never tighter.
  ScanInput sparse = scan;
  for (std::size_t i = 0; i < sparse.ranges.size(); i += 2) {
    if (std::isfinite(sparse.ranges[i])) {
      sparse.ranges[i] = kNaNF;
    }
  }
  const SweptPathClamp full =
      clamp_steering_to_swept_path(scan, kC1Yaw, kFullLeft, swept_geometry());
  const SweptPathClamp thinned =
      clamp_steering_to_swept_path(sparse, kC1Yaw, kFullLeft, swept_geometry());
  EXPECT_TRUE(thinned.clamped);
  EXPECT_GE(thinned.steering_rad, full.steering_rad);
  EXPECT_GE(inside_clearance_m(rear_axle_returns(sparse), thinned.steering_rad), -1e-9);
}

TEST(SweptPathClamp, UnusableGeometryLeavesTheSteeringAlone) {
  const ScanInput scan = c1_scan_with(side_object(+1.0));
  std::vector<SweptPathGeometry> bad(7, swept_geometry());
  bad[0].wheelbase_m = 0.0;
  bad[1].half_width_m = std::numeric_limits<double>::quiet_NaN();
  bad[2].margin_m = -0.01;
  bad[3].lookahead_m = 0.0;
  bad[4].lidar_mount_x_m = std::numeric_limits<double>::infinity();
  bad[5].lidar_mount_y_m = std::numeric_limits<double>::quiet_NaN();
  bad[6].body_front_x_m = std::numeric_limits<double>::quiet_NaN();
  for (const SweptPathGeometry& g : bad) {
    const SweptPathClamp clamp = clamp_steering_to_swept_path(scan, kC1Yaw, kFullLeft, g);
    EXPECT_FALSE(clamp.clamped);
    EXPECT_EQ(clamp.steering_rad, kFullLeft);
  }
  const SweptPathClamp nan_steering = clamp_steering_to_swept_path(
      scan, kC1Yaw, std::numeric_limits<double>::quiet_NaN(), swept_geometry());
  EXPECT_FALSE(nan_steering.clamped);
}

// End to end: a gap to the front left with the apex object beside the car on the left.
GapFollowConfig c1_follower_config(bool swept_path_clamp) {
  GapFollowConfig c = floor_config(0.0);
  c.swept_path_clamp = swept_path_clamp;
  c.swept_path = swept_geometry();
  return c;
}

ScanInput c1_left_gap_with_apex_object() {
  // A wall 1.2 m ahead of the head across the right and centre, a wall 0.6 m to the right,
  // open to the front left, and the apex object 0.5 m left of the head.
  return c1_scan_with({{1.2, -3.0, 1.2, 0.3}, {0.1, 0.5, 0.4, 0.5}, {-3.0, -0.6, 1.2, -0.6}});
}

TEST(SweptPathClamp, GapFollowerClampsAndDisabledReproducesTheOldOutput) {
  const ScanInput scan = c1_left_gap_with_apex_object();
  GapFollower on(c1_follower_config(true));
  GapFollower off(c1_follower_config(false));
  const GapFollowResult a = on.process(scan);
  const GapFollowResult b = off.process(scan);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  ASSERT_TRUE(b.gap_found);
  ASSERT_GT(b.target_vehicle_bearing_rad, 0.0);
  // Disabled: exactly the old pipeline (steering_from_bearing, then the corner override,
  // which does not fire here).
  EXPECT_FALSE(b.swept_path_clamped);
  EXPECT_FALSE(b.corner_blocked);
  EXPECT_EQ(b.steering_rad, steering_from_bearing(b.target_vehicle_bearing_rad, 1.0, 0.4189));
  EXPECT_EQ(b.wanted_steering_rad, b.steering_rad);
  // Enabled: same target, steering reduced, flag set.
  EXPECT_EQ(a.target_vehicle_bearing_rad, b.target_vehicle_bearing_rad);
  EXPECT_EQ(a.wanted_steering_rad, b.steering_rad);
  EXPECT_TRUE(a.swept_path_clamped);
  EXPECT_GT(a.steering_rad, 0.0);
  EXPECT_LT(a.steering_rad, b.steering_rad);
  EXPECT_EQ(a.steering_rad,
            clamp_steering_to_swept_path(scan, M_PI, a.wanted_steering_rad, swept_geometry())
                .steering_rad);
}

}  // namespace
}  // namespace racer_control
