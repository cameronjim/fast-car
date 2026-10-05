// L1 tests for racer_control's shared LaserScan helpers (GitHub issue 26 items a and b).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_control/laser_scan.hpp"

namespace racer_control {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kInfF = std::numeric_limits<float>::infinity();
constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();

ScanInput make_scan(std::size_t n, double angle_min, double angle_increment,
                    double range_min = 0.05, double range_max = 30.0, float fill = 5.0f) {
  ScanInput scan;
  scan.ranges.assign(n, fill);
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.range_min = range_min;
  scan.range_max = range_max;
  return scan;
}

// The old Python wall-follow test's scan: 1080 rays, -135 deg start, 0.25 deg per ray.
ScanInput old_sim_scan() { return make_scan(1080, -135.0 * M_PI / 180.0, 0.25 * M_PI / 180.0); }

// RPLIDAR C1 style: 360 degrees starting at -pi.
ScanInput full_circle_scan(std::size_t n = 360) {
  return make_scan(n, -M_PI, 2.0 * M_PI / static_cast<double>(n), 0.05, 12.0);
}

TEST(WrapAngle, MapsIntoMinusPiToPi) {
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(wrap_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(wrap_angle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(wrap_angle(M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrap_angle(5.0 * M_PI), -M_PI, 1e-9);
}

TEST(IsUsable, RejectsGarbageGeometry) {
  EXPECT_TRUE(is_usable(old_sim_scan()));
  ScanInput empty = old_sim_scan();
  empty.ranges.clear();
  EXPECT_FALSE(is_usable(empty));
  ScanInput zero_inc = old_sim_scan();
  zero_inc.angle_increment = 0.0;
  EXPECT_FALSE(is_usable(zero_inc));
  ScanInput negative_inc = old_sim_scan();
  negative_inc.angle_increment = -0.01;
  EXPECT_FALSE(is_usable(negative_inc));
  ScanInput nan_min = old_sim_scan();
  nan_min.angle_min = kNaN;
  EXPECT_FALSE(is_usable(nan_min));
  ScanInput inverted_range = old_sim_scan();
  inverted_range.range_max = inverted_range.range_min;
  EXPECT_FALSE(is_usable(inverted_range));
}

TEST(IsFullCircle, DistinguishesPartialFromFullScans) {
  EXPECT_FALSE(is_full_circle(old_sim_scan()));
  EXPECT_TRUE(is_full_circle(full_circle_scan()));
  EXPECT_TRUE(is_full_circle(full_circle_scan(1440)));
}

// Ported from the old test_wall_logic.py "known bearings map to known ray indices".
TEST(IndexForBearing, OldSimScanKnownBearings) {
  const ScanInput scan = old_sim_scan();
  const double deg = M_PI / 180.0;
  EXPECT_EQ(index_for_laser_bearing(scan, -135.0 * deg).value(), 0u);
  EXPECT_EQ(index_for_laser_bearing(scan, -90.0 * deg).value(), 180u);
  EXPECT_EQ(index_for_laser_bearing(scan, -20.0 * deg).value(), 460u);
  EXPECT_EQ(index_for_laser_bearing(scan, 0.0).value(), 540u);
  EXPECT_EQ(index_for_laser_bearing(scan, 134.75 * deg).value(), 1079u);
}

// Changed from the old "bearings outside the scan clamp to the end rays": a bearing the scan
// does not cover is now reported as not covered instead of silently reading another ray.
TEST(IndexForBearing, BearingOutsidePartialScanIsNotCovered) {
  const ScanInput scan = old_sim_scan();
  const double deg = M_PI / 180.0;
  EXPECT_FALSE(index_for_laser_bearing(scan, 135.0 * deg).has_value());
  EXPECT_FALSE(index_for_laser_bearing(scan, 180.0 * deg).has_value());
  EXPECT_FALSE(index_for_laser_bearing(scan, -200.0 * deg).has_value());
  EXPECT_FALSE(index_for_laser_bearing(scan, 200.0 * deg).has_value());
  // Within half an increment of either end still resolves to the end ray.
  EXPECT_EQ(index_for_laser_bearing(scan, -135.1 * deg).value(), 0u);
  EXPECT_EQ(index_for_laser_bearing(scan, 134.86 * deg).value(), 1079u);
}

TEST(IndexForBearing, FullCircleScanFromMinusPi) {
  const ScanInput scan = full_circle_scan();
  const double inc = scan.angle_increment;
  EXPECT_EQ(index_for_laser_bearing(scan, 0.0).value(), 180u);
  EXPECT_EQ(index_for_laser_bearing(scan, -M_PI).value(), 0u);
  EXPECT_EQ(index_for_laser_bearing(scan, M_PI).value(), 0u);  // same direction as -pi
  EXPECT_EQ(index_for_laser_bearing(scan, M_PI - inc).value(), 359u);
  EXPECT_EQ(index_for_laser_bearing(scan, M_PI - 0.25 * inc).value(), 0u);  // wraps
  EXPECT_EQ(index_for_laser_bearing(scan, M_PI / 2.0).value(), 270u);       // left
  EXPECT_EQ(index_for_laser_bearing(scan, -M_PI / 2.0).value(), 90u);       // right
  EXPECT_EQ(index_for_laser_bearing(scan, 5.0 * M_PI / 2.0).value(), 270u);
}

TEST(IndexForBearing, ScanCentredOnForward) {
  const ScanInput scan = make_scan(181, -M_PI / 2.0, M_PI / 180.0);
  EXPECT_EQ(index_for_laser_bearing(scan, 0.0).value(), 90u);
  EXPECT_EQ(index_for_laser_bearing(scan, M_PI / 2.0).value(), 180u);
  EXPECT_FALSE(index_for_laser_bearing(scan, M_PI).has_value());
}

TEST(IndexForBearing, OffCentreScanDoesNotAssumeMiddleRayIsForward) {
  // Covers [-0.2, 1.79] rad: forward is ray 20, not ray n/2 = 100.
  const ScanInput scan = make_scan(200, -0.2, 0.01);
  EXPECT_EQ(index_for_laser_bearing(scan, 0.0).value(), 20u);
}

TEST(IndexForBearing, VehicleBearingWithRearFacingLidar) {
  const ScanInput scan = full_circle_scan();
  // LiDAR yawed pi in base_link: vehicle forward is laser bearing -pi, ray 0.
  EXPECT_EQ(index_for_vehicle_bearing(scan, 0.0, M_PI).value(), 0u);
  // Vehicle left (+pi/2) is laser -pi/2.
  EXPECT_EQ(index_for_vehicle_bearing(scan, M_PI / 2.0, M_PI).value(), 90u);
  // Zero offset is the identity.
  EXPECT_EQ(index_for_vehicle_bearing(scan, M_PI / 2.0, 0.0).value(), 270u);
}

TEST(IndexForBearing, GarbageInputsAreNotCovered) {
  ScanInput scan = old_sim_scan();
  EXPECT_FALSE(index_for_laser_bearing(scan, kNaN).has_value());
  scan.angle_increment = 0.0;
  EXPECT_FALSE(index_for_laser_bearing(scan, 0.0).has_value());
}

// -- Invalid-return policy (item b) ----------------------------------------------------------

TEST(SanitizeRanges, ValidReturnsKeptAndClipped) {
  ScanInput scan = make_scan(3, -0.1, 0.1);
  scan.ranges = {1.0f, 10.0f, 0.05f};
  std::vector<double> out;
  EXPECT_EQ(sanitize_ranges(scan, 3.5, out), 3u);
  EXPECT_DOUBLE_EQ(out[0], 1.0);
  EXPECT_DOUBLE_EQ(out[1], 3.5);
  EXPECT_DOUBLE_EQ(out[2], static_cast<double>(0.05f));  // exactly range_min is valid
}

TEST(SanitizeRanges, InfAndAboveRangeMaxAreFreeSpace) {
  ScanInput scan = make_scan(3, -0.1, 0.1);
  scan.ranges = {kInfF, 31.0f, 2.0f};
  std::vector<double> out;
  EXPECT_EQ(sanitize_ranges(scan, 3.5, out), 3u);
  EXPECT_DOUBLE_EQ(out[0], 3.5);
  EXPECT_DOUBLE_EQ(out[1], 3.5);
  EXPECT_DOUBLE_EQ(out[2], 2.0);
}

TEST(SanitizeRanges, NanZeroBelowMinAndNegativeInfAreFilledFromNearestValid) {
  ScanInput scan = make_scan(8, -0.4, 0.1);
  // idx:          0      1     2     3      4      5     6      7
  scan.ranges = {kNaNF, 2.0f, 0.0f, 0.01f, 4.0f, -kInfF, kNaNF, 1.0f};
  std::vector<double> out;
  EXPECT_EQ(sanitize_ranges(scan, 3.5, out), 3u);
  EXPECT_DOUBLE_EQ(out[0], 2.0);  // leading invalid: only a right neighbour
  EXPECT_DOUBLE_EQ(out[1], 2.0);
  EXPECT_DOUBLE_EQ(out[2], 2.0);  // nearer to idx 1 than idx 4
  EXPECT_DOUBLE_EQ(out[3], 3.5);  // nearer to idx 4 (clipped 4.0 -> 3.5)
  EXPECT_DOUBLE_EQ(out[4], 3.5);
  EXPECT_DOUBLE_EQ(out[5], 3.5);  // nearer to idx 4
  EXPECT_DOUBLE_EQ(out[6], 1.0);  // nearer to idx 7
  EXPECT_DOUBLE_EQ(out[7], 1.0);
}

TEST(SanitizeRanges, EquidistantNeighboursTakeTheSmallerRange) {
  ScanInput scan = make_scan(3, -0.1, 0.1);
  scan.ranges = {3.0f, kNaNF, 1.0f};
  std::vector<double> out;
  sanitize_ranges(scan, 10.0, out);
  EXPECT_DOUBLE_EQ(out[1], 1.0);
}

TEST(SanitizeRanges, TrailingInvalidRunUsesLeftNeighbour) {
  ScanInput scan = make_scan(4, -0.1, 0.1);
  scan.ranges = {1.5f, kNaNF, 0.0f, kNaNF};
  std::vector<double> out;
  EXPECT_EQ(sanitize_ranges(scan, 10.0, out), 1u);
  for (double r : out) {
    EXPECT_DOUBLE_EQ(r, 1.5);
  }
}

TEST(SanitizeRanges, NoValidReturnsGivesZerosAndZeroCount) {
  ScanInput scan = make_scan(4, -0.1, 0.1);
  scan.ranges = {kNaNF, 0.0f, 0.001f, -kInfF};
  std::vector<double> out;
  EXPECT_EQ(sanitize_ranges(scan, 10.0, out), 0u);
  ASSERT_EQ(out.size(), 4u);
  for (double r : out) {
    EXPECT_DOUBLE_EQ(r, 0.0);
  }
}

TEST(SanitizeRanges, OutputNeverContainsNanOrInf) {
  ScanInput scan = old_sim_scan();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    switch (i % 5) {
      case 0:
        scan.ranges[i] = kNaNF;
        break;
      case 1:
        scan.ranges[i] = kInfF;
        break;
      case 2:
        scan.ranges[i] = 0.0f;
        break;
      case 3:
        scan.ranges[i] = -kInfF;
        break;
      default:
        scan.ranges[i] = 2.5f;
    }
  }
  std::vector<double> out;
  sanitize_ranges(scan, 3.5, out);
  for (double r : out) {
    EXPECT_TRUE(std::isfinite(r));
    EXPECT_GT(r, 0.0);
  }
}

TEST(SanitizeRanges, ReusesOutputBuffer) {
  const ScanInput scan = old_sim_scan();
  std::vector<double> out;
  sanitize_ranges(scan, 3.5, out);
  const double* data = out.data();
  sanitize_ranges(scan, 3.5, out);
  EXPECT_EQ(out.data(), data);
}

TEST(ResolveLaserYawOffset, ParameterUsedWhileBindingIsUnmeasured) {
  EXPECT_DOUBLE_EQ(resolve_laser_yaw_offset(std::nullopt, 0.0).value(), 0.0);
  EXPECT_DOUBLE_EQ(resolve_laser_yaw_offset(std::nullopt, M_PI).value(), M_PI);
  EXPECT_FALSE(resolve_laser_yaw_offset(std::nullopt, kNaN).has_value());
}

TEST(ResolveLaserYawOffset, BindingWinsAndConflictsRefuse) {
  EXPECT_DOUBLE_EQ(resolve_laser_yaw_offset(M_PI, 0.0).value(), M_PI);   // default param
  EXPECT_DOUBLE_EQ(resolve_laser_yaw_offset(M_PI, M_PI).value(), M_PI);  // agreeing param
  EXPECT_FALSE(resolve_laser_yaw_offset(M_PI, 0.5).has_value());         // disagreement
  EXPECT_FALSE(resolve_laser_yaw_offset(kNaN, 0.0).has_value());
}

}  // namespace
}  // namespace racer_control
