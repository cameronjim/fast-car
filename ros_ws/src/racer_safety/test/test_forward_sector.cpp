// L1 tests for racer_safety's forward-sector minimum range (forward_sector.hpp): which /scan
// returns count, which bearings are "ahead" with the real car's mount yaw (pi) and with the
// simulator's (0), the conservative fallback on garbage geometry, and the yaw resolution
// rules shared with racer_control.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "racer_safety/forward_sector.hpp"

namespace racer_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kFloatNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kFloatInf = std::numeric_limits<float>::infinity();

constexpr double kHalfAngle = 1.0;  // the PROVISIONAL committed sector, +/- 1 rad
constexpr double kCarYaw = kPi;     // vehicle_params sensors.lidar.mount_yaw_rad on the car

double deg(double degrees) { return degrees * kPi / 180.0; }

// RPLIDAR C1 as seen on the 2026-10-06 bench: 720 beams over a full turn, angle_min -pi,
// range_min 0.05 m. Every ray starts as "no return" (0.0, the driver's encoding).
struct C1Scan {
  C1Scan() : ranges(720, 0.0f) {
    geometry.angle_min_rad = -kPi;
    geometry.angle_increment_rad = 2.0 * kPi / 720.0;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
  }
  // Puts a return at the ray nearest a LASER bearing.
  void set(double laser_bearing_rad, float range_m) {
    const double offset = laser_bearing_rad - geometry.angle_min_rad;
    const auto i = static_cast<std::size_t>(std::llround(offset / geometry.angle_increment_rad));
    ranges[i % ranges.size()] = range_m;
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

// The bench run's three things in view: a box straight ahead of the nose (read at laser
// bearing 177 deg, 0.51 m), a box on the driver's left (laser -100 deg, 0.37 m), and
// something close BEHIND the car (laser 0 deg, i.e. behind on a head rotated by pi), 0.10 m.
C1Scan bench_scene() {
  C1Scan scan;
  scan.set(deg(177.0), 0.51f);
  scan.set(deg(-100.0), 0.37f);
  scan.set(deg(0.0), 0.10f);
  return scan;
}

// ---------------------------------------------------------------------------------------
// wrap_angle_rad
// ---------------------------------------------------------------------------------------

TEST(WrapAngle, WrapsIntoMinusPiToPi) {
  EXPECT_DOUBLE_EQ(wrap_angle_rad(0.0), 0.0);
  EXPECT_NEAR(wrap_angle_rad(kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(1.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(-1.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(deg(177.0) + kPi), deg(-3.0), 1e-12);
  EXPECT_TRUE(std::isnan(wrap_angle_rad(kNan)));
}

// ---------------------------------------------------------------------------------------
// is_usable_return: the invalid-return policy.
// ---------------------------------------------------------------------------------------

TEST(UsableReturn, InvalidReturnsAreIgnored) {
  const ScanGeometry g = C1Scan().geometry;
  EXPECT_FALSE(is_usable_return(kFloatNan, g));
  EXPECT_FALSE(is_usable_return(kFloatInf, g));
  EXPECT_FALSE(is_usable_return(-kFloatInf, g));
  EXPECT_FALSE(is_usable_return(0.0f, g));
  EXPECT_FALSE(is_usable_return(-0.5f, g));
  EXPECT_FALSE(is_usable_return(0.03f, g));  // below range_min 0.05
  EXPECT_FALSE(is_usable_return(12.5f, g));  // above range_max 12
  EXPECT_TRUE(is_usable_return(0.05f, g));   // exactly range_min counts
  EXPECT_TRUE(is_usable_return(12.0f, g));   // exactly range_max counts
  EXPECT_TRUE(is_usable_return(0.22f, g));
}

TEST(UsableReturn, NonFiniteBoundsAreNotApplied) {
  ScanGeometry g = C1Scan().geometry;
  g.range_min_m = kNan;
  g.range_max_m = kInf;
  EXPECT_TRUE(is_usable_return(0.01f, g));
  EXPECT_TRUE(is_usable_return(500.0f, g));
  EXPECT_FALSE(is_usable_return(0.0f, g));
}

// ---------------------------------------------------------------------------------------
// min_forward_range_m: forward sector with the real mount yaw (pi) and the sim's (0).
// ---------------------------------------------------------------------------------------

TEST(ForwardSector, CarMountYawPiSeesTheBoxAheadAndIgnoresBehindAndBeside) {
  const C1Scan scan = bench_scene();
  const double min_range = min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle);
  EXPECT_FLOAT_EQ(static_cast<float>(min_range), 0.51f);
}

TEST(ForwardSector, YawZeroOnTheSameScanLooksTheOtherWay) {
  // With yaw 0 (the simulator's convention) laser bearing 0 IS ahead, so the 0.10 m return
  // that is behind the real car is what counts. This is why the sim must not use the car's yaw.
  const C1Scan scan = bench_scene();
  const double min_range = min_forward_range_m(scan.geometry, scan.ranges, 0.0, kHalfAngle);
  EXPECT_FLOAT_EQ(static_cast<float>(min_range), 0.10f);
}

TEST(ForwardSector, SimStyleScanWithYawZeroSeesStraightAhead) {
  // racer_gym_bridge-shaped scan: 1080 beams over 4.7 rad centred on the vehicle's +x axis.
  ScanGeometry g;
  g.angle_min_rad = -2.35;
  g.angle_increment_rad = 4.7 / 1079.0;
  g.range_min_m = 0.0;
  g.range_max_m = 30.0;
  std::vector<float> ranges(1080, 10.0f);
  ranges[540] = 0.4f;  // straight ahead
  ranges[0] = 0.2f;    // far right (-2.35 rad), outside +/- 1 rad
  EXPECT_FLOAT_EQ(static_cast<float>(min_forward_range_m(g, ranges, 0.0, kHalfAngle)), 0.4f);
}

TEST(ForwardSector, SectorEdgeInsideCountsOutsideDoesNot) {
  C1Scan scan;
  scan.set(deg(180.0) - 0.98, 0.6f);  // vehicle bearing -0.98 rad: inside +/- 1
  scan.set(deg(180.0) + 1.05, 0.3f);  // vehicle bearing +1.05 rad: outside
  EXPECT_FLOAT_EQ(
      static_cast<float>(min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle)),
      0.6f);
}

TEST(ForwardSector, TheDriversLeftBoxLandsAtEightyDegreesFromAhead) {
  // Laser -100 deg on the car is vehicle +80 deg (the driver's left). A +/- 1.5 rad (86 deg)
  // sector includes it and a +/- 70 deg sector does not, which pins the yaw arithmetic; the
  // sector is symmetric, so left/right sign is not what this checks.
  C1Scan scan;
  scan.set(deg(-100.0), 0.37f);
  EXPECT_FLOAT_EQ(static_cast<float>(min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, 1.5)),
                  0.37f);
  EXPECT_EQ(min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, deg(70.0)), kInf);
}

TEST(ForwardSector, InvalidReturnsInsideTheSectorAreIgnored) {
  C1Scan scan;
  scan.set(deg(180.0), 0.0f);  // driver "no return"
  scan.set(deg(181.0), kFloatNan);
  scan.set(deg(182.0), kFloatInf);
  scan.set(deg(183.0), 0.03f);  // below range_min
  scan.set(deg(184.0), -1.0f);
  scan.set(deg(185.0), 0.8f);  // the only real return ahead
  EXPECT_FLOAT_EQ(
      static_cast<float>(min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle)),
      0.8f);
}

TEST(ForwardSector, NothingUsableInTheSectorIsInfinity) {
  const C1Scan empty_returns;
  EXPECT_EQ(min_forward_range_m(empty_returns.geometry, empty_returns.ranges, kCarYaw, kHalfAngle),
            kInf);
  EXPECT_EQ(min_forward_range_m(empty_returns.geometry, {}, kCarYaw, kHalfAngle), kInf);
}

TEST(ForwardSector, TakesTheNearestOfSeveralReturnsAhead) {
  C1Scan scan;
  scan.set(deg(170.0), 0.9f);
  scan.set(deg(190.0), 0.4f);
  scan.set(deg(200.0), 0.7f);
  EXPECT_FLOAT_EQ(
      static_cast<float>(min_forward_range_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle)),
      0.4f);
}

TEST(ForwardSector, GarbageGeometryFallsBackToTheWholeScanConservatively) {
  const C1Scan scene = bench_scene();  // whole-scan minimum is the 0.10 m return behind
  auto with = [&](auto mutate, double yaw, double half_angle) {
    C1Scan s = scene;
    mutate(s.geometry);
    return static_cast<float>(min_forward_range_m(s.geometry, s.ranges, yaw, half_angle));
  };
  auto keep = [](ScanGeometry&) {};
  EXPECT_FLOAT_EQ(with([](ScanGeometry& g) { g.angle_min_rad = kNan; }, kCarYaw, kHalfAngle),
                  0.10f);
  EXPECT_FLOAT_EQ(with([](ScanGeometry& g) { g.angle_increment_rad = kInf; }, kCarYaw, kHalfAngle),
                  0.10f);
  EXPECT_FLOAT_EQ(with([](ScanGeometry& g) { g.angle_increment_rad = 0.0; }, kCarYaw, kHalfAngle),
                  0.10f);
  EXPECT_FLOAT_EQ(with([](ScanGeometry& g) { g.angle_increment_rad = -0.01; }, kCarYaw, kHalfAngle),
                  0.10f);
  EXPECT_FLOAT_EQ(with(keep, kNan, kHalfAngle), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kNan), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, 0.0), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, -1.0), 0.10f);
}

// ---------------------------------------------------------------------------------------
// resolve_laser_yaw_rad: same rules as racer_control's resolve_laser_yaw_offset.
// ---------------------------------------------------------------------------------------

TEST(ResolveLaserYaw, CarDefaultUsesTheBinding) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, 0.0, true), std::optional<double>(kPi));
}

TEST(ResolveLaserYaw, AgreeingParameterIsAccepted) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, kPi, true), std::optional<double>(kPi));
}

TEST(ResolveLaserYaw, DisagreeingParameterRefusesToStart) {
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, 0.5, true).has_value());
}

TEST(ResolveLaserYaw, UnsetBindingUsesTheParameter) {
  EXPECT_EQ(resolve_laser_yaw_rad(std::nullopt, 0.5, true), std::optional<double>(0.5));
}

TEST(ResolveLaserYaw, SimFixtureIgnoresTheBinding) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, 0.0, false), std::optional<double>(0.0));
}

TEST(ResolveLaserYaw, NonFiniteValuesRefuse) {
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, kNan, true).has_value());
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, kInf, false).has_value());
  EXPECT_FALSE(resolve_laser_yaw_rad(kNan, 0.0, true).has_value());
}

}  // namespace
}  // namespace racer_safety
