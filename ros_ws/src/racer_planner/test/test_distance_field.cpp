#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

double node_x(int i) { return DistanceField::kXMinM + i * DistanceField::kResolutionM; }
double node_y(int j) { return DistanceField::kYMinM + j * DistanceField::kResolutionM; }

// Brute force over the seed nodes the points snap to.
double brute_node_distance(const std::vector<std::pair<int, int>>& seeds, int i, int j) {
  double best = 1e18;
  for (const auto& s : seeds) {
    const double dx = (i - s.first) * DistanceField::kResolutionM;
    const double dy = (j - s.second) * DistanceField::kResolutionM;
    best = std::min(best, std::hypot(dx, dy));
  }
  return std::min(best, DistanceField::kFarM);
}

}  // namespace

TEST(DistanceField, MatchesBruteForceOnRandomPoints) {
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> ux(DistanceField::kXMinM, DistanceField::kXMaxM);
  std::uniform_real_distribution<double> uy(DistanceField::kYMinM, DistanceField::kYMaxM);
  DistanceField f;
  for (int trial = 0; trial < 6; ++trial) {
    const int n = 1 + trial * trial * 7;
    std::vector<PointXY> pts;
    std::vector<std::pair<int, int>> seeds;
    for (int k = 0; k < n; ++k) {
      pts.push_back({ux(rng), uy(rng)});
      seeds.push_back({static_cast<int>(std::lround((pts.back().x - DistanceField::kXMinM) /
                                                    DistanceField::kResolutionM)),
                       static_cast<int>(std::lround((pts.back().y - DistanceField::kYMinM) /
                                                    DistanceField::kResolutionM))});
    }
    f.build(pts.data(), pts.size());
    for (int i = 0; i < f.nx(); i += 3) {
      for (int j = 0; j < f.ny(); j += 3) {
        const double expect = brute_node_distance(seeds, i, j);
        EXPECT_NEAR(f.distance(node_x(i), node_y(j)), expect, 1e-4)
            << trial << " " << i << " " << j;
      }
    }
    // Off node lookups are the bilinear blend of the four surrounding nodes.
    for (int k = 0; k < 50; ++k) {
      const double x = ux(rng) * 0.98, y = uy(rng) * 0.98;
      const double fx = (x - DistanceField::kXMinM) / DistanceField::kResolutionM;
      const double fy = (y - DistanceField::kYMinM) / DistanceField::kResolutionM;
      const int i = static_cast<int>(fx), j = static_cast<int>(fy);
      const double ax = fx - i, ay = fy - j;
      const double d00 = brute_node_distance(seeds, i, j),
                   d10 = brute_node_distance(seeds, i + 1, j);
      const double d01 = brute_node_distance(seeds, i, j + 1),
                   d11 = brute_node_distance(seeds, i + 1, j + 1);
      const double expect =
          (1 - ay) * ((1 - ax) * d00 + ax * d10) + ay * ((1 - ax) * d01 + ax * d11);
      EXPECT_NEAR(f.distance(x, y), expect, 1e-4);
    }
  }
}

TEST(DistanceField, DistanceToTheTrueLocationIsWithinTheQuantisation) {
  DistanceField f;
  std::vector<PointXY> pts;
  for (double y = -2.0; y <= 2.0; y += 0.013) pts.push_back({1.234, y});  // a wall at x = 1.234
  f.build(pts.data(), pts.size());
  for (double x : {0.2, 0.5, 0.9, 1.0}) {
    EXPECT_NEAR(f.distance(x, 0.0), 1.234 - x, 0.05 * std::sqrt(0.5) + 1e-9);
  }
  EXPECT_NEAR(f.distance(1.234, 0.3), 0.0, 0.04);
}

TEST(DistanceField, OutsideTheGridIsFar) {
  DistanceField f;
  PointXY p{0.5, 0.0};
  f.build(&p, 1);
  EXPECT_EQ(f.distance(4.5, 0.0), DistanceField::kFarM);
  EXPECT_EQ(f.distance(-1.5, 0.0), DistanceField::kFarM);
  EXPECT_EQ(f.distance(0.0, 3.0), DistanceField::kFarM);
  EXPECT_EQ(f.distance(0.0, -3.0), DistanceField::kFarM);
  EXPECT_EQ(f.distance(std::nan(""), 0.0), DistanceField::kFarM);
}

TEST(DistanceField, EmptyScanGivesFarEverywhere) {
  DistanceField f;
  f.build(nullptr, 0);
  EXPECT_EQ(f.distance(0.0, 0.0), DistanceField::kFarM);
  EXPECT_EQ(f.seed_count(), 0u);
}

TEST(DistanceField, PointsOutsideTheGridOrNonFiniteAreIgnored) {
  DistanceField f;
  std::vector<PointXY> pts = {{10.0, 0.0}, {0.0, 9.0}, {std::nan(""), 0.0}, {0.0, INFINITY}};
  f.build(pts.data(), pts.size());
  EXPECT_EQ(f.seed_count(), 0u);
  EXPECT_EQ(f.distance(0.0, 0.0), DistanceField::kFarM);
}

TEST(DistanceField, RebuildForgetsThePreviousScan) {
  DistanceField f;
  PointXY a{1.0, 0.0};
  f.build(&a, 1);
  EXPECT_LT(f.distance(1.0, 0.0), 0.01);
  PointXY b{-0.5, 1.0};
  f.build(&b, 1);
  EXPECT_GT(f.distance(1.0, 0.0), 1.0);
  EXPECT_LT(f.distance(-0.5, 1.0), 0.01);
}

TEST(ScanToPoints, MountYawMakesThePointsAheadOfTheCar) {
  VehicleGeometry g = test::binding_geometry();
  // One beam straight ahead of the LiDAR head in the laser frame is behind the car when the head
  // is mounted facing backwards. With yaw = mount yaw, bearing 0 in the laser frame points at
  // vehicle bearing pi (behind), which is what a backwards mounted head sees.
  const float ranges[] = {1.0f};
  PointXY out[4];
  const std::size_t n = scan_to_points(ranges, 1, 0.0, 0.01, 0.05, 12.0, M_PI, g, 0.02, out, 4);
  ASSERT_EQ(n, 1u);
  EXPECT_NEAR(out[0].x, g.lidar_mount_x_m - 1.0, 1e-9);
  EXPECT_NEAR(out[0].y, g.lidar_mount_y_m, 1e-6);
  // Same beam with no yaw is ahead.
  const std::size_t m = scan_to_points(ranges, 1, 0.0, 0.01, 0.05, 12.0, 0.0, g, 0.02, out, 4);
  ASSERT_EQ(m, 1u);
  EXPECT_NEAR(out[0].x, g.lidar_mount_x_m + 1.0, 1e-9);
}

TEST(ScanToPoints, BearingStepsCounterClockwise) {
  VehicleGeometry g = test::binding_geometry();
  const float ranges[] = {2.0f, 2.0f};
  PointXY out[4];
  ASSERT_EQ(scan_to_points(ranges, 2, 0.0, M_PI / 2.0, 0.05, 12.0, 0.0, g, 0.02, out, 4), 2u);
  EXPECT_GT(out[1].y, out[0].y + 1.0);  // second beam is 90 degrees to the left of the first
}

TEST(ScanToPoints, InvalidBeamsAreSkipped) {
  VehicleGeometry g = test::binding_geometry();
  const float ranges[] = {std::nanf(""), INFINITY, 0.01f, 20.0f, -1.0f, 1.0f};
  PointXY out[8];
  EXPECT_EQ(scan_to_points(ranges, 6, 0.0, 0.1, 0.05, 12.0, 0.0, g, 0.02, out, 8), 1u);
}

TEST(ScanToPoints, PointsInsideTheBodyPlusMarginAreDropped) {
  VehicleGeometry g = test::binding_geometry();
  // A beam from the head (mount x) pointing backwards, short enough to land on the car body.
  const float on_body[] = {0.2f};   // x = mount_x - 0.2, inside the body rectangle
  const float off_body[] = {0.6f};  // x = mount_x - 0.6, behind the rear overhang
  PointXY out[2];
  EXPECT_EQ(scan_to_points(on_body, 1, 0.0, 0.1, 0.05, 12.0, M_PI, g, 0.02, out, 2), 0u);
  EXPECT_EQ(scan_to_points(off_body, 1, 0.0, 0.1, 0.05, 12.0, M_PI, g, 0.02, out, 2), 1u);
  // The margin is part of the rule: a point 1 cm outside the rectangle is still dropped.
  const double just_outside_range = g.lidar_mount_x_m + g.rear_overhang_m + 0.01;
  const float r[] = {static_cast<float>(just_outside_range)};
  EXPECT_EQ(scan_to_points(r, 1, 0.0, 0.1, 0.05, 12.0, M_PI, g, 0.02, out, 2), 0u);
}

TEST(ScanToPoints, RespectsTheOutputCapacity) {
  VehicleGeometry g = test::binding_geometry();
  std::vector<float> ranges(100, 2.0f);
  PointXY out[10];
  EXPECT_EQ(
      scan_to_points(ranges.data(), ranges.size(), 0.0, 0.01, 0.05, 12.0, 0.0, g, 0.02, out, 10),
      10u);
}
