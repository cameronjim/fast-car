// The free-space progress field on hand-made scans: a straight corridor, an L corner, a dead-end
// box, dropouts of 3 and 20 bins, hysteresis, and the search checked against a reference.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "racer_planner/freespace_field.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

using Points = std::vector<PointXY>;

// A scene is a set of wall segments. What the car sees is made by casting beams from the LiDAR
// head, so a wall hides what is behind it exactly as in a real scan.
struct Walls {
  std::vector<std::array<double, 4>> seg;
};

void wall(Walls& w, double x1, double y1, double x2, double y2) {
  w.seg.push_back({x1, y1, x2, y2});
}

Points cast(const Walls& w, const VehicleGeometry& g, double beam_deg = 0.25) {
  Points pts;
  const int n = static_cast<int>(std::lround(360.0 / beam_deg));
  for (int i = 0; i < n; ++i) {
    const double a = -M_PI + 2.0 * M_PI * i / n;
    const double dx = std::cos(a), dy = std::sin(a);
    double best = 1e9;
    for (const auto& s : w.seg) {
      const double ex = s[2] - s[0], ey = s[3] - s[1];
      const double den = dx * ey - dy * ex;
      if (std::fabs(den) < 1e-12) continue;
      const double t = ((s[0] - g.lidar_mount_x_m) * ey - (s[1] - g.lidar_mount_y_m) * ex) / den;
      const double u = ((s[0] - g.lidar_mount_x_m) * dy - (s[1] - g.lidar_mount_y_m) * dx) / den;
      if (t > 0.05 && u >= 0.0 && u <= 1.0 && t < best) best = t;
    }
    if (best < 50.0) {
      pts.push_back({g.lidar_mount_x_m + best * dx, g.lidar_mount_y_m + best * dy});
    }
  }
  return pts;
}

// Drops the points whose bearing from the LiDAR head is within +/- half_deg of bearing_deg.
Points drop_bearing(const Points& in, const VehicleGeometry& g, double bearing_deg,
                    double half_deg) {
  Points out;
  for (const PointXY& p : in) {
    const double b = std::atan2(p.y - g.lidar_mount_y_m, p.x - g.lidar_mount_x_m) * 180.0 / M_PI;
    double d = b - bearing_deg;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    if (std::fabs(d) > half_deg) out.push_back(p);
  }
  return out;
}

struct Scene {
  VehicleGeometry g = test::binding_geometry();
  DistanceField field;
  FreespaceField fs{g};
  bool built = false;

  bool build_points(const Points& pts, const PointXY* prev = nullptr) {
    field.build(pts.data(), pts.size());
    built = fs.build(field, pts.data(), pts.size(), prev);
    return built;
  }
  bool build(const Walls& w, const PointXY* prev = nullptr) {
    return build_points(cast(w, g), prev);
  }
};

// 1 m wide corridor from behind the car to a closing wall 5 m ahead.
Walls corridor() {
  Walls pts;
  wall(pts, -1.0, 0.5, 5.0, 0.5);
  wall(pts, -1.0, -0.5, 5.0, -0.5);
  wall(pts, 5.0, -0.5, 5.0, 0.5);
  return pts;
}

// Open room 3 m wide, closed 4.6 m ahead: two far corners at about the same cost-to-reach.
Walls room(double half_width = 1.5) {
  Walls pts;
  wall(pts, -1.0, half_width, 4.6, half_width);
  wall(pts, -1.0, -half_width, 4.6, -half_width);
  wall(pts, 4.6, -half_width, 4.6, half_width);
  wall(pts, -1.0, -half_width, -1.0, half_width);
  return pts;
}

}  // namespace

TEST(FreespaceField, StraightCorridorGoalIsAtTheFarEndAndPhiFallsTowardIt) {
  Scene s;
  ASSERT_TRUE(s.build(corridor()));
  EXPECT_TRUE(s.fs.valid());
  EXPECT_GT(s.fs.goal().x, 3.5);  // the far end of the grid
  EXPECT_LT(std::fabs(s.fs.goal().y), 0.31);
  EXPECT_GT(s.fs.reach_m(), 3.0);
  // Phi decreases along the corridor toward the goal
  double prev = s.fs.phi(0.33, 0.0);
  for (double x = 0.6; x <= 3.4; x += 0.3) {
    const double p = s.fs.phi(x, 0.0);
    EXPECT_LT(p, prev) << x;
    prev = p;
  }
  EXPECT_NEAR(s.fs.phi(s.fs.goal().x, s.fs.goal().y), 0.0, 1e-3);
  // the cost-to-go from the seed is the reach of the goal, within the cost weighting
  EXPECT_NEAR(s.fs.phi(0.33, 0.0), s.fs.reach_m(), 0.15 * s.fs.reach_m());
}

TEST(FreespaceField, PhiIsFiniteEverywhereIncludingOutsideAndForBadQueries) {
  Scene s;
  ASSERT_TRUE(s.build(corridor()));
  for (double x = -3.0; x <= 7.0; x += 0.37) {
    for (double y = -4.0; y <= 4.0; y += 0.41) {
      const double p = s.fs.phi(x, y);
      ASSERT_TRUE(std::isfinite(p)) << x << " " << y;
      ASSERT_GE(p, 0.0);
    }
  }
  EXPECT_DOUBLE_EQ(s.fs.phi(100.0, 0.0), s.fs.far_value());
  EXPECT_DOUBLE_EQ(s.fs.phi(0.0, -100.0), s.fs.far_value());
  EXPECT_DOUBLE_EQ(s.fs.phi(std::nan(""), 0.0), s.fs.far_value());
  // the far value is above every Phi inside
  double mx = 0.0;
  for (int j = 0; j < s.fs.ny(); ++j) {
    for (int i = 0; i < s.fs.nx(); ++i) mx = std::max(mx, s.fs.phi_node(i, j));
  }
  EXPECT_GT(s.fs.far_value(), mx - 1e-6);
  // at the exact grid edge the lookup still works
  EXPECT_TRUE(std::isfinite(s.fs.phi(DistanceField::kXMaxM, DistanceField::kYMaxM)));
}

TEST(FreespaceField, NotValidMeansPhiIsZeroAndNothingIsReported) {
  Scene s;
  EXPECT_FALSE(s.fs.valid());
  EXPECT_DOUBLE_EQ(s.fs.phi(1.0, 0.0), 0.0);
}

TEST(FreespaceField, LCornerGoalIsPastTheCorner) {
  Scene s;
  Walls pts;
  // Left turn after 2 m. Outer wall y = -0.5 to x = 2.5 then up; inner wall y = 0.5 to x = 1.5
  // then up.
  wall(pts, -1.0, -0.5, 2.5, -0.5);
  wall(pts, 2.5, -0.5, 2.5, 6.0);
  wall(pts, -1.0, 0.5, 1.5, 0.5);
  wall(pts, 1.5, 0.5, 1.5, 6.0);
  wall(pts, -1.0, -0.5, -1.0, 0.5);
  ASSERT_TRUE(s.build(pts));
  // past the inner corner (1.5, 0.5) and turned left into the vertical leg
  EXPECT_GT(s.fs.goal().x, 1.5);
  EXPECT_GT(s.fs.goal().y, 0.3);
  EXPECT_LT(s.fs.goal().x, 2.5);
  // the field leads there: Phi is lower in the leg than at the start
  EXPECT_LT(s.fs.phi(2.0, 0.6), s.fs.phi(0.33, 0.0));
}

TEST(FreespaceField, DeadEndBoxHasNoField) {
  Scene s;
  Walls pts;
  // A box the car sits in, so tight that no goal 0.5 m of cost away exists.
  wall(pts, 0.0, -0.3, 0.7, -0.3);
  wall(pts, 0.0, 0.3, 0.7, 0.3);
  wall(pts, 0.7, -0.3, 0.7, 0.3);
  wall(pts, 0.0, -0.3, 0.0, 0.3);
  EXPECT_FALSE(s.build(pts));
  EXPECT_FALSE(s.fs.valid());
}

TEST(FreespaceField, NoPointsNoSeedAndNonFinitePointsGiveNoFieldOrIgnoreThem) {
  Scene s;
  EXPECT_FALSE(s.build_points(Points{}));
  // a wall right across the front axle point: the seed has nowhere to start
  Walls blocked;
  wall(blocked, 0.2, -2.0, 0.2, 2.0);
  wall(blocked, 0.5, -2.0, 0.5, 2.0);
  wall(blocked, 0.2, -2.0, 0.5, -2.0);
  wall(blocked, 0.2, 2.0, 0.5, 2.0);
  EXPECT_FALSE(s.build(blocked));
  // NaN and infinite points are ignored, the corridor still gives a field
  Points pts = cast(corridor(), s.g);
  pts.push_back({std::nan(""), 0.0});
  pts.push_back({0.0, INFINITY});
  EXPECT_TRUE(s.build_points(pts));
}

TEST(FreespaceField, ADropoutOfThreeBinsDoesNotSplitTheFieldAndTwentyBinsBlocksIt) {
  Scene full;
  ASSERT_TRUE(full.build(corridor()));
  const Points pts = cast(corridor(), full.g);
  // 3 bins of dropout dead ahead (bearing 0, +/- 1.5 degrees)
  Scene small;
  ASSERT_TRUE(small.build_points(drop_bearing(pts, small.g, 0.0, 1.4)));
  EXPECT_GT(small.fs.goal().x, 3.5);
  EXPECT_NEAR(small.fs.reach_m(), full.fs.reach_m(), 0.3);
  EXPECT_NEAR(small.fs.phi(2.0, 0.0), full.fs.phi(2.0, 0.0), 0.3);
  // a 3 bin dropout in a side wall changes nothing either
  Scene side;
  ASSERT_TRUE(side.build_points(drop_bearing(pts, side.g, 60.0, 1.4)));
  EXPECT_GT(side.fs.goal().x, 3.5);
  // 20 bins dead ahead (+/- 10 degrees): the corridor beyond where it fits inside the wedge is
  // unseen and the field stops short of the far end
  Scene big;
  ASSERT_TRUE(big.build_points(drop_bearing(pts, big.g, 0.0, 10.0)));
  EXPECT_LT(big.fs.goal().x, 2.5);
  EXPECT_LT(big.fs.goal().x, full.fs.goal().x - 1.0);
  // and the cells in the wedge are not traversable
  const int i =
      static_cast<int>(std::lround((3.0 - DistanceField::kXMinM) / DistanceField::kResolutionM));
  const int j =
      static_cast<int>(std::lround((0.0 - DistanceField::kYMinM) / DistanceField::kResolutionM));
  EXPECT_FALSE(big.fs.traversable(i, j));
  EXPECT_TRUE(full.fs.traversable(i, j));
}

TEST(FreespaceField, HysteresisKeepsTheGoalNearTheOldOneWhileItIsAlmostAsGood) {
  Scene s;
  const Walls pts = room();
  ASSERT_TRUE(s.build(pts));
  const PointXY first = s.fs.goal();
  // Without a previous goal the best cell is far from the other side of the room: pick the
  // mirror image of it as the "previous" goal. It is as far as the best one, so it is kept.
  const PointXY mirror{first.x, -first.y};
  ASSERT_GT(std::fabs(first.y), 0.5) << "the room should have two competing far corners";
  Scene s2;
  ASSERT_TRUE(s2.build(pts, &mirror));
  EXPECT_LE(std::hypot(s2.fs.goal().x - mirror.x, s2.fs.goal().y - mirror.y), 0.5 + 1e-9);
  EXPECT_GT(std::fabs(s2.fs.goal().y - first.y), 1.0);
  // a 0.1 m shift of the scene keeps the goal
  Walls shifted = pts;
  for (auto& sg : shifted.seg) {
    sg[0] -= 0.1;
    sg[2] -= 0.1;
  }
  Scene s3;
  ASSERT_TRUE(s3.build(shifted, &first));
  EXPECT_LE(std::hypot(s3.fs.goal().x - first.x, s3.fs.goal().y - first.y), 0.5 + 1e-9);
}

TEST(FreespaceField, HysteresisLetsGoWhenTheOldGoalIsWalledOff) {
  Scene s;
  const Walls pts = room();
  ASSERT_TRUE(s.build(pts));
  const PointXY first = s.fs.goal();
  ASSERT_GT(std::fabs(first.y), 0.5);
  // The old goal is in the corner on the other side of a wall across that half of the room.
  const PointXY old_goal{first.x, first.y > 0.0 ? -std::fabs(first.y) : std::fabs(first.y)};
  Walls walled = pts;
  if (old_goal.y < 0.0) {
    wall(walled, 1.0, -1.5, 1.0, -0.1);
  } else {
    wall(walled, 1.0, 0.1, 1.0, 1.5);
  }
  Scene s2;
  ASSERT_TRUE(s2.build(walled, &old_goal));
  // nothing near the old goal is reachable any more: the goal is in the other half
  EXPECT_GT(std::hypot(s2.fs.goal().x - old_goal.x, s2.fs.goal().y - old_goal.y), 1.0);
  EXPECT_LT(s2.fs.goal().y * old_goal.y, 0.0);
  // the same walled scene with a previous goal that is still reachable keeps it
  const PointXY open_goal{first.x, -old_goal.y};
  Scene s3;
  ASSERT_TRUE(s3.build(walled, &open_goal));
  EXPECT_LE(std::hypot(s3.fs.goal().x - open_goal.x, s3.fs.goal().y - open_goal.y), 0.5 + 1e-9);
}

TEST(FreespaceField, CellsOutsideTheTraversableSetTakeTheNearestValuePlusThePenalty) {
  Scene s;
  ASSERT_TRUE(s.build(corridor()));
  // A visible cell 0.12 m from the left wall is not traversable (under the 0.20 m radius). Its
  // value is the nearest traversable cell's plus 3 per metre of the path to it, so above it.
  const int i =
      static_cast<int>(std::lround((2.0 - DistanceField::kXMinM) / DistanceField::kResolutionM));
  const int jw =
      static_cast<int>(std::lround((0.38 - DistanceField::kYMinM) / DistanceField::kResolutionM));
  const int jt =
      static_cast<int>(std::lround((0.3 - DistanceField::kYMinM) / DistanceField::kResolutionM));
  ASSERT_TRUE(s.fs.visible(i, jw));
  ASSERT_FALSE(s.fs.traversable(i, jw));
  ASSERT_TRUE(s.fs.traversable(i, jt));
  EXPECT_GT(s.fs.phi_node(i, jw), s.fs.phi_node(i, jt));
  EXPECT_NEAR(s.fs.phi_node(i, jw), s.fs.phi_node(i, jt) + 3.0 * 0.08, 0.1);
  // behind the walls nothing is visible: the far value
  const int jb =
      static_cast<int>(std::lround((1.5 - DistanceField::kYMinM) / DistanceField::kResolutionM));
  EXPECT_FALSE(s.fs.visible(i, jb));
  EXPECT_NEAR(s.fs.phi_node(i, jb), s.fs.far_value(), 1e-5);
  // cells behind the rear bumper line are never traversable
  const int ir =
      static_cast<int>(std::lround((-0.5 - DistanceField::kXMinM) / DistanceField::kResolutionM));
  const int j0 =
      static_cast<int>(std::lround((0.0 - DistanceField::kYMinM) / DistanceField::kResolutionM));
  EXPECT_FALSE(s.fs.traversable(ir, j0));
}

TEST(FreespaceField, TheSearchMatchesABruteForceBellmanFordReference) {
  Scene s;
  Walls pts;
  // a small maze: corridor with a baffle, so the paths bend
  wall(pts, -1.0, 0.8, 5.0, 0.8);
  wall(pts, -1.0, -0.8, 5.0, -0.8);
  wall(pts, 5.0, -0.8, 5.0, 0.8);
  wall(pts, 1.6, -0.8, 1.6, 0.25);
  wall(pts, 2.8, 0.8, 2.8, -0.25);
  ASSERT_TRUE(s.build(pts));
  const FreespaceParams p;
  const int nx = s.fs.nx(), ny = s.fs.ny();
  auto cost = [&](int i, int j) {
    const double d = s.field.node(i, j);
    const double e = std::min(std::max((p.nav_pref_m - d) / p.nav_pref_m, 0.0), 1.0);
    return 1.0 + p.nav_centre_weight * e * e;
  };
  // reference cost-to-go to the goal over the traversable cells
  std::vector<double> ref(static_cast<std::size_t>(nx) * ny, 1e30);
  const int gi = static_cast<int>(
      std::lround((s.fs.goal().x - DistanceField::kXMinM) / DistanceField::kResolutionM));
  const int gj = static_cast<int>(
      std::lround((s.fs.goal().y - DistanceField::kYMinM) / DistanceField::kResolutionM));
  ASSERT_TRUE(s.fs.traversable(gi, gj));
  ref[static_cast<std::size_t>(gj) * nx + gi] = 0.0;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        if (!s.fs.traversable(i, j)) continue;
        for (int dj = -1; dj <= 1; ++dj) {
          for (int di = -1; di <= 1; ++di) {
            if (di == 0 && dj == 0) continue;
            const int a = i + di, b = j + dj;
            if (a < 0 || b < 0 || a >= nx || b >= ny || !s.fs.traversable(a, b)) continue;
            const double len =
                (di != 0 && dj != 0 ? std::sqrt(2.0) : 1.0) * DistanceField::kResolutionM;
            const double nd =
                ref[static_cast<std::size_t>(b) * nx + a] + len * 0.5 * (cost(i, j) + cost(a, b));
            double& r = ref[static_cast<std::size_t>(j) * nx + i];
            if (nd < r - 1e-12) {
              r = nd;
              changed = true;
            }
          }
        }
      }
    }
  }
  int compared = 0;
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      if (!s.fs.reached(i, j)) continue;
      ASSERT_LT(ref[static_cast<std::size_t>(j) * nx + i], 1e29) << i << " " << j;
      ASSERT_NEAR(s.fs.phi_node(i, j), ref[static_cast<std::size_t>(j) * nx + i], 2e-3)
          << i << " " << j;
      ++compared;
    }
  }
  EXPECT_GT(compared, 200);
  // the baffles make the path longer than the straight line to the goal
  EXPECT_GT(s.fs.reach_m(), std::hypot(s.fs.goal().x - 0.33, s.fs.goal().y) * 1.1);
}
