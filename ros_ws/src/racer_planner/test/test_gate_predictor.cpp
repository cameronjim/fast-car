// L1 hand cases of the gate predictor: the in-path distance of one scan along a steering angle and
// the trip decision. The exhaustive comparison against racer_safety's own code is
// test_gate_divergence.cpp.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>

#include "gate_test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::test;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Fixture {
  GateConfig cfg = binding_gate();
  const GateGeometry& g = cfg.geometry;
  const GateLimits& lim = cfg.limits;

  // Rear axle frame offsets of a head frame point: forward distance from the front bumper line.
  double ahead_of_bumper(double x_head) const {
    return x_head + g.lidar_mount_x_m - g.body_front_x_m;
  }

  // A scan with a single return on beam i at the given range.
  ScanData single(int i, double range) const {
    ScanData s = make_scan();
    s.ranges[static_cast<std::size_t>(i)] = static_cast<float>(range);
    return s;
  }

  // The beam closest to vehicle bearing b.
  int beam_at(double b) const {
    const ScanData s = make_scan();
    int best = 0;
    double bd = 1e9;
    for (int i = 0; i < 720; ++i) {
      const double d = std::fabs(wrap_pi(s.vehicle_bearing(g, i) - b));
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  }

  // The range along beam i at which the return lies exactly `rho` from the turn centre of the
  // arc at steering delta (rear axle frame, centre (0, R)); false when the beam never gets there.
  bool range_for_rho(int i, double delta, double rho, double* range) const {
    const double radius = g.wheelbase_m / std::tan(delta);
    const ScanData s = make_scan();
    const double b = s.vehicle_bearing(g, i);
    const double cb = std::cos(b), sb = std::sin(b);
    auto f = [&](double r) {
      return std::hypot(r * cb + g.lidar_mount_x_m, r * sb + g.lidar_mount_y_m - radius);
    };
    // closest approach along the ray, then bisection outward
    double lo = std::max(0.06, -(g.lidar_mount_x_m * cb + (g.lidar_mount_y_m - radius) * sb));
    double hi = 8.0;
    if (f(lo) > rho || f(hi) < rho) return false;
    for (int k = 0; k < 200; ++k) {
      const double mid = 0.5 * (lo + hi);
      (f(mid) < rho ? lo : hi) = mid;
    }
    *range = 0.5 * (lo + hi);
    return true;
  }
};

}  // namespace

TEST(GatePredictor, TheBindingGivesUsableGeometryAndLimits) {
  Fixture f;
  EXPECT_TRUE(gate_geometry_is_usable(f.g));
  EXPECT_TRUE(gate_limits_are_usable(f.lim));
  EXPECT_GT(f.lim.ttc_warning_s, f.lim.ttc_brake_s);
}

TEST(GatePredictor, WallStraightAheadStraightSteering) {
  Fixture f;
  for (double x : {0.3, 0.6, 1.0, 2.0}) {
    const ScanData s = scan_of(f.g, wall_across_road(x));
    const double d = gate_distance(f.g, s.view(), 0.0);
    // the nearest return of a wall across the road is on the axis, a float range away
    EXPECT_NEAR(d, f.ahead_of_bumper(x), 1e-5) << x;
  }
}

TEST(GatePredictor, TheSameWallWithFullLockIsMissedByTheArc) {
  Fixture f;
  const ScanData s = scan_of(f.g, wall_across_road(1.0));
  EXPECT_TRUE(std::isfinite(gate_distance(f.g, s.view(), 0.0)));
  EXPECT_EQ(gate_distance(f.g, s.view(), f.g.steering_max_rad), kInf);
  EXPECT_EQ(gate_distance(f.g, s.view(), f.g.steering_min_rad), kInf);
  // the table says the same at its two ends
  GateTable t(f.g, f.lim);
  t.build(s.view());
  EXPECT_EQ(t.at(0), kInf);
  EXPECT_EQ(t.at(kGateSteeringValues - 1), kInf);
  EXPECT_TRUE(std::isfinite(t.at(kGateSteeringValues / 2)));
}

TEST(GatePredictor, ReturnJustInsideAndJustOutsideTheStraightCorridor) {
  Fixture f;
  const int i = f.beam_at(0.17);
  const ScanData probe = make_scan();
  double bx, by;
  probe.point(f.g, i, 1.0, &bx, &by);
  ASSERT_GT(by, 0.0);
  const double hw = f.g.corridor_half_width_m;
  // The corridor is measured about the car's centreline: |y + mount_y| <= half width.
  const double r_edge = (hw - f.g.lidar_mount_y_m) / (by / 1.0);
  const double eps = 1e-5;
  const double d_in = gate_distance(f.g, f.single(i, r_edge - eps).view(), 0.0);
  const double d_out = gate_distance(f.g, f.single(i, r_edge + eps).view(), 0.0);
  EXPECT_TRUE(std::isfinite(d_in));
  EXPECT_EQ(d_out, kInf);
  // and the mirror beam (negative y)
  const int j = f.beam_at(-0.17);
  EXPECT_TRUE(std::isfinite(gate_distance(f.g, f.single(j, r_edge - eps).view(), 0.0)));
  EXPECT_EQ(gate_distance(f.g, f.single(j, r_edge + eps).view(), 0.0), kInf);
}

TEST(GatePredictor, ReturnBehindTheBumperLineOrOutsideTheSectorIsIgnored) {
  Fixture f;
  // a return right at the head is behind the front bumper line: not ahead
  EXPECT_EQ(gate_distance(f.g, f.single(f.beam_at(0.0), 0.06).view(), 0.0), kInf);
  // behind the car (vehicle bearing pi) is outside the sector
  EXPECT_EQ(gate_distance(f.g, f.single(f.beam_at(M_PI), 0.5).view(), 0.0), kInf);
  // beyond the sector half angle, even with a long range that would be in a wide arc
  const int wide = f.beam_at(f.g.sector_half_angle_rad + 0.05);
  EXPECT_EQ(gate_distance(f.g, f.single(wide, 0.5).view(), 0.0), kInf);
}

TEST(GatePredictor, ArcBandEdgesForTheBodyBandAndTheOuterCornerBand) {
  Fixture f;
  const double delta = 0.30;  // a left turn, radius about 1.07 m
  const double radius = f.g.wheelbase_m / std::tan(delta);
  const double hw = f.g.corridor_half_width_m;
  const double eps = 1e-6;
  // Beams whose returns lie on the left of the car at a large arc angle (beyond the outer corner
  // horizon) and at a small one (inside it).
  int far_beam = -1, near_beam = -1;
  for (int i = 0; i < 720; ++i) {
    const double b = make_scan().vehicle_bearing(f.g, i);
    if (std::fabs(b) > f.g.sector_half_angle_rad) continue;
    double r;
    if (!f.range_for_rho(i, delta, radius + hw - eps, &r)) continue;
    const double d = gate_distance(f.g, f.single(i, r).view(), delta);
    if (std::isfinite(d) && d > f.g.outer_corner_horizon_m + 0.05 && far_beam < 0) far_beam = i;
    if (std::isfinite(d) && d < 0.8 * f.g.outer_corner_horizon_m && near_beam < 0) near_beam = i;
  }
  ASSERT_GE(far_beam, 0) << "no beam reaches the body band beyond the horizon";
  ASSERT_GE(near_beam, 0);
  double r;
  // Far along the arc: just inside the body band (|R| + hw) still counts, just outside (the outer
  // corner band) does not, because it is past the horizon.
  ASSERT_TRUE(f.range_for_rho(far_beam, delta, radius + hw - eps, &r));
  EXPECT_TRUE(std::isfinite(gate_distance(f.g, f.single(far_beam, r).view(), delta)));
  ASSERT_TRUE(f.range_for_rho(far_beam, delta, radius + hw + eps, &r));
  EXPECT_EQ(gate_distance(f.g, f.single(far_beam, r).view(), delta), kInf);
  // Near: both count (the outer corner band within its horizon).
  ASSERT_TRUE(f.range_for_rho(near_beam, delta, radius + hw + eps, &r));
  EXPECT_TRUE(std::isfinite(gate_distance(f.g, f.single(near_beam, r).view(), delta)));
  // Past the outer corner's sweep nothing counts.
  const double outer = std::hypot(f.g.body_front_x_m, radius + hw);
  if (f.range_for_rho(near_beam, delta, outer + 1e-4, &r)) {
    EXPECT_EQ(gate_distance(f.g, f.single(near_beam, r).view(), delta), kInf);
  }
  // Inside the inner flank nothing counts.
  if (f.range_for_rho(near_beam, delta, radius - hw - 1e-4, &r)) {
    EXPECT_EQ(gate_distance(f.g, f.single(near_beam, r).view(), delta), kInf);
  }
}

TEST(GatePredictor, LeftAndRightAreMirrorImages) {
  Fixture f;
  // A scene that is not symmetric: a wall across at an angle and a pole.
  const std::vector<Segment> segs = {Segment{0.9, -0.4, 1.3, 0.6}, Segment{0.7, 0.1, 0.73, 0.13}};
  std::vector<Segment> mirrored;
  for (const Segment& s : segs) mirrored.push_back({s.x1, -s.y1, s.x2, -s.y2});
  const ScanData a = scan_of(f.g, segs);
  const ScanData b = scan_of(f.g, mirrored);
  for (int j = 0; j < kGateSteeringValues; ++j) {
    const double delta = -f.g.steering_max_rad + j * (2.0 * f.g.steering_max_rad / 32.0);
    const double da = gate_distance(f.g, a.view(), delta);
    const double db = gate_distance(f.g, b.view(), -delta);
    if (std::isfinite(da)) {
      EXPECT_NEAR(da, db, 2e-4) << j;  // the mirrored beams are a half beam off at most
    } else {
      EXPECT_EQ(da, db) << j;
    }
  }
}

TEST(GatePredictor, WouldTripAtAndAroundTheTtcThreshold) {
  Fixture f;
  const double v = 1.0;
  const double d_edge = f.lim.ttc_brake_s * v;  // d / v == ttc_brake
  EXPECT_TRUE(gate_would_trip_at(d_edge, v, 0.0, f.lim)) << "at the threshold the gate trips";
  EXPECT_TRUE(gate_would_trip_at(std::nextafter(d_edge, 0.0), v, 0.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(std::nextafter(d_edge, 10.0), v, 0.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(d_edge + 1e-6, v, 0.0, f.lim));
  EXPECT_TRUE(gate_would_trip_at(d_edge - 1e-6, v, 0.0, f.lim));
}

TEST(GatePredictor, WouldTripAtAndAroundTheDistanceFloor) {
  Fixture f;
  const double crawl = 0.01;  // the TTC is huge: only the floor can trip
  EXPECT_TRUE(gate_would_trip_at(f.lim.min_forward_clearance_m - 1e-6, crawl, 0.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(f.lim.min_forward_clearance_m, crawl, 0.0, f.lim))
      << "the floor is strict: at the floor the gate does not trip";
  EXPECT_FALSE(gate_would_trip_at(f.lim.min_forward_clearance_m + 1e-6, crawl, 0.0, f.lim));
}

TEST(GatePredictor, WouldTripUsesTheLargerOfTheRequestAndTheWheelSpeed) {
  Fixture f;
  const double d = 1.1;
  // request 0.5 alone: TTC 2.2 s, no trip; the wheels at 2.0 m/s: TTC 0.55 s, trip
  EXPECT_FALSE(gate_would_trip_at(d, 0.5, 0.5, f.lim));
  EXPECT_TRUE(gate_would_trip_at(d, 0.5, 2.0, f.lim));
  // a faster request than the wheels: the request counts
  EXPECT_TRUE(gate_would_trip_at(d, 2.0, 0.1, f.lim));
  // garbage wheel speed is ignored, never raising or lowering anything
  EXPECT_FALSE(gate_would_trip_at(d, 0.5, std::nan(""), f.lim));
  EXPECT_FALSE(gate_would_trip_at(d, 0.5, -3.0, f.lim));
}

TEST(GatePredictor, ARequestThatIsNotForwardIsNeverBraked) {
  Fixture f;
  EXPECT_FALSE(gate_would_trip_at(0.01, 0.0, 2.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(0.01, -0.5, 2.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(0.01, std::nan(""), 2.0, f.lim));
}

TEST(GatePredictor, NoObstacleOrGarbageDistanceNeverTrips) {
  Fixture f;
  EXPECT_FALSE(gate_would_trip_at(kInf, 2.0, 2.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(std::nan(""), 2.0, 2.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(0.0, 2.0, 2.0, f.lim));
  EXPECT_FALSE(gate_would_trip_at(-0.1, 2.0, 2.0, f.lim));
}

TEST(GatePredictor, WouldTripFromATableUsesTheDistanceAtTheRequestsSteering) {
  Fixture f;
  const ScanData s = scan_of(f.g, wall_across_road(1.0));
  GateTable t(f.g, f.lim);
  t.build(s.view());
  const double d = f.ahead_of_bumper(1.0);  // 0.845 m
  // 0.845 / 1.5 = 0.563 s: trips straight, not with the arc that misses the wall
  EXPECT_TRUE(would_trip(t, 0.0, 1.5, 0.0, f.lim));
  EXPECT_FALSE(would_trip(t, f.g.steering_max_rad, 1.5, 0.0, f.lim));
  // slower: 0.845 / 1.0 = 0.845 s, clear
  EXPECT_FALSE(would_trip(t, 0.0, 1.0, 0.0, f.lim));
  EXPECT_NEAR(gate_distance(f.g, s.view(), 0.0), d, 1e-5);
  // the table reads the smaller of the two grid cells around 0, one of which is the next arc over:
  // at most a centimetre or two closer than the straight distance, never farther
  EXPECT_LE(t.distance(0.0), d + 1e-5);
  EXPECT_GT(t.distance(0.0), d - 0.02);
}

TEST(GatePredictor, TheTableIsTheSmallerOfTwoNeighboursBetweenGridPoints) {
  Fixture f;
  const ScanData s = scan_of(f.g, {Segment{0.9, -0.4, 1.3, 0.6}, Segment{0.7, 0.1, 0.73, 0.13}});
  GateTable t(f.g, f.lim);
  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.distance(0.0), kInf) << "no scan yet: nothing is known";
  t.build(s.view());
  ASSERT_TRUE(t.valid());
  for (int j = 0; j < kGateSteeringValues; ++j) {
    // at a grid point the answer is never above the entry (rounding may pick the lower cell)
    EXPECT_LE(t.distance(t.steering(j)), t.at(j)) << j;
  }
  for (int j = 0; j + 1 < kGateSteeringValues; ++j) {
    const double mid = 0.5 * (t.steering(j) + t.steering(j + 1));
    EXPECT_EQ(t.distance(mid), std::min(t.at(j), t.at(j + 1))) << j;
  }
  // outside the range: the end entries
  EXPECT_EQ(t.distance(10.0),
            std::min(t.at(kGateSteeringValues - 2), t.at(kGateSteeringValues - 1)));
  EXPECT_EQ(t.distance(-10.0), std::min(t.at(0), t.at(1)));
  // NaN: the smallest entry (fail closed)
  double smallest = kInf;
  for (int j = 0; j < kGateSteeringValues; ++j) smallest = std::min(smallest, t.at(j));
  EXPECT_EQ(t.distance(std::nan("")), smallest);
  // the grid runs from the minimum to the maximum steering, ends included
  EXPECT_EQ(t.steering(0), f.g.steering_min_rad);
  EXPECT_NEAR(t.steering(kGateSteeringValues - 1), f.g.steering_max_rad, 1e-12);
}

TEST(GatePredictor, ExactDistanceIsTheGatesOwnAtAnySteeringAndNeverAboveTheTable) {
  Fixture f;
  const ScanData s = scan_of(f.g, {Segment{0.9, -0.4, 1.3, 0.6}, Segment{0.7, 0.1, 0.73, 0.13}});
  GateTable t(f.g, f.lim);
  EXPECT_EQ(t.exact_distance(0.1), kInf) << "no scan yet";
  t.build(s.view());
  for (double delta : {-0.44, -0.2, -0.0137, 0.0, 0.0071, 0.13, 0.2999, 0.44}) {
    EXPECT_EQ(t.exact_distance(delta), gate_distance(f.g, s.view(), delta)) << delta;
  }
  // exact_distance and would_trip_exact read the stored copy, not the caller's buffer
  ScanData wiped = s;
  GateTable t2(f.g, f.lim);
  t2.build(wiped.view());
  std::fill(wiped.ranges.begin(), wiped.ranges.end(), 0.0f);
  EXPECT_EQ(t2.exact_distance(0.13), gate_distance(f.g, s.view(), 0.13));
  // a scan longer than the copy's capacity: the table still builds, exact falls back to it
  GateTable small(f.g, f.lim, 100);
  small.build(s.view());
  EXPECT_EQ(small.exact_distance(0.13), small.distance(0.13));
  EXPECT_TRUE(small.valid());
  // the trip decision with the exact distance
  const double d = gate_distance(f.g, s.view(), 0.13);
  ASSERT_TRUE(std::isfinite(d));
  EXPECT_TRUE(would_trip_exact(t, 0.13, d / f.lim.ttc_brake_s * 1.01, 0.0, f.lim));
  EXPECT_FALSE(would_trip_exact(t, 0.13, d / f.lim.ttc_brake_s * 0.99, 0.0, f.lim));
}

TEST(GatePredictor, GarbageScansGiveAnInfiniteDistanceAndNoTrip) {
  Fixture f;
  GateTable t(f.g, f.lim);
  // an empty scan
  GateScan empty;
  empty.ranges = nullptr;
  empty.count = 0;
  empty.angle_min_rad = -M_PI;
  empty.angle_increment_rad = 2.0 * M_PI / 720.0;
  t.build(empty);
  EXPECT_EQ(t.distance(0.0), kInf);
  EXPECT_FALSE(would_trip(t, 0.0, 2.0, 2.0, f.lim));
  // zero beams
  std::vector<float> none;
  GateScan zero = empty;
  zero.ranges = none.data();
  t.build(zero);
  EXPECT_EQ(t.distance(0.0), kInf);
  // NaN, negative, zero, infinite and out of range returns everywhere
  for (float bad : {std::numeric_limits<float>::quiet_NaN(), -1.0f, 0.0f,
                    std::numeric_limits<float>::infinity(), 1000.0f}) {
    ScanData s = make_scan();
    std::fill(s.ranges.begin(), s.ranges.end(), bad);
    t.build(s.view());
    for (int j = 0; j < kGateSteeringValues; ++j) EXPECT_EQ(t.at(j), kInf) << bad;
    EXPECT_FALSE(would_trip(t, 0.0, 2.0, 2.0, f.lim));
  }
  // garbage mixed into a real scan changes nothing about the real returns
  ScanData s = scan_of(f.g, wall_across_road(1.0));
  const double clean = gate_distance(f.g, s.view(), 0.0);
  s.ranges[10] = std::numeric_limits<float>::quiet_NaN();
  s.ranges[11] = -2.0f;
  EXPECT_EQ(gate_distance(f.g, s.view(), 0.0), clean);
}

TEST(GatePredictor, AScanTheGateCannotPlaceGivesTheSlantRangeLikeTheGate) {
  Fixture f;
  ScanData s = scan_of(f.g, wall_across_road(1.0));
  s.angle_increment_rad = 0.0;
  double nearest = kInf;
  for (float r : s.ranges) {
    if (std::isfinite(r) && r > 0.0f) nearest = std::min(nearest, static_cast<double>(r));
  }
  EXPECT_EQ(gate_distance(f.g, s.view(), 0.0), nearest);
  EXPECT_EQ(gate_distance(f.g, s.view(), 0.3), nearest);
}

TEST(GatePredictor, BuildingATableIsFastEnough) {
  Fixture f;
  SceneGenerator gen(99);
  GateTable t(f.g, f.lim);
  std::vector<double> ms;
  for (int k = 0; k < 300; ++k) {
    const ScanData s = gen.make(f.g);
    const auto t0 = std::chrono::steady_clock::now();
    t.build(s.view());
    ms.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  std::cout << "[gate table] build ms over 300 random 720 beam scans: p50 " << ms[150] << " p99 "
            << ms[297] << " max " << ms.back() << "\n";
  EXPECT_LT(ms[150], 0.5) << "the table is built in the scan thread: budget 0.5 ms";
}
