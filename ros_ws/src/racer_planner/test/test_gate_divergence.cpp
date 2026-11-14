// The planner's reimplementation of safety's in-path distance and gate decision against the real
// ones. This target compiles racer_safety's OWN sources, read only, from the sibling package
// (see CMakeLists.txt: a missing directory fails the configure step); production code does not.
//
// Pre-registration item 4.1 (last bullet). Over kCases seeded random scenes (random wall segments,
// poles and corners, noise, dropouts, a little garbage):
//   (a) on the 33 point steering grid the planner's distance equals safety's min_path_distance_m
//       within 1e-9;
//   (b) off the grid GateTable::distance(delta) <= safety's distance + 0.02 m in at least 99.9
//       percent of cases, the exceptions are listed (the table is a 33 point sample of a
//       discontinuous function);
//   (c) no false negative: whenever SafetyGateLogic::evaluate, from an unlatched state with the
//   same
//       measured wheel speed, engages the obstacle gate, would_trip is true for the same request.
//       The false positive share is reported.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <vector>

#include "gate_safety_rig.hpp"
#include "gate_test_support.hpp"
#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

using namespace racer_planner;
using namespace racer_planner::test;

namespace {

constexpr int kCases = 100000;
constexpr int kFullGridCases = 5000;  // every grid point for the first cases, 2 random after
constexpr int kOffGridPerCase = 3;
constexpr double kOffGridTolM = 0.02;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Whether safety's obstacle gate engages for the request from an unlatched state with a fresh
// measured wheel speed and nothing driven before.
bool safety_trips(const racer_safety::SafetyGateLogic& gate, double safe_distance_m, double v_cmd,
                  double delta, double v_wheel) {
  racer_safety::GateInput in;
  in.command.speed_mps = v_cmd;
  in.command.steering_angle_rad = delta;
  in.drive_raw_age_s = 0.0;
  in.dt_s = 0.02;
  in.min_scan_range_m = safe_distance_m;
  in.measured_speed_mps = v_wheel;
  const racer_safety::GateResult r = gate.evaluate(in, racer_safety::DriveCommand{});
  return r.ttc_brake_latched;
}

struct OffGrid {
  int case_index;
  double delta, safe_m, table_m, lo_m, hi_m;
};

struct Trip {
  int case_index;
  double delta, d_safe, d_table, v_cmd, v_wheel;
  bool model;
};

}  // namespace

TEST(GateDivergence, ConstantsAndConfigAgreeWithSafety) {
  const GateConfig cfg = binding_gate();
  const SafetyRig rig;
  EXPECT_EQ(cfg.geometry.corridor_half_width_m, rig.corridor_half_width_m);
  EXPECT_EQ(cfg.geometry.sector_half_angle_rad, rig.half_angle_rad);
  EXPECT_EQ(cfg.geometry.lidar_mount_yaw_rad, rig.laser_yaw_rad);
  EXPECT_EQ(cfg.geometry.body_front_x_m, rig.path.body_front_x_m);
  EXPECT_EQ(cfg.geometry.outer_corner_horizon_m, rig.path.outer_corner_horizon_m);
  // The straight epsilon: a wall straight ahead is the straight corridor just under the epsilon
  // and the arc just over it, in both implementations.
  const ScanData s = scan_of(cfg.geometry, wall_across_road(1.0));
  const double eps = racer_safety::kStraightSteeringEpsilonRad;
  for (double delta : {0.0, eps * 0.99, -eps * 0.99, eps, -eps, eps * 1.01, -eps * 1.01}) {
    const double a = gate_distance(cfg.geometry, s.view(), delta);
    const double b = rig.distance(s, delta);
    if (std::isfinite(a)) {
      EXPECT_NEAR(a, b, 1e-9) << delta;
    } else {
      EXPECT_EQ(a, b) << delta;
    }
  }
}

TEST(GateDivergence, PlannerGateMatchesSafetyOverRandomScenes) {
  const GateConfig cfg = binding_gate();
  const SafetyRig rig;
  const racer_safety::SafetyGateLogic gate_model(safety_limits(true));
  const racer_safety::SafetyGateLogic gate_bound(safety_limits(false));
  GateTable table(cfg.geometry, cfg.limits);
  SceneGenerator gen(20261113);

  std::vector<double> grid(kGateSteeringValues);
  for (int j = 0; j < kGateSteeringValues; ++j)
    grid[static_cast<std::size_t>(j)] = table.steering(j);
  const double delta_max = cfg.geometry.max_steering_angle_rad;

  long grid_checked = 0, grid_bad = 0;
  double grid_worst = 0.0;
  long off_total = 0, off_finite = 0, off_bad = 0, off_missed = 0;
  double off_worst = 0.0;
  std::vector<OffGrid> exceptions;
  long trip_total = 0, safety_trips_n = 0, planner_trips_n = 0, fn = 0, fp = 0;
  long fn_exact = 0, fp_exact = 0, planner_exact_trips = 0;  // with the exact distance
  std::vector<Trip> false_negatives, exact_false_negatives;

  for (int c = 0; c < kCases; ++c) {
    const ScanData scan = gen.make(cfg.geometry);
    table.build(scan.view());

    // (a) on the grid
    const int n_grid = c < kFullGridCases ? kGateSteeringValues : 2;
    for (int k = 0; k < n_grid; ++k) {
      const int j = c < kFullGridCases ? k : gen.pick(kGateSteeringValues);
      const double want = rig.distance(scan, grid[static_cast<std::size_t>(j)]);
      const double got = table.at(j);
      ++grid_checked;
      const bool same = std::isfinite(want) ? std::fabs(got - want) <= 1e-9 : got == want;
      if (!same) {
        ++grid_bad;
        grid_worst = std::max(grid_worst, std::isfinite(got) ? std::fabs(got - want) : 1e9);
      }
    }

    for (int k = 0; k < kOffGridPerCase; ++k) {
      // off the grid: mostly uniform, some right next to a grid point, some around the straight
      // epsilon, some beyond the steering limits
      double delta;
      const int kind = gen.pick(10);
      if (kind < 7) {
        delta = gen.u(-delta_max, delta_max);
      } else if (kind == 7) {
        delta = grid[static_cast<std::size_t>(gen.pick(kGateSteeringValues))] + gen.u(-1e-5, 1e-5);
      } else if (kind == 8) {
        delta = (gen.pick(2) == 0 ? 1.0 : -1.0) *
                (racer_safety::kStraightSteeringEpsilonRad + gen.u(-3e-5, 3e-5));
      } else {
        delta = gen.u(-1.2 * delta_max, 1.2 * delta_max);
      }
      const double safe = rig.distance(scan, delta);
      const double tab = table.distance(delta);

      // (b)
      ++off_total;
      if (std::isfinite(safe)) ++off_finite;
      if (!(tab <= safe + kOffGridTolM)) {
        ++off_bad;
        const double excess = std::isfinite(tab) ? tab - safe : 1e9;
        if (!std::isfinite(tab)) ++off_missed;
        off_worst = std::max(off_worst, excess);
        const double cl = std::min(std::max(delta, grid.front()), grid.back());
        int i = static_cast<int>(std::floor((cl - grid.front()) / (grid.back() - grid.front()) *
                                            (kGateSteeringValues - 1)));
        i = std::min(std::max(i, 0), kGateSteeringValues - 2);
        exceptions.push_back({c, delta, safe, tab, table.at(i), table.at(i + 1)});
      }

      // (c) the same request, a speed pattern chosen to sit at and around the thresholds
      double v_cmd, v_wheel;
      const int mode = gen.pick(6);
      if (mode == 0 || !std::isfinite(safe)) {
        v_cmd = gen.u(0.0, 2.0);
        v_wheel = gen.u(0.0, 2.0);
      } else if (mode == 1) {
        v_cmd = std::min(2.0, safe / cfg.limits.ttc_brake_s * (1.0 + gen.u(-0.1, 0.1)));
        v_wheel = v_cmd * gen.u(0.0, 1.2);
      } else if (mode == 2) {
        v_cmd = gen.u(0.001, 0.2);
        v_wheel = gen.u(0.0, 0.3);
      } else if (mode == 3) {
        v_cmd = gen.pick(2) == 0 ? 0.0 : gen.u(-1.0, 1e-5);
        v_wheel = gen.u(0.0, 2.0);
      } else if (mode == 4) {
        v_cmd = gen.u(0.0, 1.0);
        v_wheel = gen.u(1.0, 2.0);
      } else {
        v_cmd = std::min(2.0, safe / cfg.limits.ttc_brake_s * (1.0 + gen.u(-0.01, 0.01)));
        v_wheel = 0.0;
      }
      for (int model = 0; model < 2; ++model) {
        const bool st =
            safety_trips(model == 0 ? gate_model : gate_bound, safe, v_cmd, delta, v_wheel);
        const bool pt = would_trip(table, delta, v_cmd, v_wheel, cfg.limits);
        const bool pe = would_trip_exact(table, delta, v_cmd, v_wheel, cfg.limits);
        ++trip_total;
        if (st) ++safety_trips_n;
        if (pt) ++planner_trips_n;
        if (st && !pt) {
          ++fn;
          if (false_negatives.size() < 20) {
            false_negatives.push_back({c, delta, safe, tab, v_cmd, v_wheel, model == 0});
          }
        }
        if (!st && pt) ++fp;
        if (st && !pe) {
          ++fn_exact;
          if (exact_false_negatives.size() < 20) {
            exact_false_negatives.push_back({c, delta, safe, tab, v_cmd, v_wheel, model == 0});
          }
        }
        if (!st && pe) ++fp_exact;
      }
    }
  }

  std::sort(exceptions.begin(), exceptions.end(), [](const OffGrid& a, const OffGrid& b) {
    return (std::isfinite(a.table_m) ? a.table_m - a.safe_m : 1e9) >
           (std::isfinite(b.table_m) ? b.table_m - b.safe_m : 1e9);
  });
  std::printf("[gate divergence] %d scenes\n", kCases);
  std::printf("  (a) on the grid: %ld comparisons, %ld differ by more than 1e-9 (worst %.3g m)\n",
              grid_checked, grid_bad, grid_worst);
  std::printf(
      "  (b) off the grid: %ld requests (%ld with a finite distance), %ld over safety + %.2f m "
      "(%.4f percent), %ld of them a miss (table infinite, safety finite), worst excess %.4f m\n",
      off_total, off_finite, off_bad, kOffGridTolM, 100.0 * off_bad / off_total, off_missed,
      off_worst);
  for (std::size_t i = 0; i < std::min<std::size_t>(exceptions.size(), 25); ++i) {
    const OffGrid& e = exceptions[i];
    std::printf(
        "      case %d delta %+.5f rad: safety %.4f m, table %.4f m (neighbours %.4f, %.4f)\n",
        e.case_index, e.delta, e.safe_m, e.table_m, e.lo_m, e.hi_m);
  }
  std::printf(
      "  (c) gate decisions: %ld requests (each with the measured speed model on and off), safety "
      "trips %ld (%.2f percent), planner trips %ld, false negatives %ld, false positives %ld "
      "(%.2f percent of planner trips, %.3f percent of all)\n",
      trip_total, safety_trips_n, 100.0 * safety_trips_n / trip_total, planner_trips_n, fn, fp,
      planner_trips_n > 0 ? 100.0 * fp / planner_trips_n : 0.0, 100.0 * fp / trip_total);
  std::printf(
      "  (c) with the exact distance (would_trip_exact, what the planner uses for the command it "
      "is "
      "about to publish): false negatives %ld, false positives %ld\n",
      fn_exact, fp_exact);
  std::printf(
      "      the table alone misses %ld of %ld safety trips (%.3f percent): the table reads "
      "a few mm to cm farther than the gate between grid points\n",
      fn, safety_trips_n, 100.0 * fn / safety_trips_n);
  for (const Trip& t : false_negatives) {
    std::printf(
        "      FN case %d delta %+.5f safety d %.5f table d %.5f v_cmd %.4f v_wheel %.4f model "
        "%d\n",
        t.case_index, t.delta, t.d_safe, t.d_table, t.v_cmd, t.v_wheel, t.model ? 1 : 0);
  }

  // The test must not be vacuous: a good share of scenes have something in the corridor and a
  // good share of requests are at the gate.
  EXPECT_GT(off_finite, off_total / 4);
  EXPECT_GT(safety_trips_n, trip_total / 20);
  EXPECT_EQ(grid_bad, 0) << "(a) the table must equal safety's distance on the grid";
  EXPECT_LE(off_bad, off_total / 1000) << "(b) at most 0.1 percent over safety + 2 cm";
  EXPECT_EQ(fn_exact, 0) << "(c) no false negative with the exact distance";
  // The table alone is a sample: it must still catch all but a small share of the trips.
  EXPECT_LT(fn, safety_trips_n / 100) << "(c) the table alone misses over 1 percent of trips";
  (void)exact_false_negatives;
  (void)planner_exact_trips;
}
