// Planner closed loop with no ROS (the L5 stand-in): a simulated car, synthetic worlds from wall
// segments, an oracle local track from the geometry and scans ray cast from the LiDAR pose at
// 10 Hz with 0.02 m noise and 0.1 s of latency. The simulated car is the model with a 10 percent
// slower speed lag and the command delay. v_max is 0.6. Seeded and deterministic.
#include <gtest/gtest.h>

#include <functional>
#include <iostream>

#include "sim_world.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::sim;

namespace {

SimSetup base_setup(const World& w) {
  SimSetup s;
  s.world = &w;
  s.geometry = test::binding_geometry();
  s.planner_dynamics = test::plain_dynamics();
  s.config.logic.v_max_mps = 0.6;
  s.config.tick_s = 0.02;
  s.config.mppi.seed = 1;
  s.config.max_samples = 768;
  s.config.mppi.samples = 768;
  return s;
}

void report(const char* name, const SimResult& r) {
  std::cout << "[closed loop] " << name << ": finished " << r.finished << ", time " << r.sim_time_s
            << " s, mean speed " << r.mean_speed_mps << " m/s, min body clearance "
            << r.min_body_clearance_m << " m, distance " << r.distance_m << " m, max solve "
            << r.max_solve_s * 1e3 << " ms, driving " << r.ticks_driving << " no_track "
            << r.ticks_no_track << " blocked " << r.ticks_blocked << " no_scan " << r.ticks_no_scan
            << "\n";
}

void expect_good_run(const SimResult& r) {
  EXPECT_TRUE(r.finished) << "did not finish in time";
  EXPECT_GE(r.min_body_clearance_m, 0.05) << "body got within 5 cm of a wall";
  EXPECT_GT(r.mean_speed_mps, 0.35);
  EXPECT_EQ(r.ticks_blocked, 0);
  EXPECT_EQ(r.ticks_no_track, 0);
}

}  // namespace

TEST(PlannerClosedLoop, StraightCorridor) {
  const World w = straight_corridor();
  SimSetup s = base_setup(w);
  s.max_time_s = 30.0;
  s.done = [](double x, double, double) { return x > 8.5; };
  const SimResult r = run_closed_loop(s);
  report("a straight corridor", r);
  expect_good_run(r);
}

TEST(PlannerClosedLoop, LeftBend) {
  const World w = left_bend();
  SimSetup s = base_setup(w);
  s.max_time_s = 30.0;
  s.done = [](double, double y, double) { return y > 3.5; };
  const SimResult r = run_closed_loop(s);
  report("b left bend", r);
  expect_good_run(r);
}

TEST(PlannerClosedLoop, RightBend) {
  const World w = right_bend();
  SimSetup s = base_setup(w);
  s.max_time_s = 30.0;
  s.done = [](double, double y, double) { return y < -3.5; };
  const SimResult r = run_closed_loop(s);
  report("c right bend", r);
  expect_good_run(r);
}

TEST(PlannerClosedLoop, StadiumTwoLaps) {
  const World w = stadium();
  SimSetup s = base_setup(w);
  s.start_x = 1.0;
  s.max_time_s = 60.0;
  const double two_laps = 2.0 * w.perimeter;
  s.done = [two_laps](double, double, double progress) { return progress >= two_laps; };
  const SimResult r = run_closed_loop(s);
  report("d stadium, two laps", r);
  expect_good_run(r);
}

TEST(PlannerClosedLoop, WallAcrossTheCorridorStopsBeforeIt) {
  const World w = corridor_with_wall(2.5);
  SimSetup s = base_setup(w);
  s.max_time_s = 15.0;
  const SimResult r = run_closed_loop(s);
  report("e wall across the corridor", r);
  EXPECT_GE(r.min_body_clearance_m, 0.05) << "touched or nearly touched the wall";
  EXPECT_LT(r.final_speed_mps, 0.02);
  EXPECT_TRUE(r.final_state == PlannerState::Blocked || r.final_state == PlannerState::Driving)
      << "final state " << static_cast<int>(r.final_state);
  EXPECT_GT(r.final_x, 1.0) << "it should have driven toward the wall first";
  EXPECT_LT(r.final_x, 2.5);
}

// The noise seeds change the run, not the verdict.
TEST(PlannerClosedLoop, BendsAcrossSeeds) {
  const World left = left_bend();
  const World right = right_bend();
  for (std::uint64_t seed : {2u, 3u, 4u}) {
    for (int dir = 0; dir < 2; ++dir) {
      SimSetup s = base_setup(dir == 0 ? left : right);
      s.max_time_s = 30.0;
      s.seed = seed;
      s.config.mppi.seed = seed;
      if (dir == 0) {
        s.done = [](double, double y, double) { return y > 3.5; };
      } else {
        s.done = [](double, double y, double) { return y < -3.5; };
      }
      const SimResult r = run_closed_loop(s);
      report(dir == 0 ? "bend left, seed sweep" : "bend right, seed sweep", r);
      expect_good_run(r);
    }
  }
}

// ---------------------------------------------------------------------------------------------
// v0.1: ovals at two lane widths in both directions, with and without a track, with a track that
// is wrong the way the first tracer was, on a plant with a speed dead zone and a steering lag the
// planner's model does not have. Two laps each, zero contact.

namespace {

constexpr double kDeadZone = 0.20;
constexpr double kFloorMargin = 0.03;

SimSetup oval_setup(const World& w, TrackMode mode) {
  SimSetup s = base_setup(w);
  s.track_mode = mode;
  s.config.mppi.speed_floor_mps = kDeadZone + kFloorMargin;
  s.plant_dead_zone_mps = kDeadZone;
  s.plant_extra_steer_tau_s = 0.10;
  s.start_x = 1.0;
  s.max_time_s = 150.0;
  const double two_laps = 2.0 * w.perimeter;
  s.done = [two_laps](double, double, double progress) { return progress >= two_laps; };
  return s;
}

void run_ovals(TrackMode mode, const char* name) {
  for (double lane : {1.0, 1.3}) {
    for (int dir = 0; dir < 2; ++dir) {
      World w = oval(lane, 0.4, 2.0);
      if (dir == 1) w.mirror_y();
      SimSetup s = oval_setup(w, mode);
      const SimResult r = run_closed_loop(s);
      std::string label = std::string(name) + ", lane " + std::to_string(lane).substr(0, 3) +
                          (dir == 0 ? ", ccw" : ", cw");
      report(label.c_str(), r);
      EXPECT_TRUE(r.finished) << label << ": did not finish two laps in time";
      EXPECT_GE(r.min_body_clearance_m, 0.02) << label << ": too close to a wall";
      EXPECT_EQ(r.ticks_blocked, 0) << label;
      EXPECT_EQ(r.ticks_no_track, 0) << label;
      EXPECT_GT(r.mean_speed_mps, 0.25) << label;
      if (mode == TrackMode::None) {
        EXPECT_EQ(r.ticks_no_track_conf, r.ticks_driving) << label << ": confidence should be 0";
      }
    }
  }
}

}  // namespace

TEST(PlannerClosedLoop, OvalsWithTheOracleTrack) { run_ovals(TrackMode::Oracle, "oracle track"); }

TEST(PlannerClosedLoop, OvalsWithNoTrackAtAll) { run_ovals(TrackMode::None, "no track"); }

TEST(PlannerClosedLoop, OvalsWithTheTrackPushedTowardTheOuterWallInCorners) {
  run_ovals(TrackMode::Biased, "track pushed 0.25 m outward");
}
