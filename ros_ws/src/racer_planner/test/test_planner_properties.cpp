// L2 style property test (a seeded generator loop) and Planner level state behaviour: for random
// tracks, scans and states, including NaN and empty inputs, the published command is finite,
// speed in [0, v_cap], steering within the vehicle_params limits.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "racer_planner/freespace_field.hpp"
#include "racer_planner/planner.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

// The planner's speed floor is the bundle's dead zone plus mppi.speed_floor_margin_mps.
DynamicsParams dynamics_with_floor(double floor_mps) {
  DynamicsParams d = test::plain_dynamics();
  d.nominal.speed_deadband_mps = std::max(0.0, floor_mps - MppiParams{}.speed_floor_margin_mps);
  return d;
}

PlannerConfig small_config(double v_max) {
  PlannerConfig c;
  c.logic.v_max_mps = v_max;
  c.mppi.samples = 128;
  c.max_samples = 128;
  c.mppi.seed = 5;
  c.logic.require_gated_for_start = false;  // these tests run without safety_node
  return c;
}

LocalTrackData good_track(double stamp) {
  LocalTrackData t;
  t.stamp_s = stamp;
  t.frame_id = "base_link";
  t.station_spacing_m = kTrackStationSpacingM;
  t.valid_count = kTrackStations;
  for (int k = 0; k < kTrackStations; ++k) {
    t.x_m.push_back(k * kTrackStationSpacingM);
    t.y_m.push_back(0.0);
    t.half_width_left_m.push_back(0.5);
    t.half_width_right_m.push_back(0.5);
    t.sigma_m.push_back(0.02);
  }
  return t;
}

// A 1 m wide corridor closed 5 m ahead, dense enough that every degree of the scan has a return.
std::vector<PointXY> corridor_points() {
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 5.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  for (double y = -0.5; y <= 0.5; y += 0.02) pts.push_back({5.0, y});
  return pts;
}

DistanceField corridor_field() {
  DistanceField f;
  const std::vector<PointXY> pts = corridor_points();
  f.build(pts.data(), pts.size());
  return f;
}

}  // namespace

TEST(PlannerProperties, CommandIsAlwaysFiniteAndWithinTheCapAndTheSteeringLimits) {
  const VehicleGeometry g = test::binding_geometry();
  std::mt19937_64 rng(2026);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  auto uni = [&](double lo, double hi) { return lo + (hi - lo) * u01(rng); };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  auto weird = [&](double v) {
    const double r = u01(rng);
    if (r < 0.04) return nan;
    if (r < 0.06) return inf;
    if (r < 0.08) return -inf;
    if (r < 0.10) return v * 1e6;
    return v;
  };
  int driving = 0, other = 0;
  for (int scenario = 0; scenario < 120; ++scenario) {
    PlannerConfig cfg = small_config(uni(0.1, 0.9));
    // This generator feeds garbage on most ticks, so 25 consecutive good ticks would almost never
    // happen: the warm start is one tick here (its own properties are tested below).
    cfg.logic.start_ticks = 1;
    cfg.logic.use_lap_guidance = u01(rng) < 0.3;
    const double floor_mps = u01(rng) < 0.5 ? 0.23 : MppiParams{}.speed_floor_margin_mps;
    Planner planner(cfg, dynamics_with_floor(floor_mps), g);
    const double reverse_limit = std::max(cfg.recovery.recover_speed_mps, floor_mps);
    DistanceField field;
    FreespaceField freespace(g);
    {
      std::vector<PointXY> pts;
      const int n = static_cast<int>(uni(0, 400));
      for (int i = 0; i < n; ++i) pts.push_back({weird(uni(-1.5, 4.5)), weird(uni(-3.0, 3.0))});
      if (u01(rng) < 0.2) pts.clear();
      field.build(pts.data(), pts.size());
      if (u01(rng) < 0.7) freespace.build(field, pts.data(), pts.size(), nullptr);
    }
    double now = 1000.0;
    double prev_speed = 0.0;
    int ramp_run = 0;
    for (int tick = 0; tick < 80; ++tick, now += 0.02) {
      LocalTrackData track = good_track(now - uni(0.0, 0.5));
      track.valid_count = static_cast<int>(uni(-1, 20));
      for (int k = 0; k < kTrackStations; ++k) {
        track.x_m[k] = weird(uni(-0.5, 3.5));
        track.y_m[k] = weird(uni(-1.0, 1.0));
        track.half_width_left_m[k] = weird(uni(-0.2, 2.0));
        track.half_width_right_m[k] = weird(uni(-0.2, 2.0));
        track.sigma_m[k] = weird(0.05);
      }
      if (u01(rng) < 0.6) track = good_track(now - 0.1);
      if (u01(rng) < 0.05) track.x_m.pop_back();
      if (u01(rng) < 0.05) track.frame_id = "map";
      track.boundary_residual_m = u01(rng) < 0.1 ? nan : uni(0.0, 0.4);
      track.intrusion_fraction = u01(rng) < 0.1 ? nan : uni(0.0, 0.2);
      TickInput in;
      in.now_s = now;
      in.has_odom = u01(rng) < 0.95;
      in.odom_stamp_s = u01(rng) < 0.7 ? now : now - uni(0.0, 0.2);
      in.wheel_speed_mps = weird(uni(-0.5, 2.0));
      in.has_scan = u01(rng) < 0.95;
      in.scan_stamp_s = u01(rng) < 0.7 ? now - 0.1 : weird(now - uni(0.0, 0.6));
      in.field = u01(rng) < 0.05 ? nullptr : &field;
      in.freespace = u01(rng) < 0.9 ? &freespace : nullptr;
      in.track = u01(rng) < 0.05 ? nullptr : &track;
      in.has_guidance = u01(rng) < 0.5;
      in.guidance_stamp_s = now - uni(0.0, 3.0);
      in.guidance_v_cap_mps = weird(uni(-0.2, 1.5));
      const TickOutput out = planner.tick(in);
      ASSERT_TRUE(std::isfinite(out.cmd.speed_mps)) << scenario << "/" << tick;
      ASSERT_TRUE(std::isfinite(out.cmd.steering_rad)) << scenario << "/" << tick;
      if (out.state == PlannerState::Recovering) {
        ASSERT_GE(out.cmd.speed_mps, -reverse_limit - 1e-12);
      } else {
        ASSERT_GE(out.cmd.speed_mps, 0.0) << "state " << static_cast<int>(out.state);
        // 0 or at least the floor, except the (at most two) ramp ticks of a start from rest
        const bool in_dead_zone = out.cmd.speed_mps > 0.0 && out.cmd.speed_mps < floor_mps - 1e-12;
        if (in_dead_zone) {
          ++ramp_run;
          ASSERT_TRUE(prev_speed == 0.0 || prev_speed < floor_mps - 1e-12)
              << "dead zone entered from " << prev_speed;
          ASSERT_LE(ramp_run, 2) << "more than two ticks inside the dead zone";
          ASSERT_LE(out.cmd.speed_mps - prev_speed, 0.45 * g.max_acceleration_mps2 * 0.02 + 1e-12);
        } else {
          ramp_run = 0;
        }
      }
      ASSERT_LE(out.cmd.speed_mps, std::max(0.0, out.v_cap_mps) + 1e-12);
      ASSERT_LE(out.v_cap_mps, cfg.logic.v_max_mps + 1e-12);
      ASSERT_GE(out.cmd.steering_rad, g.steering_min_rad);
      ASSERT_LE(out.cmd.steering_rad, g.steering_max_rad);
      ASSERT_LE(static_cast<int>(out.state), static_cast<int>(PlannerState::Recovering));
      if (out.state != PlannerState::Driving && out.state != PlannerState::Recovering) {
        ASSERT_EQ(out.cmd.speed_mps, 0.0) << "state " << static_cast<int>(out.state);
        ++other;
      } else {
        ++driving;
      }
      prev_speed = out.cmd.speed_mps;
      ASSERT_TRUE(std::isfinite(out.est_speed_mps));
      ASSERT_TRUE(std::isfinite(out.est_steering_rad));
      ASSERT_TRUE(std::isfinite(out.solve_time_s));
    }
  }
  EXPECT_GT(driving, 100);  // the generator does reach the solver
  EXPECT_GT(other, 100);
}

namespace {

struct Rig {
  VehicleGeometry g = test::binding_geometry();
  DistanceField field = corridor_field();
  FreespaceField freespace{g};
  std::vector<PointXY> points = corridor_points();
  LocalTrackData track;
  Planner planner;
  double now = 50.0;
  double v = 0.3;

  explicit Rig(double v_max = 0.5, double floor_mps = 0.03)
      : planner(small_config(v_max), dynamics_with_floor(floor_mps), g) {
    set_points(points);
  }

  // Replaces the scene (distance field and free-space field).
  void set_points(const std::vector<PointXY>& pts) {
    points = pts;
    field.build(points.data(), points.size());
    freespace.build(field, points.data(), points.size(), nullptr);
  }

  TickOutput tick(bool odom = true, bool scan = true, bool trk = true) {
    TickInput in;
    in.now_s = now;
    in.has_odom = odom;
    in.odom_stamp_s = now;
    in.wheel_speed_mps = v;
    in.has_scan = scan;
    in.scan_stamp_s = now - 0.1;
    in.field = &field;
    in.freespace = &freespace;
    track = good_track(now - 0.1);
    in.track = trk ? &track : nullptr;
    now += 0.02;
    return planner.tick(in);
  }
};

}  // namespace

TEST(Planner, HealthyInputsDriveForwardUnderTheCap) {
  Rig r(0.5);
  TickOutput out;
  for (int i = 0; i < 50; ++i) out = r.tick();
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_GT(out.cmd.speed_mps, 0.2);
  EXPECT_LE(out.cmd.speed_mps, 0.5);
  EXPECT_EQ(out.samples, 128);
  EXPECT_GT(out.solve_time_s, 0.0);
  EXPECT_NEAR(out.est_speed_mps, 0.3, 1e-12);
  EXPECT_NEAR(out.v_cap_mps, 0.5, 1e-12);
  EXPECT_TRUE(r.planner.plan_valid());
  EXPECT_GE(out.scan_age_s, 0.09);
  EXPECT_GE(out.track_age_s, 0.09);
}

TEST(Planner, SilenceOnScanAndOdometryGivesZeroSpeedAndTheMatchingState) {
  {
    Rig r;
    for (int i = 0; i < 10; ++i) r.tick();
    const TickOutput out = r.tick(false, true, true);
    EXPECT_EQ(out.state, PlannerState::NoOdom);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
  {
    Rig r;
    for (int i = 0; i < 10; ++i) r.tick();
    const TickOutput out = r.tick(true, false, true);
    EXPECT_EQ(out.state, PlannerState::NoScan);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
}

TEST(Planner, SilenceOnTheTrackKeepsDrivingOnTheScanAtZeroConfidence) {
  Rig r(0.5);
  TickOutput with_track;
  for (int i = 0; i < 80; ++i) with_track = r.tick();
  EXPECT_EQ(with_track.state, PlannerState::Driving);
  EXPECT_NEAR(with_track.track_confidence, 1.0, 1e-9);
  TickOutput out;
  for (int i = 0; i < 80; ++i) out = r.tick(true, true, false);
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_DOUBLE_EQ(out.track_confidence, 0.0);
  EXPECT_GT(out.cmd.speed_mps, 0.1);
  EXPECT_NEAR(out.v_cap_mps, 0.5 * 0.7, 1e-12);  // the free-space speed fraction
  EXPECT_EQ(out.block_reason, BlockReason::None);
  EXPECT_TRUE(out.has_freespace_goal);
  EXPECT_GT(out.freespace_goal_x_m, 2.0);
  EXPECT_GT(out.freespace_reach_m, 2.0);
}

TEST(Planner, StaleWheelSpeedStopsTheCar) {
  Rig r;
  for (int i = 0; i < 10; ++i) r.tick();
  TickInput in;
  in.now_s = r.now;
  in.has_odom = true;
  in.odom_stamp_s = r.now - 0.5;
  in.wheel_speed_mps = 0.3;
  in.has_scan = true;
  in.scan_stamp_s = r.now - 0.1;
  in.field = &r.field;
  r.track = good_track(r.now - 0.1);
  in.track = &r.track;
  EXPECT_EQ(r.planner.tick(in).state, PlannerState::NoOdom);
}

TEST(Planner, TheConfidenceFallsAtOnceWhenTheTrackGoesAndRisesSlowlyWhenItReturns) {
  Rig r;
  for (int i = 0; i < 80; ++i) r.tick();
  const TickOutput gone = r.tick(true, true, false);
  EXPECT_DOUBLE_EQ(gone.track_confidence, 0.0);  // at once
  const TickOutput back = r.tick();
  EXPECT_NEAR(back.track_confidence, 0.02, 1e-9);  // 1 per second at 50 Hz
  TickOutput later;
  for (int i = 0; i < 10; ++i) later = r.tick();
  EXPECT_NEAR(later.track_confidence, 0.22, 1e-9);
  EXPECT_EQ(later.state, PlannerState::Driving);
  EXPECT_GT(later.cmd.speed_mps, 0.0);
}

TEST(Planner, NoTrackAndNoFreeSpaceFieldIsBlockedForWantOfAPlan) {
  Rig r;
  for (int i = 0; i < 30; ++i) r.tick();  // past the 25 tick warm start
  TickInput in;
  in.now_s = r.now;
  in.has_odom = true;
  in.odom_stamp_s = r.now;
  in.wheel_speed_mps = 0.3;
  in.has_scan = true;
  in.scan_stamp_s = r.now - 0.1;
  in.field = &r.field;
  in.freespace = nullptr;
  in.track = nullptr;
  const TickOutput out = r.planner.tick(in);
  EXPECT_EQ(out.state, PlannerState::Blocked);
  EXPECT_EQ(out.block_reason, BlockReason::BelowSpeedFloor);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
  EXPECT_FALSE(out.has_freespace_goal);
  EXPECT_FALSE(r.planner.plan_valid());
}

TEST(Planner, SpeedRampIsLimitedByTheVehicleAcceleration) {
  Rig r(0.5);
  r.v = 0.0;
  const TickOutput first = r.tick();
  EXPECT_LE(first.cmd.speed_mps, r.g.max_acceleration_mps2 * 0.02 + 1e-12);
}

TEST(Planner, WheelBlockedByAWallReportsBlockedWithZeroSpeed) {
  Rig r;
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.1, y});
  r.set_points(wall);
  TickOutput out;
  for (int i = 0; i < 30; ++i) out = r.tick();  // past the 25 tick warm start
  EXPECT_EQ(out.state, PlannerState::Blocked);
  EXPECT_EQ(out.block_reason, BlockReason::AllCollide);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(out.collision_fraction, 1.0);
}

TEST(Planner, TrackOlderThanTheAgeRampDropsToZeroConfidenceButStillDrives) {
  Rig r;
  for (int i = 0; i < 80; ++i) r.tick();
  TickInput in;
  in.now_s = r.now;
  in.has_odom = true;
  in.odom_stamp_s = r.now;
  in.wheel_speed_mps = 0.3;
  in.has_scan = true;
  in.scan_stamp_s = r.now - 0.1;
  in.field = &r.field;
  in.freespace = &r.freespace;
  r.track = good_track(r.now - 0.7);
  in.track = &r.track;
  const TickOutput out = r.planner.tick(in);
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_DOUBLE_EQ(out.track_confidence, 0.0);
}

TEST(Planner, ABadSelfCheckLowersTheConfidenceInsteadOfStopping) {
  Rig r;
  for (int i = 0; i < 80; ++i) r.tick();
  TickInput in;
  in.now_s = r.now;
  in.has_odom = true;
  in.odom_stamp_s = r.now;
  in.wheel_speed_mps = 0.3;
  in.has_scan = true;
  in.scan_stamp_s = r.now - 0.1;
  in.field = &r.field;
  in.freespace = &r.freespace;
  r.track = good_track(r.now - 0.1);
  r.track.intrusion_fraction = 0.09;  // half trust
  in.track = &r.track;
  TickOutput out;
  for (int i = 0; i < 3; ++i) {
    in.now_s = r.now += 0.02;
    in.odom_stamp_s = in.now_s;
    in.scan_stamp_s = in.now_s - 0.1;
    r.track.stamp_s = in.now_s - 0.1;
    out = r.planner.tick(in);
  }
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_NEAR(out.track_confidence, 0.5, 1e-9);
  EXPECT_NEAR(out.v_cap_mps, 0.5 * (0.7 + 0.3 * 0.5), 1e-12);
}

TEST(Planner, DelayTicksComeFromTheBundleCommandDelay) {
  Rig r;
  EXPECT_EQ(r.planner.delay_ticks(), 4);  // 0.08 s at 50 Hz
}

TEST(Planner, SteeringIsLowPassedAndStaysInsideTheRateLimit) {
  Rig r(0.5);
  for (int i = 0; i < 30; ++i) r.tick();
  // Put a wall right ahead: the solver now wants a hard steering change.
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.75, y});
  r.field.build(wall.data(), wall.size());
  const double max_step = 0.45 * r.g.steering_max_rate_rad_per_s * 0.02;
  double prev = r.tick().cmd.steering_rad;
  double biggest = 0.0;
  for (int i = 0; i < 60; ++i) {
    const double cur = r.tick().cmd.steering_rad;
    biggest = std::max(biggest, std::fabs(cur - prev));
    EXPECT_LE(std::fabs(cur - prev), max_step + 1e-12);
    prev = cur;
  }
  EXPECT_GT(biggest, 0.0);
}

TEST(Planner, LeavingDrivingRestartsTheLowPassFromTheLastPublishedSteering) {
  Rig r(0.5);
  TickOutput last;
  for (int i = 0; i < 30; ++i) last = r.tick();
  const TickOutput stopped = r.tick(true, false, true);  // no scan: held steering, speed 0
  EXPECT_EQ(stopped.state, PlannerState::NoScan);
  EXPECT_DOUBLE_EQ(stopped.cmd.steering_rad, last.cmd.steering_rad);
  const TickOutput back = r.tick();
  EXPECT_EQ(back.state, PlannerState::Driving);
  EXPECT_LE(std::fabs(back.cmd.steering_rad - stopped.cmd.steering_rad),
            0.45 * r.g.steering_max_rate_rad_per_s * 0.02 + 1e-12);
}

TEST(Planner, AStartFromRestHasTwoTicksInsideTheDeadZoneAtMostAndNeverOutrunsTheLimit) {
  Rig r(0.6, 0.23);
  r.v = 0.0;
  std::vector<double> speeds;
  TickOutput out;
  for (int i = 0; i < 60; ++i) {
    out = r.tick();
    speeds.push_back(out.cmd.speed_mps);
  }
  EXPECT_EQ(out.state, PlannerState::Driving);
  int inside = 0;
  double prev = 0.0;
  const double step = 0.45 * r.g.max_acceleration_mps2 * 0.02;  // the planner's own rate limit
  for (double v : speeds) {
    if (v > 0.0 && v < 0.23 - 1e-12) ++inside;
    EXPECT_LE(v - prev, step + 1e-12);  // equal ramp steps, none above 0.45 of the limit
    prev = v;
  }
  EXPECT_EQ(inside, 2);  // the only exception: two ramp ticks from rest
  EXPECT_GE(speeds.back(), 0.23 - 1e-12);
  std::size_t k = 0;
  while (k < speeds.size() && speeds[k] == 0.0) ++k;
  ASSERT_LT(k + 2, speeds.size());
  EXPECT_NEAR(speeds[k], 0.23 / 3.0, 1e-9);
  EXPECT_NEAR(speeds[k + 1], 2 * 0.23 / 3.0, 1e-9);
  EXPECT_NEAR(speeds[k + 2], 0.23, 1e-9);
}

TEST(Planner, AfterAStallLongerThanTheWatchdogTheSpeedRampRestartsFromZero) {
  // Captured on the canary under CPU load: a 61 ms gap between /drive_raw messages tripped
  // safety_node's watchdog (its output went to 0) and the next command, at full speed, was
  // rate limited by safety (a rate_limit event). The planner now restarts the ramp from 0.
  const double step = 0.45 * test::binding_geometry().max_acceleration_mps2 * 0.02;
  Rig r(0.6, 0.23);
  TickOutput out;
  for (int i = 0; i < 80; ++i) out = r.tick();
  ASSERT_GT(out.cmd.speed_mps, 0.4);
  EXPECT_FALSE(out.gap_reset);
  const double steering_before = out.cmd.steering_rad;
  r.now += 0.04;  // a 60 ms gap since the previous tick
  out = r.tick();
  EXPECT_TRUE(out.gap_reset);
  EXPECT_LE(out.cmd.speed_mps, step + 1e-12);  // from 0, one ramp step
  EXPECT_NEAR(out.cmd.steering_rad, steering_before, 0.03);
  double prev = out.cmd.speed_mps;
  for (int i = 0; i < 12; ++i) {
    out = r.tick();
    EXPECT_FALSE(out.gap_reset);
    EXPECT_LE(out.cmd.speed_mps - prev, step + 1e-12);  // the ramp never outruns the limit
    prev = out.cmd.speed_mps;
  }
  EXPECT_GT(prev, 0.4);  // and it gets back to speed
}

TEST(Planner, ASmallJitterInTheTickTimeDoesNotRestartTheRamp) {
  Rig r(0.6, 0.23);
  TickOutput out;
  for (int i = 0; i < 80; ++i) out = r.tick();
  const double before = out.cmd.speed_mps;
  r.now += 0.02;  // a 40 ms gap: under the 50 ms threshold
  out = r.tick();
  EXPECT_FALSE(out.gap_reset);
  EXPECT_NEAR(out.cmd.speed_mps, before, 0.1);
}
