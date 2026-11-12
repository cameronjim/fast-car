// L1 tests for racer_control's physics speed law and latency statistics (race_speed.hpp,
// 2026-11-12). Hand-computed cases with the car's own numbers, boundary and garbage inputs,
// monotonicity and the inverse relation between the stopping distance and the stopping speed.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "racer_control/race_speed.hpp"
#include "racer_control/reactive_speed.hpp"

namespace racer_control {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kWheelbase = 0.3302;  // chassis.wheelbase_m
constexpr double kLock = 0.449;        // steering.max_angle_rad

// -- lateral limit --------------------------------------------------------------------------------

TEST(LateralLimitSpeed, FullLockOnTheCarIsSqrtOfALatTimesTheTurnRadius) {
  // R = L / tan(0.449) = 0.3302 / 0.48182 = 0.6853 m; v = sqrt(2 * 0.6853) = 1.1707 m/s.
  const double radius = kWheelbase / std::tan(kLock);
  EXPECT_NEAR(radius, 0.6853, 1e-3);
  EXPECT_NEAR(lateral_limit_speed(kLock, kWheelbase, 2.0), std::sqrt(2.0 * radius), 1e-12);
  EXPECT_NEAR(lateral_limit_speed(kLock, kWheelbase, 2.0), 1.1707, 1e-3);
}

TEST(LateralLimitSpeed, LeftAndRightAreTheSameAndAStraightHasNoLimit) {
  EXPECT_DOUBLE_EQ(lateral_limit_speed(0.2, kWheelbase, 3.0),
                   lateral_limit_speed(-0.2, kWheelbase, 3.0));
  EXPECT_EQ(lateral_limit_speed(0.0, kWheelbase, 2.0), kInf);
  EXPECT_EQ(lateral_limit_speed(1e-12, kWheelbase, 2.0), kInf);
}

TEST(LateralLimitSpeed, FallsWithSteeringAndRisesWithTheAccelerationBound) {
  double previous = kInf;
  for (double delta = 0.01; delta <= kLock; delta += 0.01) {
    const double v = lateral_limit_speed(delta, kWheelbase, 2.0);
    EXPECT_LT(v, previous) << delta;
    previous = v;
  }
  EXPECT_GT(lateral_limit_speed(0.3, kWheelbase, 4.0), lateral_limit_speed(0.3, kWheelbase, 2.0));
  // Doubling the acceleration bound scales the speed by sqrt(2).
  EXPECT_NEAR(lateral_limit_speed(0.3, kWheelbase, 4.0) / lateral_limit_speed(0.3, kWheelbase, 2.0),
              std::sqrt(2.0), 1e-12);
}

TEST(LateralLimitSpeed, GarbageFailsSlow) {
  EXPECT_EQ(lateral_limit_speed(kNaN, kWheelbase, 2.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(kInf, kWheelbase, 2.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, 0.0, 2.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, -0.1, 2.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, kNaN, 2.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, kWheelbase, 0.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, kWheelbase, -1.0), 0.0);
  EXPECT_EQ(lateral_limit_speed(0.3, kWheelbase, kNaN), 0.0);
}

// -- stopping limit -------------------------------------------------------------------------------

TEST(StoppingLimitSpeed, HandComputedCase) {
  // a = 1 m/s^2, latency 0.2 s, 0.5 m of room: v = 1 * (sqrt(0.04 + 1.0) - 0.2) = 0.81980.
  const double v = stopping_limit_speed(0.65, 0.15, 1.0, 0.2);
  EXPECT_NEAR(v, std::sqrt(1.04) - 0.2, 1e-12);
  EXPECT_NEAR(v, 0.8198, 1e-4);
  // It stops exactly in the room: latency * v + v^2 / 2a = 0.5.
  EXPECT_NEAR(stopping_distance(v, 1.0, 0.2), 0.5, 1e-12);
}

TEST(StoppingLimitSpeed, IsTheInverseOfTheStoppingDistance) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> room(0.01, 3.0);
  std::uniform_real_distribution<double> decel(0.3, 6.0);
  std::uniform_real_distribution<double> latency(0.0, 0.5);
  for (int i = 0; i < 500; ++i) {
    const double r = room(rng);
    const double a = decel(rng);
    const double t = latency(rng);
    const double v = stopping_limit_speed(r + 0.1, 0.1, a, t);
    EXPECT_NEAR(stopping_distance(v, a, t), r, 1e-9);
  }
}

TEST(StoppingLimitSpeed, MoreRoomMeansMoreSpeedAndNoRoomMeansNone) {
  double previous = 0.0;
  for (double d = 0.2; d < 3.0; d += 0.1) {
    const double v = stopping_limit_speed(d, 0.15, 1.0, 0.2);
    EXPECT_GE(v, previous) << d;
    previous = v;
  }
  EXPECT_EQ(stopping_limit_speed(0.15, 0.15, 1.0, 0.2), 0.0);  // exactly at the standoff
  EXPECT_EQ(stopping_limit_speed(0.10, 0.15, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(-1.0, 0.15, 1.0, 0.2), 0.0);
  // No latency: v = sqrt(2 a room).
  EXPECT_NEAR(stopping_limit_speed(1.5, 0.5, 2.0, 0.0), std::sqrt(2.0 * 2.0 * 1.0), 1e-12);
}

TEST(StoppingLimitSpeed, GarbageFailsSlow) {
  EXPECT_EQ(stopping_limit_speed(kNaN, 0.1, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(kInf, 0.1, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(1.0, kNaN, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(1.0, 0.1, 0.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(1.0, 0.1, -1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(1.0, 0.1, 1.0, -0.2), 0.0);
  EXPECT_EQ(stopping_limit_speed(1.0, 0.1, 1.0, kNaN), 0.0);
}

TEST(StoppingDistance, KnownValuesAndGuards) {
  EXPECT_NEAR(stopping_distance(1.0, 1.0, 0.2), 0.2 + 0.5, 1e-12);
  EXPECT_NEAR(stopping_distance(0.64, 1.0, 0.2), 0.128 + 0.2048, 1e-12);
  EXPECT_EQ(stopping_distance(0.0, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_distance(-1.0, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_distance(kNaN, 1.0, 0.2), 0.0);
  EXPECT_EQ(stopping_distance(1.0, 0.0, 0.2), kInf);
  EXPECT_EQ(stopping_distance(1.0, kNaN, 0.2), kInf);
  EXPECT_NEAR(stopping_distance(1.0, 1.0, -0.5), 0.5, 1e-12);  // a negative latency counts as 0
}

// -- side proximity cap ---------------------------------------------------------------------------

TEST(SideProximityCap, NoCapWithoutAMeasurementOrFarFromTheWall) {
  EXPECT_EQ(side_proximity_cap(std::nullopt, 0.2, 0.36, 1.0), kInf);
  EXPECT_EQ(side_proximity_cap(0.2, 0.2, 0.36, 1.0), kInf);  // exactly at the distance
  EXPECT_EQ(side_proximity_cap(0.5, 0.2, 0.36, 1.0), kInf);
  EXPECT_EQ(side_proximity_cap(0.05, 0.0, 0.36, 1.0), kInf);  // off
  EXPECT_EQ(side_proximity_cap(0.05, -0.1, 0.36, 1.0), kInf);
  EXPECT_EQ(side_proximity_cap(kNaN, 0.2, 0.36, 1.0), kInf);
  EXPECT_EQ(side_proximity_cap(0.05, kNaN, 0.36, 1.0), kInf);
}

TEST(SideProximityCap, FallsLinearlyFromTheMaximumToTheMinimumAtTheWall) {
  EXPECT_NEAR(side_proximity_cap(0.0, 0.2, 0.36, 1.0), 0.36, 1e-12);
  EXPECT_NEAR(side_proximity_cap(0.1, 0.2, 0.36, 1.0), 0.68, 1e-12);
  EXPECT_NEAR(side_proximity_cap(-0.05, 0.2, 0.36, 1.0), 0.36, 1e-12);  // clamped at the wall
  // Continuous at the distance: just inside it the cap is just under the maximum.
  EXPECT_NEAR(side_proximity_cap(0.2 - 1e-9, 0.2, 0.36, 1.0), 1.0, 1e-7);
}

TEST(SideProximityCap, SmoothAndMonotoneInTheClearance) {
  double previous = side_proximity_cap(0.0, 0.2, 0.36, 1.0);
  for (double c = 0.005; c < 0.2; c += 0.005) {
    const double cap = side_proximity_cap(c, 0.2, 0.36, 1.0);
    EXPECT_GE(cap, previous);
    EXPECT_LE(cap - previous, (1.0 - 0.36) / 0.2 * 0.005 + 1e-9);  // the slope is bounded
    previous = cap;
  }
}

// -- the composed target --------------------------------------------------------------------------

RaceSpeedConfig car_config() {
  RaceSpeedConfig c;
  c.wheelbase_m = kWheelbase;
  c.a_lat_max_mps2 = 2.0;
  c.brake_decel_mps2 = 1.0;
  c.stop_latency_s = 0.2;
  c.standoff_m = 0.15;
  c.max_speed_mps = 1.0;
  c.min_speed_mps = 0.36;
  c.side_cap_distance_m = 0.2;
  c.side_cap_min_speed_mps = 0.4;
  return c;
}

TEST(RaceTargetSpeed, NoLimitBindingGivesTheCeiling) {
  RaceSpeedInputs in;
  in.free_path_m = 3.0;
  const RaceSpeedTarget t = race_target_speed(in, car_config());
  EXPECT_EQ(t.lateral_mps, kInf);
  EXPECT_GT(t.stopping_mps, 1.0);
  EXPECT_EQ(t.side_mps, kInf);
  EXPECT_DOUBLE_EQ(t.target_mps, 1.0);
}

TEST(RaceTargetSpeed, EachLimitCanBeTheBindingOne) {
  RaceSpeedConfig c = car_config();
  RaceSpeedInputs corner;
  corner.steering_rad = kLock;
  corner.free_path_m = 3.0;
  c.a_lat_max_mps2 = 1.0;  // so the corner, not the 1.0 m/s ceiling, binds
  EXPECT_NEAR(race_target_speed(corner, c).target_mps,
              std::sqrt(1.0 * kWheelbase / std::tan(kLock)), 1e-12);
  c.a_lat_max_mps2 = 2.0;

  RaceSpeedInputs wall;
  wall.free_path_m = 0.65;  // 0.5 m of room: 0.8198 m/s
  EXPECT_NEAR(race_target_speed(wall, c).target_mps, std::sqrt(1.04) - 0.2, 1e-12);

  RaceSpeedInputs side;
  side.free_path_m = 3.0;
  side.side_clearance_m = 0.1;  // half way: 0.4 + 0.6 * 0.5 = 0.7
  EXPECT_NEAR(race_target_speed(side, c).target_mps, 0.7, 1e-12);
}

TEST(RaceTargetSpeed, TheTargetIsTheMinimumOfTheLimits) {
  RaceSpeedConfig c = car_config();
  RaceSpeedInputs in;
  in.steering_rad = kLock;
  in.free_path_m = 0.65;
  in.side_clearance_m = 0.1;
  const RaceSpeedTarget t = race_target_speed(in, c);
  EXPECT_DOUBLE_EQ(t.target_mps, std::max(c.min_speed_mps, std::min({c.max_speed_mps, t.lateral_mps,
                                                                     t.stopping_mps, t.side_mps})));
}

TEST(RaceTargetSpeed, TheFloorHoldsAgainstTheStoppingLimitAndTheCeilingAgainstTheFloor) {
  RaceSpeedConfig c = car_config();
  RaceSpeedInputs blocked;
  blocked.free_path_m = 0.05;  // inside the standoff: v_stop 0
  const RaceSpeedTarget t = race_target_speed(blocked, c);
  EXPECT_EQ(t.stopping_mps, 0.0);
  EXPECT_DOUBLE_EQ(t.target_mps, c.min_speed_mps);
  c.min_speed_mps = 2.0;  // a floor above the ceiling never lifts the target past it
  EXPECT_DOUBLE_EQ(race_target_speed(blocked, c).target_mps, c.max_speed_mps);
}

TEST(RaceTargetSpeed, NoFreePathMeansNoStoppingLimitAndGarbageSteeringFailsToTheFloor) {
  RaceSpeedConfig c = car_config();
  RaceSpeedInputs in;
  EXPECT_DOUBLE_EQ(race_target_speed(in, c).target_mps, 1.0);
  in.steering_rad = kNaN;
  EXPECT_DOUBLE_EQ(race_target_speed(in, c).target_mps, c.min_speed_mps);
}

TEST(RaceTargetSpeed, PropertyAlwaysWithinTheFloorAndTheCeiling) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> steer(-0.6, 0.6);
  std::uniform_real_distribution<double> path(-0.5, 4.0);
  std::uniform_real_distribution<double> side(-0.1, 0.5);
  const RaceSpeedConfig c = car_config();
  for (int i = 0; i < 2000; ++i) {
    RaceSpeedInputs in;
    in.steering_rad = steer(rng);
    if (i % 3 != 0) {
      in.free_path_m = path(rng);
    }
    if (i % 2 == 0) {
      in.side_clearance_m = side(rng);
    }
    const double v = race_target_speed(in, c).target_mps;
    ASSERT_GE(v, c.min_speed_mps);
    ASSERT_LE(v, c.max_speed_mps);
  }
}

// -- the speed chain's tail -----------------------------------------------------------------------

TEST(ReactiveSpeedCommandUpdateTarget, RampsAtTheLimiterAndKeepsItsBound) {
  ReactiveSpeedConfig config;
  config.min_speed_mps = 0.36;
  config.max_speed_mps = 1.0;
  config.max_acceleration_mps2 = 1.0;
  ReactiveSpeedCommand command(config);
  EXPECT_EQ(command.update_target(1.0, 0.0), 0.0);  // no elapsed time, no increase
  double previous = 0.0;
  for (int i = 0; i < 100; ++i) {
    const double v = command.update_target(1.0, 0.02);
    EXPECT_LE(v - previous, 0.02 * 1.0 + 1e-12);  // a * dt
    previous = v;
  }
  EXPECT_NEAR(previous, 1.0, 1e-9);
  // Braking is not rate limited.
  EXPECT_DOUBLE_EQ(command.update_target(0.4, 0.02), 0.4);
  // A non-finite target falls to the floor.
  EXPECT_DOUBLE_EQ(command.update_target(kNaN, 0.02), 0.36);
  command.reset();
  EXPECT_EQ(command.update_target(1.0, 0.02), 0.02);
}

TEST(ReactiveSpeedCommandUpdateTarget, TheLowPassIsAppliedBeforeTheLimiterWhenConfigured) {
  ReactiveSpeedConfig config;
  config.max_speed_mps = 1.0;
  config.max_acceleration_mps2 = 100.0;
  config.speed_time_constant_s = 0.5;
  ReactiveSpeedCommand command(config);
  command.update_target(1.0, 0.0);
  const double v = command.update_target(1.0, 0.02);
  EXPECT_NEAR(v, 0.02 / (0.5 + 0.02), 1e-12);
}

// -- percentile window ----------------------------------------------------------------------------

TEST(PercentileWindow, NearestRankOnKnownValues) {
  PercentileWindow w(10);
  for (int i = 1; i <= 10; ++i) {
    w.push(i);
  }
  EXPECT_EQ(w.size(), 10u);
  EXPECT_EQ(w.percentile(0.5), 5.0);    // rank ceil(5) = 5
  EXPECT_EQ(w.percentile(0.99), 10.0);  // rank ceil(9.9) = 10
  EXPECT_EQ(w.percentile(0.1), 1.0);
  EXPECT_EQ(w.percentile(0.0), 1.0);  // rank at least 1
  EXPECT_EQ(w.percentile(1.0), 10.0);
  EXPECT_EQ(w.percentile(7.0), 10.0);  // clamped
  EXPECT_EQ(w.percentile(-3.0), 1.0);
  EXPECT_EQ(w.percentile(kNaN), 10.0);  // garbage reads the top
  EXPECT_EQ(w.max(), 10.0);
}

TEST(PercentileWindow, OldSamplesFallOutAndGarbageIsIgnored) {
  PercentileWindow w(3);
  for (const double v : {100.0, 1.0, 2.0, 3.0}) {
    w.push(v);
  }
  w.push(kNaN);
  w.push(kInf);
  EXPECT_EQ(w.size(), 3u);
  EXPECT_EQ(w.max(), 3.0);  // the 100 is gone
  EXPECT_EQ(w.percentile(0.5), 2.0);
  w.clear();
  EXPECT_EQ(w.size(), 0u);
  EXPECT_EQ(w.percentile(0.5), 0.0);
  EXPECT_EQ(w.max(), 0.0);
}

TEST(PercentileWindow, ZeroCapacityStillHoldsOneSampleAndPushNeverReallocates) {
  PercentileWindow one(0);
  one.push(4.0);
  one.push(5.0);
  EXPECT_EQ(one.size(), 1u);
  EXPECT_EQ(one.percentile(0.5), 5.0);
  PercentileWindow w(64);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> value(0.0, 10.0);
  for (int i = 0; i < 1000; ++i) {
    w.push(value(rng));
    const double p50 = w.percentile(0.5);
    EXPECT_LE(p50, w.percentile(0.99));
    EXPECT_LE(w.percentile(0.99), w.max() + 1e-12);
  }
}

}  // namespace
}  // namespace racer_control
