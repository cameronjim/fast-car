// L1 tests for the obstacle gate (step 3b of SafetyGateLogic::evaluate: TTC brake on the
// REQUESTED speed + distance floor, one latch with hysteresis). Rewritten 2026-10-06 after
// the bench limit cycle described in gate_logic.hpp ("THE OBSTACLE GATE AND ITS LATCH") and
// docs/notes/ttc-limit-cycle-2026-10-06.md.
//
// Limits are plain fixtures in the shape of config/vehicle_params.yaml, NOT read from it
// (gate_logic never includes the generated binding), same pattern as test_gate_logic.cpp.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.02;  // 50 Hz, the real control rate

// The bag: 2026-10-06T18-51-39_car_teleop. gap_follow_node requested a steady ~0.48 m/s
// with an obstacle ~0.22 m ahead.
constexpr double kBagRequestMps = 0.48;
constexpr double kBagRangeM = 0.22;
// 286 s of that run at 50 Hz.
constexpr int kBagCycles = 286 * 50;

SafetyLimits make_limits(std::optional<double> ttc_warning_s, std::optional<double> ttc_brake_s,
                         std::optional<double> min_forward_clearance_m) {
  SafetyLimits limits;
  limits.steering_min_rad = -0.4189;
  limits.steering_max_rad = 0.4189;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;  // 0.19 m/s per 50 Hz cycle, as on the car
  limits.ttc_warning_s = ttc_warning_s;
  limits.ttc_brake_s = ttc_brake_s;
  limits.min_forward_clearance_m = min_forward_clearance_m;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
}

// The thresholds the car ran with on 2026-10-06 (0.5 s brake, 1.0 s warn), floor disabled:
// isolates the TTC half of the fix.
SafetyLimits bag_night_limits() { return make_limits(1.0, 0.5, std::nullopt); }

// What config/vehicle_params.yaml holds after this fix: 1.0 s brake, 2.0 s warn, 0.30 m floor.
SafetyLimits committed_limits() { return make_limits(2.0, 1.0, 0.30); }

GateInput cycle_input(double speed_mps, double range_m, bool latched) {
  GateInput input;
  input.command = DriveCommand{0.0, speed_mps};
  input.drive_raw_age_s = 0.0;
  input.dt_s = kDt;
  input.min_scan_range_m = range_m;
  input.ttc_brake_latched = latched;
  return input;
}

bool has_activation(const GateResult& result, GateSource source, EventSeverity severity) {
  for (const auto& a : result.activations) {
    if (a.source == source && a.severity == severity) {
      return true;
    }
  }
  return false;
}

std::size_t count_records(const std::vector<SafetyEventRecord>& records, GateSource source,
                          EventSeverity severity, EventPhase phase) {
  std::size_t n = 0;
  for (const auto& r : records) {
    if (r.source == source && r.severity == severity && r.phase == phase) {
      ++n;
    }
  }
  return n;
}

// Drives the gate the way safety_node does: previous output, latch, steering-hold timer and
// event tracker all threaded from cycle to cycle.
struct Loop {
  explicit Loop(SafetyLimits limits, double dt_s = kDt) : gate(limits), dt(dt_s) {}

  GateResult step(double speed_mps, double range_m, double steering_rad = 0.0) {
    GateInput input = cycle_input(speed_mps, range_m, latched);
    input.command.steering_angle_rad = steering_rad;
    input.dt_s = dt;
    input.obstacle_hold_timer_s = hold_timer_s;
    return finish(gate.evaluate(input, previous));
  }

  // Threads one already-evaluated cycle (used for hand-built inputs such as a stale cycle).
  GateResult finish(GateResult result) {
    previous = result.output;
    latched = result.ttc_brake_latched;
    hold_timer_s = result.obstacle_hold_timer_s;
    now_s += dt;
    for (auto& r : tracker.update(result.activations, now_s, result.releases)) {
      records.push_back(r);
    }
    return result;
  }

  SafetyGateLogic gate;
  double dt;
  GateEventTracker tracker;
  DriveCommand previous{0.0, 0.0};
  bool latched = false;
  std::optional<double> hold_timer_s;
  double now_s = 0.0;
  std::vector<SafetyEventRecord> records;
};

// ---------------------------------------------------------------------------------------
// The 2026-10-06 limit cycle, reproduced with the exact bag numbers.
// ---------------------------------------------------------------------------------------

TEST(ObstacleGateLimitCycle, BagScenarioOutputStaysZeroOnEveryCycleWithTheNightsThresholds) {
  // Pre-fix this produced 0.00, 0.19, 0.38, 0.00, ... (TTC judged on the ramped output).
  Loop loop(bag_night_limits());
  for (int i = 0; i < kBagCycles; ++i) {
    const GateResult result = loop.step(kBagRequestMps, kBagRangeM);
    ASSERT_EQ(result.output.speed_mps, 0.0) << "cycle " << i << " leaked throttle";
    ASSERT_TRUE(result.ttc_brake_latched) << "cycle " << i;
    ASSERT_TRUE(result.zero_throttle) << "cycle " << i;
  }
  // One intervention for the whole obstacle episode, never released, and the rate limiter
  // never fights the brake (pre-fix: 780 flips and a rate_limit event on every ramp).
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      0u);
  EXPECT_EQ(count_records(loop.records, GateSource::kRateLimit, EventSeverity::kWarning,
                          EventPhase::kEngage),
            0u);
  EXPECT_EQ(loop.records.size(), 1u);
}

TEST(ObstacleGateLimitCycle, BagScenarioOutputStaysZeroWithTheCommittedThresholdsAndFloor) {
  Loop loop(committed_limits());
  for (int i = 0; i < kBagCycles; ++i) {
    const GateResult result = loop.step(kBagRequestMps, kBagRangeM);
    ASSERT_EQ(result.output.speed_mps, 0.0) << "cycle " << i;
  }
  EXPECT_EQ(loop.records.size(), 1u);
}

TEST(ObstacleGateLimitCycle, MidRampOutputIsBrakedOnTheVeryNextCycle) {
  // The car was at 0.38 m/s of the pre-fix ramp: the request's TTC (0.46 s) violates the
  // brake threshold, so the output goes straight to 0 (braking is never rate-limited).
  SafetyGateLogic gate(bag_night_limits());
  const GateResult result =
      gate.evaluate(cycle_input(kBagRequestMps, kBagRangeM, false), DriveCommand{0.0, 0.38});
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_FALSE(has_activation(result, GateSource::kRateLimit, EventSeverity::kWarning));
}

TEST(ObstacleGateLimitCycle, TtcIsJudgedOnTheRequestNotOnTheRateLimitedOutput) {
  // From rest the limiter would output 0.19 m/s (TTC 1.16 s, which passes 0.5 s). The
  // request's TTC is 0.46 s, so the gate must brake.
  SafetyGateLogic gate(bag_night_limits());
  const GateResult result =
      gate.evaluate(cycle_input(kBagRequestMps, kBagRangeM, false), DriveCommand{0.0, 0.0});
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_TRUE(has_activation(result, GateSource::kTtc, EventSeverity::kBrake));
  ASSERT_FALSE(result.activations.empty());
  EXPECT_NE(result.activations[0].detail.find("requested speed"), std::string::npos);
}

TEST(ObstacleGateLimitCycle, WarningZoneIsAlsoJudgedOnTheRequest) {
  // Request 5 m/s at 4 m: request TTC 0.8 s is in the warning zone even though the first
  // rate-limited output (0.19 m/s, TTC 21 s) would not be.
  SafetyGateLogic gate(bag_night_limits());
  const GateResult result = gate.evaluate(cycle_input(5.0, 4.0, false), DriveCommand{0.0, 0.0});
  EXPECT_FALSE(result.ttc_brake_latched);
  EXPECT_NEAR(result.output.speed_mps, 9.51 * kDt, 1e-12);  // still rate-limited
  EXPECT_TRUE(has_activation(result, GateSource::kTtc, EventSeverity::kInfo));
  EXPECT_TRUE(has_activation(result, GateSource::kRateLimit, EventSeverity::kWarning));
}

// ---------------------------------------------------------------------------------------
// Release: only when the REQUEST's TTC exceeds the release threshold.
// ---------------------------------------------------------------------------------------

TEST(ObstacleGateRelease, ReleasesOnlyWhenRequestTtcClearsTheWarningThresholdThenRampsUp) {
  Loop loop(bag_night_limits());
  for (int i = 0; i < 50; ++i) {
    loop.step(kBagRequestMps, kBagRangeM);
  }
  ASSERT_TRUE(loop.latched);

  // The obstacle backs away. Request TTC 0.625 s .. exactly 1.0 s: above the 0.5 s brake
  // threshold (pre-fix this would already have let the car go) but not ABOVE the 1.0 s
  // warning threshold, so the latch holds and the output stays 0.
  for (const double range_m : {0.30, 0.40, 0.47, 0.48}) {
    const GateResult held = loop.step(kBagRequestMps, range_m);
    EXPECT_TRUE(held.ttc_brake_latched) << "released at range " << range_m;
    EXPECT_EQ(held.output.speed_mps, 0.0) << "range " << range_m;
    EXPECT_TRUE(held.releases.empty());
    EXPECT_FALSE(has_activation(held, GateSource::kTtc, EventSeverity::kInfo))
        << "advisory emitted while latched";
  }

  // 0.49 m: request TTC 1.02 s > 1.0 s -> release on this cycle, with the note.
  const GateResult released = loop.step(kBagRequestMps, 0.49);
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_FALSE(released.zero_throttle);
  ASSERT_EQ(released.releases.size(), 1u);
  EXPECT_EQ(released.releases[0].source, GateSource::kTtc);
  EXPECT_EQ(released.releases[0].severity, EventSeverity::kBrake);
  EXPECT_NE(released.releases[0].detail.find("ttc brake released"), std::string::npos);
  // The rate limit still applies to what is output after the release.
  EXPECT_NEAR(released.output.speed_mps, 9.51 * kDt, 1e-9);

  const GateResult next = loop.step(kBagRequestMps, 0.49);
  EXPECT_NEAR(next.output.speed_mps, 2.0 * 9.51 * kDt, 1e-9);
  EXPECT_TRUE(next.releases.empty());
  const GateResult last = loop.step(kBagRequestMps, 0.49);
  EXPECT_NEAR(last.output.speed_mps, kBagRequestMps, 1e-9);
  EXPECT_FALSE(last.ttc_brake_latched);

  // Exactly one engage and one release record for the whole episode, and the release
  // record says why.
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
  ASSERT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      1u);
  for (const auto& r : loop.records) {
    if (r.phase == EventPhase::kRelease && r.source == GateSource::kTtc) {
      EXPECT_NE(r.detail.find("ttc brake released"), std::string::npos);
      EXPECT_GT(r.duration_s, 0.0);
    }
  }
}

TEST(ObstacleGateRelease, WithoutAWarningThresholdTheHysteresisFactorSetsTheReleasePoint) {
  // Brake 0.5 s, no warning: release needs TTC > 0.5 * 2.0 = 1.0 s.
  SafetyGateLogic gate(make_limits(std::nullopt, 0.5, std::nullopt));
  const GateResult held = gate.evaluate(cycle_input(0.48, 0.48, true), DriveCommand{});
  EXPECT_TRUE(held.ttc_brake_latched);  // TTC exactly 1.0 s, not above it
  const GateResult released = gate.evaluate(cycle_input(0.48, 0.49, true), DriveCommand{});
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_EQ(released.releases.size(), 1u);
}

TEST(ObstacleGateRelease, AWarningThresholdNotAboveTheBrakeFallsBackToTheFactor) {
  // Misconfigured warning (0.4 s <= 0.5 s brake) must not become a release point BELOW the
  // brake threshold: the factor applies instead (release above 1.0 s).
  SafetyGateLogic gate(make_limits(0.4, 0.5, std::nullopt));
  EXPECT_TRUE(gate.evaluate(cycle_input(0.48, 0.40, true), DriveCommand{}).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(cycle_input(0.48, 0.49, true), DriveCommand{}).ttc_brake_latched);
}

TEST(ObstacleGateRelease, DefaultHysteresisFactorsMeetTheSpec) {
  const SafetyLimits defaults;
  EXPECT_GE(defaults.ttc_release_hysteresis_factor, 2.0);
  EXPECT_GT(defaults.clearance_release_factor, 1.0);
}

TEST(ObstacleGateRelease, NonFiniteHysteresisFactorNeverReleasesFailClosed) {
  SafetyLimits limits = make_limits(std::nullopt, 0.5, std::nullopt);
  limits.ttc_release_hysteresis_factor = kNan;
  SafetyGateLogic gate(limits);
  // Even with nothing in the sector, a NaN release threshold holds the latch.
  const GateResult result = gate.evaluate(cycle_input(0.48, kInf, true), DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_EQ(result.output.speed_mps, 0.0);
}

TEST(ObstacleGateRelease, StoppedRequestReleasesATtcOnlyLatch) {
  // A zero request has no TTC (infinite), so the TTC half clears; with no floor configured
  // the latch releases even though the obstacle is still close.
  SafetyGateLogic gate(bag_night_limits());
  const GateResult result = gate.evaluate(cycle_input(0.0, kBagRangeM, true), DriveCommand{});
  EXPECT_FALSE(result.ttc_brake_latched);
  EXPECT_EQ(result.releases.size(), 1u);
}

TEST(ObstacleGateRelease, NothingInTheSectorReleases) {
  SafetyGateLogic gate(committed_limits());
  const GateResult result = gate.evaluate(cycle_input(0.48, kInf, true), DriveCommand{});
  EXPECT_FALSE(result.ttc_brake_latched);
  ASSERT_EQ(result.releases.size(), 1u);
  EXPECT_NE(result.releases[0].detail.find("ttc brake released"), std::string::npos);
}

TEST(ObstacleGateRelease, GarbageRangeNeverReleasesFailClosed) {
  SafetyGateLogic gate(committed_limits());
  for (const double garbage : {kNan, 0.0, -1.0, -kInf}) {
    const GateResult result = gate.evaluate(cycle_input(0.48, garbage, true), DriveCommand{});
    EXPECT_TRUE(result.ttc_brake_latched) << "garbage range " << garbage << " released";
    EXPECT_EQ(result.output.speed_mps, 0.0);
    EXPECT_TRUE(result.releases.empty());
    ASSERT_FALSE(result.activations.empty());
    EXPECT_NE(result.activations[0].detail.find("latch held"), std::string::npos);
  }
}

TEST(ObstacleGateRelease, GarbageRangeNeverTrips) {
  SafetyGateLogic gate(committed_limits());
  for (const double garbage : {kNan, 0.0, -1.0, -kInf}) {
    const GateResult result = gate.evaluate(cycle_input(0.48, garbage, false), DriveCommand{});
    EXPECT_FALSE(result.ttc_brake_latched) << "garbage range " << garbage << " tripped";
  }
}

TEST(ObstacleGateRelease, NoThresholdsAtAllReleasesAStaleLatch) {
  // Only reachable with an input latch from a differently configured gate: with both halves
  // disabled nothing can hold it.
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, std::nullopt));
  const GateResult result = gate.evaluate(cycle_input(0.48, 0.1, true), DriveCommand{});
  EXPECT_FALSE(result.ttc_brake_latched);
  EXPECT_EQ(result.releases.size(), 1u);
  EXPECT_FALSE(has_activation(result, GateSource::kTtc, EventSeverity::kInfo));
}

// ---------------------------------------------------------------------------------------
// Distance floor.
// ---------------------------------------------------------------------------------------

TEST(ObstacleGateFloor, CrawlRequestBelowTheFloorBrakesRegardlessOfSpeed) {
  // 0.05 m/s at 0.25 m is a TTC of 5 s, far above any TTC threshold; the floor brakes anyway.
  SafetyGateLogic gate(committed_limits());
  const GateResult result = gate.evaluate(cycle_input(0.05, 0.25, false), DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kTtc);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kBrake);
  EXPECT_NE(result.activations[0].detail.find("forward clearance"), std::string::npos);
}

TEST(ObstacleGateFloor, FloorWorksWithTheTtcHalfDisabled) {
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, 0.30));
  EXPECT_TRUE(gate.evaluate(cycle_input(0.05, 0.25, false), DriveCommand{}).ttc_brake_latched);
}

TEST(ObstacleGateFloor, ExactlyAtTheFloorDoesNotTrip) {
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, 0.30));
  EXPECT_FALSE(gate.evaluate(cycle_input(0.05, 0.30, false), DriveCommand{}).ttc_brake_latched);
}

TEST(ObstacleGateFloor, StoppedRequestDoesNotTripTheFloor) {
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, 0.30));
  const GateResult result = gate.evaluate(cycle_input(0.0, 0.10, false), DriveCommand{});
  EXPECT_FALSE(result.ttc_brake_latched);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(ObstacleGateFloor, NothingInTheSectorDoesNotTripTheFloor) {
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, 0.30));
  EXPECT_FALSE(gate.evaluate(cycle_input(0.05, kInf, false), DriveCommand{}).ttc_brake_latched);
}

TEST(ObstacleGateFloor, ReleasesOnlyAboveFloorTimesReleaseFactor) {
  // Floor 0.30 m, factor 1.5: release needs range > 0.45 m. Request 0.05 m/s so the TTC
  // half (2.0 s release) is clear well before that (0.45 / 0.05 = 9 s). 0.44 / 0.46 rather
  // than exactly 0.45 because 0.30 * 1.5 is not exactly 0.45 in binary floating point.
  SafetyGateLogic gate(committed_limits());
  EXPECT_TRUE(gate.evaluate(cycle_input(0.05, 0.35, true), DriveCommand{}).ttc_brake_latched);
  EXPECT_TRUE(gate.evaluate(cycle_input(0.05, 0.44, true), DriveCommand{}).ttc_brake_latched);
  const GateResult released = gate.evaluate(cycle_input(0.05, 0.46, true), DriveCommand{});
  EXPECT_FALSE(released.ttc_brake_latched);
  ASSERT_EQ(released.releases.size(), 1u);
  EXPECT_NE(released.releases[0].detail.find("ttc brake released"), std::string::npos);
}

TEST(ObstacleGateFloor, StoppedRequestKeepsAFloorLatchWhileTheObstacleIsClose) {
  SafetyGateLogic gate(committed_limits());
  const GateResult result = gate.evaluate(cycle_input(0.0, 0.20, true), DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_TRUE(result.releases.empty());
}

TEST(ObstacleGateFloor, ReverseRequestPassesWhileLatchedAndTheLatchHolds) {
  // Backing away from the obstacle is allowed; the latch stays until the clearance is back.
  SafetyGateLogic gate(committed_limits());
  GateInput input = cycle_input(-1.0, 0.20, true);
  input.dt_s = 1e6;  // keep the rate limiter out of this test
  const GateResult result = gate.evaluate(input, DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, -1.0);
}

TEST(ObstacleGateFloor, TripWinsOverAMisconfiguredReleaseFactor) {
  // clearance_release_factor 0.5 would put the release clearance (0.15 m) inside the floor
  // (0.30 m). A forward request at 0.20 m still trips, so the output can never pulse.
  SafetyLimits limits = make_limits(std::nullopt, std::nullopt, 0.30);
  limits.clearance_release_factor = 0.5;
  SafetyGateLogic gate(limits);
  const GateResult result = gate.evaluate(cycle_input(0.05, 0.20, true), DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_EQ(result.output.speed_mps, 0.0);
}

TEST(ObstacleGateFloor, TtcAndFloorTrippingTogetherReportTheTtcDetail) {
  SafetyGateLogic gate(committed_limits());
  const GateResult result =
      gate.evaluate(cycle_input(kBagRequestMps, kBagRangeM, false), DriveCommand{});
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_NE(result.activations[0].detail.find("time-to-collision"), std::string::npos);
}

// ---------------------------------------------------------------------------------------
// Documented safety properties preserved.
// ---------------------------------------------------------------------------------------

TEST(ObstacleGateProperties, SteeringIsKeptAndStillRateLimitedDuringALatchedBrake) {
  SafetyGateLogic gate(committed_limits());
  GateInput input = cycle_input(kBagRequestMps, kBagRangeM, false);
  input.command.steering_angle_rad = 0.2;
  const GateResult result = gate.evaluate(input, DriveCommand{0.1, 0.0});
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_NEAR(result.output.steering_angle_rad, 0.1 + 3.2 * kDt, 1e-12);
}

TEST(ObstacleGateProperties, ABrakeFromSpeedIsNotRateLimited) {
  SafetyGateLogic gate(committed_limits());
  const GateResult result = gate.evaluate(cycle_input(3.0, 0.5, false), DriveCommand{0.0, 3.0});
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_FALSE(has_activation(result, GateSource::kRateLimit, EventSeverity::kWarning));
}

TEST(ObstacleGateProperties, WatchdogHasPriorityAndHoldsTheLatch) {
  SafetyGateLogic gate(committed_limits());
  GateInput input = cycle_input(kBagRequestMps, kInf, true);  // would release if evaluated
  input.drive_raw_age_s = 5.0;
  const GateResult result = gate.evaluate(input, DriveCommand{0.15, 0.0});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.15);
  ASSERT_EQ(result.activations.size(), 2u);
  EXPECT_EQ(result.activations[0].source, GateSource::kWatchdog);
  EXPECT_EQ(result.activations[1].source, GateSource::kTtc);
  EXPECT_EQ(result.activations[1].severity, EventSeverity::kBrake);
  EXPECT_TRUE(result.releases.empty());
}

TEST(ObstacleGateProperties, CommandSanityHoldsTheLatch) {
  SafetyGateLogic gate(committed_limits());
  GateInput input = cycle_input(kNan, kInf, true);
  const GateResult result = gate.evaluate(input, DriveCommand{});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_TRUE(has_activation(result, GateSource::kCommandSanity, EventSeverity::kBrake));
  EXPECT_TRUE(has_activation(result, GateSource::kTtc, EventSeverity::kBrake));
}

TEST(ObstacleGateProperties, WatchdogBlipDoesNotSplitTheEngagement) {
  Loop loop(committed_limits());
  for (int i = 0; i < 10; ++i) {
    loop.step(kBagRequestMps, kBagRangeM);
  }
  // One stale cycle in the middle of the episode.
  GateInput stale = cycle_input(kBagRequestMps, kBagRangeM, loop.latched);
  stale.drive_raw_age_s = 1.0;
  const GateResult blip = loop.gate.evaluate(stale, loop.previous);
  loop.latched = blip.ttc_brake_latched;
  for (auto& r : loop.tracker.update(blip.activations, loop.now_s += kDt, blip.releases)) {
    loop.records.push_back(r);
  }
  for (int i = 0; i < 10; ++i) {
    loop.step(kBagRequestMps, kBagRangeM);
  }
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      0u);
}

TEST(ObstacleGateProperties, UnconfiguredTtcWithFloorOnlyNeverEmitsAnAdvisory) {
  SafetyGateLogic gate(make_limits(std::nullopt, std::nullopt, 0.30));
  const GateResult result = gate.evaluate(cycle_input(5.0, 1.0, false), DriveCommand{});
  EXPECT_FALSE(has_activation(result, GateSource::kTtc, EventSeverity::kInfo));
}

// ---------------------------------------------------------------------------------------
// Steering hold while parked on the obstacle latch (gate_logic.hpp, "STEERING HOLD WHILE
// PARKED ON THE OBSTACLE LATCH"). Added 2026-10-06 late after the owner watched the servo
// hunt on the stand while the latch held the car still.
// ---------------------------------------------------------------------------------------

// A dt and hold time that are exact in binary floating point, so "engages after exactly
// 0.5 s" is an exact cycle count: the timer reads 0, 0.125, 0.25, 0.375, 0.5 on the first
// five latched cycles and the hold engages on the fifth.
constexpr double kHoldDt = 0.125;
constexpr double kHoldAfterS = 0.5;
constexpr int kCyclesBeforeHold = 4;  // timer 0 .. 0.375
constexpr double kParkRangeM = 0.10;  // inside the 0.20 m floor: a forward request latches
constexpr double kClearRangeM = kInf;
constexpr double kRequestMps = 0.3;

// The values config/vehicle_params.yaml holds after this change (schema 0.7.3): 0.45 s warn
// (release line), 0.35 s brake, 0.20 m floor, 0.5 s steering hold.
SafetyLimits hold_limits(double hold_after_s = kHoldAfterS) {
  SafetyLimits limits = make_limits(0.45, 0.35, 0.20);
  limits.obstacle_steering_hold_after_s = hold_after_s;
  return limits;
}

const GateActivation* find_activation(const GateResult& result, GateSource source,
                                      EventSeverity severity) {
  for (const auto& a : result.activations) {
    if (a.source == source && a.severity == severity) {
      return &a;
    }
  }
  return nullptr;
}

// Drives on a clear road with steering `steering_rad` until the output has converged.
void settle_clear(Loop& loop, double steering_rad) {
  for (int i = 0; i < 20; ++i) {
    loop.step(kRequestMps, kClearRangeM, steering_rad);
  }
  ASSERT_DOUBLE_EQ(loop.previous.steering_angle_rad, steering_rad);
  ASSERT_FALSE(loop.latched);
}

TEST(ObstacleGateSteeringHold, EngagesOnlyAfterTheConfiguredTimeThenFreezesTheHoldStartAngle) {
  Loop loop(hold_limits(), kHoldDt);
  settle_clear(loop, 0.1);

  // Parked on the obstacle. Before the hold time the steering passes as today: these steps
  // (0.1 rad each at most) are inside the 3.2 rad/s * 0.125 s = 0.4 rad rate limit, so the
  // output is exactly the request, cycle by cycle.
  const double before_hold[kCyclesBeforeHold] = {0.2, 0.3, 0.1, 0.25};
  for (int i = 0; i < kCyclesBeforeHold; ++i) {
    const GateResult r = loop.step(kRequestMps, kParkRangeM, before_hold[i]);
    ASSERT_TRUE(r.ttc_brake_latched);
    EXPECT_EQ(r.output.speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, before_hold[i]) << "cycle " << i;
    ASSERT_TRUE(r.obstacle_hold_timer_s.has_value());
    EXPECT_DOUBLE_EQ(*r.obstacle_hold_timer_s, kHoldDt * i);
    EXPECT_EQ(find_activation(r, GateSource::kTtc, EventSeverity::kInfo), nullptr)
        << "hold engaged early, cycle " << i;
  }

  // Fifth latched cycle: 0.5 s held at zero. The steering freezes at the angle the output had
  // when the hold started (0.25, the previous output), whatever is requested now.
  const GateResult engaged = loop.step(kRequestMps, kParkRangeM, -0.3);
  ASSERT_TRUE(engaged.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(*engaged.obstacle_hold_timer_s, kHoldAfterS);
  EXPECT_DOUBLE_EQ(engaged.output.steering_angle_rad, 0.25);
  EXPECT_EQ(engaged.output.speed_mps, 0.0);
  const GateActivation* hold = find_activation(engaged, GateSource::kTtc, EventSeverity::kInfo);
  ASSERT_NE(hold, nullptr);
  EXPECT_NE(hold->detail.find("steering held while obstacle-latched"), std::string::npos);
  EXPECT_NE(hold->detail.find("0.250000"), std::string::npos) << hold->detail;
  // The hold is not a rate-limit clamp.
  EXPECT_FALSE(has_activation(engaged, GateSource::kRateLimit, EventSeverity::kWarning));

  // Frozen while the request keeps wandering, for as long as the latch holds.
  for (const double request : {-0.4, 0.4, 0.0, -0.1, 0.35, 0.05, -0.25, 0.3}) {
    const GateResult r = loop.step(kRequestMps, kParkRangeM, request);
    EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, 0.25) << "request " << request;
    EXPECT_EQ(r.output.speed_mps, 0.0);
    EXPECT_TRUE(r.releases.empty());
  }

  // The obstacle clears: the latch releases and the steering is live again (rate-limited from
  // the held angle) on the same cycle, with the hold's release note.
  const GateResult released = loop.step(kRequestMps, kClearRangeM, -0.1);
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_FALSE(released.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(released.output.steering_angle_rad, -0.1);
  EXPECT_EQ(find_activation(released, GateSource::kTtc, EventSeverity::kInfo), nullptr);
  bool hold_note = false;
  for (const auto& note : released.releases) {
    if (note.source == GateSource::kTtc && note.severity == EventSeverity::kInfo) {
      hold_note = note.detail.find("steering hold released") != std::string::npos;
    }
  }
  EXPECT_TRUE(hold_note);

  // ONE hold engagement and one release for the episode; the brake's own pair is unchanged.
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kEngage), 1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kRelease),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      1u);
  for (const auto& r : loop.records) {
    if (r.source == GateSource::kTtc && r.severity == EventSeverity::kInfo &&
        r.phase == EventPhase::kRelease) {
      EXPECT_NE(r.detail.find("steering hold released"), std::string::npos);
    }
  }
}

TEST(ObstacleGateSteeringHold, ABrakeShorterThanTheHoldTimeNeverHolds) {
  Loop loop(hold_limits(), kHoldDt);
  settle_clear(loop, 0.0);
  const double requests[kCyclesBeforeHold] = {0.1, -0.1, 0.2, 0.15};
  for (int i = 0; i < kCyclesBeforeHold; ++i) {
    const GateResult r = loop.step(kRequestMps, kParkRangeM, requests[i]);
    ASSERT_TRUE(r.ttc_brake_latched);
    EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, requests[i]);
  }
  const GateResult released = loop.step(kRequestMps, kClearRangeM, 0.05);
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_DOUBLE_EQ(released.output.steering_angle_rad, 0.05);
  for (const auto& note : released.releases) {
    EXPECT_NE(note.severity, EventSeverity::kInfo) << "a hold that never engaged released";
  }
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kEngage), 0u);
}

TEST(ObstacleGateSteeringHold, AtTheRealRateSteeringIsRateLimitedUntilTheHoldThenFrozen) {
  // 50 Hz, the committed 0.5 s: the latched car steers toward a far request at 3.2 rad/s *
  // 0.02 s = 0.064 rad per cycle, and the hold freezes it about 0.5 s after the latch.
  Loop loop(hold_limits(), kDt);
  settle_clear(loop, 0.0);
  int engaged_cycle = -1;
  double angle_before = 0.0;
  for (int i = 0; i < 60; ++i) {
    const double previous = loop.previous.steering_angle_rad;
    // Wander between the two locks every 5 cycles so the request is always far away.
    const double request = (i / 5) % 2 == 0 ? 0.4 : -0.4;
    const GateResult r = loop.step(kRequestMps, kParkRangeM, request);
    ASSERT_TRUE(r.ttc_brake_latched);
    ASSERT_EQ(r.output.speed_mps, 0.0);
    if (find_activation(r, GateSource::kTtc, EventSeverity::kInfo) != nullptr) {
      if (engaged_cycle < 0) {
        engaged_cycle = i;
        angle_before = previous;
      }
      EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, angle_before) << "cycle " << i;
    } else {
      ASSERT_LT(engaged_cycle, 0) << "hold let go at cycle " << i << " while still latched";
      EXPECT_NEAR(std::fabs(r.output.steering_angle_rad - previous), 3.2 * kDt, 1e-12)
          << "cycle " << i;
      EXPECT_TRUE(has_activation(r, GateSource::kRateLimit, EventSeverity::kWarning));
    }
  }
  // The timer starts at 0 on the latching cycle (cycle 0), so the hold engages once 0.5 s of
  // dt has accumulated: cycle 25, or 26 if the floating-point sum of 0.02 lands a hair short.
  ASSERT_GE(engaged_cycle, 25);
  EXPECT_LE(engaged_cycle, 26);
}

TEST(ObstacleGateSteeringHold, AReverseRequestWhileHeldGivesSteeringBackAndRestartsTheTimer) {
  Loop loop(hold_limits(), kHoldDt);
  settle_clear(loop, 0.2);
  for (int i = 0; i <= kCyclesBeforeHold; ++i) {
    loop.step(kRequestMps, kParkRangeM, 0.2);
  }
  ASSERT_TRUE(loop.hold_timer_s.has_value());
  ASSERT_TRUE(loop.gate.steering_hold_engaged(loop.hold_timer_s));

  // Backing away: the latch stays (obstacle inside the release clearance), the reverse request
  // passes, the output is no longer zero, so the hold lets go and steering follows.
  const GateResult reversing = loop.step(-0.5, kParkRangeM, -0.1);
  EXPECT_TRUE(reversing.ttc_brake_latched);
  EXPECT_LT(reversing.output.speed_mps, 0.0);
  EXPECT_FALSE(reversing.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(reversing.output.steering_angle_rad, -0.1);
  ASSERT_EQ(reversing.releases.size(), 1u);
  EXPECT_EQ(reversing.releases[0].severity, EventSeverity::kInfo);
  EXPECT_NE(reversing.releases[0].detail.find("steering hold released"), std::string::npos);

  // Forward again into the obstacle: zero output, the timer restarts from 0, live steering.
  const GateResult again = loop.step(kRequestMps, kParkRangeM, 0.0);
  EXPECT_EQ(again.output.speed_mps, 0.0);
  ASSERT_TRUE(again.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(*again.obstacle_hold_timer_s, 0.0);
  EXPECT_DOUBLE_EQ(again.output.steering_angle_rad, 0.0);
}

TEST(ObstacleGateSteeringHold, ALatchedZeroRequestStillRollingBackwardsDoesNotStartTheTimer) {
  // Previous output -1.0 m/s, zero request while latched: the increase toward zero is
  // rate-limited, so the output is not yet zero and the timer stays cleared.
  SafetyGateLogic gate(hold_limits(0.0));
  GateInput input = cycle_input(0.0, kParkRangeM, true);
  input.obstacle_hold_timer_s = 3.0;
  const GateResult result = gate.evaluate(input, DriveCommand{0.1, -1.0});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_LT(result.output.speed_mps, 0.0);
  EXPECT_FALSE(result.obstacle_hold_timer_s.has_value());
  EXPECT_EQ(find_activation(result, GateSource::kTtc, EventSeverity::kInfo), nullptr);
  ASSERT_EQ(result.releases.size(), 1u);  // the input timer said "engaged"
  EXPECT_EQ(result.releases[0].severity, EventSeverity::kInfo);
}

TEST(ObstacleGateSteeringHold, AWatchdogBlipKeepsTheHoldAndHoldsTheTimer) {
  Loop loop(hold_limits(), kHoldDt);
  settle_clear(loop, -0.15);
  for (int i = 0; i <= kCyclesBeforeHold; ++i) {
    loop.step(kRequestMps, kParkRangeM, -0.15);
  }
  ASSERT_TRUE(loop.gate.steering_hold_engaged(loop.hold_timer_s));
  const double timer_before = *loop.hold_timer_s;

  GateInput stale = cycle_input(kRequestMps, kParkRangeM, loop.latched);
  stale.command.steering_angle_rad = 0.3;
  stale.drive_raw_age_s = 1.0;
  stale.dt_s = kHoldDt;
  stale.obstacle_hold_timer_s = loop.hold_timer_s;
  const GateResult blip = loop.finish(loop.gate.evaluate(stale, loop.previous));
  EXPECT_EQ(blip.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(blip.output.steering_angle_rad, -0.15);
  ASSERT_TRUE(blip.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(*blip.obstacle_hold_timer_s, timer_before);  // held, not advanced
  EXPECT_TRUE(has_activation(blip, GateSource::kWatchdog, EventSeverity::kBrake));
  EXPECT_TRUE(has_activation(blip, GateSource::kTtc, EventSeverity::kBrake));
  const GateActivation* hold = find_activation(blip, GateSource::kTtc, EventSeverity::kInfo);
  ASSERT_NE(hold, nullptr);
  EXPECT_NE(hold->detail.find("steering held while obstacle-latched"), std::string::npos);

  const GateResult after = loop.step(kRequestMps, kParkRangeM, 0.3);
  EXPECT_DOUBLE_EQ(after.output.steering_angle_rad, -0.15);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kEngage), 1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kRelease),
      0u);
}

TEST(ObstacleGateSteeringHold, AShortCircuitBeforeTheHoldTimeHoldsTheTimerWithoutHolding) {
  SafetyGateLogic gate(hold_limits());
  GateInput input = cycle_input(kNan, kParkRangeM, true);  // command sanity
  input.obstacle_hold_timer_s = 0.25;
  const GateResult result = gate.evaluate(input, DriveCommand{0.05, 0.0});
  EXPECT_TRUE(has_activation(result, GateSource::kCommandSanity, EventSeverity::kBrake));
  ASSERT_TRUE(result.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(*result.obstacle_hold_timer_s, 0.25);
  EXPECT_EQ(find_activation(result, GateSource::kTtc, EventSeverity::kInfo), nullptr);
}

TEST(ObstacleGateSteeringHold, CommandSanityWithAnEngagedHoldKeepsReportingIt) {
  SafetyGateLogic gate(hold_limits());
  GateInput input = cycle_input(kNan, kParkRangeM, true);
  input.obstacle_hold_timer_s = 2.0;
  const GateResult result = gate.evaluate(input, DriveCommand{0.05, 0.0});
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.05);
  EXPECT_TRUE(has_activation(result, GateSource::kTtc, EventSeverity::kInfo));
}

TEST(ObstacleGateSteeringHold, AShortCircuitWithoutTheLatchDoesNotApplyTheHold) {
  // Watchdog/sanity keep their own behaviour (zero speed, previous steering) and, with no
  // latch, clear the timer and report no hold, even from an inconsistent input timer.
  SafetyGateLogic gate(hold_limits());
  GateInput input = cycle_input(kRequestMps, kParkRangeM, false);
  input.drive_raw_age_s = 5.0;
  input.obstacle_hold_timer_s = 2.0;
  const GateResult result = gate.evaluate(input, DriveCommand{0.2, 0.0});
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.2);
  EXPECT_EQ(result.output.speed_mps, 0.0);
  EXPECT_FALSE(result.obstacle_hold_timer_s.has_value());
  EXPECT_EQ(find_activation(result, GateSource::kTtc, EventSeverity::kInfo), nullptr);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kWatchdog);
}

TEST(ObstacleGateSteeringHold, GarbageTimerInputRestartsTheTimerAndDoesNotHold) {
  SafetyGateLogic gate(hold_limits());
  for (const double garbage : {kNan, kInf, -kInf, -1.0}) {
    GateInput input = cycle_input(kRequestMps, kParkRangeM, true);
    input.command.steering_angle_rad = 0.1;
    input.dt_s = 1.0;  // keep the steering rate limiter out of this test
    input.obstacle_hold_timer_s = garbage;
    const GateResult result = gate.evaluate(input, DriveCommand{0.0, 0.0});
    ASSERT_TRUE(result.obstacle_hold_timer_s.has_value()) << garbage;
    EXPECT_DOUBLE_EQ(*result.obstacle_hold_timer_s, 0.0) << garbage;
    EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.1) << garbage;
    EXPECT_EQ(find_activation(result, GateSource::kTtc, EventSeverity::kInfo), nullptr);
  }
}

TEST(ObstacleGateSteeringHold, GarbageDtDoesNotAdvanceTheTimer) {
  SafetyGateLogic gate(hold_limits());
  GateInput input = cycle_input(kRequestMps, kParkRangeM, true);
  input.obstacle_hold_timer_s = 0.375;
  input.dt_s = kNan;
  const GateResult result = gate.evaluate(input, DriveCommand{});
  ASSERT_TRUE(result.obstacle_hold_timer_s.has_value());
  EXPECT_DOUBLE_EQ(*result.obstacle_hold_timer_s, 0.375);
}

TEST(ObstacleGateSteeringHold, UnsetOrNanHoldTimeNeverHolds) {
  const SafetyLimits defaults;
  EXPECT_TRUE(std::isinf(defaults.obstacle_steering_hold_after_s));
  EXPECT_GT(defaults.obstacle_steering_hold_after_s, 0.0);
  for (const double hold_after_s : {kInf, kNan}) {
    SafetyGateLogic gate(hold_limits(hold_after_s));
    GateInput input = cycle_input(kRequestMps, kParkRangeM, true);
    input.command.steering_angle_rad = 0.1;
    input.dt_s = 1.0;  // keep the steering rate limiter out of this test
    input.obstacle_hold_timer_s = 1e9;
    const GateResult result = gate.evaluate(input, DriveCommand{});
    EXPECT_FALSE(gate.steering_hold_engaged(result.obstacle_hold_timer_s));
    EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.1);
  }
}

TEST(ObstacleGateSteeringHold, ZeroHoldTimeFreezesOnTheFirstLatchedCycle) {
  SafetyGateLogic gate(hold_limits(0.0));
  GateInput input = cycle_input(kRequestMps, kParkRangeM, false);
  input.command.steering_angle_rad = 0.3;
  const GateResult result = gate.evaluate(input, DriveCommand{-0.05, 0.0});
  EXPECT_TRUE(result.ttc_brake_latched);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, -0.05);
  EXPECT_TRUE(has_activation(result, GateSource::kTtc, EventSeverity::kInfo));
}

TEST(ObstacleGateSteeringHold, TheHeldAngleIsBoundsClampedAndNeverNan) {
  // Neither previous output is producible by the gate itself; defensive only.
  SafetyGateLogic gate(hold_limits(0.0));
  const GateInput input = cycle_input(kRequestMps, kParkRangeM, true);
  EXPECT_DOUBLE_EQ(gate.evaluate(input, DriveCommand{0.9, 0.0}).output.steering_angle_rad, 0.4189);
  EXPECT_DOUBLE_EQ(gate.evaluate(input, DriveCommand{kNan, 0.0}).output.steering_angle_rad, 0.0);
}

TEST(ObstacleGateSteeringHold, NotLatchedMeansNoTimerAndNoHold) {
  SafetyGateLogic gate(hold_limits());
  GateInput input = cycle_input(kRequestMps, kClearRangeM, false);
  input.obstacle_hold_timer_s = 0.25;  // stale input with no latch
  const GateResult result = gate.evaluate(input, DriveCommand{});
  EXPECT_FALSE(result.obstacle_hold_timer_s.has_value());
  EXPECT_TRUE(result.releases.empty());
}

// ---------------------------------------------------------------------------------------
// GateEventTracker release notes.
// ---------------------------------------------------------------------------------------

TEST(GateEventTrackerReleaseNotes, NoteBecomesTheReleaseDetailFirstNoteWins) {
  GateEventTracker tracker;
  const std::vector<GateActivation> engaged{
      GateActivation{GateSource::kTtc, EventSeverity::kBrake, "tripped"}};
  ASSERT_EQ(tracker.update(engaged, 0.0).size(), 1u);
  const std::vector<GateActivation> notes{
      GateActivation{GateSource::kTtc, EventSeverity::kBrake, "ttc brake released: first"},
      GateActivation{GateSource::kTtc, EventSeverity::kBrake, "second"}};
  const auto records = tracker.update({}, 1.5, notes);
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records[0].phase, EventPhase::kRelease);
  EXPECT_DOUBLE_EQ(records[0].duration_s, 1.5);
  EXPECT_NE(records[0].detail.find("ttc brake released: first"), std::string::npos);
  EXPECT_EQ(records[0].detail.find("second"), std::string::npos);
}

TEST(GateEventTrackerReleaseNotes, NoteForSomethingNotReleasingCreatesNoRecord) {
  GateEventTracker tracker;
  const std::vector<GateActivation> notes{
      GateActivation{GateSource::kTtc, EventSeverity::kBrake, "ttc brake released"}};
  EXPECT_TRUE(tracker.update({}, 0.0, notes).empty());
}

}  // namespace
}  // namespace racer_safety
