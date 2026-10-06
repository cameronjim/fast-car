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

// Drives the gate the way safety_node does: previous output, latch and event tracker all
// threaded from cycle to cycle.
struct Loop {
  explicit Loop(SafetyLimits limits) : gate(limits) {}

  GateResult step(double speed_mps, double range_m) {
    GateInput input = cycle_input(speed_mps, range_m, latched);
    GateResult result = gate.evaluate(input, previous);
    previous = result.output;
    latched = result.ttc_brake_latched;
    now_s += kDt;
    for (auto& r : tracker.update(result.activations, now_s, result.releases)) {
      records.push_back(r);
    }
    return result;
  }

  SafetyGateLogic gate;
  GateEventTracker tracker;
  DriveCommand previous{0.0, 0.0};
  bool latched = false;
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
