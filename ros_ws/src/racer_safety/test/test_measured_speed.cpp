// L1 tests for "JUDGED SPEED FROM THE MEASURED WHEEL SPEED" (gate_logic.hpp, 2026-11-12, schema
// 0.15.0): with a fresh measured wheel speed and a usable model the TTC divides by
// max(measured, min(request, measured + a_peak * latency)) instead of the request raised to the
// decaying bound, and without either the gate is the one of test_ttc_speed.cpp, unchanged. The
// numbers are the 2026-11-12 race bag's (docs/notes/safety-race-retune-2026-11-12.md): latency
// 0.22 s, peak acceleration 2.2 m/s^2, TTC 0.60 / 0.71 s, floor 0.10 m. Includes the two brakes
// of that bag the change removes (judged 1.5 m/s with the wheels at 0.87, and a request of 0.74
// m/s from a car at rest). Limits are plain fixtures, NOT read from the yaml.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.02;
constexpr double kLatency = 0.22;
constexpr double kPeak = 2.2;
constexpr double kReach = kPeak * kLatency;  // 0.484 m/s gained before a brake bites

SafetyLimits make_limits(std::optional<MeasuredSpeedModel> m = MeasuredSpeedModel{kLatency, kPeak},
                         std::optional<double> brake = 0.60) {
  SafetyLimits limits;
  limits.steering_min_rad = -0.449;
  limits.steering_max_rad = 0.449;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.ttc_warning_s = 0.71;
  limits.ttc_brake_s = brake;
  limits.min_forward_clearance_m = 0.10;
  limits.speed_bound_deceleration_mps2 = 1.0;
  limits.measured_speed_model = m;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
}

GateInput forward(double request_mps, double range_m, std::optional<double> measured_mps,
                  double bound_mps = 0.0, bool latched = false) {
  GateInput input;
  input.command = DriveCommand{0.0, request_mps};
  input.dt_s = kDt;
  input.min_scan_range_m = range_m;
  input.measured_speed_mps = measured_mps;
  input.speed_bound_forward_mps = bound_mps;
  input.ttc_brake_latched = latched;
  return input;
}

GateInput backward(double request_mps, double range_m, std::optional<double> measured_mps) {
  GateInput input;
  input.command = DriveCommand{0.0, request_mps};
  input.dt_s = kDt;
  input.min_rear_scan_range_m = range_m;
  input.measured_speed_mps = measured_mps;
  return input;
}

const DriveCommand kCruising{0.0, 1.0};  // the last output equals a 1.0 m/s request: no ramp

TEST(MeasuredSpeed, TheTtcDividesByTheMeasuredSpeedRaisedTowardTheRequestByWhatTheCarCanGain) {
  const SafetyGateLogic gate(make_limits());
  // Wheels 0.8, request 1.2 (reachable 0.8 + 0.484 = 1.284, so judged 1.2): TTC 0.6 s at 0.72 m.
  EXPECT_TRUE(gate.evaluate(forward(1.2, 0.71, 0.8), kCruising).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(forward(1.2, 0.73, 0.8), kCruising).ttc_brake_latched);
  // Request 2.0 is capped at 1.284: TTC 0.6 s at 0.7704 m (the old gate would use 2.0: 1.2 m).
  EXPECT_TRUE(gate.evaluate(forward(2.0, 0.76, 0.8), kCruising).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(forward(2.0, 0.78, 0.8), kCruising).ttc_brake_latched);
  const SafetyGateLogic legacy(make_limits(std::nullopt));
  EXPECT_TRUE(legacy.evaluate(forward(2.0, 0.78, 0.8), kCruising).ttc_brake_latched);
}

TEST(MeasuredSpeed, ARequestBelowTheMeasuredSpeedIsRaisedToIt) {
  const SafetyGateLogic gate(make_limits());
  // The follower dropped to 0.36 but the wheels still read 0.8: judged 0.8, TTC 0.6 s at 0.48 m.
  EXPECT_TRUE(gate.evaluate(forward(0.36, 0.47, 0.8), kCruising).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(forward(0.36, 0.49, 0.8), kCruising).ttc_brake_latched);
}

TEST(MeasuredSpeed, TheDecayingBoundIsNotUsedWhileTheWheelsAreMeasured) {
  const SafetyGateLogic gate(make_limits());
  // 2026-11-12, 1751.6 s: last output 1.5 m/s, request 1.33, wheels 0.87, body distance 0.89 m.
  // The old gate judges 1.5 (0.59 s, trips); here 1.33 is reachable (0.87 + 0.484 = 1.354) and the
  // TTC is 0.67 s.
  const GateInput in = forward(1.33, 0.89, 0.87, 1.5);
  EXPECT_FALSE(gate.evaluate(in, DriveCommand{0.0, 1.5}).ttc_brake_latched);
  const SafetyGateLogic legacy(make_limits(std::nullopt));
  EXPECT_TRUE(legacy.evaluate(in, DriveCommand{0.0, 1.5}).ttc_brake_latched);
}

TEST(MeasuredSpeed, ACarAtRestIsNotJudgedAtARequestItCannotReach) {
  const SafetyGateLogic gate(make_limits());
  // 2026-11-12, 1755.8 s: wheels 0.10, request 0.74, 0.43 m of path. Judged 0.584: 0.74 s.
  EXPECT_FALSE(
      gate.evaluate(forward(0.74, 0.431, 0.10), DriveCommand{0.0, 0.74}).ttc_brake_latched);
  const SafetyGateLogic legacy(make_limits(std::nullopt));
  EXPECT_TRUE(
      legacy.evaluate(forward(0.74, 0.431, 0.10), DriveCommand{0.0, 0.74}).ttc_brake_latched);
}

TEST(MeasuredSpeed, MeasuredSpeedAwayFromTheSideCountsAsZero) {
  const SafetyGateLogic gate(make_limits());
  // Rolling backward at 0.5 with a forward request of 0.8: judged min(0.8, 0 + 0.484) = 0.484.
  EXPECT_TRUE(gate.evaluate(forward(0.8, 0.28, -0.5), DriveCommand{0.0, 0.8}).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(forward(0.8, 0.30, -0.5), DriveCommand{0.0, 0.8}).ttc_brake_latched);
}

TEST(MeasuredSpeed, TheRearGateJudgesTheReverseSpeedTheSameWay) {
  const SafetyGateLogic gate(make_limits());
  // Reversing at 0.8 with a reverse request of 2.0: judged 1.284, TTC 0.6 s at 0.7704 m.
  EXPECT_TRUE(
      gate.evaluate(backward(-2.0, 0.76, -0.8), DriveCommand{0.0, -2.0}).reverse_brake_latched);
  EXPECT_FALSE(
      gate.evaluate(backward(-2.0, 0.78, -0.8), DriveCommand{0.0, -2.0}).reverse_brake_latched);
}

TEST(MeasuredSpeed, WithoutAFreshMeasurementTheRequestRaisedToTheBoundDecides) {
  const SafetyGateLogic gate(make_limits());
  EXPECT_TRUE(gate.evaluate(forward(2.0, 0.78, std::nullopt), kCruising).ttc_brake_latched);
  for (const double bad : {kNan, kInf, -kInf}) {
    EXPECT_TRUE(gate.evaluate(forward(2.0, 0.78, bad), kCruising).ttc_brake_latched) << bad;
  }
  // The bound still holds the judged speed up with no measurement.
  EXPECT_TRUE(gate.evaluate(forward(0.36, 0.5, std::nullopt, 1.5), DriveCommand{0.0, 1.5})
                  .ttc_brake_latched);
}

TEST(MeasuredSpeed, AnUnsetOrUnusableModelIsTheOldGate) {
  // Every model the gate must not trust leaves the request judged whole (2.0 m/s, 0.78 m, wheels
  // 0.8: a usable model would not trip).
  const SafetyGateLogic usable(make_limits());
  EXPECT_FALSE(usable.evaluate(forward(2.0, 0.78, 0.8), kCruising).ttc_brake_latched);
  const std::optional<MeasuredSpeedModel> bad_models[] = {
      std::nullopt,
      MeasuredSpeedModel{kNan, kPeak},
      MeasuredSpeedModel{kLatency, kNan},
      MeasuredSpeedModel{kInf, kPeak},
      MeasuredSpeedModel{kLatency, kInf},
      MeasuredSpeedModel{0.0, kPeak},
      MeasuredSpeedModel{-0.1, kPeak},
      MeasuredSpeedModel{kLatency, 0.0},
      MeasuredSpeedModel{kLatency, -1.0},
  };
  for (const auto& m : bad_models) {
    const SafetyGateLogic gate(make_limits(m));
    EXPECT_TRUE(gate.evaluate(forward(2.0, 0.78, 0.8), kCruising).ttc_brake_latched);
  }
}

TEST(MeasuredSpeed, NoTtcBrakeThresholdMeansOnlyTheFloorWhateverTheModel) {
  const SafetyGateLogic gate(make_limits(MeasuredSpeedModel{kLatency, kPeak}, std::nullopt));
  EXPECT_FALSE(gate.evaluate(forward(1.0, 0.3, 0.8), kCruising).ttc_brake_latched);
  EXPECT_TRUE(gate.evaluate(forward(1.0, 0.05, 0.8), kCruising).ttc_brake_latched);
}

TEST(MeasuredSpeed, ARequestAwayFromTheObstacleIsNeverBraked) {
  const SafetyGateLogic gate(make_limits());
  EXPECT_FALSE(gate.evaluate(forward(-0.5, 0.2, 0.8), kCruising).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(forward(0.0, 0.2, 0.8), kCruising).ttc_brake_latched);
}

TEST(MeasuredSpeed, LatchAndReleaseKeepTheirThresholdsOnTheJudgedSpeed) {
  const SafetyGateLogic gate(make_limits());
  // Wheels 0.8, request 1.0 (reachable): judged 1.0. Release needs TTC above 0.71 s: 0.71 m.
  EXPECT_TRUE(gate.evaluate(forward(1.0, 0.70, 0.8, 0.0, true), kCruising).ttc_brake_latched);
  const GateResult released = gate.evaluate(forward(1.0, 0.72, 0.8, 0.0, true), kCruising);
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_EQ(released.releases.size(), 1u);
}

TEST(MeasuredSpeed, ThisCanOnlyRemoveATripNeverAddOne) {
  // Over a grid of requests, wheel speeds, bounds and ranges: whenever the model gate trips the old
  // gate trips too (the judged speed is never above the old one).
  const SafetyGateLogic gate(make_limits());
  const SafetyGateLogic legacy(make_limits(std::nullopt));
  for (double request = 0.1; request <= 2.0; request += 0.15) {
    for (double wheels = -0.3; wheels <= 2.0; wheels += 0.2) {
      for (double bound = 0.0; bound <= 2.0; bound += 0.5) {
        for (double range = 0.05; range <= 2.0; range += 0.07) {
          const GateInput in = forward(request, range, wheels, bound);
          const bool model_trips = gate.evaluate(in, kCruising).ttc_brake_latched;
          const bool old_trips = legacy.evaluate(in, kCruising).ttc_brake_latched;
          EXPECT_TRUE(!model_trips || old_trips)
              << request << " " << wheels << " " << bound << " " << range;
        }
      }
    }
  }
}

}  // namespace
}  // namespace racer_safety
