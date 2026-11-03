// L1 tests for "SPEED FOR THE TTC" (gate_logic.hpp, 2026-11-02, schema 0.13.0): the TTC divides
// by the request raised to the decaying bound of recent outputs and to the fresh measured wheel
// speed, never lowered below the request. Includes the 2026-10-24 wall-follow contact (request
// dropped 1.97 -> 1.0 m/s while the car was still faster) and a re-run of the 2026-10-06 limit
// cycle with the bound on, which must stay fixed.
//
// Limits are plain fixtures in the shape of config/vehicle_params.yaml, NOT read from it.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.02;
constexpr double kDecel = 1.0;  // actuation.stop_deceleration_mps2, schema 0.13.0

SafetyLimits make_limits(std::optional<double> decel = kDecel) {
  SafetyLimits limits;
  limits.steering_min_rad = -0.4189;
  limits.steering_max_rad = 0.4189;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.ttc_warning_s = 0.45;
  limits.ttc_brake_s = 0.34;
  limits.min_forward_clearance_m = 0.05;
  limits.speed_bound_deceleration_mps2 = decel;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
}

GateInput input_for(double speed_mps, double range_m, double bound_f = 0.0, double bound_r = 0.0,
                    double rear_range_m = kInf) {
  GateInput input;
  input.command = DriveCommand{0.0, speed_mps};
  input.dt_s = kDt;
  input.min_scan_range_m = range_m;
  input.min_rear_scan_range_m = rear_range_m;
  input.speed_bound_forward_mps = bound_f;
  input.speed_bound_reverse_mps = bound_r;
  return input;
}

// ---------------------------------------------------------------------------------------
// The output bound.
// ---------------------------------------------------------------------------------------

TEST(TtcSpeedBound, UnsetDecelerationKeepsNoBoundAndTheRequestDecides) {
  const SafetyGateLogic gate(make_limits(std::nullopt));
  // Previous output 2.0 m/s, request 1.0, range 0.4: TTC on the request is 0.4 s > 0.34.
  const GateResult r = gate.evaluate(input_for(1.0, 0.4, 2.0), DriveCommand{0.0, 2.0});
  EXPECT_EQ(r.speed_bound_forward_mps, 0.0);
  EXPECT_EQ(r.speed_bound_reverse_mps, 0.0);
  EXPECT_FALSE(r.ttc_brake_latched);
}

TEST(TtcSpeedBound, TheBoundIsTheLastOutputOrTheDecayedEarlierOne) {
  const SafetyGateLogic gate(make_limits());
  // Faster previous output than the carried bound: the output wins.
  EXPECT_DOUBLE_EQ(
      gate.evaluate(input_for(1.0, kInf, 0.5), DriveCommand{0.0, 1.2}).speed_bound_forward_mps,
      1.2);
  // Slower previous output: the carried bound, minus decel x dt.
  EXPECT_NEAR(
      gate.evaluate(input_for(1.0, kInf, 1.97), DriveCommand{0.0, 1.0}).speed_bound_forward_mps,
      1.97 - kDecel * kDt, 1e-12);
  // The reverse bound follows the reverse output the same way.
  EXPECT_DOUBLE_EQ(gate.evaluate(input_for(-0.5, kInf, 0.0, 0.1), DriveCommand{0.0, -0.5})
                       .speed_bound_reverse_mps,
                   0.5);
  EXPECT_DOUBLE_EQ(gate.evaluate(input_for(-0.5, kInf, 0.0, 0.1), DriveCommand{0.0, -0.5})
                       .speed_bound_forward_mps,
                   0.0);
}

TEST(TtcSpeedBound, GarbageCarriedBoundRestartsFromTheLastOutput) {
  const SafetyGateLogic gate(make_limits());
  for (const double bad : {kNan, kInf, -kInf}) {
    EXPECT_DOUBLE_EQ(
        gate.evaluate(input_for(0.5, kInf, bad), DriveCommand{0.0, 0.3}).speed_bound_forward_mps,
        0.3)
        << bad;
  }
}

TEST(TtcSpeedBound, GarbageDecelerationKeepsNoMemory) {
  for (const double bad : {kNan, kInf, 0.0, -1.0}) {
    const SafetyGateLogic gate(make_limits(bad));
    EXPECT_DOUBLE_EQ(
        gate.evaluate(input_for(0.5, kInf, 1.9), DriveCommand{0.0, 0.3}).speed_bound_forward_mps,
        0.3)
        << bad;
  }
}

TEST(TtcSpeedBound, ANonFinitePreviousOutputNeverPoisonsTheBound) {
  const SafetyGateLogic gate(make_limits());
  const GateResult r = gate.evaluate(input_for(0.5, kInf, 0.4), DriveCommand{0.0, kNan});
  EXPECT_NEAR(r.speed_bound_forward_mps, 0.4 - kDecel * kDt, 1e-12);
  EXPECT_NEAR(r.speed_bound_reverse_mps, 0.0, 1e-12);
}

TEST(TtcSpeedBound, TheWatchdogShortCircuitStillAdvancesTheBound) {
  const SafetyGateLogic gate(make_limits());
  GateInput input = input_for(1.0, kInf, 1.5);
  input.drive_raw_age_s = 1.0;  // stale
  const GateResult r = gate.evaluate(input, DriveCommand{0.0, 0.0});
  EXPECT_NEAR(r.speed_bound_forward_mps, 1.5 - kDecel * kDt, 1e-12);
}

TEST(TtcSpeedBound, TheWallFollowContactOf2026_10_24) {
  // Bag 2026-10-24T23-13-18 t 701.18: wall_follow dropped its request from 1.97 to 1.0 m/s on
  // the approach and the car was still about 1.15 m/s (scan matching) at the brake. A wall
  // 0.40 m from the body: TTC on the request alone is 0.40 s (no brake); on the bound, which
  // decays from 1.97 at 1 m/s^2, it is 0.40 / 1.93 s on the second cycle after the drop.
  const SafetyGateLogic gate(make_limits());
  const SafetyGateLogic request_only(make_limits(std::nullopt));
  DriveCommand previous{0.0, 1.97};
  double bound = 1.97;
  bool latched_any = false;
  for (int i = 0; i < 5; ++i) {
    const GateResult r = gate.evaluate(input_for(1.0, 0.40, bound), previous);
    latched_any = latched_any || r.ttc_brake_latched;
    previous = r.output;
    bound = r.speed_bound_forward_mps;
  }
  EXPECT_TRUE(latched_any);
  EXPECT_FALSE(
      request_only.evaluate(input_for(1.0, 0.40, 1.97), DriveCommand{0.0, 1.97}).ttc_brake_latched);
}

TEST(TtcSpeedBound, OnceDecayedTheRequestDecidesAgain) {
  // (1.97 - 1.0) / 1 m/s^2 = 0.97 s after the drop the bound is back at the request.
  const SafetyGateLogic gate(make_limits());
  DriveCommand previous{0.0, 1.0};
  double bound = 1.97;
  for (int i = 0; i < 49; ++i) {  // 0.98 s
    const GateResult r = gate.evaluate(input_for(1.0, kInf, bound), previous);
    previous = r.output;
    bound = r.speed_bound_forward_mps;
  }
  EXPECT_DOUBLE_EQ(bound, 1.0);
  // TTC 0.35 s on the request: no brake.
  EXPECT_FALSE(gate.evaluate(input_for(1.0, 0.35, bound), previous).ttc_brake_latched);
}

TEST(TtcSpeedBound, AZeroOrAwayRequestNeverTripsWhateverTheBound) {
  // The gate judges only a request toward the obstacle: the bound raises the speed of such a
  // request, it does not create one.
  const SafetyGateLogic gate(make_limits());
  EXPECT_FALSE(gate.evaluate(input_for(0.0, 0.06, 2.0), DriveCommand{0.0, 2.0}).ttc_brake_latched);
  EXPECT_FALSE(gate.evaluate(input_for(-0.3, 0.06, 2.0), DriveCommand{0.0, 2.0}).ttc_brake_latched);
}

// ---------------------------------------------------------------------------------------
// The measured wheel speed.
// ---------------------------------------------------------------------------------------

TEST(TtcSpeedMeasured, AFasterMeasuredSpeedRaisesTheTtcSpeed) {
  const SafetyGateLogic gate(make_limits());
  GateInput input = input_for(0.5, 0.3);  // TTC 0.6 s on the request
  EXPECT_FALSE(gate.evaluate(input, DriveCommand{0.0, 0.5}).ttc_brake_latched);
  input.measured_speed_mps = 1.0;  // TTC 0.3 s on the measured speed
  EXPECT_TRUE(gate.evaluate(input, DriveCommand{0.0, 0.5}).ttc_brake_latched);
}

TEST(TtcSpeedMeasured, ASlowerMeasuredSpeedNeverLowersItBelowTheRequest) {
  // The car crawling at 0.1 m/s while asked for 1.0 m/s (the October corners ran at about 0.55
  // of the request): the TTC stays the request's, 0.3 / 1.0 s, and brakes.
  const SafetyGateLogic gate(make_limits());
  GateInput input = input_for(1.0, 0.3);
  input.measured_speed_mps = 0.1;
  EXPECT_TRUE(gate.evaluate(input, DriveCommand{0.0, 1.0}).ttc_brake_latched);
}

TEST(TtcSpeedMeasured, NonFiniteMeasurementsAreIgnored) {
  const SafetyGateLogic gate(make_limits());
  for (const double bad : {kNan, kInf, -kInf}) {
    GateInput input = input_for(0.5, 0.3);
    input.measured_speed_mps = bad;
    EXPECT_FALSE(gate.evaluate(input, DriveCommand{0.0, 0.5}).ttc_brake_latched) << bad;
  }
}

TEST(TtcSpeedMeasured, RollingBackwardsRaisesTheReverseGateOnly) {
  const SafetyGateLogic gate(make_limits());
  GateInput input = input_for(-0.3, 0.3, 0.0, 0.0, 0.3);  // reverse request, wall 0.3 m behind
  input.measured_speed_mps = -1.0;                        // rolling back at 1 m/s
  const GateResult r = gate.evaluate(input, DriveCommand{0.0, -0.3});
  EXPECT_TRUE(r.reverse_brake_latched);  // 0.3 / 1.0 s
  EXPECT_FALSE(r.ttc_brake_latched);     // the forward gate sees no forward request
  input.measured_speed_mps = std::nullopt;
  EXPECT_FALSE(gate.evaluate(input, DriveCommand{0.0, -0.3}).reverse_brake_latched);  // 1.0 s
}

// ---------------------------------------------------------------------------------------
// The 2026-10-06 limit cycle stays fixed with the bound on.
// ---------------------------------------------------------------------------------------

TEST(TtcSpeedNoLimitCycle, TheBenchRequestIsBrakedOnEveryCycleAndReleasesCleanly) {
  // docs/notes/ttc-limit-cycle-2026-10-06.md: 0.48 m/s requested steadily with an obstacle
  // 0.13 m from the body (TTC 0.27 s). With the bound threaded and the gate's own braking
  // feeding the previous output, the output must be zero on every cycle (no 3-cycle pulse), and
  // once the obstacle moves away it must release once and ramp without re-tripping.
  const SafetyGateLogic gate(make_limits());
  GateEventTracker tracker;
  DriveCommand previous{0.0, 0.0};
  bool latched = false;
  double bound_f = 0.0;
  double bound_r = 0.0;
  std::size_t engages = 0;
  std::size_t releases = 0;
  double now = 0.0;
  auto step = [&](double range) {
    GateInput input = input_for(0.48, range, bound_f, bound_r);
    input.ttc_brake_latched = latched;
    const GateResult r = gate.evaluate(input, previous);
    for (const auto& rec : tracker.update(r.activations, now += kDt, r.releases)) {
      if (rec.source == GateSource::kTtc && rec.severity == EventSeverity::kBrake) {
        engages += rec.phase == EventPhase::kEngage ? 1 : 0;
        releases += rec.phase == EventPhase::kRelease ? 1 : 0;
      }
    }
    previous = r.output;
    latched = r.ttc_brake_latched;
    bound_f = r.speed_bound_forward_mps;
    bound_r = r.speed_bound_reverse_mps;
    return r;
  };
  for (int i = 0; i < 286 * 50; ++i) {
    ASSERT_EQ(step(0.13).output.speed_mps, 0.0) << "cycle " << i;
  }
  EXPECT_EQ(engages, 1u);
  for (int i = 0; i < 100; ++i) {
    const GateResult r = step(0.5);  // TTC 1.04 s
    ASSERT_FALSE(r.ttc_brake_latched) << "cycle " << i;
  }
  EXPECT_NEAR(previous.speed_mps, 0.48, 1e-9);
  EXPECT_EQ(engages, 1u);
  EXPECT_EQ(releases, 1u);
}

}  // namespace
}  // namespace racer_safety
