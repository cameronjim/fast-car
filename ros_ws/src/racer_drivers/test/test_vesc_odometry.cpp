// L1 unit tests for include/racer_drivers/vesc_odometry.hpp (claude-docs/12-testing.md L1).
//
// Fixture drivetrain values below are deliberately NOT the car's (7 pole pairs, 5:1, 50 mm
// wheel): these tests check the arithmetic against hand-computed cases. The car's own numbers
// are only ever read from the generated binding, in test_vesc_odometry_binding.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "racer_drivers/vesc_odometry.hpp"

namespace vo = racer_drivers::vesc_odometry;

namespace {

vo::DrivetrainConfig fixture() {
  vo::DrivetrainConfig config;
  config.pole_pairs = 7.0;
  config.gear_ratio = 5.0;
  config.wheel_radius_m = 0.05;
  return config;
}

constexpr double kTight = 1e-12;

}  // namespace

// --- conversion --------------------------------------------------------------------------

TEST(VescOdometryConversion, HandComputedCase) {
  // 4200 ERPM / 7 pole pairs = 600 motor RPM; / 5 = 120 wheel RPM = 2 rev/s;
  // 2 rev/s * 2 pi * 0.05 m = 0.2 pi m/s.
  const auto config = fixture();
  EXPECT_NEAR(vo::erpm_to_motor_rad_per_s(4200.0, config), 600.0 * 2.0 * M_PI / 60.0, kTight);
  EXPECT_NEAR(vo::erpm_to_wheel_rad_per_s(4200.0, config), 4.0 * M_PI, kTight);
  EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(4200.0, config), 0.2 * M_PI, kTight);
}

TEST(VescOdometryConversion, ZeroErpmIsZeroSpeed) {
  EXPECT_EQ(vo::erpm_to_wheel_speed_mps(0.0, fixture()), 0.0);
}

TEST(VescOdometryConversion, PositiveErpmIsForwardNegativeIsReverse) {
  // claude-docs/06-vehicle-params.md: positive drive torque is forward, so the VESC's positive
  // rotation (the direction positive current turns it) is forward travel.
  const auto config = fixture();
  EXPECT_GT(vo::erpm_to_wheel_speed_mps(1000.0, config), 0.0);
  EXPECT_LT(vo::erpm_to_wheel_speed_mps(-1000.0, config), 0.0);
}

TEST(VescOdometryConversion, OddInSign) {
  const auto config = fixture();
  for (double erpm : {1.0, 250.0, 900.0, 6000.0, 60000.0}) {
    EXPECT_DOUBLE_EQ(vo::erpm_to_wheel_speed_mps(-erpm, config),
                     -vo::erpm_to_wheel_speed_mps(erpm, config));
  }
}

TEST(VescOdometryConversion, LinearInErpm) {
  const auto config = fixture();
  const double k = vo::mps_per_erpm(config);
  for (double erpm : {-6000.0, -1.0, 0.5, 900.0, 6000.0}) {
    EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(erpm, config), k * erpm, 1e-12);
  }
}

TEST(VescOdometryConversion, InverseRoundTrips) {
  const auto config = fixture();
  for (double speed : {-3.0, -0.01, 0.0, 0.44, 2.91, 15.0}) {
    EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(vo::wheel_speed_mps_to_erpm(speed, config), config),
                speed, 1e-12);
  }
}

TEST(VescOdometryConversion, EachDrivetrainFieldScalesTheRightWay) {
  const auto base = fixture();
  const double v = vo::erpm_to_wheel_speed_mps(3000.0, base);
  auto more_poles = base;
  more_poles.pole_pairs *= 2.0;
  EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(3000.0, more_poles), v / 2.0, kTight);
  auto taller_gear = base;
  taller_gear.gear_ratio *= 2.0;
  EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(3000.0, taller_gear), v / 2.0, kTight);
  auto bigger_wheel = base;
  bigger_wheel.wheel_radius_m *= 2.0;
  EXPECT_NEAR(vo::erpm_to_wheel_speed_mps(3000.0, bigger_wheel), v * 2.0, kTight);
}

// --- validate ----------------------------------------------------------------------------

TEST(VescOdometryValidate, AcceptsAPositiveConfig) { EXPECT_FALSE(vo::validate(fixture())); }

TEST(VescOdometryValidate, RejectsZeroNegativeNanAndInfPerField) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double bad : {0.0, -1.0, nan, inf}) {
    auto config = fixture();
    config.pole_pairs = bad;
    ASSERT_TRUE(vo::validate(config));
    EXPECT_NE(vo::validate(config)->find("pole_pairs"), std::string::npos);

    config = fixture();
    config.gear_ratio = bad;
    ASSERT_TRUE(vo::validate(config));
    EXPECT_NE(vo::validate(config)->find("gear_ratio"), std::string::npos);

    config = fixture();
    config.wheel_radius_m = bad;
    ASSERT_TRUE(vo::validate(config));
    EXPECT_NE(vo::validate(config)->find("nominal_radius_m"), std::string::npos);
  }
}

// --- integrator --------------------------------------------------------------------------

using Result = vo::AlongTrackIntegrator::Result;

TEST(AlongTrackIntegrator, FirstSampleAddsNothing) {
  vo::AlongTrackIntegrator integrator(0.2);
  EXPECT_EQ(integrator.add(10.0, 1.0), Result::kFirstSample);
  EXPECT_EQ(integrator.distance_m(), 0.0);
}

TEST(AlongTrackIntegrator, ConstantSpeedIsSpeedTimesTime) {
  vo::AlongTrackIntegrator integrator(0.2);
  integrator.add(0.0, 0.5);
  for (int i = 1; i <= 100; ++i) {
    EXPECT_EQ(integrator.add(i * 0.02, 0.5), Result::kIntegrated);
  }
  EXPECT_NEAR(integrator.distance_m(), 0.5 * 2.0, 1e-12);
}

TEST(AlongTrackIntegrator, TrapezoidOnARamp) {
  // Speed ramps 0 -> 1 m/s over 1 s: exact area 0.5 m, and the trapezoid rule is exact on a
  // linear ramp.
  vo::AlongTrackIntegrator integrator(0.2);
  for (int i = 0; i <= 50; ++i) {
    integrator.add(i * 0.02, i * 0.02);
  }
  EXPECT_NEAR(integrator.distance_m(), 0.5, 1e-12);
}

TEST(AlongTrackIntegrator, ReverseSubtractsSoAThreePointTurnNetsOut) {
  vo::AlongTrackIntegrator integrator(0.2);
  double t = 0.0;
  integrator.add(t, 0.4);
  for (int i = 0; i < 50; ++i) integrator.add(t += 0.02, 0.4);   // forward 1 s
  integrator.add(t += 0.02, -0.4);                               // reverse
  for (int i = 0; i < 49; ++i) integrator.add(t += 0.02, -0.4);  // reverse ~1 s
  // Forward 0.4 m, then one trapezoid straddling the reversal (averages to 0), then -0.392 m.
  EXPECT_NEAR(integrator.distance_m(), 0.4 - 0.4 * 0.98, 1e-9);
  EXPECT_LT(integrator.distance_m(), 0.4);
}

TEST(AlongTrackIntegrator, NonFiniteSampleIsDroppedAndThePreviousKept) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  vo::AlongTrackIntegrator integrator(0.2);
  EXPECT_EQ(integrator.add(nan, 1.0), Result::kRejectedNonFinite);
  integrator.add(0.0, 1.0);
  EXPECT_EQ(integrator.add(0.02, nan), Result::kRejectedNonFinite);
  EXPECT_EQ(integrator.add(0.03, inf), Result::kRejectedNonFinite);
  EXPECT_EQ(integrator.add(inf, 1.0), Result::kRejectedNonFinite);
  EXPECT_EQ(integrator.distance_m(), 0.0);
  EXPECT_EQ(integrator.add(0.04, 1.0), Result::kIntegrated);
  EXPECT_NEAR(integrator.distance_m(), 0.04, 1e-12);
}

TEST(AlongTrackIntegrator, DuplicateAndBackwardStampsAreDropped) {
  vo::AlongTrackIntegrator integrator(0.2);
  integrator.add(1.0, 1.0);
  integrator.add(1.02, 1.0);
  const double before = integrator.distance_m();
  EXPECT_EQ(integrator.add(1.02, 1.0), Result::kRejectedNonMonotonic);
  EXPECT_EQ(integrator.add(0.5, 1.0), Result::kRejectedNonMonotonic);
  EXPECT_EQ(integrator.distance_m(), before);
  // The next good sample integrates from the last ACCEPTED one (1.02), not the rejected ones.
  EXPECT_EQ(integrator.add(1.04, 1.0), Result::kIntegrated);
  EXPECT_NEAR(integrator.distance_m(), before + 0.02, 1e-12);
}

TEST(AlongTrackIntegrator, GapIsNotBridged) {
  vo::AlongTrackIntegrator integrator(0.2);
  integrator.add(0.0, 1.0);
  integrator.add(0.1, 1.0);  // 0.1 m
  EXPECT_EQ(integrator.add(5.0, 1.0), Result::kRestartedAfterGap);
  EXPECT_NEAR(integrator.distance_m(), 0.1, 1e-12);  // nothing invented over 4.9 s
  EXPECT_EQ(integrator.add(5.1, 1.0), Result::kIntegrated);
  EXPECT_NEAR(integrator.distance_m(), 0.2, 1e-12);
}

TEST(AlongTrackIntegrator, GapExactlyAtTheLimitIsIntegrated) {
  vo::AlongTrackIntegrator integrator(0.25);
  integrator.add(0.0, 2.0);
  EXPECT_EQ(integrator.add(0.25, 2.0), Result::kIntegrated);
  EXPECT_NEAR(integrator.distance_m(), 0.5, 1e-12);
}

// --- fault names -------------------------------------------------------------------------

TEST(VescFaultName, KnownCodes) {
  EXPECT_EQ(vo::fault_name(0), "NONE");
  EXPECT_EQ(vo::fault_name(1), "OVER_VOLTAGE");
  EXPECT_EQ(vo::fault_name(2), "UNDER_VOLTAGE");
  EXPECT_EQ(vo::fault_name(4), "ABS_OVER_CURRENT");
  EXPECT_EQ(vo::fault_name(5), "OVER_TEMP_FET");
  EXPECT_EQ(vo::fault_name(6), "OVER_TEMP_MOTOR");
  EXPECT_EQ(vo::fault_name(18), "UNBALANCED_CURRENTS");
}

TEST(VescFaultName, UnknownCodesAreNamedNotDropped) {
  EXPECT_EQ(vo::fault_name(19), "UNKNOWN_19");
  EXPECT_EQ(vo::fault_name(-3), "UNKNOWN_-3");
  EXPECT_EQ(vo::fault_name(255), "UNKNOWN_255");
}
