// L1 unit tests for racer_drivers' ROS-free mapping and output sequencing
// (claude-docs/12-testing.md L1: "every branch, every boundary value (exactly at bound,
// epsilon over, NaN, inf)"). No ROS, no sysfs, no kernel: everything here runs against
// pwm_mapping.hpp's pure functions and pwm_sink.hpp's InMemoryPwmChannel.
//
// The calibration numbers in `nominal_config()` are TEST FIXTURES, not the vehicle's
// constants: they are chosen to be asymmetric on purpose so a mapping bug that happens to
// work for a symmetric 1000/1500/2000 calibration still fails here. The real values reach
// the node only from the generated vehicle_params binding (CLAUDE.md invariant 2).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "racer_drivers/pwm_mapping.hpp"
#include "racer_drivers/pwm_output_driver.hpp"
#include "racer_drivers/pwm_sink.hpp"

namespace {

using racer_drivers::CommandState;
using racer_drivers::InMemoryPwmChannel;
using racer_drivers::MappingConfig;
using racer_drivers::PulsePair;
using racer_drivers::PwmOutputDriver;
using racer_drivers::RequiredField;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;

MappingConfig nominal_config() {
  MappingConfig config;
  // Asymmetric on both channels (neutral is NOT the midpoint) so an implementation that
  // interpolates one straight line across the whole range fails these tests.
  config.steering.min_us = 1000.0;
  config.steering.neutral_us = 1400.0;
  config.steering.max_us = 2000.0;
  config.throttle.min_us = 1100.0;
  config.throttle.neutral_us = 1500.0;
  config.throttle.max_us = 1900.0;
  config.steering_min_angle_rad = -0.4;
  config.steering_max_angle_rad = 0.2;
  config.speed_full_scale_mps = 4.0;
  config.left_is_pwm_max = true;
  config.drive_timeout_s = 0.1;
  return config;
}

// -- find_missing_fields -------------------------------------------------------------------

TEST(FindMissingFields, AllPresentReturnsNullopt) {
  const std::vector<RequiredField> fields = {{"a", 1.0}, {"b", 2.0}};
  EXPECT_FALSE(racer_drivers::find_missing_fields(fields).has_value());
}

TEST(FindMissingFields, EmptyListIsNotAFailure) {
  EXPECT_FALSE(racer_drivers::find_missing_fields({}).has_value());
}

TEST(FindMissingFields, NamesTheSingleMissingField) {
  const std::vector<RequiredField> fields = {{"steering.pwm_min_us", std::nullopt}, {"b", 2.0}};
  const auto message = racer_drivers::find_missing_fields(fields);
  ASSERT_TRUE(message.has_value());
  EXPECT_NE(message->find("steering.pwm_min_us"), std::string::npos);
}

TEST(FindMissingFields, NamesEveryMissingField) {
  const std::vector<RequiredField> fields = {
      {"steering.pwm_min_us", std::nullopt},
      {"actuation.throttle_pwm_max_us", std::nullopt},
      {"limits.global_speed_cap_mps", 20.0},
  };
  const auto message = racer_drivers::find_missing_fields(fields);
  ASSERT_TRUE(message.has_value());
  EXPECT_NE(message->find("steering.pwm_min_us"), std::string::npos);
  EXPECT_NE(message->find("actuation.throttle_pwm_max_us"), std::string::npos);
  EXPECT_EQ(message->find("limits.global_speed_cap_mps"), std::string::npos);
}

// -- validate_config -----------------------------------------------------------------------

TEST(ValidateConfig, NominalIsAccepted) {
  EXPECT_FALSE(racer_drivers::validate_config(nominal_config()).has_value());
}

TEST(ValidateConfig, RejectsNonFiniteSteeringCalibration) {
  MappingConfig config = nominal_config();
  config.steering.min_us = kNaN;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering.neutral_us = kInf;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering.max_us = -kInf;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsNonFiniteThrottleCalibration) {
  MappingConfig config = nominal_config();
  config.throttle.min_us = kNaN;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.throttle.neutral_us = kNaN;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.throttle.max_us = kInf;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsUnorderedSteeringCalibration) {
  MappingConfig config = nominal_config();
  config.steering.neutral_us = config.steering.min_us;  // exactly at bound: not strictly inside
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering.neutral_us = config.steering.max_us;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering.min_us = 2100.0;  // min above max
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsUnorderedThrottleCalibration) {
  MappingConfig config = nominal_config();
  config.throttle.neutral_us = config.throttle.min_us;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.throttle.neutral_us = config.throttle.max_us;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsNonFiniteAngleLimits) {
  MappingConfig config = nominal_config();
  config.steering_min_angle_rad = -kInf;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering_max_angle_rad = kNaN;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsOneSidedAngleRange) {
  MappingConfig config = nominal_config();
  config.steering_min_angle_rad = 0.0;  // exactly at the bound the check rejects
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering_max_angle_rad = 0.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.steering_min_angle_rad = 0.1;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsBadSpeedFullScale) {
  MappingConfig config = nominal_config();
  config.speed_full_scale_mps = 0.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.speed_full_scale_mps = -1.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.speed_full_scale_mps = kNaN;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

TEST(ValidateConfig, RejectsBadTimeout) {
  MappingConfig config = nominal_config();
  config.drive_timeout_s = 0.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.drive_timeout_s = kInf;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
}

// -- is_stale ------------------------------------------------------------------------------

struct StaleCase {
  const char* name;
  bool has_command;
  double age_s;
  bool expected_stale;
};

TEST(IsStale, TableDriven) {
  const std::vector<StaleCase> cases = {
      {"no command yet", false, 0.0, true},
      {"no command yet, junk age", false, kNaN, true},
      {"fresh", true, 0.0, false},
      {"just inside the timeout", true, 0.1 - kEps, false},
      {"exactly at the timeout is stale (fail closed on the boundary)", true, 0.1, true},
      {"epsilon over", true, 0.1 + kEps, true},
      {"far over", true, 10.0, true},
      {"NaN age", true, kNaN, true},
      {"inf age", true, kInf, true},
      {"negative age (clock went backwards) is not stale", true, -0.5, false},
  };
  for (const StaleCase& c : cases) {
    CommandState state;
    state.has_command = c.has_command;
    state.age_s = c.age_s;
    EXPECT_EQ(racer_drivers::is_stale(state, 0.1), c.expected_stale) << c.name;
  }
}

// -- steering_angle_to_pulse_us --------------------------------------------------------------

struct SteeringCase {
  const char* name;
  double angle_rad;
  double expected_us;
};

TEST(SteeringAngleToPulse, TableDrivenLeftIsPwmMax) {
  const MappingConfig config = nominal_config();  // -0.4 .. +0.2 rad, 1000/1400/2000 us
  const std::vector<SteeringCase> cases = {
      {"zero is exactly neutral", 0.0, 1400.0},
      {"full left (max angle)", 0.2, 2000.0},
      {"half left", 0.1, 1700.0},
      {"epsilon over full left clamps", 0.2 + kEps, 2000.0},
      {"far over full left clamps", 100.0, 2000.0},
      {"full right (min angle)", -0.4, 1000.0},
      {"half right", -0.2, 1200.0},
      {"epsilon over full right clamps", -0.4 - kEps, 1000.0},
      {"far over full right clamps", -100.0, 1000.0},
      {"NaN is neutral", kNaN, 1400.0},
      {"+inf is neutral", kInf, 1400.0},
      {"-inf is neutral", -kInf, 1400.0},
  };
  for (const SteeringCase& c : cases) {
    EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, c.angle_rad), c.expected_us, 1e-6)
        << c.name;
  }
}

TEST(SteeringAngleToPulse, TableDrivenLeftIsPwmMin) {
  MappingConfig config = nominal_config();
  config.left_is_pwm_max = false;  // the other bench-calibrated polarity
  const std::vector<SteeringCase> cases = {
      {"zero is exactly neutral", 0.0, 1400.0},
      {"full left now goes to pwm_min_us", 0.2, 1000.0},
      {"half left", 0.1, 1200.0},
      {"full right now goes to pwm_max_us", -0.4, 2000.0},
      {"half right", -0.2, 1700.0},
      {"NaN is still neutral", kNaN, 1400.0},
  };
  for (const SteeringCase& c : cases) {
    EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, c.angle_rad), c.expected_us, 1e-6)
        << c.name;
  }
}

TEST(SteeringAngleToPulse, NeverLeavesTheCalibratedRange) {
  const MappingConfig config = nominal_config();
  for (double angle = -5.0; angle <= 5.0; angle += 0.01) {
    const double pulse = racer_drivers::steering_angle_to_pulse_us(config, angle);
    EXPECT_GE(pulse, config.steering.min_us);
    EXPECT_LE(pulse, config.steering.max_us);
  }
}

// -- speed_to_pulse_us -----------------------------------------------------------------------

struct SpeedCase {
  const char* name;
  double speed_mps;
  double expected_us;
};

TEST(SpeedToPulse, TableDriven) {
  const MappingConfig config = nominal_config();  // full scale 4 m/s, 1100/1500/1900 us
  const std::vector<SpeedCase> cases = {
      {"zero is exactly neutral", 0.0, 1500.0},
      {"full scale forward", 4.0, 1900.0},
      {"half scale forward", 2.0, 1700.0},
      {"epsilon over full scale clamps", 4.0 + kEps, 1900.0},
      {"far over clamps", 1000.0, 1900.0},
      {"full scale reverse", -4.0, 1100.0},
      {"half scale reverse", -2.0, 1300.0},
      {"epsilon over reverse clamps", -4.0 - kEps, 1100.0},
      {"far under clamps", -1000.0, 1100.0},
      {"NaN is neutral", kNaN, 1500.0},
      {"+inf is neutral", kInf, 1500.0},
      {"-inf is neutral", -kInf, 1500.0},
  };
  for (const SpeedCase& c : cases) {
    EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, c.speed_mps), c.expected_us, 1e-6)
        << c.name;
  }
}

TEST(SpeedToPulse, NeverLeavesTheCalibratedRange) {
  const MappingConfig config = nominal_config();
  for (double speed = -50.0; speed <= 50.0; speed += 0.1) {
    const double pulse = racer_drivers::speed_to_pulse_us(config, speed);
    EXPECT_GE(pulse, config.throttle.min_us);
    EXPECT_LE(pulse, config.throttle.max_us);
  }
}

// -- compute_outputs / neutral_outputs --------------------------------------------------------

TEST(ComputeOutputs, NoCommandYetIsNeutralOnBothChannels) {
  const MappingConfig config = nominal_config();
  CommandState state;  // has_command == false
  state.steering_angle_rad = 0.2;
  state.speed_mps = 4.0;
  const PulsePair pulses = racer_drivers::compute_outputs(config, state);
  EXPECT_DOUBLE_EQ(pulses.steering_us, config.steering.neutral_us);
  EXPECT_DOUBLE_EQ(pulses.throttle_us, config.throttle.neutral_us);
}

TEST(ComputeOutputs, StaleCommandIsNeutralOnBothChannels) {
  const MappingConfig config = nominal_config();
  CommandState state;
  state.has_command = true;
  state.age_s = 0.5;
  state.steering_angle_rad = 0.2;
  state.speed_mps = 4.0;
  const PulsePair pulses = racer_drivers::compute_outputs(config, state);
  EXPECT_DOUBLE_EQ(pulses.steering_us, config.steering.neutral_us);
  EXPECT_DOUBLE_EQ(pulses.throttle_us, config.throttle.neutral_us);
}

TEST(ComputeOutputs, FreshCommandIsMapped) {
  const MappingConfig config = nominal_config();
  CommandState state;
  state.has_command = true;
  state.age_s = 0.01;
  state.steering_angle_rad = 0.1;
  state.speed_mps = 2.0;
  const PulsePair pulses = racer_drivers::compute_outputs(config, state);
  EXPECT_NEAR(pulses.steering_us, 1700.0, 1e-6);
  EXPECT_NEAR(pulses.throttle_us, 1700.0, 1e-6);
}

TEST(NeutralOutputs, IsTheCalibratedNeutral) {
  const MappingConfig config = nominal_config();
  const PulsePair pulses = racer_drivers::neutral_outputs(config);
  EXPECT_DOUBLE_EQ(pulses.steering_us, 1400.0);
  EXPECT_DOUBLE_EQ(pulses.throttle_us, 1500.0);
}

// -- pulse_us_to_duty_ns ----------------------------------------------------------------------

TEST(PulseToDuty, TableDriven) {
  constexpr unsigned long long kPeriod = 20000000ULL;  // 50 Hz
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(1500.0, kPeriod), 1500000ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(1000.4, kPeriod), 1000400ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(0.0, kPeriod), 0ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(-5.0, kPeriod), 0ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(kNaN, kPeriod), 0ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(kInf, kPeriod), 0ULL);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(-kInf, kPeriod), 0ULL);
  // Exactly at, and past, the period: clamped to the period, never longer (EINVAL).
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(20000.0, kPeriod), kPeriod);
  EXPECT_EQ(racer_drivers::pulse_us_to_duty_ns(30000.0, kPeriod), kPeriod);
}

// -- PwmOutputDriver sequencing (in-memory sinks, no kernel) ------------------------------------

class DriverFixture : public ::testing::Test {
 protected:
  MappingConfig config{nominal_config()};
  InMemoryPwmChannel steering;
  InMemoryPwmChannel throttle;
  static constexpr unsigned long long kPeriod = 20000000ULL;
};

TEST_F(DriverFixture, StartsBothChannelsAtNeutral) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  EXPECT_TRUE(steering.started);
  EXPECT_TRUE(throttle.started);
  EXPECT_EQ(steering.period_ns, kPeriod);
  ASSERT_EQ(steering.duty_writes.size(), 1U);
  EXPECT_EQ(steering.duty_writes.front(), 1400000ULL);
  ASSERT_EQ(throttle.duty_writes.size(), 1U);
  EXPECT_EQ(throttle.duty_writes.front(), 1500000ULL);
}

TEST_F(DriverFixture, UpdateWritesTheMappedPulseToBothChannels) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  CommandState state;
  state.has_command = true;
  state.age_s = 0.0;
  state.steering_angle_rad = 0.1;
  state.speed_mps = 2.0;
  const PulsePair pulses = driver.update(state);
  EXPECT_NEAR(pulses.steering_us, 1700.0, 1e-6);
  EXPECT_EQ(steering.duty_writes.back(), 1700000ULL);
  EXPECT_EQ(throttle.duty_writes.back(), 1700000ULL);
}

TEST_F(DriverFixture, UpdateWithNoCommandWritesNeutral) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  driver.update(CommandState{});
  EXPECT_EQ(steering.duty_writes.back(), 1400000ULL);
  EXPECT_EQ(throttle.duty_writes.back(), 1500000ULL);
}

TEST_F(DriverFixture, ForceNeutralWritesNeutralWithoutDisabling) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  CommandState state;
  state.has_command = true;
  state.steering_angle_rad = 0.2;
  state.speed_mps = 4.0;
  driver.update(state);
  driver.force_neutral();
  EXPECT_EQ(steering.duty_writes.back(), 1400000ULL);
  EXPECT_EQ(throttle.duty_writes.back(), 1500000ULL);
  EXPECT_TRUE(steering.enabled);
  EXPECT_TRUE(throttle.enabled);
}

TEST_F(DriverFixture, StopWritesNeutralThenDisables) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  CommandState state;
  state.has_command = true;
  state.steering_angle_rad = 0.2;
  state.speed_mps = 4.0;
  driver.update(state);
  driver.stop();
  EXPECT_EQ(steering.duty_writes.back(), 1400000ULL);
  EXPECT_EQ(throttle.duty_writes.back(), 1500000ULL);
  EXPECT_FALSE(steering.enabled);
  EXPECT_FALSE(throttle.enabled);
}

TEST_F(DriverFixture, StopIsIdempotent) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  driver.stop();
  const std::size_t writes_after_first_stop = steering.duty_writes.size();
  driver.stop();
  EXPECT_EQ(steering.duty_writes.size(), writes_after_first_stop);
}

TEST_F(DriverFixture, NoUpdateEverLeavesANonNeutralPulseAfterStop) {
  PwmOutputDriver driver(config, steering, throttle, kPeriod);
  driver.start();
  for (double angle = -0.4; angle <= 0.2; angle += 0.05) {
    CommandState state;
    state.has_command = true;
    state.steering_angle_rad = angle;
    state.speed_mps = 3.0;
    driver.update(state);
  }
  driver.stop();
  EXPECT_EQ(steering.duty_writes.back(), 1400000ULL);
  EXPECT_EQ(throttle.duty_writes.back(), 1500000ULL);
}

}  // namespace
