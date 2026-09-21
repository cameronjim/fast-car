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
  // Full scale (the open-loop throttle map's actuation.throttle_full_scale_mps) is
  // deliberately BELOW the cap (limits.global_speed_cap_mps), as it is in the committed
  // config, so the tests below can tell the two apart.
  config.speed_full_scale_mps = 4.0;
  config.speed_cap_mps = 10.0;
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

TEST(ValidateConfig, RejectsBadSpeedCap) {
  MappingConfig config = nominal_config();
  config.speed_cap_mps = 0.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.speed_cap_mps = -1.0;
  EXPECT_TRUE(racer_drivers::validate_config(config).has_value());
  config = nominal_config();
  config.speed_cap_mps = kNaN;
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
      // Fail closed on a backwards clock, like racer_safety's watchdog and racer_tools'
      // should_use_zero_command. This case asserted `false` (fresh) before the 2026-09-14
      // command-path review; with `age_s >= timeout_s` alone, a negative age read as fresh
      // and left the last commanded pulse on the wire.
      {"negative age (clock went backwards) is stale", true, -0.5, true},
      {"large negative age is stale", true, -1e6, true},
      {"negative epsilon is stale", true, -kEps, true},
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
  // full scale 4 m/s, cap 10 m/s, 1100/1500/1900 us. Every boundary below is against the
  // FULL SCALE, which is what sets the pulse; the cap only bounds the command first.
  const MappingConfig config = nominal_config();
  const std::vector<SpeedCase> cases = {
      {"zero is exactly neutral", 0.0, 1500.0},
      {"full scale forward", 4.0, 1900.0},
      {"half scale forward", 2.0, 1700.0},
      {"epsilon over full scale saturates", 4.0 + kEps, 1900.0},
      {"between full scale and cap saturates", 7.0, 1900.0},
      {"at the cap saturates", 10.0, 1900.0},
      {"far over the cap is clamped then saturates", 1000.0, 1900.0},
      {"full scale reverse", -4.0, 1100.0},
      {"half scale reverse", -2.0, 1300.0},
      {"epsilon over reverse saturates", -4.0 - kEps, 1100.0},
      {"between full scale and cap reverse saturates", -7.0, 1100.0},
      {"far under the cap is clamped then saturates", -1000.0, 1100.0},
      {"NaN is neutral", kNaN, 1500.0},
      {"+inf is neutral", kInf, 1500.0},
      {"-inf is neutral", -kInf, 1500.0},
  };
  for (const SpeedCase& c : cases) {
    EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, c.speed_mps), c.expected_us, 1e-6)
        << c.name;
  }
}

// The full scale, not the cap, is what a mid-range command is scaled against: with the
// committed 5 m/s full scale over a 1000/1500/2000 us channel, 1 m/s is 100 us off neutral,
// which is the whole point of GitHub issue #40. Against the old 20 m/s cap-as-full-scale it
// was 25 us, plausibly inside the VESC's PPM deadband.
TEST(SpeedToPulse, FullScaleNotTheCapSetsTheSlope) {
  MappingConfig config = nominal_config();
  config.throttle.min_us = 1000.0;
  config.throttle.neutral_us = 1500.0;
  config.throttle.max_us = 2000.0;
  config.speed_full_scale_mps = 5.0;  // actuation.throttle_full_scale_mps, committed value
  config.speed_cap_mps = 20.0;        // limits.global_speed_cap_mps, committed value
  ASSERT_FALSE(racer_drivers::validate_config(config).has_value());
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, 1.0), 1600.0, 1e-6);
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, -1.0), 1400.0, 1e-6);
}

// A command ABOVE the full scale but BELOW the cap is not rejected and not zeroed: it
// saturates the pulse at the channel end. Nothing about the cap changed -- it is still the
// clamp it always was.
TEST(SpeedToPulse, AboveFullScaleBelowCapSaturates) {
  const MappingConfig config = nominal_config();  // full scale 4, cap 10
  ASSERT_GT(config.speed_cap_mps, config.speed_full_scale_mps);
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, 6.0), config.throttle.max_us, 1e-6);
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, -6.0), config.throttle.min_us, 1e-6);
}

// A command above the CAP is clamped to the cap before anything else happens, exactly as
// before this field was split out.
TEST(SpeedToPulse, AboveCapIsClampedFirst) {
  MappingConfig config = nominal_config();
  // Full scale ABOVE the cap makes the clamp observable: without the cap clamp, 10 m/s would
  // map to the channel end; with it, the command is first cut to 5 m/s, i.e. half scale.
  config.speed_full_scale_mps = 10.0;
  config.speed_cap_mps = 5.0;
  ASSERT_FALSE(racer_drivers::validate_config(config).has_value());
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, 10.0), 1700.0, 1e-6);
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, 1000.0), 1700.0, 1e-6);
  EXPECT_NEAR(racer_drivers::speed_to_pulse_us(config, -1000.0), 1300.0, 1e-6);
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

// -- validate_channel_assignment -------------------------------------------------------------
//
// Added by the 2026-09-14 command-path review. Every one of the four chip/channel parameters
// used to default to 0, so `ros2 run racer_drivers pwm_output_node` with no arguments aimed
// BOTH pulses at pwmchip0/pwm0 and the throttle write silently overwrote the steering write
// 50 times a second.

TEST(ValidateChannelAssignment, DistinctChannelsOnOneChipAreFine) {
  EXPECT_FALSE(racer_drivers::validate_channel_assignment(0, 0, 0, 1).has_value());
}

TEST(ValidateChannelAssignment, DistinctChipsAreFineEvenWithTheSameChannelIndex) {
  EXPECT_FALSE(racer_drivers::validate_channel_assignment(0, 0, 3, 0).has_value());
}

TEST(ValidateChannelAssignment, TheSameChipAndChannelIsRefusedAndNamed) {
  const auto reason = racer_drivers::validate_channel_assignment(0, 0, 0, 0);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("pwmchip0/pwm0"), std::string::npos);
}

TEST(ValidateChannelAssignment, TheSameNonZeroChipAndChannelIsAlsoRefused) {
  EXPECT_TRUE(racer_drivers::validate_channel_assignment(4, 2, 4, 2).has_value());
}

// -- steering sign convention, end of the chain -----------------------------------------------
//
// claude-docs/06-vehicle-params.md: road-wheel angle in radians, LEFT POSITIVE. This is the
// last hop of that convention before the wire, and `left_is_pwm_max` is the one bench-
// calibrated unknown in it (docs/notes/first-boot-runbook.md). The earlier hops are pinned in
// racer_tools/test/test_keymap.py, test_twist_teleop.py and racer_safety/test/
// test_gate_logic.cpp's SteeringSign group.

TEST(SteeringSign, PositiveIsLeftAndGoesToThePwmEndTheFlagNames) {
  MappingConfig config = nominal_config();
  config.left_is_pwm_max = true;
  EXPECT_GT(racer_drivers::steering_angle_to_pulse_us(config, 0.1), config.steering.neutral_us);
  EXPECT_LT(racer_drivers::steering_angle_to_pulse_us(config, -0.1), config.steering.neutral_us);

  config.left_is_pwm_max = false;
  EXPECT_LT(racer_drivers::steering_angle_to_pulse_us(config, 0.1), config.steering.neutral_us);
  EXPECT_GT(racer_drivers::steering_angle_to_pulse_us(config, -0.1), config.steering.neutral_us);
}

TEST(SteeringSign, FlippingTheFlagMirrorsThePulseAboutNeutralForBothPolarities) {
  MappingConfig max_config = nominal_config();
  MappingConfig min_config = nominal_config();
  min_config.left_is_pwm_max = false;
  // The calibration is deliberately asymmetric (1000/1400/2000 with -0.4..+0.2 rad), so this
  // asserts the two polarities use the SAME two half-ranges, just swapped -- not that the
  // pulse is a mirror image about neutral, which an asymmetric calibration is not.
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(max_config, 0.2),
              max_config.steering.max_us, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(min_config, 0.2),
              min_config.steering.min_us, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(max_config, -0.4),
              max_config.steering.min_us, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(min_config, -0.4),
              min_config.steering.max_us, 1e-6);
}

// -- golden: the committed steering calibration, config/vehicle_params.yaml -----------------
//
// MEASURED 2026-09-21 on the car, wheels off the ground, mux armed, commanded via the Jetson
// PWM (docs/notes/bench-session-2026-09-20.md's steering endpoint follow-up): left mechanical
// stop 1094 us, right mechanical stop 1875 us, neutral 1500 us, and a SHORTER pulse turns the
// wheels LEFT (so steering.pwm_left_bound is "pwm_min_us", i.e. left_is_pwm_max is false).
//
// These numbers are typed in here deliberately, not read out of vehicle_params.yaml: this is
// the L1 layer (claude-docs/12-testing.md), which per this file's own CMakeLists.txt
// deliberately never includes the generated vehicle_params binding, so it is table-fixture
// values ALL THE WAY DOWN. What ties this fixture back to the real committed file is
// test_pwm_output_node_launch.py's L3 suite, which reads config/vehicle_params.yaml directly
// and would fail the moment these two disagree. If the committed calibration changes, this
// fixture must be updated to match, on purpose -- that is what "golden" means here.
MappingConfig committed_steering_config() {
  MappingConfig config = nominal_config();
  config.steering.min_us = 1094.0;
  config.steering.neutral_us = 1500.0;
  config.steering.max_us = 1875.0;
  config.steering_min_angle_rad = -0.4189;
  config.steering_max_angle_rad = 0.4189;
  config.left_is_pwm_max = false;  // pwm_left_bound: "pwm_min_us" -- shorter pulse is LEFT
  return config;
}

TEST(SteeringAngleToPulse, CommittedCalibrationFullLeftIsPwmMinUs) {
  const MappingConfig config = committed_steering_config();
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, 0.4189), 1094.0, 1e-6);
}

TEST(SteeringAngleToPulse, CommittedCalibrationFullRightIsPwmMaxUs) {
  const MappingConfig config = committed_steering_config();
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, -0.4189), 1875.0, 1e-6);
}

TEST(SteeringAngleToPulse, CommittedCalibrationZeroIsNeutral) {
  const MappingConfig config = committed_steering_config();
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, 0.0), 1500.0, 1e-6);
}

TEST(SteeringAngleToPulse, CommittedCalibrationOutOfRangeAnglesClampToTheEndpointsNeverBeyond) {
  const MappingConfig config = committed_steering_config();
  // Epsilon and far over both directions: clamps exactly at the endpoint, never past it.
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, 0.4189 + kEps), 1094.0, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, 100.0), 1094.0, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, -0.4189 - kEps), 1875.0, 1e-6);
  EXPECT_NEAR(racer_drivers::steering_angle_to_pulse_us(config, -100.0), 1875.0, 1e-6);
}

TEST(SteeringAngleToPulse, CommittedCalibrationNeverLeaves1094To1875ForAnyAngle) {
  const MappingConfig config = committed_steering_config();
  for (double angle = -50.0; angle <= 50.0; angle += 0.01) {
    const double pulse = racer_drivers::steering_angle_to_pulse_us(config, angle);
    EXPECT_GE(pulse, 1094.0) << "angle=" << angle;
    EXPECT_LE(pulse, 1875.0) << "angle=" << angle;
  }
  // Non-finite input is neutral, also inside the range.
  for (double angle : {kNaN, kInf, -kInf}) {
    const double pulse = racer_drivers::steering_angle_to_pulse_us(config, angle);
    EXPECT_GE(pulse, 1094.0);
    EXPECT_LE(pulse, 1875.0);
  }
}

// -- reverse / below-neutral pulses ------------------------------------------------------------
//
// The mapping itself still maps a negative speed to a below-neutral pulse; whether a negative
// speed ever REACHES it is now the teleop nodes' `allow_reverse` parameter (default false,
// see racer_tools/keymap.py). This pins that the two ends of that decision stay separate: the
// driver is not where reverse is disabled.

TEST(SpeedToPulse, NegativeSpeedStillMapsBelowNeutralWhateverTheTeleopClampDoes) {
  const MappingConfig config = nominal_config();
  EXPECT_LT(racer_drivers::speed_to_pulse_us(config, -1.0), config.throttle.neutral_us);
  EXPECT_GE(racer_drivers::speed_to_pulse_us(config, -1.0), config.throttle.min_us);
  EXPECT_DOUBLE_EQ(racer_drivers::speed_to_pulse_us(config, 0.0), config.throttle.neutral_us);
}

TEST(SpeedToPulse, TheFullSpanStaysInsideTheMuxPwmValidityWindow) {
  // firmware/safety_mux's pwm_is_valid_us accepts [min_us, max_us] INCLUSIVE, from the same
  // config/vehicle_params.yaml fields this map's ends come from, so a saturated pulse at
  // either end is still a valid pulse to the mux rather than something it cuts on.
  const MappingConfig config = nominal_config();
  for (double speed = -100.0; speed <= 100.0; speed += 0.5) {
    const double pulse = racer_drivers::speed_to_pulse_us(config, speed);
    EXPECT_GE(pulse, config.throttle.min_us);
    EXPECT_LE(pulse, config.throttle.max_us);
  }
}

// -- wait_until: the bounded retry SysfsPwmChannel::start() waits for udev with ---------------
//
// Added 2026-09-21 after the first real Jetson bring-up (docs/notes/build-log.md). start()
// used to wait only for the exported channel's DIRECTORY to appear, which the kernel creates
// synchronously on the export write -- so the wait always fell through on its first check and
// the next write raced udev's chgrp of the attribute files and lost ("Permission denied" on
// pwm0/enable, unprivileged, in the car container). It now waits on a predicate that also
// requires those attributes to be writable.
//
// The udev race itself cannot be staged in a unit test (no kernel, no PWM chip, and as root
// every access(W_OK) succeeds anyway). What IS testable, and what the fix actually rests on,
// is that this retry helper polls a not-yet-true condition instead of giving up on the first
// look -- which is precisely what the old loop failed to do for the condition that mattered.

TEST(WaitUntil, ReturnsTrueImmediatelyWithoutSleepingWhenAlreadySatisfied) {
  int calls = 0;
  EXPECT_TRUE(racer_drivers::wait_until(
      [&calls] {
        ++calls;
        return true;
      },
      50, 0));
  EXPECT_EQ(calls, 1);
}

TEST(WaitUntil, KeepsPollingUntilTheConditionBecomesTrue) {
  // The regression this pins: a condition that is false on the first look and true a few
  // polls later must still be seen. The old loop would have surfaced this as a hard failure.
  int calls = 0;
  EXPECT_TRUE(racer_drivers::wait_until(
      [&calls] {
        ++calls;
        return calls >= 4;
      },
      50, 0));
  EXPECT_EQ(calls, 4);
}

TEST(WaitUntil, GivesUpAfterTheBudgetRatherThanRetryingForever) {
  // Bounded, on purpose: this is the actuator path, and "never became writable" must reach
  // the operator as a refuse-to-start, not as a node that hangs in start() (CLAUDE.md
  // invariant 1's fail-closed stance, and pwm_output_node's own refuse-to-start discipline).
  int calls = 0;
  EXPECT_FALSE(racer_drivers::wait_until(
      [&calls] {
        ++calls;
        return false;
      },
      5, 0));
  // Five polls in the loop plus the final confirming call before reporting failure.
  EXPECT_EQ(calls, 6);
}

TEST(WaitUntil, ZeroAttemptsStillEvaluatesTheConditionExactlyOnce) {
  int calls = 0;
  EXPECT_TRUE(racer_drivers::wait_until(
      [&calls] {
        ++calls;
        return true;
      },
      0, 0));
  EXPECT_EQ(calls, 1);
}

}  // namespace
