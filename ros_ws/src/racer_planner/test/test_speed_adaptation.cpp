// L1 and L2 tests of the online speed adaptation (speed_adaptation.hpp) and of the speed
// correction in the model step: off is exactly the bundle's model, the estimate converges on a
// plant with a known bias and gain, every hard limit holds, reset and bad samples zero it, and
// random input sequences never leave the bounds (property, seeded).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/speed_adaptation.hpp"

using namespace racer_planner;

namespace {

SpeedAdaptationParams on() {
  SpeedAdaptationParams p;
  p.enabled = true;
  return p;
}

VehicleGeometry synthetic() {
  VehicleGeometry g;
  g.wheelbase_m = 0.5;
  g.width_m = 0.3;
  g.rear_overhang_m = 0.1;
  g.front_overhang_m = 0.1;
  g.steering_min_rad = -0.4;
  g.steering_max_rad = 0.4;
  g.steering_max_rate_rad_per_s = 2.0;
  g.max_acceleration_mps2 = 5.0;
  g.stop_deceleration_mps2 = 1.0;
  return g;
}

DynamicsParams nominal() {
  DynamicsParams p;
  p.dt_s = 0.05;
  p.nominal.speed_tau_s = 0.25;
  p.nominal.steer_tau_s = 0.1;
  p.nominal.accel_max_mps2 = 2.0;
  p.nominal.decel_max_mps2 = 1.5;
  p.nominal.speed_deadband_mps = 0.2;
  return p;
}

// Feed n samples of a plant a_meas = a_pred * (1 + gain) + bias + noise, a_pred excited.
void feed(SpeedAdaptation* a, double bias, double gain, int n, double noise, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> apred(-1.5, 1.5);
  std::normal_distribution<double> nz(0.0, noise);
  for (int i = 0; i < n; ++i) {
    const double ap = apred(rng);
    a->update(ap, ap * (1.0 + gain) + bias + nz(rng), 1.0, 0.02, true);
  }
}

}  // namespace

TEST(SpeedAdaptation, DisabledDoesNothing) {
  SpeedAdaptation a(SpeedAdaptationParams{});
  EXPECT_FALSE(a.update(0.5, 0.0, 1.0, 0.02, true));
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
  EXPECT_EQ(a.correction().gain, 0.0);
  EXPECT_FALSE(a.status().enabled);
  EXPECT_EQ(a.status().updates, 0u);
}

TEST(SpeedAdaptation, StartsAtZero) {
  SpeedAdaptation a(on());
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
  EXPECT_EQ(a.correction().gain, 0.0);
  EXPECT_EQ(a.status().resets, 0u);
}

TEST(SpeedAdaptation, ConvergesOnAKnownPlant) {
  SpeedAdaptation a(on());
  feed(&a, -0.2, 0.15, 2000, 0.05, 1);
  EXPECT_NEAR(a.correction().bias_mps2, -0.2, 0.02);
  EXPECT_NEAR(a.correction().gain, 0.15, 0.03);
  EXPECT_FALSE(a.status().at_limit);
  EXPECT_GT(a.status().updates, 1900u);
}

TEST(SpeedAdaptation, ExactPlantIsLeftAlone) {
  SpeedAdaptation a(on());
  feed(&a, 0.0, 0.0, 500, 0.0, 2);
  EXPECT_NEAR(a.correction().bias_mps2, 0.0, 1e-12);
  EXPECT_NEAR(a.correction().gain, 0.0, 1e-12);
}

TEST(SpeedAdaptation, HardLimitsHold) {
  SpeedAdaptation a(on());
  feed(&a, 3.0, 0.0, 500, 0.0, 3);  // innovation 3.0 is under innovation_max 4.0
  EXPECT_DOUBLE_EQ(a.correction().bias_mps2, a.params().bias_max_mps2);
  EXPECT_TRUE(a.status().at_limit);
  SpeedAdaptation b(on());
  feed(&b, 0.0, -0.8, 500, 0.0, 4);
  EXPECT_DOUBLE_EQ(b.correction().gain, -b.params().gain_max);
  EXPECT_TRUE(b.status().at_limit);
}

TEST(SpeedAdaptation, ParametersCannotLoosenTheLimits) {
  SpeedAdaptationParams p = on();
  p.gain_max = 5.0;                                            // pulled to 0.9
  p.bias_max_mps2 = std::numeric_limits<double>::quiet_NaN();  // pulled to 0
  p.forgetting = 0.1;                                          // pulled to 0.9
  SpeedAdaptation a(p);
  EXPECT_LE(a.params().gain_max, 0.9);
  EXPECT_EQ(a.params().bias_max_mps2, 0.0);
  EXPECT_GE(a.params().forgetting, 0.9);
  feed(&a, 1.0, 0.0, 200, 0.0, 5);
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
}

TEST(SpeedAdaptation, ResetZeroes) {
  SpeedAdaptation a(on());
  feed(&a, -0.3, 0.1, 500, 0.0, 6);
  ASSERT_NE(a.correction().bias_mps2, 0.0);
  a.reset();
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
  EXPECT_EQ(a.correction().gain, 0.0);
  EXPECT_EQ(a.status().updates, 0u);
  EXPECT_EQ(a.status().resets, 1u);
}

TEST(SpeedAdaptation, NonFiniteSampleResets) {
  SpeedAdaptation a(on());
  feed(&a, -0.3, 0.1, 500, 0.0, 7);
  EXPECT_FALSE(a.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 0.02, true));
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
  EXPECT_EQ(a.status().resets, 1u);
  EXPECT_FALSE(a.update(0.0, std::numeric_limits<double>::infinity(), 1.0, 0.02, true));
  EXPECT_EQ(a.status().resets, 2u);
}

TEST(SpeedAdaptation, GatedSamplesAreSkipped) {
  SpeedAdaptation a(on());
  EXPECT_FALSE(a.update(0.5, 0.0, 1.0, 0.02, false));  // not driving forward
  EXPECT_FALSE(a.update(0.5, 0.0, 0.1, 0.02, true));   // under min_speed_mps
  EXPECT_FALSE(a.update(0.5, 0.0, 1.0, 0.001, true));  // interval too short
  EXPECT_FALSE(a.update(0.5, 0.0, 1.0, 0.5, true));    // interval too long
  EXPECT_FALSE(a.update(0.0, 10.0, 1.0, 0.02, true));  // innovation above the limit
  EXPECT_EQ(a.status().skipped, 5u);
  EXPECT_EQ(a.status().updates, 0u);
  EXPECT_EQ(a.correction().bias_mps2, 0.0);
  EXPECT_TRUE(a.update(0.5, 0.4, 1.0, 0.02, true));
}

// L2 property: whatever finite or non-finite inputs arrive, the correction stays finite and
// inside its hard limits, and with no excitation of the gain (a_pred always 0) it does not drift.
TEST(SpeedAdaptationProperty, AlwaysBoundedAndFinite) {
  std::mt19937 rng(20261124);
  std::uniform_real_distribution<double> u(-20.0, 20.0);
  std::uniform_real_distribution<double> v(-1.0, 3.0);
  std::uniform_real_distribution<double> dtd(0.0, 0.3);
  std::uniform_int_distribution<int> pick(0, 99);
  for (int trial = 0; trial < 50; ++trial) {
    SpeedAdaptationParams p = on();
    p.bias_max_mps2 = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
    p.gain_max = std::uniform_real_distribution<double>(0.0, 0.5)(rng);
    p.forgetting = std::uniform_real_distribution<double>(0.9, 1.0)(rng);
    SpeedAdaptation a(p);
    for (int i = 0; i < 2000; ++i) {
      double ap = u(rng), am = u(rng);
      if (pick(rng) == 0) am = std::numeric_limits<double>::quiet_NaN();
      a.update(ap, am, v(rng), dtd(rng), pick(rng) < 90);
      const SpeedCorrection c = a.correction();
      ASSERT_TRUE(std::isfinite(c.bias_mps2));
      ASSERT_TRUE(std::isfinite(c.gain));
      ASSERT_LE(std::fabs(c.bias_mps2), p.bias_max_mps2 + 1e-15);
      ASSERT_LE(std::fabs(c.gain), p.gain_max + 1e-15);
    }
  }
}

TEST(SpeedAdaptationProperty, NoGainDriftWithoutExcitation) {
  SpeedAdaptation a(on());
  std::mt19937 rng(9);
  std::normal_distribution<double> nz(0.0, 0.3);
  for (int i = 0; i < 20000; ++i) a.update(0.0, nz(rng), 1.0, 0.02, true);
  EXPECT_EQ(a.correction().gain, 0.0);  // phi = [1, 0]: the gain row is never excited
  EXPECT_LE(std::fabs(a.correction().bias_mps2), 0.2);
}

TEST(SpeedCorrectionInModel, ZeroCorrectionIsBitIdentical) {
  const DynamicsModel plain(nominal(), synthetic(), 0.05);
  DynamicsModel corrected(nominal(), synthetic(), 0.05);
  corrected.set_speed_correction(SpeedCorrection{});
  State s{0.0, 0.0, 0.1, 0.8, 0.05};
  State t = s;
  for (int i = 0; i < 100; ++i) {
    const Control u{1.3 * std::sin(0.1 * i) + 0.6, 0.3 * std::cos(0.07 * i)};
    s = plain.step(s, u);
    t = corrected.step(t, u);
    ASSERT_EQ(s.x, t.x);
    ASSERT_EQ(s.y, t.y);
    ASSERT_EQ(s.yaw, t.yaw);
    ASSERT_EQ(s.v, t.v);
    ASSERT_EQ(s.delta, t.delta);
  }
}

TEST(SpeedCorrectionInModel, ScalesTheSpeedStepOnlyWhenDriving) {
  const DynamicsModel plain(nominal(), synthetic(), 0.05);
  DynamicsModel corrected(nominal(), synthetic(), 0.05);
  corrected.set_speed_correction(SpeedCorrection{-0.2, 0.1});
  const State s{0.0, 0.0, 0.0, 0.5, 0.0};
  const Control drive{1.0, 0.0};
  const double dv = plain.step(s, drive).v - s.v;
  EXPECT_NEAR(corrected.step(s, drive).v - s.v, dv * 1.1 - 0.2 * 0.05, 1e-12);
  // The steering, the yaw and the position of this step do not depend on the correction.
  EXPECT_EQ(corrected.step(s, drive).delta, plain.step(s, drive).delta);
  EXPECT_EQ(corrected.step(s, drive).x, plain.step(s, drive).x);
  // A command in the dead zone (target 0): no correction, a standing car stays standing.
  const Control stop{0.1, 0.0};
  EXPECT_EQ(corrected.step(s, stop).v, plain.step(s, stop).v);
  EXPECT_EQ(corrected.step(State{}, stop).v, 0.0);
  // A non-finite correction is refused (treated as none).
  corrected.set_speed_correction(SpeedCorrection{std::numeric_limits<double>::quiet_NaN(), 0.1});
  EXPECT_EQ(corrected.step(s, drive).v, plain.step(s, drive).v);
}

// Closed loop on a synthetic plant that accelerates 20 percent slower than the model and with a
// constant drag: the adapted one-step prediction error falls well below the unadapted one.
TEST(SpeedCorrectionInModel, AdaptationReducesPredictionErrorOnASlowPlant) {
  const DynamicsModel model(nominal(), synthetic(), 0.05);
  DynamicsModel adapted(nominal(), synthetic(), 0.05);
  SpeedAdaptation a(on());
  double v = 0.6;
  double err_plain = 0.0, err_adapted = 0.0;
  int n = 0;
  for (int i = 0; i < 3000; ++i) {
    const Control u{0.9 + 0.4 * std::sin(0.013 * i), 0.0};
    const State s{0.0, 0.0, 0.0, v, 0.0};
    const double a_pred = (model.step(s, u).v - v) / 0.05;
    const double a_true = 0.8 * a_pred - 0.1;
    const double v_next = v + a_true * 0.05;
    if (i > 1500) {
      err_plain += std::pow(model.step(s, u).v - v_next, 2);
      err_adapted += std::pow(adapted.step(s, u).v - v_next, 2);
      ++n;
    }
    a.update(a_pred, a_true, v, 0.05, true);
    adapted.set_speed_correction(a.correction());
    v = v_next;
  }
  ASSERT_GT(n, 0);
  EXPECT_LT(err_adapted, 0.01 * err_plain);
  EXPECT_NEAR(a.correction().gain, -0.2, 0.02);
  EXPECT_NEAR(a.correction().bias_mps2, -0.1, 0.02);
}
