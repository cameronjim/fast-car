// Model step against hand computed values, the sign cases and every clamp. The geometry here is
// synthetic (round numbers so the arithmetic can be done by hand), not the car.
#include <gtest/gtest.h>

#include <cmath>

#include "racer_planner/dynamics_model.hpp"

using namespace racer_planner;

namespace {

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
  p.dt_s = 0.1;
  p.nominal.speed_gain = 1.0;
  p.nominal.speed_tau_s = 0.1;
  p.nominal.steer_tau_s = 0.1;
  p.nominal.accel_max_mps2 = 100.0;  // clamps off unless a test turns them on
  p.nominal.decel_max_mps2 = 100.0;
  return p;
}

const double kA = 1.0 - std::exp(-1.0);  // a_d = a_v for dt == tau

}  // namespace

TEST(DynamicsModel, HandComputedStep) {
  const auto g = synthetic();
  const DynamicsModel m(nominal(), g, 0.1);
  State s{1.0, 2.0, 0.5, 1.0, 0.2};
  const State n = m.step(s, Control{2.0, 0.3});
  // delta_target = 0.3, delta_next = 0.2 + kA * 0.1; rate limit 2.0 * 0.1 = 0.2 not binding.
  EXPECT_NEAR(n.delta, 0.2 + kA * 0.1, 1e-12);
  EXPECT_NEAR(n.v, 1.0 + kA * 1.0, 1e-12);
  const double yaw_rate = 1.0 * std::tan(0.2) / 0.5;
  EXPECT_NEAR(n.yaw, 0.5 + yaw_rate * 0.1, 1e-12);
  EXPECT_NEAR(n.x, 1.0 + 1.0 * std::cos(0.5) * 0.1, 1e-12);
  EXPECT_NEAR(n.y, 2.0 + 1.0 * std::sin(0.5) * 0.1, 1e-12);
}

TEST(DynamicsModel, RestStaysAtRest) {
  const DynamicsModel m(nominal(), synthetic(), 0.1);
  const State n = m.step(State{}, Control{});
  EXPECT_EQ(n.x, 0.0);
  EXPECT_EQ(n.y, 0.0);
  EXPECT_EQ(n.yaw, 0.0);
  EXPECT_EQ(n.v, 0.0);
  EXPECT_EQ(n.delta, 0.0);
}

TEST(DynamicsModel, PositiveSteeringGivesPositiveYawRate) {
  const DynamicsModel m(nominal(), synthetic(), 0.1);
  State s;
  s.v = 1.0;
  double yaw_left = 0.0, yaw_right = 0.0;
  State l = s, r = s;
  for (int i = 0; i < 20; ++i) {
    l = m.step(l, Control{1.0, 0.2});
    r = m.step(r, Control{1.0, -0.2});
  }
  yaw_left = l.yaw;
  yaw_right = r.yaw;
  EXPECT_GT(yaw_left, 0.0);
  EXPECT_LT(yaw_right, 0.0);
  EXPECT_NEAR(yaw_left, -yaw_right, 1e-9);
  EXPECT_GT(l.y, 0.0);  // a left turn moves the car to +y
  EXPECT_LT(r.y, 0.0);
}

TEST(DynamicsModel, LeftAndRightGainsApplyToTheirOwnSide) {
  auto p = nominal();
  p.nominal.steer_gain_left = 2.0;
  p.nominal.steer_gain_right = 0.5;
  p.nominal.steer_offset_rad = 0.0;
  const DynamicsModel m(p, synthetic(), 0.1);
  const State l = m.step(State{}, Control{0.0, 0.1});
  const State r = m.step(State{}, Control{0.0, -0.1});
  EXPECT_NEAR(l.delta, kA * 0.2, 1e-12);    // 2.0 * 0.1
  EXPECT_NEAR(r.delta, -kA * 0.05, 1e-12);  // 0.5 * -0.1
  // zero uses the left gain (steering_cmd >= 0)
  p.nominal.steer_offset_rad = 0.1;
  const DynamicsModel m2(p, synthetic(), 0.1);
  EXPECT_NEAR(m2.step(State{}, Control{0.0, 0.0}).delta, kA * 0.1, 1e-12);
}

TEST(DynamicsModel, SteeringTargetIsClampedToTheLimits) {
  const DynamicsModel m(nominal(), synthetic(), 0.1);
  // Target far past 0.4 clamps to 0.4; the rate limit (0.2) is not binding from delta 0.35.
  const State n = m.step(State{0, 0, 0, 0, 0.35}, Control{0.0, 5.0});
  EXPECT_NEAR(n.delta, 0.35 + kA * (0.4 - 0.35), 1e-12);
  const State k = m.step(State{0, 0, 0, 0, -0.35}, Control{0.0, -5.0});
  EXPECT_NEAR(k.delta, -0.35 - kA * 0.05, 1e-12);
}

TEST(DynamicsModel, SteeringRateIsClamped) {
  auto g = synthetic();
  g.steering_max_rate_rad_per_s = 0.5;  // 0.05 rad per 0.1 s step
  const DynamicsModel m(nominal(), g, 0.1);
  EXPECT_NEAR(m.step(State{}, Control{0.0, 0.4}).delta, 0.05, 1e-12);
  EXPECT_NEAR(m.step(State{}, Control{0.0, -0.4}).delta, -0.05, 1e-12);
}

TEST(DynamicsModel, AccelerationAndDecelerationAreClamped) {
  auto p = nominal();
  p.nominal.accel_max_mps2 = 1.0;  // 0.1 m/s per step
  p.nominal.decel_max_mps2 = 2.0;  // 0.2 m/s per step
  const DynamicsModel m(p, synthetic(), 0.1);
  EXPECT_NEAR(m.step(State{}, Control{5.0, 0.0}).v, 0.1, 1e-12);
  EXPECT_NEAR(m.step(State{0, 0, 0, 1.0, 0}, Control{0.0, 0.0}).v, 0.8, 1e-12);
}

TEST(DynamicsModel, SpeedNeverGoesNegative) {
  auto p = nominal();
  p.nominal.speed_gain = 1.0;
  const DynamicsModel m(p, synthetic(), 0.1);
  const State n = m.step(State{0, 0, 0, 0.05, 0}, Control{-2.0, 0.0});
  EXPECT_EQ(n.v, 0.0);
}

TEST(DynamicsModel, SpeedGainScalesTheTargetSpeed) {
  auto p = nominal();
  p.nominal.speed_gain = 0.5;
  const DynamicsModel m(p, synthetic(), 0.1);
  EXPECT_NEAR(m.step(State{}, Control{1.0, 0.0}).v, kA * 0.5, 1e-12);
}

TEST(DynamicsModel, UndersteerReducesYawRate) {
  auto p = nominal();
  p.nominal.understeer_s2_per_m2 = 1.0;
  const DynamicsModel m(p, synthetic(), 0.1);
  const State n = m.step(State{0, 0, 0, 2.0, 0.2}, Control{2.0, 0.2});
  EXPECT_NEAR(n.yaw, 2.0 * std::tan(0.2) / 0.5 / (1.0 + 4.0) * 0.1, 1e-12);
}

TEST(DynamicsModel, YawIsNotWrapped) {
  const DynamicsModel m(nominal(), synthetic(), 0.1);
  const State n = m.step(State{0, 0, 3.1, 1.0, 0.4}, Control{1.0, 0.4});
  EXPECT_GT(n.yaw, M_PI);
}

TEST(DynamicsModel, DifferentStepsGiveConsistentFirstOrderLags) {
  // Two half steps of the lag are the same as one whole step when nothing is clamped.
  const auto g = synthetic();
  const DynamicsModel whole(nominal(), g, 0.1);
  const DynamicsModel half(nominal(), g, 0.05);
  const State a = whole.step(State{}, Control{1.0, 0.0});
  State b = half.step(State{}, Control{1.0, 0.0});
  b = half.step(b, Control{1.0, 0.0});
  EXPECT_NEAR(a.v, b.v, 1e-12);
}

TEST(DynamicsModel, ResidualBiasAddsToAccelerationAndYawRate) {
  auto p = nominal();
  p.residual.enabled = true;
  p.residual.weights.assign(kMlpWeightCount, 0.0);
  // b3 is the last two entries; everything else zero so the network outputs b3 exactly.
  p.residual.weights[kMlpWeightCount - 2] = 0.5;
  p.residual.weights[kMlpWeightCount - 1] = -0.25;
  p.residual.output_scale = {{2.0, 4.0}};
  const DynamicsModel m(p, synthetic(), 0.1);
  const State n = m.step(State{}, Control{0.0, 0.0});
  EXPECT_NEAR(n.v, 2.0 * 0.5 * 0.1, 1e-12);      // dv += r0 * dt
  EXPECT_NEAR(n.yaw, 4.0 * -0.25 * 0.1, 1e-12);  // yaw += r1 * dt
}

TEST(DynamicsModel, ResidualUsesNormalisedInputsAndTanh) {
  auto p = nominal();
  p.residual.enabled = true;
  p.residual.weights.assign(kMlpWeightCount, 0.0);
  // Layer 1: unit i takes input 0. Layer 2: unit 0 takes h1[0]. Layer 3: out0 = h2[0].
  p.residual.weights[0 * kMlpIn + 0] = 1.0;
  const int b1 = kMlpHidden * kMlpIn;
  const int w2 = b1 + kMlpHidden;
  const int b2 = w2 + kMlpHidden * kMlpHidden;
  const int w3 = b2 + kMlpHidden;
  p.residual.weights[w2 + 0 * kMlpHidden + 0] = 1.0;
  p.residual.weights[w3 + 0 * kMlpHidden + 0] = 1.0;
  p.residual.input_mean = {{0.5, 0.0, 0.0, 0.0}};
  p.residual.input_std = {{2.0, 1.0, 1.0, 1.0}};
  const DynamicsModel m(p, synthetic(), 0.1);
  double r[kMlpOut];
  m.residual(2.5, 0.0, 0.0, 0.0, r);
  EXPECT_NEAR(r[0], std::tanh(std::tanh((2.5 - 0.5) / 2.0)), 1e-12);
  EXPECT_NEAR(r[1], 0.0, 1e-12);
}

TEST(DynamicsModel, DisabledResidualIsIgnoredEvenWithWeights) {
  auto p = nominal();
  p.residual.enabled = false;
  p.residual.weights.assign(kMlpWeightCount, 1.0);
  const DynamicsModel with(p, synthetic(), 0.1);
  const DynamicsModel without(nominal(), synthetic(), 0.1);
  const State a = with.step(State{0, 0, 0, 0.5, 0.1}, Control{0.7, 0.2});
  const State b = without.step(State{0, 0, 0, 0.5, 0.1}, Control{0.7, 0.2});
  EXPECT_EQ(a.v, b.v);
  EXPECT_EQ(a.yaw, b.yaw);
}
