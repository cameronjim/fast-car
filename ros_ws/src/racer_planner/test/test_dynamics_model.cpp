// Model step against hand computed values, the sign cases and every clamp. The geometry here is
// synthetic (round numbers so the arithmetic can be done by hand), not the car.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

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

// Dead zone (bundle schema 1.1.0): target = gain * cmd when |cmd| >= deadband, else 0.
DynamicsParams with_dead_zone(double deadband, double gain = 1.0) {
  auto p = nominal();
  p.nominal.speed_deadband_mps = deadband;
  p.nominal.speed_gain = gain;
  return p;
}

TEST(DynamicsModel, CommandBelowTheDeadZoneKeepsACarAtRestAtRest) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  EXPECT_EQ(m.step(State{}, Control{0.19, 0.0}).v, 0.0);
  EXPECT_EQ(m.step(State{}, Control{0.05, 0.0}).v, 0.0);
  EXPECT_EQ(m.step(State{}, Control{-0.1, 0.0}).v, 0.0);
}

TEST(DynamicsModel, CommandAtTheDeadZoneEdgeMoves) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  EXPECT_NEAR(m.step(State{}, Control{0.2, 0.0}).v, kA * 0.2, 1e-12);
}

TEST(DynamicsModel, CommandAboveTheDeadZoneIsUnchangedAndGainApplies) {
  const DynamicsModel m(with_dead_zone(0.2, 0.9), synthetic(), 0.1);
  EXPECT_NEAR(m.step(State{}, Control{0.5, 0.0}).v, kA * 0.9 * 0.5, 1e-12);
}

TEST(DynamicsModel, RollingCarDecaysToZeroUnderACommandInsideTheDeadZone) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  // v = 0.4, target 0: v' = 0.4 - kA * 0.4 (the gain is not applied inside the zone).
  EXPECT_NEAR(m.step(State{0, 0, 0, 0.4, 0}, Control{0.1, 0.0}).v, 0.4 * (1.0 - kA), 1e-12);
  State s{0, 0, 0, 0.4, 0};
  for (int i = 0; i < 60; ++i) s = m.step(s, Control{0.1, 0.0});
  EXPECT_LT(s.v, 1e-9);
}

TEST(DynamicsModel, RollingCarKeepsMovingJustAboveTheDeadZone) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  State s{0, 0, 0, 0.4, 0};
  for (int i = 0; i < 60; ++i) s = m.step(s, Control{0.25, 0.0});
  EXPECT_NEAR(s.v, 0.25, 1e-6);
}

TEST(DynamicsModel, DeadZoneIsSymmetricInTheSignOfTheCommand) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  // A reverse command inside the zone has a target of 0, beyond it the target is negative.
  EXPECT_NEAR(m.step(State{0, 0, 0, 0.4, 0}, Control{-0.1, 0.0}).v, 0.4 * (1.0 - kA), 1e-12);
  EXPECT_EQ(m.step(State{0, 0, 0, 0.1, 0}, Control{-0.5, 0.0}).v, 0.0);
}

TEST(DynamicsModel, ZeroDeadZoneIsTheOldModel) {
  const DynamicsModel with(with_dead_zone(0.0), synthetic(), 0.1);
  const DynamicsModel without(nominal(), synthetic(), 0.1);
  for (double cmd : {-0.3, 0.0, 0.05, 0.4}) {
    EXPECT_EQ(with.step(State{0, 0, 0, 0.2, 0.1}, Control{cmd, 0.1}).v,
              without.step(State{0, 0, 0, 0.2, 0.1}, Control{cmd, 0.1}).v);
  }
}

TEST(DynamicsModel, DeadZoneDoesNotTouchSteering) {
  const DynamicsModel a(with_dead_zone(0.5), synthetic(), 0.1);
  const DynamicsModel b(nominal(), synthetic(), 0.1);
  const State s{0, 0, 0, 0.3, 0.05};
  EXPECT_EQ(a.step(s, Control{0.1, 0.2}).delta, b.step(s, Control{0.1, 0.2}).delta);
  EXPECT_EQ(a.step(s, Control{0.1, 0.2}).yaw, b.step(s, Control{0.1, 0.2}).yaw);
}

TEST(DynamicsModel, StepTrigAndStepAgreeAcrossTheDeadZone) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  const State s{0.1, 0.2, 0.7, 0.3, 0.05};
  for (double cmd : {0.0, 0.1, 0.2, 0.3}) {
    const State a = m.step(s, Control{cmd, 0.1});
    const State b = m.step_trig(s, Control{cmd, 0.1}, std::cos(s.yaw), std::sin(s.yaw), nullptr);
    EXPECT_EQ(a.v, b.v);
    EXPECT_EQ(a.x, b.x);
  }
}

TEST(DynamicsModel, NaNCommandStillGivesNaNSpeed) {
  const DynamicsModel m(with_dead_zone(0.2), synthetic(), 0.1);
  EXPECT_TRUE(std::isnan(m.step(State{}, Control{std::nan(""), 0.0}).v));
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

// The residual's forward pass is vectorised (transposed hidden layers; in the fast mode also a
// polynomial exp based tanh and a split output sum): in both modes it must equal a plain
// std::tanh forward pass in the bundle's layout to rounding, for inputs from tiny to saturating
// and infinite, and pass a NaN on. The float32 path (owner decision 30) is held to the same
// reference with its own, looser bound.
struct ResidualErrors {
  double max_abs = 0.0;  // largest |got - want|
  double max_rel = 0.0;  // largest |got - want| / max(|want|, 1)
};

// weight_span: weights are uniform in +-span/2. 6 (the pinned tests' choice) saturates the tanh
// units; 1.2 is the size of a trained net's weights.
// float_weights: round every weight to the nearest float first, as the bundle stores them
// (residual_mlp.f32), so the float32 path loses nothing in the weights. operating_only: inputs in
// the car's operating range instead of the stress sweep.
static ResidualErrors check_residual_against_a_std_tanh_forward_pass(bool fast_mlp, bool float32,
                                                                     double weight_span, double tol,
                                                                     bool float_weights = false,
                                                                     bool operating_only = false) {
  DynamicsParams p = nominal();
  p.fast_mlp = fast_mlp;
  p.float32_mlp = float32;
  p.residual.enabled = true;
  p.residual.input_mean = {0.5, 0.0, 0.6, 0.0};
  p.residual.input_std = {0.4, 0.2, 0.4, 0.2};
  p.residual.output_scale = {0.7, 0.3};
  p.residual.weights.resize(kMlpWeightCount);
  std::uint32_t state = 12345u;
  for (double& w : p.residual.weights) {
    state = state * 1664525u + 1013904223u;
    w = (static_cast<double>(state) / 4294967296.0 - 0.5) * weight_span;
    if (float_weights) w = static_cast<double>(static_cast<float>(w));
  }
  const DynamicsModel m(p, synthetic(), 0.1);
  auto reference = [&](const double in4[4], double out[2]) {
    const auto& r = p.residual;
    const double* w1 = r.weights.data();
    const double* b1 = w1 + kMlpHidden * kMlpIn;
    const double* w2 = b1 + kMlpHidden;
    const double* b2 = w2 + kMlpHidden * kMlpHidden;
    const double* w3 = b2 + kMlpHidden;
    const double* b3 = w3 + kMlpOut * kMlpHidden;
    double x[4], h1[kMlpHidden], h2[kMlpHidden];
    for (int j = 0; j < 4; ++j) x[j] = (in4[j] - r.input_mean[j]) / r.input_std[j];
    for (int i = 0; i < kMlpHidden; ++i) {
      double a = b1[i];
      for (int j = 0; j < kMlpIn; ++j) a += w1[i * kMlpIn + j] * x[j];
      h1[i] = std::tanh(a);
    }
    for (int i = 0; i < kMlpHidden; ++i) {
      double a = b2[i];
      for (int j = 0; j < kMlpHidden; ++j) a += w2[i * kMlpHidden + j] * h1[j];
      h2[i] = std::tanh(a);
    }
    for (int i = 0; i < kMlpOut; ++i) {
      double a = b3[i];
      for (int j = 0; j < kMlpHidden; ++j) a += w3[i * kMlpHidden + j] * h2[j];
      out[i] = r.output_scale[i] * a;
    }
  };
  ResidualErrors e;
  auto compare = [&](const double in4[4]) {
    double got[2], want[2];
    m.residual(in4[0], in4[1], in4[2], in4[3], got);
    reference(in4, want);
    for (int i = 0; i < 2; ++i) {
      const double err = std::fabs(got[i] - want[i]);
      EXPECT_LE(err, tol * std::max(std::fabs(want[i]), 1.0));
      e.max_abs = std::max(e.max_abs, err);
      e.max_rel = std::max(e.max_rel, err / std::max(std::fabs(want[i]), 1.0));
    }
  };
  if (operating_only) {
    // wheel speed 0 to 1.5 m/s, steering angle +-0.4 rad, speed command 0 to 1.5, steering +-0.4:
    // a 9 x 9 x 9 x 9 grid
    for (int a = 0; a < 9; ++a)
      for (int b = 0; b < 9; ++b)
        for (int c = 0; c < 9; ++c)
          for (int d = 0; d < 9; ++d) {
            const double in4[4] = {1.5 * a / 8.0, -0.4 + 0.1 * b, 1.5 * c / 8.0, -0.4 + 0.1 * d};
            compare(in4);
          }
    return e;
  }
  for (double scale : {1e-9, 1e-3, 0.1, 1.0, 10.0, 1e3}) {
    for (int k = 0; k < 50; ++k) {
      const double in4[4] = {scale * std::sin(1.0 + k), scale * std::cos(2.0 * k),
                             scale * std::sin(0.3 * k), scale * std::cos(0.7 + k)};
      compare(in4);
    }
  }
  // Infinite inputs saturate the first layer (tanh(+-inf) = +-1), the output stays finite.
  for (double inf : {INFINITY, -INFINITY}) {
    const double in4[4] = {inf, 0.1, 0.6, 0.2};
    double want[2];
    reference(in4, want);
    EXPECT_TRUE(std::isfinite(want[0]) && std::isfinite(want[1]));
    compare(in4);
  }
  double out[2];
  m.residual(std::nan(""), 0.0, 0.0, 0.0, out);
  EXPECT_TRUE(std::isnan(out[0]) && std::isnan(out[1]));
  return e;
}

static void expect_residual_equals_a_std_tanh_forward_pass(bool fast_mlp) {
  check_residual_against_a_std_tanh_forward_pass(fast_mlp, false, 6.0, 1e-12);
}

TEST(DynamicsModel, ResidualEqualsAStdTanhForwardPass) {
  expect_residual_equals_a_std_tanh_forward_pass(false);
}

TEST(DynamicsModel, FastResidualEqualsAStdTanhForwardPass) {
  expect_residual_equals_a_std_tanh_forward_pass(true);
}

// Owner decision 30: the float32 path (DynamicsParams::float32_mlp) against the double reference.
// The bound applies to this path only; the double paths above keep 1e-12. Error is
// |got - want| / max(|want|, 1), in the std::tanh variant and the vectorised fast variant, with
// weights as the bundle stores them (float) and standardised inputs rounded to float.
//
// Measured (aarch64, gcc 11, -O3; the table below is printed by the test):
//   trained size weights (+-0.6), operating range of inputs:  5.0e-7 (std::tanh), 4.8e-7 (fast)
//   the real dyn-v0.7p bundle on the same grid:               3.8e-7 (std::tanh), 3.1e-7 (fast)
//   (scratch run, bundle not in the repo; outputs up to 1.5)
// That is the float rounding of a 32 term sum (about sqrt(32) * 6e-8), so single precision cannot
// be much closer than 3e-7: the bound is 1e-6, twice the worst measurement. The owner's "about
// 1e-7" of decision 30 is met in order of magnitude only, and the worst golden rollout error of
// the bundle contract stays under 3e-8 (test_bundle_loader).
// Saturating weights (+-3, tanh units pinned at +-1) and inputs up to 1e3 push the pre-activations
// into the thousands, where float rounding of the pre-activation alone is 1e-4 of the tanh slope
// region: those sweeps are held to looser bounds, each 3 to 6 times its measurement.
constexpr double kFloat32Bound = 1e-6;
constexpr double kFloat32StressBound = 1e-5;
constexpr double kFloat32SaturatingBound = 1e-4;

TEST(DynamicsModel, Float32ResidualMatchesTheDoubleReferenceWithinItsBound) {
  for (const bool fast : {false, true}) {
    // trained size weights: operating range, then the stress sweep
    const ResidualErrors op =
        check_residual_against_a_std_tanh_forward_pass(fast, true, 1.2, kFloat32Bound, true, true);
    const ResidualErrors stress = check_residual_against_a_std_tanh_forward_pass(
        fast, true, 1.2, kFloat32StressBound, true, false);
    // saturating weights
    const ResidualErrors sat_op = check_residual_against_a_std_tanh_forward_pass(
        fast, true, 6.0, kFloat32SaturatingBound, true, true);
    const ResidualErrors sat = check_residual_against_a_std_tanh_forward_pass(
        fast, true, 6.0, kFloat32SaturatingBound, true, false);
    std::printf(
        "float32 mlp fast=%d: span 1.2 operating rel %.2e (bound %.0e), stress rel %.2e (bound "
        "%.0e); span 6 operating rel %.2e, stress rel %.2e (bound %.0e)\n",
        fast, op.max_rel, kFloat32Bound, stress.max_rel, kFloat32StressBound, sat_op.max_rel,
        sat.max_rel, kFloat32SaturatingBound);
  }
}

// The float32 switch is off by default and the double path is untouched by it: with the flag off
// the model is bit for bit the one without the field (it is a plain member of the params).
TEST(DynamicsModel, Float32IsOffByDefaultAndTheDoublePathIsBitIdenticalWithoutIt) {
  DynamicsParams p = nominal();
  EXPECT_FALSE(p.float32_mlp);
  p.residual.enabled = true;
  p.residual.output_scale = {0.7, 0.3};
  p.residual.weights.resize(kMlpWeightCount);
  std::uint32_t state = 777u;
  for (double& w : p.residual.weights) {
    state = state * 1664525u + 1013904223u;
    w = (static_cast<double>(state) / 4294967296.0 - 0.5) * 1.2;
  }
  DynamicsParams q = p;
  q.float32_mlp = true;
  const DynamicsModel off(p, synthetic(), 0.1);
  const DynamicsModel on(q, synthetic(), 0.1);
  double a[2], b[2];
  off.residual(0.5, 0.1, 0.7, 0.2, a);
  on.residual(0.5, 0.1, 0.7, 0.2, b);
  EXPECT_NE(a[0], b[0]) << "the float32 pass is a different pass";
  EXPECT_NEAR(a[0], b[0], 1e-6);
  EXPECT_NEAR(a[1], b[1], 1e-6);
  // rolled through the step: the state after 20 steps stays within 1e-5 of the double rollout
  State sa{0, 0, 0, 0.5, 0.0}, sb = sa;
  for (int k = 0; k < 20; ++k) {
    const Control u{0.8, 0.1 * std::sin(0.3 * k)};
    sa = off.step(sa, u);
    sb = on.step(sb, u);
  }
  EXPECT_NEAR(sa.x, sb.x, 1e-5);
  EXPECT_NEAR(sa.yaw, sb.yaw, 1e-5);
  EXPECT_NEAR(sa.v, sb.v, 1e-5);
}
