// L1 tests for racer_control::clamp_for_float32_publish (claude-docs/12-testing.md).
#include <gtest/gtest.h>

#include "racer_control/float32_wire_margin.hpp"

namespace racer_control {
namespace {

TEST(Float32WireMargin, ValueWellWithinBoundsPassesThroughUnchanged) {
  EXPECT_FLOAT_EQ(clamp_for_float32_publish(0.2, -0.4189, 0.4189), 0.2f);
}

TEST(Float32WireMargin, ValueExactlyAtUpperBoundIsPulledInsideByTheMargin) {
  const float out = clamp_for_float32_publish(0.4189, -0.4189, 0.4189);
  EXPECT_LT(out, 0.4189f);
  EXPECT_GT(out, 0.4189f - 1e-3f);  // still very close to the bound, not a large derate
}

TEST(Float32WireMargin, ValueExactlyAtLowerBoundIsPulledInsideByTheMargin) {
  const float out = clamp_for_float32_publish(-0.4189, -0.4189, 0.4189);
  EXPECT_GT(out, -0.4189f);
  EXPECT_LT(out, -0.4189f + 1e-3f);
}

TEST(Float32WireMargin, ValueSlightlyBeyondUpperBoundIsClampedToTheMarginBound) {
  const float out_at_bound = clamp_for_float32_publish(0.4189, -0.4189, 0.4189);
  const float out_beyond = clamp_for_float32_publish(0.5, -0.4189, 0.4189);
  EXPECT_FLOAT_EQ(out_beyond, out_at_bound);
}

TEST(Float32WireMargin, AsymmetricBoundsHandledIndependently) {
  // Speed's real bounds: min_velocity_mps=-5.0, global_speed_cap_mps=20.0.
  EXPECT_LT(clamp_for_float32_publish(20.0, -5.0, 20.0), 20.0f);
  EXPECT_GT(clamp_for_float32_publish(-5.0, -5.0, 20.0), -5.0f);
  EXPECT_FLOAT_EQ(clamp_for_float32_publish(10.0, -5.0, 20.0), 10.0f);
}

}  // namespace
}  // namespace racer_control
