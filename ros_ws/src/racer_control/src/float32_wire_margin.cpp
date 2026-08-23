#include "racer_control/float32_wire_margin.hpp"

namespace racer_control {

namespace {
constexpr float kFloatWireSafetyMargin = 1e-4f;
}  // namespace

float clamp_for_float32_publish(double value, double min_bound, double max_bound) {
  const float margin_min = static_cast<float>(min_bound) + kFloatWireSafetyMargin;
  const float margin_max = static_cast<float>(max_bound) - kFloatWireSafetyMargin;
  float value_f = static_cast<float>(value);
  if (value_f > margin_max) {
    value_f = margin_max;
  } else if (value_f < margin_min) {
    value_f = margin_min;
  }
  return value_f;
}

}  // namespace racer_control
