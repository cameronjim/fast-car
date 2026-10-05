#include "racer_control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace racer_control {

double Pid::update(double error, double dt_s) {
  if (!std::isfinite(error)) {
    return 0.0;
  }
  const bool can_integrate = has_previous_ && std::isfinite(dt_s) && dt_s > 0.0;
  double derivative = 0.0;
  if (can_integrate) {
    const double limit = std::abs(gains_.integral_limit);
    integral_ = std::min(std::max(integral_ + error * dt_s, -limit), limit);
    derivative = (error - previous_error_) / dt_s;
  }
  previous_error_ = error;
  has_previous_ = true;
  return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
}

void Pid::reset() {
  integral_ = 0.0;
  previous_error_ = 0.0;
  has_previous_ = false;
}

}  // namespace racer_control
