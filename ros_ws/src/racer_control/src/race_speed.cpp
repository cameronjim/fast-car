#include "racer_control/race_speed.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_control {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

double lateral_limit_speed(double steering_rad, double wheelbase_m, double a_lat_max_mps2) {
  if (!std::isfinite(steering_rad) || !std::isfinite(wheelbase_m) || !(wheelbase_m > 0.0) ||
      !std::isfinite(a_lat_max_mps2) || !(a_lat_max_mps2 > 0.0)) {
    return 0.0;
  }
  const double t = std::abs(std::tan(steering_rad));
  if (!(t > 1e-9)) {
    return kInf;
  }
  return std::sqrt(a_lat_max_mps2 * wheelbase_m / t);
}

double stopping_limit_speed(double free_path_m, double standoff_m, double decel_mps2,
                            double latency_s) {
  if (!std::isfinite(free_path_m) || !std::isfinite(standoff_m) || !std::isfinite(decel_mps2) ||
      !(decel_mps2 > 0.0) || !std::isfinite(latency_s) || latency_s < 0.0) {
    return 0.0;
  }
  const double room = free_path_m - standoff_m;
  if (!(room > 0.0)) {
    return 0.0;
  }
  return decel_mps2 * (std::sqrt(latency_s * latency_s + 2.0 * room / decel_mps2) - latency_s);
}

double stopping_distance(double speed_mps, double decel_mps2, double latency_s) {
  if (!std::isfinite(speed_mps) || !(speed_mps > 0.0)) {
    return 0.0;
  }
  if (!std::isfinite(decel_mps2) || !(decel_mps2 > 0.0)) {
    return kInf;
  }
  return std::max(latency_s, 0.0) * speed_mps + speed_mps * speed_mps / (2.0 * decel_mps2);
}

double side_proximity_cap(std::optional<double> clearance_m, double distance_m,
                          double min_speed_mps, double max_speed_mps) {
  if (!clearance_m || !std::isfinite(*clearance_m) || !std::isfinite(distance_m) ||
      !(distance_m > 0.0) || *clearance_m >= distance_m) {
    return kInf;
  }
  const double fraction = std::max(*clearance_m, 0.0) / distance_m;
  return min_speed_mps + (max_speed_mps - min_speed_mps) * fraction;
}

double free_path_horizon(double max_speed_mps, double decel_mps2, double latency_s,
                         double standoff_m) {
  if (!std::isfinite(max_speed_mps) || !(max_speed_mps > 0.0) || !std::isfinite(standoff_m)) {
    return 0.0;
  }
  const double stop = stopping_distance(max_speed_mps, decel_mps2, latency_s);
  if (!std::isfinite(stop)) {
    return 3.0;
  }
  return std::min(3.0, stop + standoff_m + 0.3);
}

RaceSpeedTarget race_target_speed(const RaceSpeedInputs& inputs, const RaceSpeedConfig& config) {
  RaceSpeedTarget out;
  out.lateral_mps =
      lateral_limit_speed(inputs.steering_rad, config.wheelbase_m, config.a_lat_max_mps2);
  out.stopping_mps = inputs.free_path_m
                         ? stopping_limit_speed(*inputs.free_path_m, config.standoff_m,
                                                config.brake_decel_mps2, config.stop_latency_s)
                         : kInf;
  out.side_mps = side_proximity_cap(inputs.side_clearance_m, config.side_cap_distance_m,
                                    config.side_cap_min_speed_mps, config.max_speed_mps);
  const double limited = std::min(std::min(config.max_speed_mps, out.lateral_mps),
                                  std::min(out.stopping_mps, out.side_mps));
  // The floor, then never above the ceiling.
  out.target_mps = std::min(std::max(config.min_speed_mps, limited), config.max_speed_mps);
  return out;
}

PercentileWindow::PercentileWindow(std::size_t capacity)
    : ring_(std::max<std::size_t>(capacity, 1), 0.0), scratch_(ring_.size(), 0.0) {}

void PercentileWindow::push(double value) {
  if (!std::isfinite(value)) {
    return;
  }
  ring_[next_] = value;
  next_ = (next_ + 1) % ring_.size();
  count_ = std::min(count_ + 1, ring_.size());
}

void PercentileWindow::clear() {
  next_ = 0;
  count_ = 0;
}

double PercentileWindow::percentile(double q) {
  if (count_ == 0) {
    return 0.0;
  }
  const auto begin = scratch_.begin();
  const auto end = begin + static_cast<std::ptrdiff_t>(count_);
  std::copy(ring_.begin(), ring_.begin() + static_cast<std::ptrdiff_t>(count_), begin);
  const double qc = std::isfinite(q) ? std::min(std::max(q, 0.0), 1.0) : 1.0;
  const auto rank = std::max<std::size_t>(
      static_cast<std::size_t>(std::ceil(qc * static_cast<double>(count_))), 1);
  const auto nth = begin + static_cast<std::ptrdiff_t>(rank - 1);
  std::nth_element(begin, nth, end);
  return *nth;
}

double PercentileWindow::max() const {
  if (count_ == 0) {
    return 0.0;
  }
  return *std::max_element(ring_.begin(), ring_.begin() + static_cast<std::ptrdiff_t>(count_));
}

}  // namespace racer_control
