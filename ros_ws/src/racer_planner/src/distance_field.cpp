#include "racer_planner/distance_field.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

namespace {
constexpr double kNoSeed = 1e12;  // "infinite" in squared cells, finite so the transform has no NaN

// Felzenszwalb and Huttenlocher exact 1D squared distance transform of f over n samples.
void edt_1d(const double* f, int n, double* d, int* v, double* z) {
  int k = 0;
  v[0] = 0;
  z[0] = -1e300;
  z[1] = 1e300;
  for (int q = 1; q < n; ++q) {
    double s;
    for (;;) {
      const int p = v[k];
      s = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) /
          (2.0 * q - 2.0 * p);
      if (s <= z[k] && k > 0) {
        --k;
      } else {
        break;
      }
    }
    if (s <= z[k]) {  // k == 0 and the new parabola dominates everywhere
      v[0] = q;
      z[0] = -1e300;
      z[1] = 1e300;
      continue;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e300;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const double dq = static_cast<double>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}
}  // namespace

DistanceField::DistanceField()
    : nx_(static_cast<int>(std::lround((kXMaxM - kXMinM) / kResolutionM)) + 1),
      ny_(static_cast<int>(std::lround((kYMaxM - kYMinM) / kResolutionM)) + 1) {
  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_;
  const std::size_t longest = static_cast<std::size_t>(std::max(nx_, ny_));
  dist_.assign(cells, static_cast<float>(kFarM));
  f_.assign(cells, kNoSeed);
  col_.assign(longest + 1, 0.0);
  out_.assign(longest + 1, 0.0);
  v_.assign(longest + 1, 0);
  z_.assign(longest + 2, 0.0);
}

void DistanceField::build(const PointXY* points, std::size_t count) {
  std::fill(f_.begin(), f_.end(), kNoSeed);
  seeds_ = 0;
  for (std::size_t p = 0; p < count; ++p) {
    if (!std::isfinite(points[p].x) || !std::isfinite(points[p].y)) continue;
    const double fx = (points[p].x - kXMinM) * kInvRes;
    const double fy = (points[p].y - kYMinM) * kInvRes;
    if (fx < -0.5 || fy < -0.5 || fx > nx_ - 0.5 || fy > ny_ - 0.5) continue;
    const int i = std::min(std::max(static_cast<int>(std::lround(fx)), 0), nx_ - 1);
    const int j = std::min(std::max(static_cast<int>(std::lround(fy)), 0), ny_ - 1);
    f_[static_cast<std::size_t>(j) * nx_ + i] = 0.0;
    ++seeds_;
  }
  if (seeds_ == 0) {
    std::fill(dist_.begin(), dist_.end(), static_cast<float>(kFarM));
    return;
  }
  // Pass 1: along y for every column.
  for (int i = 0; i < nx_; ++i) {
    for (int j = 0; j < ny_; ++j) col_[j] = f_[static_cast<std::size_t>(j) * nx_ + i];
    edt_1d(col_.data(), ny_, out_.data(), v_.data(), z_.data());
    for (int j = 0; j < ny_; ++j) f_[static_cast<std::size_t>(j) * nx_ + i] = out_[j];
  }
  // Pass 2: along x for every row.
  for (int j = 0; j < ny_; ++j) {
    double* row = &f_[static_cast<std::size_t>(j) * nx_];
    edt_1d(row, nx_, out_.data(), v_.data(), z_.data());
    for (int i = 0; i < nx_; ++i) {
      const double m = std::sqrt(out_[i]) * kResolutionM;
      dist_[static_cast<std::size_t>(j) * nx_ + i] = static_cast<float>(std::min(m, kFarM));
    }
  }
}

std::size_t scan_to_points(const float* ranges, std::size_t beam_count, double angle_min,
                           double angle_increment, double range_min, double range_max,
                           double mount_yaw_rad, const VehicleGeometry& g, double body_margin_m,
                           PointXY* out, std::size_t max_points) {
  std::size_t n = 0;
  const double x_lo = g.body_min_x_m() - body_margin_m;
  const double x_hi = g.body_max_x_m() + body_margin_m;
  const double y_hi = g.width_m * 0.5 + body_margin_m;
  for (std::size_t i = 0; i < beam_count && n < max_points; ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) continue;
    const double bearing = angle_min + static_cast<double>(i) * angle_increment + mount_yaw_rad;
    const double x = g.lidar_mount_x_m + r * std::cos(bearing);
    const double y = g.lidar_mount_y_m + r * std::sin(bearing);
    if (x >= x_lo && x <= x_hi && std::fabs(y) <= y_hi) continue;
    out[n++] = PointXY{x, y};
  }
  return n;
}

}  // namespace racer_planner
