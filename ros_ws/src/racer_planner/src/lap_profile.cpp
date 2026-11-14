#include "racer_planner/lap_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_planner {

namespace {
// Position of s on the grid, clamped to the stations: index and fraction.
inline void locate(const LapProfile& p, double s_m, std::size_t* i, double* f) {
  const double u = (s_m - p.s0_m) / p.spacing_m;
  if (!(u > 0.0)) {  // also NaN
    *i = 0;
    *f = 0.0;
    return;
  }
  const double last = static_cast<double>(p.n - 1);
  if (u >= last) {
    *i = p.n - 2;
    *f = 1.0;
    return;
  }
  const double fl = std::floor(u);
  *i = static_cast<std::size_t>(fl);
  *f = u - fl;
}
}  // namespace

double LapProfile::v_bound_at(double s_m) const {
  if (!active || n < kLapProfileMinStations) return std::numeric_limits<double>::infinity();
  std::size_t i;
  double f;
  locate(*this, s_m, &i, &f);
  return v_bound[i] + f * (v_bound[i + 1] - v_bound[i]);
}

double LapProfile::q_at(double s_m) const {
  if (!active || n < kLapProfileMinStations) return 0.0;
  const double u = (s_m - s0_m) / spacing_m;
  const double last = static_cast<double>(n - 1);
  if (u > last) {
    // Beyond the last station: the last slope, continued.
    const double slope = (q[n - 1] - q[n - 2]) / spacing_m;
    return q[n - 1] + slope * (s_m - (s0_m + last * spacing_m));
  }
  std::size_t i;
  double f;
  locate(*this, s_m, &i, &f);
  return q[i] + f * (q[i + 1] - q[i]);
}

LapProfileStatus make_lap_profile(const float* s, const float* v, const float* q, std::size_t n_s,
                                  std::size_t n_v, std::size_t n_q, double stamp_s, double now_s,
                                  LapProfile* out) {
  out->active = false;
  if (n_s == 0 && n_v == 0 && n_q == 0) return LapProfileStatus::kNone;
  if (n_s != n_v || n_s != n_q) return LapProfileStatus::kBad;
  const std::size_t n = n_s;
  if (n < kLapProfileMinStations || n > kLapProfileCapacity) return LapProfileStatus::kBad;
  if (s == nullptr || v == nullptr || q == nullptr) return LapProfileStatus::kBad;
  if (!std::isfinite(stamp_s) || !std::isfinite(now_s)) return LapProfileStatus::kBad;
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isfinite(s[i]) || !std::isfinite(v[i]) || !std::isfinite(q[i])) {
      return LapProfileStatus::kBad;
    }
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = static_cast<double>(s[i + 1]) - static_cast<double>(s[i]);
    if (!(std::fabs(ds - kLapStationSpacingM) <= kLapSpacingToleranceM)) {
      return LapProfileStatus::kBad;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!(v[i] >= 0.0f)) return LapProfileStatus::kBad;
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    if (!(static_cast<double>(q[i + 1]) <= static_cast<double>(q[i]) + kLapCostToGoToleranceS)) {
      return LapProfileStatus::kBad;
    }
  }
  const double age = now_s - stamp_s;
  if (!(age >= 0.0 && age <= kLapProfileMaxAgeS)) return LapProfileStatus::kBad;

  out->stamp_s = stamp_s;
  out->s0_m = static_cast<double>(s[0]);
  out->spacing_m =
      (static_cast<double>(s[n - 1]) - static_cast<double>(s[0])) / static_cast<double>(n - 1);
  out->n = n;
  for (std::size_t i = 0; i < n; ++i) {
    out->v_bound[i] = static_cast<double>(v[i]);
    out->q[i] = static_cast<double>(q[i]);
  }
  out->active = true;
  return LapProfileStatus::kOk;
}

}  // namespace racer_planner
