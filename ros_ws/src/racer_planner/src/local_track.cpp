#include "racer_planner/local_track.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

TrackValidity validate_track(const LocalTrackData& t) {
  const std::size_t k = static_cast<std::size_t>(kTrackStations);
  if (t.x_m.size() != k || t.y_m.size() != k || t.half_width_left_m.size() != k ||
      t.half_width_right_m.size() != k || t.sigma_m.size() != k || t.valid_count > kTrackStations ||
      t.valid_count < 0) {
    return TrackValidity::WrongLength;
  }
  if (!(std::fabs(t.station_spacing_m - kTrackStationSpacingM) <= 1e-6)) {
    return TrackValidity::WrongSpacing;
  }
  if (t.frame_id != "base_link") return TrackValidity::WrongFrame;
  for (int i = 0; i < t.valid_count; ++i) {
    if (!std::isfinite(t.x_m[i]) || !std::isfinite(t.y_m[i]) ||
        !std::isfinite(t.half_width_left_m[i]) || !std::isfinite(t.half_width_right_m[i]) ||
        !std::isfinite(t.sigma_m[i])) {
      return TrackValidity::NonFinite;
    }
  }
  if (t.valid_count == 0) return TrackValidity::NoTrack;
  return TrackValidity::Ok;
}

bool TrackProjector::build(const LocalTrackData& track, const Pose2& frame_change) {
  n_ = 0;
  if (validate_track(track) != TrackValidity::Ok) return false;
  const int n = std::min(track.valid_count, kTrackStations);
  if (n < 2) return false;
  const double c = std::cos(frame_change.yaw);
  const double s = std::sin(frame_change.yaw);
  for (int i = 0; i < n; ++i) {
    px_[i] = c * track.x_m[i] - s * track.y_m[i] + frame_change.x;
    py_[i] = s * track.x_m[i] + c * track.y_m[i] + frame_change.y;
    hwl_[i] = std::max(0.0, track.half_width_left_m[i]);
    hwr_[i] = std::max(0.0, track.half_width_right_m[i]);
  }
  s_at_[0] = 0.0;
  double last_ux = 1.0, last_uy = 0.0;
  for (int i = 0; i + 1 < n; ++i) {
    const double dx = px_[i + 1] - px_[i];
    const double dy = py_[i + 1] - py_[i];
    const double len = std::hypot(dx, dy);
    if (len > 1e-9) {
      ux_[i] = dx / len;
      uy_[i] = dy / len;
      last_ux = ux_[i];
      last_uy = uy_[i];
    } else {
      ux_[i] = last_ux;
      uy_[i] = last_uy;
    }
    len_[i] = len;
    s_at_[i + 1] = s_at_[i] + len;
  }
  n_ = n;
  return true;
}

namespace {
struct SegEval {
  double t;      // clamped distance along the segment, metres
  double dist2;  // squared distance to the clamped point
};
}  // namespace

int TrackProjector::locate(double x, double y) const {
  int best = 0;
  double best_d2 = 1e300;
  for (int i = 0; i + 1 < n_; ++i) {
    const double rx = x - px_[i];
    const double ry = y - py_[i];
    const double t = std::min(std::max(rx * ux_[i] + ry * uy_[i], 0.0), len_[i]);
    const double ex = rx - t * ux_[i];
    const double ey = ry - t * uy_[i];
    const double d2 = ex * ex + ey * ey;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

Projection TrackProjector::project(double x, double y, int* hint) const {
  Projection out;
  if (n_ < 2) return out;
  const int last = n_ - 2;
  int lo = 0, hi = last;
  if (hint != nullptr) {
    lo = std::max(0, std::min(*hint, last) - 1);
    hi = std::min(last, std::min(*hint, last) + 2);
  }
  int best = lo;
  double best_d2 = 1e300;
  for (int i = lo; i <= hi; ++i) {
    const double rx = x - px_[i];
    const double ry = y - py_[i];
    const double t = std::min(std::max(rx * ux_[i] + ry * uy_[i], 0.0), len_[i]);
    const double ex = rx - t * ux_[i];
    const double ey = ry - t * uy_[i];
    const double d2 = ex * ex + ey * ey;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  if (hint != nullptr) *hint = best;
  const double rx = x - px_[best];
  const double ry = y - py_[best];
  const double along = rx * ux_[best] + ry * uy_[best];
  // The first segment extends backwards (the car may be a little behind station 0); the last is
  // capped so progress saturates at the end of what is visible.
  double t_seg = along;
  if (best != 0) t_seg = std::max(t_seg, 0.0);
  t_seg = std::min(t_seg, len_[best]);
  out.s = s_at_[best] + t_seg;
  out.d = ux_[best] * ry - uy_[best] * rx;
  const double frac = len_[best] > 1e-9 ? std::min(std::max(t_seg / len_[best], 0.0), 1.0) : 0.0;
  out.half_left = hwl_[best] + frac * (hwl_[best + 1] - hwl_[best]);
  out.half_right = hwr_[best] + frac * (hwr_[best + 1] - hwr_[best]);
  return out;
}

}  // namespace racer_planner
