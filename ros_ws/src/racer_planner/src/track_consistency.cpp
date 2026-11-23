#include "racer_planner/track_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_planner {

namespace {

// Distance from (px, py) to the polyline through the n points (xs, ys).
double distance_to_polyline(double px, double py, const double* xs, const double* ys, int n) {
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i + 1 < n; ++i) {
    const double ux = xs[i + 1] - xs[i];
    const double uy = ys[i + 1] - ys[i];
    const double l2 = ux * ux + uy * uy;
    double s = 0.0;
    if (l2 > 1e-12) s = std::clamp(((px - xs[i]) * ux + (py - ys[i]) * uy) / l2, 0.0, 1.0);
    best = std::min(best, std::hypot(px - (xs[i] + s * ux), py - (ys[i] + s * uy)));
  }
  return best;
}

double min_half_width(const LocalTrackData& t, int n) {
  double m = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    m = std::min(
        m, 0.5 * (std::max(0.0, t.half_width_left_m[i]) + std::max(0.0, t.half_width_right_m[i])));
  }
  return m;
}

// dst = src for a track that passed validate_track, without allocating when dst's vectors have
// room for kTrackStations entries (assign within capacity) and the frame name fits.
void copy_track(LocalTrackData* dst, const LocalTrackData& src) {
  dst->stamp_s = src.stamp_s;
  dst->frame_id.assign(src.frame_id);
  dst->station_spacing_m = src.station_spacing_m;
  dst->x_m.assign(src.x_m.begin(), src.x_m.end());
  dst->y_m.assign(src.y_m.begin(), src.y_m.end());
  dst->half_width_left_m.assign(src.half_width_left_m.begin(), src.half_width_left_m.end());
  dst->half_width_right_m.assign(src.half_width_right_m.begin(), src.half_width_right_m.end());
  dst->sigma_m.assign(src.sigma_m.begin(), src.sigma_m.end());
  dst->valid_count = src.valid_count;
  dst->boundary_residual_m = src.boundary_residual_m;
  dst->intrusion_fraction = src.intrusion_fraction;
}

}  // namespace

ConsistencyResult judge_consistency(const LocalTrackData& candidate,
                                    const LocalTrackData& reference,
                                    const Pose2& reference_in_candidate,
                                    const TrackConsistencyParams& p) {
  ConsistencyResult r;
  if (validate_track(candidate) != TrackValidity::Ok ||
      validate_track(reference) != TrackValidity::Ok) {
    return r;
  }
  const int k = std::clamp(p.compare_stations, 1, kTrackStations);
  const int n_ref = reference.valid_count;
  const int n_new = std::min(candidate.valid_count, k);
  if (n_ref < 2 || n_new < 2) return r;
  const double c = std::cos(reference_in_candidate.yaw);
  const double s = std::sin(reference_in_candidate.yaw);
  double xs[kTrackStations], ys[kTrackStations];
  for (int i = 0; i < n_ref; ++i) {
    xs[i] = c * reference.x_m[i] - s * reference.y_m[i] + reference_in_candidate.x;
    ys[i] = s * reference.x_m[i] + c * reference.y_m[i] + reference_in_candidate.y;
  }
  double jump = 0.0;
  for (int i = 0; i < n_new; ++i) {
    jump = std::max(jump, distance_to_polyline(candidate.x_m[i], candidate.y_m[i], xs, ys, n_ref));
  }
  const double w_new = min_half_width(candidate, n_new);
  const double w_ref = min_half_width(reference, std::min(n_ref, k));
  r.judged = std::isfinite(jump) && std::isfinite(w_new) && std::isfinite(w_ref);
  if (!r.judged) return r;
  r.centre_jump_m = jump;
  r.half_width_ratio = w_ref > 1e-6 ? w_new / w_ref : 1.0;
  r.consistent = !(jump > p.max_centre_jump_m && r.half_width_ratio < p.min_half_width_ratio);
  return r;
}

TrackGate::TrackGate(const TrackConsistencyParams& p) : p_(p) {
  const auto n = static_cast<std::size_t>(kTrackStations);
  accepted_.frame_id.reserve(32);
  accepted_.x_m.reserve(n);
  accepted_.y_m.reserve(n);
  accepted_.half_width_left_m.reserve(n);
  accepted_.half_width_right_m.reserve(n);
  accepted_.sigma_m.reserve(n);
}

void TrackGate::reset() {
  have_accepted_ = false;
  have_judged_ = false;
  bridging_ = false;
  bridged_in_row_ = 0;
  pick_ = TrackPick::kNone;
  result_ = ConsistencyResult{};
}

bool TrackGate::is_new_frame(const LocalTrackData* newest) const {
  return newest != nullptr && (!have_judged_ || newest->stamp_s != judged_stamp_s_);
}

const LocalTrackData* TrackGate::select(const LocalTrackData* newest, double now_s,
                                        const Pose2& accepted_in_newest, bool covered) {
  if (newest == nullptr) {
    pick_ = TrackPick::kNone;
    return nullptr;
  }
  if (!p_.enabled) {
    pick_ = TrackPick::kNewest;
    return newest;
  }
  if (is_new_frame(newest)) {
    have_judged_ = true;
    judged_stamp_s_ = newest->stamp_s;
    bridging_ = false;
    result_ = ConsistencyResult{};
    const bool newest_ok = validate_track(*newest) == TrackValidity::Ok;
    // Only a valid new frame that is later than the accepted one, with the dead reckoning
    // covering the time between them, is judged; and only one frame in a row is ever bridged.
    if (newest_ok && have_accepted_ && bridged_in_row_ < 1 && newest->stamp_s > accepted_.stamp_s &&
        covered) {
      result_ = judge_consistency(*newest, accepted_, accepted_in_newest, p_);
      bridging_ = result_.judged && !result_.consistent;
    }
    if (bridging_) {
      ++bridged_in_row_;
      ++bridged_total_;
    } else {
      bridged_in_row_ = 0;
      if (newest_ok) {
        copy_track(&accepted_, *newest);
        have_accepted_ = true;
      } else {
        have_accepted_ = false;  // an invalid track is used as it is (confidence 0), never bridged
      }
    }
  }
  // Bounded: an accepted track older than max_hold_age_s never stands in for a newer one (a NaN
  // age counts as too old).
  if (bridging_ && have_accepted_ && now_s - accepted_.stamp_s <= p_.max_hold_age_s) {
    pick_ = TrackPick::kBridged;
    return &accepted_;
  }
  pick_ = TrackPick::kNewest;
  return newest;
}

}  // namespace racer_planner
