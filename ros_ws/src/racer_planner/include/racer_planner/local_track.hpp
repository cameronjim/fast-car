// The local track from perception (racer_pnp_msgs/LocalTrack) as a plain struct, its contract
// validation, and polyline projection: arc length s along the centreline, signed lateral offset
// d (left positive) and the half widths at s.
#ifndef RACER_PLANNER_LOCAL_TRACK_HPP_
#define RACER_PLANNER_LOCAL_TRACK_HPP_

#include <array>
#include <string>
#include <vector>

namespace racer_planner {

constexpr int kTrackStations = 16;
constexpr double kTrackStationSpacingM = 0.20;

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct LocalTrackData {
  double stamp_s = 0.0;
  std::string frame_id;
  double station_spacing_m = 0.0;
  std::vector<double> x_m, y_m, half_width_left_m, half_width_right_m, sigma_m;
  int valid_count = 0;
  double boundary_residual_m = 0.0;
  double intrusion_fraction = 0.0;
};

enum class TrackValidity {
  Ok,
  NoTrack,       // valid_count is 0
  WrongLength,   // an array is not 16 long, or valid_count above 16
  WrongSpacing,  // station_spacing_m is not 0.20 within 1e-6
  WrongFrame,    // frame is not base_link
  NonFinite,     // a value among the first valid_count entries is not finite
};

// Contract validation (claude-docs/13 section 4.1). Ok means the message may be used.
TrackValidity validate_track(const LocalTrackData& t);

struct Projection {
  double s = 0.0;          // arc length from station 0, capped at the last valid station
  double d = 0.0;          // signed lateral offset, left of the travel direction positive
  double half_left = 0.0;  // half width at s
  double half_right = 0.0;
};

class TrackProjector {
 public:
  // Builds the polyline of the first `valid_count` stations, moved into another frame by
  // `frame_change` (the pose of the track's frame origin expressed in the target frame, so a
  // point p in the track frame becomes R(yaw) p + (x, y)). Returns false when there are fewer
  // than 2 usable stations.
  bool build(const LocalTrackData& track, const Pose2& frame_change);

  bool valid() const { return n_ >= 2; }
  int station_count() const { return n_; }
  // Arc length of the last valid station: the part of the track the car can see.
  double s_valid_end() const { return n_ >= 2 ? s_at_[n_ - 1] : 0.0; }

  // Index of the nearest segment, by brute force.
  int locate(double x, double y) const;
  // Projection with a segment hint (in and out) for cheap sequential queries along a rollout.
  Projection project(double x, double y, int* hint) const;

 private:
  int n_ = 0;
  std::array<double, kTrackStations> px_{}, py_{}, s_at_{}, hwl_{}, hwr_{};
  std::array<double, kTrackStations> ux_{}, uy_{}, len_{};  // per segment i (station i to i+1)
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_LOCAL_TRACK_HPP_
