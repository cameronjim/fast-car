// Distance to the nearest scan point on a 0.05 m grid in the scan's base_link frame, rebuilt once
// per scan with an exact two pass Euclidean distance transform. Storage is allocated once.
#ifndef RACER_PLANNER_DISTANCE_FIELD_HPP_
#define RACER_PLANNER_DISTANCE_FIELD_HPP_

#include <cstddef>
#include <vector>

#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct PointXY {
  double x;
  double y;
};

class DistanceField {
 public:
  static constexpr double kResolutionM = 0.05;
  static constexpr double kXMinM = -1.0;
  static constexpr double kXMaxM = 4.0;
  static constexpr double kYMinM = -2.5;
  static constexpr double kYMaxM = 2.5;
  // Value returned outside the grid and where no point is near.
  static constexpr double kFarM = 10.0;

  DistanceField();

  // Rebuilds the field from scan points already in base_link. No allocation.
  void build(const PointXY* points, std::size_t count);

  // Bilinear lookup in metres. Outside the grid: kFarM.
  double distance(double x, double y) const {
    const double fx = (x - kXMinM) * kInvRes;
    const double fy = (y - kYMinM) * kInvRes;
    if (!(fx >= 0.0 && fy >= 0.0 && fx < static_cast<double>(nx_ - 1) &&
          fy < static_cast<double>(ny_ - 1))) {
      return kFarM;
    }
    const int i = static_cast<int>(fx);
    const int j = static_cast<int>(fy);
    const double ax = fx - i;
    const double ay = fy - j;
    const float* row0 = &dist_[static_cast<std::size_t>(j) * nx_ + i];
    const float* row1 = row0 + nx_;
    const double d0 = row0[0] + ax * (row0[1] - row0[0]);
    const double d1 = row1[0] + ax * (row1[1] - row1[0]);
    return d0 + ay * (d1 - d0);
  }

  // The distance at grid node (i, j), metres, no interpolation. Node (i, j) is at
  // (kXMinM + i * res, kYMinM + j * res).
  double node(int i, int j) const { return dist_[static_cast<std::size_t>(j) * nx_ + i]; }

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  std::size_t seed_count() const { return seeds_; }

 private:
  static constexpr double kInvRes = 1.0 / kResolutionM;
  int nx_;
  int ny_;
  std::size_t seeds_ = 0;
  std::vector<float> dist_;  // metres, nodes at (kXMinM + i * res, kYMinM + j * res)
  std::vector<double> f_;    // working: squared distances in cells
  std::vector<double> col_;  // working
  std::vector<double> out_;  // working
  std::vector<int> v_;       // working
  std::vector<double> z_;    // working
};

// Beams of a LaserScan to base_link points. Bearing of beam i is angle_min + i * increment plus
// the mount yaw; the range is from the LiDAR head at (mount_x, mount_y). Invalid beams (not
// finite, or outside [range_min, range_max]) are skipped, and so are points inside the body
// rectangle grown by body_margin_m (the same rule as the perception input). Returns the number
// of points written (at most max_points).
std::size_t scan_to_points(const float* ranges, std::size_t beam_count, double angle_min,
                           double angle_increment, double range_min, double range_max,
                           double mount_yaw_rad, const VehicleGeometry& geometry,
                           double body_margin_m, PointXY* out, std::size_t max_points);

}  // namespace racer_planner

#endif  // RACER_PLANNER_DISTANCE_FIELD_HPP_
