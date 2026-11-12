// Free-space progress field: where the scan alone says the car can go, as a potential Phi over
// the DistanceField grid (0.05 m, x from -1.0 to 4.0, y from -2.5 to 2.5) in the scan's base_link
// frame. Built once per scan, ROS-free, no allocation after construction.
//
//   * visible region: a 360 bin (1 degree) minimum range image about the LiDAR head. A run of up to
//     5 empty bins takes the smaller of its two neighbouring ranges, a longer run is range 0
//     (unseen). A cell is visible when its range from the head is below the image range of its bin.
//   * traversable cell: distance field value at least nav_radius_m, visible, not behind the rear
//     bumper line.
//   * cost per metre in a traversable cell: 1 + w * clip((pref - d) / pref, 0, 1)^2; an edge
//     between 8-connected neighbours costs its length times the mean of the two cell costs.
//   * seed: the traversable cell nearest the front axle point (wheelbase, 0), within seed_max_m.
//   * goal: the reached cell with the largest cost-to-reach g, kept near the previous scan's goal
//     (hysteresis) while it is at least hysteresis_keep of the best. Goal g under min_goal_cost_m:
//     no field.
//   * Phi: cost-to-go to the goal over the traversable cells; every other visible cell takes the
//     value of its nearest traversable cell plus outside_penalty_per_m per metre; every remaining
//     cell max finite Phi + 1. Bilinear lookup, finite everywhere.
#ifndef RACER_PLANNER_FREESPACE_FIELD_HPP_
#define RACER_PLANNER_FREESPACE_FIELD_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct FreespaceParams {
  double nav_radius_m = 0.20;
  double nav_centre_weight = 2.0;
  double nav_pref_m = 0.45;
  double outside_penalty_per_m = 3.0;
  double seed_max_m = 0.5;
  double hysteresis_radius_m = 0.5;
  double hysteresis_keep = 0.8;
  double min_goal_cost_m = 0.5;
  int max_gap_bins = 5;
};

class FreespaceField {
 public:
  static constexpr int kBins = 360;

  explicit FreespaceField(const VehicleGeometry& geometry,
                          const FreespaceParams& params = FreespaceParams{});

  // Rebuilds from the distance field and the scan points (base_link) it was built from.
  // `prev_goal` is the previous scan's goal in the same frame (null for none). Returns valid().
  bool build(const DistanceField& field, const PointXY* points, std::size_t count,
             const PointXY* prev_goal);

  bool valid() const { return valid_; }
  // Cost-to-go (metres of weighted path) from (x, y), bilinear. Finite everywhere; the far value
  // outside the grid. Meaningless (0) when not valid().
  double phi(double x, double y) const {
    if (!valid_) return 0.0;
    return phi_lookup(x, y);
  }
  PointXY goal() const { return goal_; }
  // Cost-to-reach of the goal from the seed, metres.
  double reach_m() const { return reach_m_; }
  double far_value() const { return far_; }

  // Test and diagnostics access.
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  bool traversable(int i, int j) const { return trav_[idx(i, j)] != 0; }
  bool visible(int i, int j) const { return vis_[idx(i, j)] != 0; }
  bool reached(int i, int j) const { return reached_[idx(i, j)] != 0; }
  double phi_node(int i, int j) const { return phi_[idx(i, j)]; }
  double g_node(int i, int j) const { return g_[idx(i, j)]; }

 private:
  std::size_t idx(int i, int j) const {
    return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_) +
           static_cast<std::size_t>(i);
  }
  double phi_lookup(double x, double y) const;
  void build_visibility(const PointXY* points, std::size_t count);
  // Dijkstra over the traversable cells from one seed with the per metre cell costs: out gets the
  // cost-to-reach (infinity where not reached).
  void dijkstra_from(std::size_t seed, float* out);
  // Values of the visible cells that are not reached: nearest reached cell's Phi plus the
  // outside penalty per metre of path length.
  void fill_outside();
  // Indexed binary heap over cell numbers keyed by an external array.
  void heap_clear();
  bool heap_empty() const { return heap_size_ == 0; }
  void heap_push(std::size_t cell, const float* key);  // inserts or decreases the key
  std::size_t heap_pop(const float* key);
  void sift_up(int i, const float* key);
  void sift_down(int i, const float* key);

  FreespaceParams p_;
  int nx_;
  int ny_;
  std::size_t cells_;
  double head_x_, head_y_;
  double rear_x_;
  double front_axle_x_;

  // geometry of the cells seen from the head, fixed
  std::vector<std::uint16_t> cell_bin_;
  std::vector<float> cell_range_;

  // per scan
  float vis_range_[kBins];
  std::vector<unsigned char> vis_, trav_, reached_;
  std::vector<float> cost_;   // per metre, traversable cells only
  std::vector<float> g_;      // cost to reach from the seed
  std::vector<float> phi_;    // result
  std::vector<float> key_;    // working key of the fill
  std::vector<float> carry_;  // working: Phi of the source a fill value came from
  std::vector<int> heap_, pos_;
  int heap_size_ = 0;
  bool valid_ = false;
  PointXY goal_{0.0, 0.0};
  double reach_m_ = 0.0;
  double far_ = 0.0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_FREESPACE_FIELD_HPP_
