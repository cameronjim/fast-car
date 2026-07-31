// Raceline: a loaded, closed-loop reference path (roadmap task S.2).
//
// ROS-free (no rclcpp), so it is unit-testable with gtest (claude-docs/12-testing.md L1)
// with no ROS install. Loading happens once at node init (`tracker_node` calls
// `Raceline::load_from_csv` in its constructor, before the 50 Hz timer starts) --
// claude-docs/10-conventions.md's "no heap allocation in the 50 Hz control path after
// init" applies to `nearest_index`/`advance_to_lookahead` (read-only index arithmetic over
// an already-sized std::vector), not to loading itself.
//
// File format: the CSV produced by tools/raceline (tools/raceline/io.py) --
// `#`-commented provenance header, then a header row, then
// `s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps` rows. This class parses that
// same format independently in C++ (deliberately not sharing code with the Python writer --
// each deployment target parses the shared interchange file on its own, the normal pattern
// for this kind of file).
#ifndef RACER_CONTROL_RACELINE_HPP_
#define RACER_CONTROL_RACELINE_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace racer_control {

struct RacelinePoint {
  double s_m;
  double x_m;
  double y_m;
  double heading_rad;
  double curvature_1pm;
  double target_speed_mps;
};

class RacelineLoadError : public std::runtime_error {
 public:
  explicit RacelineLoadError(const std::string& what) : std::runtime_error(what) {}
};

class Raceline {
 public:
  // Direct construction from already-parsed points (used by tests that don't want to
  // round-trip through a CSV file).
  explicit Raceline(std::vector<RacelinePoint> points);

  // Parses a tools/raceline-format CSV. Throws RacelineLoadError on any I/O or format
  // problem (missing file, wrong column header, non-numeric field, empty body) -- there is
  // no silent fallback to an empty/degenerate raceline.
  static Raceline load_from_csv(const std::string& path);

  std::size_t size() const { return points_.size(); }
  const RacelinePoint& at(std::size_t index) const { return points_.at(index); }

  // Index of the closest raceline point to (x, y). O(n) linear scan; n is a few hundred
  // points and this runs once per 50 Hz control cycle, which is cheap in absolute terms
  // and allocates no memory.
  std::size_t nearest_index(double x, double y) const;

  // Starting from `from_index`, walks forward (with wraparound, since the raceline is a
  // closed loop) until the raceline point is at least `lookahead_m` from (x, y), and
  // returns that point's index. If the whole loop is shorter than lookahead_m, returns the
  // point farthest from (x, y).
  std::size_t advance_to_lookahead(std::size_t from_index, double x, double y,
                                   double lookahead_m) const;

 private:
  std::vector<RacelinePoint> points_;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_RACELINE_HPP_
