// Dead reckoning of the car's own motion over the last fraction of a second, from the wheel
// speed and the modelled steering angle, so a scan or track that is 0.1 to 0.3 s old is used at
// the right place. There is no pose in this system: this is only the short term bicycle
// integral between two timestamps.
#ifndef RACER_PLANNER_EGO_MOTION_HPP_
#define RACER_PLANNER_EGO_MOTION_HPP_

#include <array>
#include <cstddef>

#include "racer_planner/local_track.hpp"  // Pose2

namespace racer_planner {

class EgoMotion {
 public:
  // yaw_rate = v * tan(delta) / wheelbase / (1 + understeer * v^2), as in the model step.
  EgoMotion(double wheelbase_m, double understeer_s2_per_m2);

  // Samples are held (zero order) until the next one. Non finite values and times that do not
  // advance are ignored.
  void push(double t_s, double v_mps, double delta_rad);
  void clear();
  std::size_t size() const { return count_; }

  // Pose of the car at t_to expressed in the car frame at t_from. For t_to < t_from this is the
  // inverse of the forward motion. `covered` (optional) is false when t_from is older than the
  // oldest sample (the first sample is extrapolated backwards) or the buffer is empty.
  Pose2 relative_pose(double t_from_s, double t_to_s, bool* covered = nullptr) const;

 private:
  struct Sample {
    double t = 0.0;
    double v = 0.0;
    double delta = 0.0;
  };
  static constexpr std::size_t kCapacity = 256;
  const Sample& at(std::size_t i) const {  // i = 0 is the oldest
    return buf_[(head_ + i) % kCapacity];
  }
  Pose2 forward(double t_from_s, double t_to_s, bool* covered) const;
  void advance(Pose2* pose, const Sample& s, double dt) const;

  double wheelbase_;
  double understeer_;
  std::array<Sample, kCapacity> buf_{};
  std::size_t head_ = 0;  // index of the oldest sample
  std::size_t count_ = 0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_EGO_MOTION_HPP_
