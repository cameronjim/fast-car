// A planner on a straight corridor with an explicit clock, shared by the timing tests: solve() and
// publish() (or tick()) at given times, a scene that can be changed under it.
#ifndef RACER_PLANNER_TEST_PLANNER_RIG_HPP_
#define RACER_PLANNER_TEST_PLANNER_RIG_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "racer_planner/freespace_field.hpp"
#include "racer_planner/planner.hpp"
#include "test_support.hpp"

namespace racer_planner::rig {

inline DynamicsParams dynamics_with_floor(double floor_mps) {
  DynamicsParams d = test::plain_dynamics();
  d.nominal.speed_deadband_mps = std::max(0.0, floor_mps - MppiParams{}.speed_floor_margin_mps);
  return d;
}

inline PlannerConfig small_config(double v_max) {
  PlannerConfig c;
  c.logic.v_max_mps = v_max;
  c.mppi.samples = 128;
  c.max_samples = 128;
  c.mppi.seed = 5;
  return c;
}

inline LocalTrackData good_track(double stamp, double y_offset = 0.0) {
  LocalTrackData t;
  t.stamp_s = stamp;
  t.frame_id = "base_link";
  t.station_spacing_m = kTrackStationSpacingM;
  t.valid_count = kTrackStations;
  for (int k = 0; k < kTrackStations; ++k) {
    t.x_m.push_back(k * kTrackStationSpacingM);
    t.y_m.push_back(y_offset);
    t.half_width_left_m.push_back(0.5);
    t.half_width_right_m.push_back(0.5);
    t.sigma_m.push_back(0.02);
  }
  return t;
}

inline std::vector<PointXY> corridor_points() {
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 5.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  for (double y = -0.5; y <= 0.5; y += 0.02) pts.push_back({5.0, y});
  return pts;
}

// A planner on a straight corridor with an explicit clock: solve() and publish() at given times.
struct Rig {
  VehicleGeometry g = test::binding_geometry();
  DistanceField field;
  FreespaceField freespace{g};
  std::vector<PointXY> points = corridor_points();
  LocalTrackData track;
  Planner planner;
  double v = 0.3;
  bool odom = true;
  bool scan = true;
  bool has_track = true;
  bool gated = true;
  double gated_steering = 0.0;

  explicit Rig(PlannerConfig cfg = small_config(0.5), double floor_mps = 0.23)
      : planner(cfg, dynamics_with_floor(floor_mps), g) {
    set_points(points);
  }

  void set_points(const std::vector<PointXY>& pts) {
    points = pts;
    field.build(points.data(), points.size());
    freespace.build(field, points.data(), points.size(), nullptr);
  }

  TickInput input(double now) {
    TickInput in;
    in.now_s = now;
    in.has_odom = odom;
    in.odom_stamp_s = now;
    in.wheel_speed_mps = v;
    in.has_scan = scan;
    in.scan_stamp_s = now - 0.1;
    in.field = &field;
    in.freespace = &freespace;
    track = good_track(now - 0.1);
    in.track = has_track ? &track : nullptr;
    in.has_gated = gated;
    in.gated_stamp_s = now;
    in.gated_speed_mps = 1.0;  // a gate that passes everything
    in.gated_steering_rad = gated_steering;
    return in;
  }
  PublishInput pub_input(double now) const {
    PublishInput p;
    p.now_s = now;
    p.has_gated = gated;
    p.gated_stamp_s = now;
    p.gated_steering_rad = gated_steering;
    return p;
  }
  TickOutput tick(double now) { return planner.tick(input(now)); }
};

// Ticks at 20 ms from t0 until the planner has left STARTING (or n ticks); returns the last output
// and advances *t.
inline TickOutput run_ticks(Rig& r, double* t, int n) {
  TickOutput out;
  for (int i = 0; i < n; ++i) {
    out = r.tick(*t);
    *t += 0.02;
  }
  return out;
}

}  // namespace racer_planner::rig

#endif  // RACER_PLANNER_TEST_PLANNER_RIG_HPP_
