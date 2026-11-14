// A small closed loop simulator for the planner (no ROS): synthetic worlds made of wall segments,
// a LiDAR ray caster, an oracle local track computed from the geometry, and a simulated car that
// is the same model with a slower speed lag and a command delay. Seeded, deterministic.
#ifndef RACER_PLANNER_SIM_WORLD_HPP_
#define RACER_PLANNER_SIM_WORLD_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/planner.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner::sim {

struct Seg {
  double x1, y1, x2, y2;
};

struct World {
  std::vector<Seg> walls;
  std::vector<std::pair<double, double>> centre;  // dense polyline in the travel direction
  bool closed = false;
  double lane_m = 1.0;
  double perimeter = 0.0;  // total arc length of the centreline (closed: including the wrap)

  void add_wall(double x1, double y1, double x2, double y2) { walls.push_back({x1, y1, x2, y2}); }

  // Arc from angle a0 to a1 (radians, either direction) around (cx, cy), as short wall segments.
  void add_arc_wall(double cx, double cy, double r, double a0, double a1) {
    const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(a1 - a0) / (M_PI / 36.0))));
    for (int i = 0; i < n; ++i) {
      const double t0 = a0 + (a1 - a0) * i / n, t1 = a0 + (a1 - a0) * (i + 1) / n;
      add_wall(cx + r * std::cos(t0), cy + r * std::sin(t0), cx + r * std::cos(t1),
               cy + r * std::sin(t1));
    }
  }

  void add_centre_line(double x1, double y1, double x2, double y2) {
    const double len = std::hypot(x2 - x1, y2 - y1);
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.02)));
    for (int i = 0; i < n; ++i) centre.push_back({x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n});
  }
  void add_centre_arc(double cx, double cy, double r, double a0, double a1) {
    const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(a1 - a0) * r / 0.02)));
    for (int i = 0; i < n; ++i) {
      const double t = a0 + (a1 - a0) * i / n;
      centre.push_back({cx + r * std::cos(t), cy + r * std::sin(t)});
    }
  }
  void finish_centre(double last_x, double last_y) {
    if (!closed) centre.push_back({last_x, last_y});
    perimeter = 0.0;
    const std::size_t n = centre.size();
    for (std::size_t i = 0; i + 1 < n; ++i) {
      perimeter += std::hypot(centre[i + 1].first - centre[i].first,
                              centre[i + 1].second - centre[i].second);
    }
    if (closed) {
      perimeter += std::hypot(centre[0].first - centre[n - 1].first,
                              centre[0].second - centre[n - 1].second);
    }
  }
  void mirror_y() {
    for (auto& s : walls) {
      s.y1 = -s.y1;
      s.y2 = -s.y2;
    }
    for (auto& c : centre) c.second = -c.second;
  }
};

inline World straight_corridor(double length = 12.0) {
  World w;
  w.add_wall(-1.0, 0.5, length, 0.5);
  w.add_wall(-1.0, -0.5, length, -0.5);
  w.add_centre_line(-1.0, 0.0, length, 0.0);
  w.finish_centre(length, 0.0);
  return w;
}

// 90 degree left bend, lane 1.0, inner radius 0.4 (centreline radius 0.9). Heads +x, then +y.
inline World left_bend() {
  World w;
  const double ri = 0.4, ro = 1.4, rc = 0.9;
  w.add_wall(-1.0, 0.5, 2.0, 0.5);
  w.add_wall(-1.0, -0.5, 2.0, -0.5);
  w.add_arc_wall(2.0, rc, ri, -M_PI / 2, 0.0);
  w.add_arc_wall(2.0, rc, ro, -M_PI / 2, 0.0);
  w.add_wall(2.0 + ri, rc, 2.0 + ri, 4.5);
  w.add_wall(2.0 + ro, rc, 2.0 + ro, 4.5);
  w.add_centre_line(-1.0, 0.0, 2.0, 0.0);
  w.add_centre_arc(2.0, rc, rc, -M_PI / 2, 0.0);
  w.add_centre_line(2.0 + rc, rc, 2.0 + rc, 4.5);
  w.finish_centre(2.0 + rc, 4.5);
  return w;
}

inline World right_bend() {
  World w = left_bend();
  w.mirror_y();
  return w;
}

// Counter clockwise oval: straights of length `len`, island end radius `ri`, lane width `lane`.
// The bottom straight runs from (0, 0) to (len, 0).
inline World oval(double lane = 1.0, double ri = 0.4, double len = 2.0) {
  World w;
  w.closed = true;
  w.lane_m = lane;
  const double ro = ri + lane, rc = ri + lane / 2.0, h = lane / 2.0;
  w.add_wall(0.0, h, len, h);
  w.add_wall(0.0, 2 * rc - h, len, 2 * rc - h);
  w.add_arc_wall(len, rc, ri, -M_PI / 2, M_PI / 2);
  w.add_arc_wall(0.0, rc, ri, M_PI / 2, 3 * M_PI / 2);
  w.add_wall(0.0, -h, len, -h);
  w.add_wall(0.0, 2 * rc + h, len, 2 * rc + h);
  w.add_arc_wall(len, rc, ro, -M_PI / 2, M_PI / 2);
  w.add_arc_wall(0.0, rc, ro, M_PI / 2, 3 * M_PI / 2);
  w.add_centre_line(0.0, 0.0, len, 0.0);
  w.add_centre_arc(len, rc, rc, -M_PI / 2, M_PI / 2);
  w.add_centre_line(len, 2 * rc, 0.0, 2 * rc);
  w.add_centre_arc(0.0, rc, rc, M_PI / 2, 3 * M_PI / 2);
  w.finish_centre(0.0, 0.0);
  return w;
}

// Counter clockwise stadium: lane 1.0, straights 2.0 m, island radius 0.4.
inline World stadium() { return oval(1.0, 0.4, 2.0); }

// Corridor with a wall across it, 2.5 m ahead of the origin. The oracle centreline runs on through
// it (the track does not know about the obstacle, only the scan does).
inline World corridor_with_wall(double wall_x = 2.5) {
  World w = straight_corridor();
  w.add_wall(wall_x, -0.5, wall_x, 0.5);
  return w;
}

// ---- geometry helpers ------------------------------------------------------------------------

inline double point_seg_dist(double px, double py, const Seg& s) {
  const double dx = s.x2 - s.x1, dy = s.y2 - s.y1;
  const double l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((px - s.x1) * dx + (py - s.y1) * dy) / l2 : 0.0;
  t = std::min(std::max(t, 0.0), 1.0);
  return std::hypot(px - (s.x1 + t * dx), py - (s.y1 + t * dy));
}

inline bool seg_intersect(const Seg& a, const Seg& b) {
  auto cross = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
  const double d1 = cross(a.x2 - a.x1, a.y2 - a.y1, b.x1 - a.x1, b.y1 - a.y1);
  const double d2 = cross(a.x2 - a.x1, a.y2 - a.y1, b.x2 - a.x1, b.y2 - a.y1);
  const double d3 = cross(b.x2 - b.x1, b.y2 - b.y1, a.x1 - b.x1, a.y1 - b.y1);
  const double d4 = cross(b.x2 - b.x1, b.y2 - b.y1, a.x2 - b.x1, a.y2 - b.y1);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

inline double seg_seg_dist(const Seg& a, const Seg& b) {
  if (seg_intersect(a, b)) return 0.0;
  return std::min(std::min(point_seg_dist(a.x1, a.y1, b), point_seg_dist(a.x2, a.y2, b)),
                  std::min(point_seg_dist(b.x1, b.y1, a), point_seg_dist(b.x2, b.y2, a)));
}

// Exact clearance between the body rectangle and the walls (0 when touching).
inline double body_clearance(const World& w, const VehicleGeometry& g, double x, double y,
                             double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double xs[2] = {g.body_min_x_m(), g.body_max_x_m()};
  const double hw = g.width_m / 2.0;
  double cx[4], cy[4];
  int k = 0;
  for (double bx : {xs[0], xs[1]}) {
    for (double by : {-hw, hw}) {
      cx[k] = x + c * bx - s * by;
      cy[k] = y + s * bx + c * by;
      ++k;
    }
  }
  // corners order: (min,-hw) (min,hw) (max,-hw) (max,hw); edges between neighbours
  const int edges[4][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}};
  double best = 1e9;
  for (const Seg& wall : w.walls) {
    for (const auto& e : edges) {
      best = std::min(best, seg_seg_dist(Seg{cx[e[0]], cy[e[0]], cx[e[1]], cy[e[1]]}, wall));
    }
  }
  return best;
}

inline double ray_cast(const World& w, double ox, double oy, double bearing, double max_range) {
  const double dx = std::cos(bearing), dy = std::sin(bearing);
  double best = max_range;
  for (const Seg& s : w.walls) {
    const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
    const double den = dx * ey - dy * ex;
    if (std::fabs(den) < 1e-12) continue;
    const double t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den;
    const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
    if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best) best = t;
  }
  return best;
}

// Oracle local track for a car pose: the 16 stations along the centreline from the point nearest
// the rear axle, in base_link, constant half width of lane / 2 on both sides. Open worlds end
// where the centreline ends (valid_count shrinks).
//
// With bias_outer_m > 0 the stations in corners are pushed that far toward the outer wall (the
// way the first tracer was wrong on the car) and the self-check numbers are those the tracer
// reported there: intrusion 0.2 and boundary residual 0.25 m. On straights the track is exact.
inline LocalTrackData oracle_track(const World& w, double x, double y, double yaw, double stamp,
                                   double bias_outer_m = 0.0) {
  LocalTrackData t;
  t.stamp_s = stamp;
  t.frame_id = "base_link";
  t.station_spacing_m = kTrackStationSpacingM;
  t.boundary_residual_m = 0.0;
  t.intrusion_fraction = 0.0;
  const std::size_t n = w.centre.size();
  std::size_t nearest = 0;
  double best = 1e18;
  for (std::size_t i = 0; i < n; ++i) {
    const double d = std::hypot(w.centre[i].first - x, w.centre[i].second - y);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  int valid = 0;
  for (int k = 0; k < kTrackStations; ++k) {
    std::size_t idx =
        nearest + static_cast<std::size_t>(std::lround(k * kTrackStationSpacingM / 0.02));
    bool ok = true;
    if (idx >= n) {
      if (w.closed) {
        idx %= n;
      } else {
        idx = n - 1;
        ok = false;
      }
    }
    double px = w.centre[idx].first, py = w.centre[idx].second;
    if (bias_outer_m > 0.0) {
      // turning rate over +/- 0.3 m of the centreline: the push is full from 0.5 rad/m
      const std::size_t span = 15;
      const std::size_t i0 = (idx + n - span) % n, i1 = (idx + span) % n;
      const std::size_t i2 = (idx + n - 1) % n, i3 = (idx + 1) % n;
      auto heading = [&](std::size_t a, std::size_t b) {
        return std::atan2(w.centre[b].second - w.centre[a].second,
                          w.centre[b].first - w.centre[a].first);
      };
      double turn = heading(idx, i1) - heading(i0, idx);
      while (turn > M_PI) turn -= 2.0 * M_PI;
      while (turn < -M_PI) turn += 2.0 * M_PI;
      const double rate = turn / 0.6;
      const double push = bias_outer_m * std::min(1.0, std::fabs(rate) / 0.5);
      const double hd = heading(i2, i3);
      const double outward = rate > 0.0 ? -1.0 : 1.0;  // away from the centre of the turn
      px += outward * push * (-std::sin(hd));
      py += outward * push * std::cos(hd);
      if (push > 0.05) {
        t.intrusion_fraction = 0.2;
        t.boundary_residual_m = 0.25;
      }
    }
    const double wx = px - x, wy = py - y;
    t.x_m.push_back(c * wx + s * wy);
    t.y_m.push_back(-s * wx + c * wy);
    t.half_width_left_m.push_back(w.lane_m / 2.0);
    t.half_width_right_m.push_back(w.lane_m / 2.0);
    t.sigma_m.push_back(0.02);
    if (ok) valid = k + 1;
  }
  t.valid_count = valid;
  return t;
}

// ---- the closed loop -------------------------------------------------------------------------

struct SimResult {
  double min_body_clearance_m = 1e9;
  double mean_speed_mps = 0.0;
  double final_speed_mps = 0.0;
  double final_x = 0.0, final_y = 0.0;
  double distance_m = 0.0;
  double sim_time_s = 0.0;
  bool finished = false;
  PlannerState final_state = PlannerState::Starting;
  int ticks_driving = 0, ticks_no_track = 0, ticks_blocked = 0, ticks_no_scan = 0, ticks_other = 0;
  int ticks_no_track_conf = 0;  // driving with track confidence 0
  int max_consecutive_blocked = 0;
  int ticks_recovering = 0;
  double max_solve_s = 0.0;
};

enum class TrackMode {
  Oracle,  // the geometry's centreline
  None,    // no track message at all
  Biased,  // pushed bias_outer_m toward the outer wall in corners, self-check as the tracer's
};

struct SimSetup {
  const World* world = nullptr;
  TrackMode track_mode = TrackMode::Oracle;
  double track_bias_outer_m = 0.25;
  // Plant features the planner's model does not have.
  double plant_dead_zone_mps = 0.0;      // a speed command below this gives a zero target speed
  double plant_extra_steer_tau_s = 0.0;  // extra first-order steering lag
  VehicleGeometry geometry;
  DynamicsParams planner_dynamics;
  PlannerConfig config;
  double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;
  double max_time_s = 60.0;
  double plant_speed_lag_factor = 1.1;  // the simulated car is 10 percent slower to respond
  double scan_noise_m = 0.02;
  double scan_dropout = 0.03;
  double sensor_latency_s = 0.1;
  std::uint64_t seed = 1;
  // Finished predicate on (x, y, unwrapped centreline progress from the start).
  std::function<bool(double, double, double)> done;
};

inline SimResult run_closed_loop(const SimSetup& setup) {
  const World& world = *setup.world;
  const VehicleGeometry& g = setup.geometry;
  const double tick = setup.config.tick_s;
  Planner planner(setup.config, setup.planner_dynamics, g);

  DynamicsParams plant_params = setup.planner_dynamics;
  plant_params.nominal.speed_tau_s *= setup.plant_speed_lag_factor;
  plant_params.nominal.steer_tau_s += setup.plant_extra_steer_tau_s;
  plant_params.nominal.speed_deadband_mps =
      std::max(plant_params.nominal.speed_deadband_mps, setup.plant_dead_zone_mps);
  const DynamicsModel plant(plant_params, g, tick);
  const int delay_ticks =
      static_cast<int>(std::lround(plant_params.nominal.command_delay_s / tick));
  std::deque<Control> fifo(static_cast<std::size_t>(delay_ticks), Control{});

  std::mt19937_64 rng(setup.seed);
  std::normal_distribution<double> noise(0.0, setup.scan_noise_m);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  constexpr int kBeams = 720;
  const double inc = 2.0 * M_PI / kBeams;
  const double angle_min = -M_PI;
  const double range_min = 0.05, range_max = 12.0;
  std::vector<float> ranges(kBeams);
  std::vector<PointXY> points(kBeams);

  struct Pending {
    double deliver_s;
    double stamp_s;
    std::vector<PointXY> pts;
    LocalTrackData track;
  };
  std::deque<Pending> pending;
  DistanceField field;
  FreespaceField freespace(g);
  bool have_prev_goal = false;
  PointXY prev_goal{0.0, 0.0};
  LocalTrackData track;
  bool has_scan = false, has_track = false;
  double scan_stamp = 0.0;

  State car;
  car.x = setup.start_x;
  car.y = setup.start_y;
  car.yaw = setup.start_yaw;

  SimResult run;
  double last_cmd_speed = 0.0;
  double last_cmd_steering = 0.0;
  int blocked_run = 0;
  double speed_sum = 0.0;
  int speed_n = 0;
  // unwrapped centreline progress from the start
  const std::size_t n_centre = world.centre.size();
  auto nearest_index = [&](double x, double y) {
    std::size_t best_i = 0;
    double best = 1e18;
    for (std::size_t i = 0; i < n_centre; ++i) {
      const double d = std::hypot(world.centre[i].first - x, world.centre[i].second - y);
      if (d < best) {
        best = d;
        best_i = i;
      }
    }
    return best_i;
  };
  const double s_start = 0.02 * static_cast<double>(nearest_index(car.x, car.y));
  double progress = 0.0, last_s = s_start;

  const int total_ticks = static_cast<int>(setup.max_time_s / tick);
  const int ticks_per_scan = static_cast<int>(std::lround(0.1 / tick));
  for (int n = 0; n < total_ticks; ++n) {
    const double t = n * tick;
    if (n % ticks_per_scan == 0) {
      Pending p;
      p.stamp_s = t;
      p.deliver_s = t + setup.sensor_latency_s;
      const double lx =
          car.x + std::cos(car.yaw) * g.lidar_mount_x_m - std::sin(car.yaw) * g.lidar_mount_y_m;
      const double ly =
          car.y + std::sin(car.yaw) * g.lidar_mount_x_m + std::cos(car.yaw) * g.lidar_mount_y_m;
      for (int i = 0; i < kBeams; ++i) {
        const double b = angle_min + i * inc;
        const double bearing = car.yaw + b + g.lidar_mount_yaw_rad;
        double r = ray_cast(world, lx, ly, bearing, range_max + 1.0);
        if (r >= range_max || u01(rng) < setup.scan_dropout) {
          ranges[i] = std::numeric_limits<float>::infinity();
        } else {
          ranges[i] = static_cast<float>(std::max(range_min, r + noise(rng)));
        }
      }
      const std::size_t m =
          scan_to_points(ranges.data(), kBeams, angle_min, inc, range_min, range_max,
                         g.lidar_mount_yaw_rad, g, 0.02, points.data(), points.size());
      p.pts.assign(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(m));
      p.track =
          oracle_track(world, car.x, car.y, car.yaw, t,
                       setup.track_mode == TrackMode::Biased ? setup.track_bias_outer_m : 0.0);
      pending.push_back(std::move(p));
    }
    while (!pending.empty() && pending.front().deliver_s <= t + 1e-9) {
      field.build(pending.front().pts.data(), pending.front().pts.size());
      freespace.build(field, pending.front().pts.data(), pending.front().pts.size(),
                      have_prev_goal ? &prev_goal : nullptr);
      have_prev_goal = freespace.valid();
      if (have_prev_goal) prev_goal = freespace.goal();
      scan_stamp = pending.front().stamp_s;
      track = pending.front().track;
      has_scan = true;
      has_track = setup.track_mode != TrackMode::None;
      pending.pop_front();
    }

    TickInput in;
    in.now_s = t;
    in.has_odom = true;
    in.odom_stamp_s = t;
    in.wheel_speed_mps = car.v;
    in.has_scan = has_scan;
    in.scan_stamp_s = scan_stamp;
    in.has_gated = true;  // the sim has no safety node: the gated /drive passes what was published
    in.gated_stamp_s = t;
    in.gated_speed_mps = last_cmd_speed;
    in.gated_steering_rad = last_cmd_steering;
    in.field = has_scan ? &field : nullptr;
    in.freespace = has_scan ? &freespace : nullptr;
    in.track = has_track ? &track : nullptr;
    const TickOutput out = planner.tick(in);
    last_cmd_speed = out.cmd.speed_mps;
    last_cmd_steering = out.cmd.steering_rad;
    if (out.state == PlannerState::Recovering) ++run.ticks_recovering;
    run.max_solve_s = std::max(run.max_solve_s, out.solve_time_s);
    run.final_state = out.state;
    switch (out.state) {
      case PlannerState::Driving:
        ++run.ticks_driving;
        if (out.track_confidence == 0.0) ++run.ticks_no_track_conf;
        break;
      case PlannerState::NoTrack:
        ++run.ticks_no_track;
        break;
      case PlannerState::Blocked:
        ++run.ticks_blocked;
        break;
      case PlannerState::NoScan:
        ++run.ticks_no_scan;
        break;
      default:
        ++run.ticks_other;
        break;
    }

    if (out.state == PlannerState::Blocked) {
      ++blocked_run;
      run.max_consecutive_blocked = std::max(run.max_consecutive_blocked, blocked_run);
    } else {
      blocked_run = 0;
    }
    fifo.push_back(Control{out.cmd.speed_mps, out.cmd.steering_rad});
    Control acting = delay_ticks > 0 ? fifo.front() : fifo.back();
    if (delay_ticks > 0) fifo.pop_front();
    const State prev = car;
    if (car.v < 0.0 || acting.speed_cmd < 0.0) {
      // Reverse: the model has no reverse, so a first order speed toward the command (the same
      // dead zone and lag), steering from the model, rear axle kinematics.
      const double dz = plant_params.nominal.speed_deadband_mps;
      const double target = std::fabs(acting.speed_cmd) < dz
                                ? 0.0
                                : plant_params.nominal.speed_gain * acting.speed_cmd;
      const double a_v = 1.0 - std::exp(-tick / plant_params.nominal.speed_tau_s);
      State steer_only = car;
      steer_only.v = 0.0;
      steer_only = plant.step(steer_only, Control{0.0, acting.steering_cmd});
      const double dv =
          std::min(std::max(a_v * (target - car.v), -plant_params.nominal.decel_max_mps2 * tick),
                   plant_params.nominal.accel_max_mps2 * tick);
      const double v = std::min(car.v + dv, 0.0);
      const double yaw_rate = v * std::tan(car.delta) / g.wheelbase_m;
      car.x += v * std::cos(car.yaw) * tick;
      car.y += v * std::sin(car.yaw) * tick;
      car.yaw += yaw_rate * tick;
      car.v = v;
      car.delta = steer_only.delta;
    } else {
      car = plant.step(car, acting);
    }
    run.distance_m += std::hypot(car.x - prev.x, car.y - prev.y);

    run.min_body_clearance_m =
        std::min(run.min_body_clearance_m, body_clearance(world, g, car.x, car.y, car.yaw));
    speed_sum += car.v;
    ++speed_n;
    run.sim_time_s = t + tick;

    // centreline progress (unwrapped for closed worlds)
    const double s_now = 0.02 * static_cast<double>(nearest_index(car.x, car.y));
    double ds = s_now - last_s;
    if (world.closed) {
      if (ds < -0.5 * world.perimeter) ds += world.perimeter;
      if (ds > 0.5 * world.perimeter) ds -= world.perimeter;
    }
    progress += ds;
    last_s = s_now;
    if (setup.done && setup.done(car.x, car.y, progress)) {
      run.finished = true;
      break;
    }
  }
  run.mean_speed_mps = speed_n > 0 ? speed_sum / speed_n : 0.0;
  run.final_speed_mps = car.v;
  run.final_x = car.x;
  run.final_y = car.y;
  return run;
}

}  // namespace racer_planner::sim

#endif  // RACER_PLANNER_SIM_WORLD_HPP_
