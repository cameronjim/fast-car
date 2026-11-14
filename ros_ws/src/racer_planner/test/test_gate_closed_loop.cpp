// Closed loop on a synthetic oval with a faithful in-test copy of the safety gate (racer_safety's
// own compiled sources, read only, see CMakeLists.txt):
//
//  * the gate aware cost: at 1.2 and 1.5 m/s caps the planner with w_gate 0 makes the gate brake;
//    with the default it engages zero times over 10 laps, and the lap time is within 1.10 times the
//    w_gate 0 lap time measured with the gate's brakes disabled in the test world;
//  * the lap profile: a hand made profile (bound 0.6 on the ends, 1.2 on the straights, cap 1.2)
//    keeps the driven speed under bound + 0.05 m/s at every station, and the lap is faster than
//    with a flat cap of 0.6.
//
// The world is sim_world.hpp's (ray cast scans at 10 Hz with noise, dropouts and 0.1 s latency, an
// oracle local track, the planner's model with a slower speed lag, a speed dead zone and extra
// steering lag as the plant). The loop below is run_closed_loop with the gate table, the gate and
// the profile added. Seeded and deterministic.
#include <gtest/gtest.h>

#include <deque>
#include <functional>
#include <iostream>
#include <optional>

#include "gate_safety_rig.hpp"
#include "sim_world.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::sim;
using namespace racer_planner::test;

namespace {

constexpr double kDeadZone = 0.20;
constexpr double kFloorMargin = 0.03;
// Clearance margins of the planner just under the gate's own geometry (shipped: 0.15 m and
// 0.25 s). With the shipped ones the planner never makes these ovals engage the gate even with the
// term off (second test below); with these it does, at both caps.
constexpr double kTightSoftBaseM = 0.12;
constexpr double kTightHeadwayS = 0.15;

struct LoopSetup {
  const World* world = nullptr;
  double v_cap_mps = 1.2;
  double w_gate = 300.0;
  bool brakes_enabled = true;  // the plant takes the gate's output (true) or the planner's command
  bool use_gate_table = true;  // the planner is told the gate's view of the scan
  double laps = 10.0;
  double max_time_s = 400.0;
  std::uint64_t seed = 1;
  // Lap profile: the speed bound at a station (unwrapped stations are taken modulo the lap) and
  // the time to finish from it, both null for no profile.
  std::function<double(double)> bound_of_station;
  std::function<double(double)> q_of_station;
  int samples = 512;
  // The oracle track is pushed this far toward the outer wall in corners (the way the first
  // tracer was wrong on the car: a track the planner follows into the gate's corridor).
  double track_bias_outer_m = 0.0;
  // The planner's own clearance margins, below zero for the defaults.
  double d_soft_base_m = -1.0;
  double t_headway_s = -1.0;
  double gate_ttc_margin = -1.0;
  double gate_clear_margin = -1.0;
  double w_trust = -1.0;  // below zero: the default
};

struct LoopResult {
  bool finished = false;
  double sim_time_s = 0.0;
  int engagements = 0;  // obstacle gate latch, false to true transitions
  int latched_ticks = 0;
  int would_trip_ticks = 0;  // ticks the gate decision (applied or not) was a brake
  double min_clearance_m = 1e9;
  double mean_speed_mps = 0.0;
  double max_speed_mps = 0.0;
  double max_over_bound_mps = 0.0;  // worst (driven speed - bound at the car's station)
  double worst_station_m = 0.0;     // where, and the time
  double worst_time_s = 0.0;
  double max_solve_s = 0.0;
  int ticks_blocked = 0;
};

LoopResult run_loop(const LoopSetup& su) {
  const World& world = *su.world;
  const VehicleGeometry g = test::binding_geometry();
  const GateConfig gc = binding_gate();
  const SafetyRig rig;
  const racer_safety::SafetyGateLogic safety(safety_limits(true));

  PlannerConfig cfg;
  cfg.logic.v_max_mps = su.v_cap_mps;
  cfg.tick_s = 0.02;
  cfg.mppi.seed = su.seed;
  cfg.max_samples = su.samples;
  cfg.mppi.samples = su.samples;
  cfg.mppi.w_gate = su.w_gate;
  cfg.mppi.speed_floor_margin_mps = kFloorMargin;
  if (su.d_soft_base_m >= 0.0) cfg.mppi.d_soft_base_m = su.d_soft_base_m;
  if (su.t_headway_s >= 0.0) cfg.mppi.t_headway_s = su.t_headway_s;
  if (su.gate_ttc_margin >= 0.0) cfg.mppi.gate_ttc_margin = su.gate_ttc_margin;
  if (su.gate_clear_margin >= 0.0) cfg.mppi.gate_clear_margin = su.gate_clear_margin;
  if (su.w_trust >= 0.0) cfg.mppi.w_trust = su.w_trust;
  DynamicsParams dyn = test::plain_dynamics();
  dyn.v_max_mps = su.v_cap_mps;
  dyn.nominal.speed_deadband_mps = kDeadZone;
  const double tick = cfg.tick_s;
  Planner planner(cfg, dyn, g);

  DynamicsParams plant_params = dyn;
  plant_params.nominal.speed_tau_s *= 1.1;
  plant_params.nominal.steer_tau_s += 0.10;
  const DynamicsModel plant(plant_params, g, tick);
  const int delay_ticks =
      static_cast<int>(std::lround(plant_params.nominal.command_delay_s / tick));
  std::deque<Control> fifo(static_cast<std::size_t>(delay_ticks), Control{});

  std::mt19937_64 rng(su.seed);
  std::normal_distribution<double> noise(0.0, 0.02);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  constexpr int kBeams = 720;
  const double inc = 2.0 * M_PI / kBeams;
  const double angle_min = -M_PI;
  const double range_min = 0.05, range_max = 12.0;
  std::vector<float> ranges(kBeams);
  std::vector<PointXY> points(kBeams);

  struct Pending {
    double deliver_s, stamp_s;
    std::vector<PointXY> pts;
    std::vector<float> ranges;
    LocalTrackData track;
  };
  std::deque<Pending> pending;
  DistanceField field;
  FreespaceField freespace(g);
  GateTable table(gc.geometry, gc.limits);
  bool have_prev_goal = false;
  PointXY prev_goal{0.0, 0.0};
  LocalTrackData track;
  bool has_scan = false;
  double scan_stamp = 0.0;
  ScanData scan_data = make_scan(kBeams);
  scan_data.range_min_m = range_min;
  scan_data.range_max_m = range_max;
  bool have_gate_scan = false;

  State car;
  car.x = 1.0;
  LoopResult run;
  Control last_applied{};

  // gate state
  racer_safety::DriveCommand prev_out{};
  bool latched = false, rev_latched = false;
  std::optional<double> hold_timer;
  double bound_fwd = 0.0, bound_rev = 0.0;
  Command last_cmd{};

  // lap profile
  LapProfile profile;
  double next_profile_s = 0.0;
  const double lap_m = world.perimeter;

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
  double station = s_start;  // 0 .. lap_m
  double speed_sum = 0.0;
  int speed_n = 0;

  const int total_ticks = static_cast<int>(su.max_time_s / tick);
  const int ticks_per_scan = static_cast<int>(std::lround(0.1 / tick));
  for (int n = 0; n < total_ticks; ++n) {
    const double t = n * tick;
    if (n % ticks_per_scan == 0) {
      Pending p;
      p.stamp_s = t;
      p.deliver_s = t + 0.1;
      const double lx =
          car.x + std::cos(car.yaw) * g.lidar_mount_x_m - std::sin(car.yaw) * g.lidar_mount_y_m;
      const double ly =
          car.y + std::sin(car.yaw) * g.lidar_mount_x_m + std::cos(car.yaw) * g.lidar_mount_y_m;
      for (int i = 0; i < kBeams; ++i) {
        const double b = angle_min + i * inc;
        const double bearing = car.yaw + b + g.lidar_mount_yaw_rad;
        const double r = ray_cast(world, lx, ly, bearing, range_max + 1.0);
        if (r >= range_max || u01(rng) < 0.03) {
          ranges[i] = std::numeric_limits<float>::infinity();
        } else {
          ranges[i] = static_cast<float>(std::max(range_min, r + noise(rng)));
        }
      }
      const std::size_t m =
          scan_to_points(ranges.data(), kBeams, angle_min, inc, range_min, range_max,
                         g.lidar_mount_yaw_rad, g, 0.02, points.data(), points.size());
      p.pts.assign(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(m));
      p.ranges = ranges;
      p.track = oracle_track(world, car.x, car.y, car.yaw, t, su.track_bias_outer_m);
      pending.push_back(std::move(p));
    }
    while (!pending.empty() && pending.front().deliver_s <= t + 1e-9) {
      Pending& f = pending.front();
      field.build(f.pts.data(), f.pts.size());
      freespace.build(field, f.pts.data(), f.pts.size(), have_prev_goal ? &prev_goal : nullptr);
      have_prev_goal = freespace.valid();
      if (have_prev_goal) prev_goal = freespace.goal();
      scan_stamp = f.stamp_s;
      track = f.track;
      scan_data.ranges = f.ranges;
      table.build(scan_data.view());
      have_gate_scan = true;
      has_scan = true;
      pending.pop_front();
    }

    // The lap profile the producer would send: every 0.1 s, the car's station at that time.
    if (su.bound_of_station && t >= next_profile_s) {
      next_profile_s = t + 0.1;
      std::vector<float> s, v, q;
      for (int i = 0; i < 61; ++i) {
        const double st = station + 0.1 * i;
        s.push_back(static_cast<float>(st));
        v.push_back(static_cast<float>(su.bound_of_station(st)));
        q.push_back(static_cast<float>(su.q_of_station(st)));
      }
      make_lap_profile(s.data(), v.data(), q.data(), s.size(), v.size(), q.size(), t, t, &profile);
    }

    TickInput in;
    in.now_s = t;
    in.has_odom = true;
    in.odom_stamp_s = t;
    in.wheel_speed_mps = car.v;
    in.has_scan = has_scan;
    in.scan_stamp_s = scan_stamp;
    in.has_gated = true;
    in.gated_stamp_s = t;
    in.gated_speed_mps = last_applied.speed_cmd;
    in.field = has_scan ? &field : nullptr;
    in.freespace = has_scan ? &freespace : nullptr;
    in.track = has_scan ? &track : nullptr;
    in.gate = su.use_gate_table && have_gate_scan ? &table : nullptr;
    in.lap = su.bound_of_station ? &profile : nullptr;
    const TickOutput out = planner.tick(in);
    run.max_solve_s = std::max(run.max_solve_s, out.solve_time_s);
    if (out.state == PlannerState::Blocked) ++run.ticks_blocked;
    last_cmd = out.cmd;

    // The safety gate, from the scan it would hold now and the wheel speed, every cycle.
    racer_safety::GateInput gi;
    gi.command.speed_mps = out.cmd.speed_mps;
    gi.command.steering_angle_rad = out.cmd.steering_rad;
    gi.drive_raw_age_s = 0.0;
    gi.dt_s = tick;
    gi.min_scan_range_m = have_gate_scan ? rig.distance(scan_data, out.cmd.steering_rad)
                                         : std::numeric_limits<double>::infinity();
    gi.ttc_brake_latched = latched;
    gi.reverse_brake_latched = rev_latched;
    gi.obstacle_hold_timer_s = hold_timer;
    gi.speed_bound_forward_mps = bound_fwd;
    gi.speed_bound_reverse_mps = bound_rev;
    gi.measured_speed_mps = car.v;
    const racer_safety::GateResult gr = safety.evaluate(gi, prev_out);
    if (gr.ttc_brake_latched && !latched) ++run.engagements;
    if (gr.ttc_brake_latched) ++run.latched_ticks;
    latched = gr.ttc_brake_latched;
    rev_latched = gr.reverse_brake_latched;
    hold_timer = gr.obstacle_hold_timer_s;
    bound_fwd = gr.speed_bound_forward_mps;
    bound_rev = gr.speed_bound_reverse_mps;
    prev_out = gr.output;
    if (gr.zero_throttle) ++run.would_trip_ticks;

    const Control applied = su.brakes_enabled
                                ? Control{gr.output.speed_mps, gr.output.steering_angle_rad}
                                : Control{out.cmd.speed_mps, out.cmd.steering_rad};
    last_applied = applied;
    fifo.push_back(applied);
    Control acting = delay_ticks > 0 ? fifo.front() : fifo.back();
    if (delay_ticks > 0) fifo.pop_front();
    const State prev = car;
    if (car.v < 0.0 || acting.speed_cmd < 0.0) {
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
      car.x += v * std::cos(car.yaw) * tick;
      car.y += v * std::sin(car.yaw) * tick;
      car.yaw += v * std::tan(car.delta) / g.wheelbase_m * tick;
      car.v = v;
      car.delta = steer_only.delta;
    } else {
      car = plant.step(car, acting);
    }
    (void)prev;
    run.min_clearance_m =
        std::min(run.min_clearance_m, body_clearance(world, g, car.x, car.y, car.yaw));
    speed_sum += car.v;
    ++speed_n;
    run.max_speed_mps = std::max(run.max_speed_mps, car.v);
    run.sim_time_s = t + tick;

    const double s_now = 0.02 * static_cast<double>(nearest_index(car.x, car.y));
    double ds = s_now - last_s;
    if (ds < -0.5 * lap_m) ds += lap_m;
    if (ds > 0.5 * lap_m) ds -= lap_m;
    progress += ds;
    last_s = s_now;
    station = s_now;
    if (su.bound_of_station && t > 3.0) {
      const double over = car.v - su.bound_of_station(station);
      if (over > run.max_over_bound_mps) {
        run.max_over_bound_mps = over;
        run.worst_station_m = station;
        run.worst_time_s = t;
      }
    }
    if (progress >= su.laps * lap_m) {
      run.finished = true;
      break;
    }
  }
  run.mean_speed_mps = speed_n > 0 ? speed_sum / speed_n : 0.0;
  return run;
}

void report(const char* name, const LoopResult& r) {
  std::cout << "[gate closed loop] " << name << ": finished " << r.finished << ", time "
            << r.sim_time_s << " s, engagements " << r.engagements << " (latched ticks "
            << r.latched_ticks << "), mean speed " << r.mean_speed_mps << " m/s, max speed "
            << r.max_speed_mps << ", min clearance " << r.min_clearance_m << " m, max solve "
            << r.max_solve_s * 1e3 << " ms, blocked ticks " << r.ticks_blocked;
  if (r.max_over_bound_mps != 0.0) {
    std::cout << ", worst over the bound " << r.max_over_bound_mps << " at station "
              << r.worst_station_m << " t " << r.worst_time_s;
  }
  std::cout << "\n";
}

World make_oval(int dir) {
  World w = oval(1.0, 0.4, 2.0);
  if (dir == 1) w.mirror_y();
  return w;
}

}  // namespace

// The gate aware cost, at both caps and in both directions. The planner's own clearance margins
// (d_soft_base_m, t_headway_s) are set below the gate's in this test: with the shipped margins the
// planner keeps far enough from the walls of these ovals that the gate never engages even with the
// term off (see the next test), so this is the configuration in which the term has work to do.
TEST(GateClosedLoop, DefaultWeightNeverEngagesTheGateAndCostsNoMoreThanTenPercentOfLapTime) {
  for (double cap : {1.2, 1.5}) {
    for (int dir = 0; dir < 2; ++dir) {
      const World w = make_oval(dir);
      const std::string label =
          "cap " + std::to_string(cap).substr(0, 3) + (dir == 0 ? " ccw" : " cw");
      // Without the gate term and with the gate's brakes disabled in the world: the reference lap
      // time, and how often the gate would have braked.
      LoopSetup off;
      off.world = &w;
      off.v_cap_mps = cap;
      off.w_gate = 0.0;
      off.brakes_enabled = false;
      off.d_soft_base_m = kTightSoftBaseM;
      off.t_headway_s = kTightHeadwayS;
      const LoopResult r0 = run_loop(off);
      report((label + ", w_gate 0, gate brakes disabled").c_str(), r0);
      // With the default weight and the gate's brakes applied.
      LoopSetup on = off;
      on.w_gate = MppiParams{}.w_gate;
      on.brakes_enabled = true;
      const LoopResult r1 = run_loop(on);
      report((label + ", default w_gate, gate brakes applied").c_str(), r1);
      EXPECT_TRUE(r0.finished) << label;
      EXPECT_TRUE(r1.finished) << label;
      EXPECT_GE(r0.engagements, 1) << label << ": without the term the gate engages";
      EXPECT_EQ(r1.engagements, 0) << label << ": with the default weight the gate never engages";
      EXPECT_LE(r1.sim_time_s, 1.10 * r0.sim_time_s) << label << ": lap time cost over 10 percent";
      EXPECT_GE(r1.min_clearance_m, 0.02) << label;
    }
  }
}

// With the shipped clearance margins the planner already keeps clear of the gate in these ovals,
// and the term must not slow it down there.
TEST(GateClosedLoop, TheTermCostsNothingWhereThePlannerAlreadyKeepsClear) {
  for (double cap : {1.2, 1.5}) {
    const World w = make_oval(0);
    const std::string label = "shipped margins, cap " + std::to_string(cap).substr(0, 3) + " ccw";
    LoopSetup off;
    off.world = &w;
    off.v_cap_mps = cap;
    off.w_gate = 0.0;
    off.brakes_enabled = false;
    const LoopResult r0 = run_loop(off);
    report((label + ", w_gate 0, gate brakes disabled").c_str(), r0);
    LoopSetup on = off;
    on.w_gate = MppiParams{}.w_gate;
    on.brakes_enabled = true;
    const LoopResult r1 = run_loop(on);
    report((label + ", default w_gate, gate brakes applied").c_str(), r1);
    EXPECT_TRUE(r0.finished && r1.finished) << label;
    EXPECT_EQ(r1.engagements, 0) << label;
    EXPECT_LE(r1.sim_time_s, 1.10 * r0.sim_time_s) << label;
  }
}

namespace {

// The hand made profile of the oval (sim_world.hpp oval(1.0, 0.4, 2.0)): 0.6 m/s on the two ends
// (the half circles), 1.2 m/s on the straights. Between them the bound ramps over kRampM metres of
// straight: a literal step is not a profile any car with a speed lag can follow (the producer's
// bounds are speeds the car really drove, so they never fall faster than the car can brake); the
// step was tried and the driven speed ran 0.05 to 0.11 m/s over it at its edges for every w_trust
// from 400 to 3e6.
constexpr double kEndBound = 0.6;
constexpr double kStraightBound = 1.2;
constexpr double kRampM = 0.8;

double hand_bound(double station_m, double lap_m) {
  const double straight = 2.0;
  const double arc = (lap_m - 2.0 * straight) / 2.0;
  const double s = station_m - lap_m * std::floor(station_m / lap_m);
  double d_to_end;  // metres of straight to the nearest end, 0 on an end
  if (s < straight) {
    d_to_end = std::min(s, straight - s);
  } else if (s < straight + arc) {
    d_to_end = 0.0;
  } else if (s < 2.0 * straight + arc) {
    d_to_end = std::min(s - (straight + arc), (2.0 * straight + arc) - s);
  } else {
    d_to_end = 0.0;
  }
  return kEndBound + (kStraightBound - kEndBound) * std::min(1.0, d_to_end / kRampM);
}

}  // namespace

TEST(GateClosedLoop, LapProfileKeepsTheSpeedUnderTheBoundAndLapsFasterThanAFlatLowCap) {
  const World w = make_oval(0);
  const double lap_m = w.perimeter;
  auto bound = [&](double st) { return hand_bound(st, lap_m); };
  // Time to finish at the bound speed, from any unwrapped station (Q(L + x) = Q(x) - T_best).
  auto cum_time = [&](double st) {
    double t = 0.0;
    const double ds = 0.01;
    for (double x = 0.0; x < st; x += ds) t += ds / bound(x + 0.5 * ds);
    return t;
  };
  const double t_best = cum_time(lap_m);
  auto q = [&](double st) { return t_best - cum_time(st); };

  LoopSetup with;
  with.world = &w;
  with.v_cap_mps = kStraightBound;
  with.laps = 3.0;
  with.bound_of_station = bound;
  with.q_of_station = q;
  const LoopResult rp = run_loop(with);
  report("lap profile, bound 0.6 on the ends 1.2 on the straights (0.8 m ramps), cap 1.2", rp);

  LoopSetup flat;
  flat.world = &w;
  flat.v_cap_mps = kEndBound;
  flat.laps = 3.0;
  const LoopResult rf = run_loop(flat);
  report("flat cap 0.6", rf);

  EXPECT_TRUE(rp.finished);
  EXPECT_TRUE(rf.finished);
  EXPECT_LE(rp.max_over_bound_mps, 0.05) << "driven speed over the bound at some station";
  EXPECT_LT(rp.sim_time_s, rf.sim_time_s) << "the profile should lap faster than a flat 0.6";
  EXPECT_GE(rp.min_clearance_m, 0.02);
  EXPECT_EQ(rp.engagements, 0);
}
