// L2: properties of the planner's output timing for ANY sequence of tick times (jitter, bunching,
// stalls) and any solver outputs, seeded generators. The important one (design 14.7): over every
// window between two publishes the published steering changes by at most f * steering_rate * (t_j
// - t_i) plus one hysteresis quantum, and the speed magnitude grows by at most f * max_accel *
// (t_j - t_i); slowing is not limited. Also: the output stays inside the cap and the steering
// limits, STARTING never publishes a non-zero speed, and nothing is commanded between 0 and the
// speed floor except the dead zone ramp from rest, which takes the same wall time however the
// ticks land.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "planner_rig.hpp"

using namespace racer_planner;
using namespace racer_planner::rig;

namespace {

struct Sample {
  double t = 0.0;
  Command cmd;
  PlannerState state = PlannerState::Starting;
  double elapsed = 0.0;
  bool bunched = false;
  bool gap_reset = false;
  double v_cap = 0.0;
};

struct Generator {
  std::mt19937_64 rng;
  std::uniform_real_distribution<double> u01{0.0, 1.0};
  explicit Generator(std::uint64_t seed) : rng(seed) {}
  double uni(double lo, double hi) { return lo + (hi - lo) * u01(rng); }

  // The next tick gap: mostly on time, some jitter, bunched ticks right after a late one, late
  // ticks of 40 to 90 ms and the occasional stall of a few hundred ms.
  double gap() {
    const double r = u01(rng);
    if (r < 0.55) return 0.02;
    if (r < 0.70) return uni(0.015, 0.025);
    if (r < 0.82) return uni(0.0005, 0.0099);  // bunched
    if (r < 0.94) return uni(0.026, 0.09);
    if (r < 0.985) return uni(0.09, 0.2);
    return uni(0.2, 0.6);
  }
};

// The scene a random scenario shows to the planner at time t.
struct Scene {
  Generator& gen;
  double v = 0.0;
  double y_offset = 0.0;
  double offset_target = 0.0;

  void step(double dt) {
    // a wheel speed that wanders between rest and 0.7 m/s
    v = std::min(std::max(v + gen.uni(-0.6, 0.6) * dt * 5.0, 0.0), 0.7);
    if (gen.u01(gen.rng) < 0.03) offset_target = gen.uni(-0.45, 0.45);
    y_offset += (offset_target - y_offset) * std::min(1.0, dt * 8.0);
  }
};

void check_windows(const std::vector<Sample>& s, const VehicleGeometry& g, double fraction,
                   double hysteresis, double floor_mps, double tick, const char* label) {
  const double steer_rate = fraction * g.steering_max_rate_rad_per_s;
  const double accel = fraction * g.max_acceleration_mps2;
  for (std::size_t i = 0; i < s.size(); ++i) {
    ASSERT_TRUE(std::isfinite(s[i].cmd.speed_mps) && std::isfinite(s[i].cmd.steering_rad)) << label;
    ASSERT_GE(s[i].cmd.steering_rad, g.steering_min_rad) << label;
    ASSERT_LE(s[i].cmd.steering_rad, g.steering_max_rad) << label;
    if (s[i].state == PlannerState::Starting) {
      ASSERT_EQ(s[i].cmd.speed_mps, 0.0) << label << " STARTING published a speed";
    }
    if (s[i].state != PlannerState::Recovering) {
      ASSERT_GE(s[i].cmd.speed_mps, 0.0) << label << " state " << static_cast<int>(s[i].state);
    }
    if (s[i].state == PlannerState::Driving) {
      ASSERT_LE(s[i].cmd.speed_mps, s[i].v_cap + 1e-12) << label;
    }
    if (s[i].state != PlannerState::Driving && s[i].state != PlannerState::Recovering) {
      ASSERT_EQ(s[i].cmd.speed_mps, 0.0) << label << " state " << static_cast<int>(s[i].state);
    }
  }
  for (std::size_t i = 0; i < s.size(); ++i) {
    for (std::size_t j = i + 1; j < s.size(); ++j) {
      const double dt = s[j].t - s[i].t;
      ASSERT_LE(std::fabs(s[j].cmd.steering_rad - s[i].cmd.steering_rad),
                steer_rate * dt + hysteresis + 1e-9)
          << label << " steering window " << i << ".." << j << " dt " << dt;
      ASSERT_LE(std::fabs(s[j].cmd.speed_mps) - std::fabs(s[i].cmd.speed_mps), accel * dt + 1e-9)
          << label << " speed window " << i << ".." << j << " dt " << dt;
    }
  }
  // Nothing between 0 and the floor except the ramp from rest, whose wall time is the same
  // however the ticks land: the elapsed time summed over one run of such commands stays under
  // ceil(floor / step) ticks.
  if (floor_mps > 0.0) {
    const double up = accel * tick;
    const double ramp_wall = std::ceil(floor_mps / up - 1e-9) * tick;
    double run = 0.0;
    double prev_speed = 0.0;
    for (const Sample& x : s) {
      if (x.bunched) continue;
      if (x.gap_reset) prev_speed = 0.0;  // a gap long enough for the watchdog: a start from rest
      const bool in_dz = x.cmd.speed_mps > 0.0 && x.cmd.speed_mps < floor_mps - 1e-12;
      if (in_dz) {
        ASSERT_LT(prev_speed, floor_mps - 1e-12)
            << label << " dead zone entered from " << prev_speed;
        run += x.elapsed;
        ASSERT_LT(run, ramp_wall + 1e-9) << label << " ramp took " << run << " s";
      } else {
        run = 0.0;
      }
      prev_speed = x.cmd.speed_mps;
    }
  }
}

}  // namespace

TEST(PlannerTimingProperties, SlewAllowanceHoldsOverEveryWindowForAnyTickTimes) {
  const VehicleGeometry g = test::binding_geometry();
  int driving_ticks = 0, bunched = 0, late = 0, total = 0;
  for (int scenario = 0; scenario < 24; ++scenario) {
    Generator gen(9000 + static_cast<std::uint64_t>(scenario));
    PlannerConfig cfg = small_config(gen.uni(0.3, 0.9));
    cfg.logic.start_ticks = scenario % 2 == 0 ? 25 : 3;
    cfg.logic.require_gated_for_start = scenario % 3 != 0;
    cfg.logic.steer_output_tau_s = scenario % 4 == 0 ? 0.0 : 0.04;
    const double floor_mps = scenario % 2 == 0 ? 0.23 : 0.03;
    Rig r(cfg, floor_mps);
    Scene scene{gen};
    scene.v = gen.uni(0.0, 0.5);
    double t = 5000.0;
    std::vector<Sample> samples;
    Command last{0.0, 0.0};
    for (int k = 0; k < 420; ++k) {
      const double dt = k == 0 ? 0.0 : gen.gap();
      t += dt;
      scene.step(dt);
      r.v = scene.v;
      TickInput in = r.input(t);
      const LocalTrackData bent = good_track(t - 0.1, scene.y_offset);
      in.track = gen.u01(gen.rng) < 0.04 ? nullptr : &bent;
      in.gated_speed_mps = 1.0;
      in.gated_steering_rad = last.steering_rad;  // safety follows what was published
      if (gen.u01(gen.rng) < 0.02) in.has_odom = false;
      if (gen.u01(gen.rng) < 0.02) in.odom_stamp_s = t - gen.uni(0.05, 0.3);
      if (gen.u01(gen.rng) < 0.02) in.scan_stamp_s = t - gen.uni(0.2, 0.8);
      const TickOutput out = r.planner.tick(in);
      Sample x;
      x.t = t;
      x.cmd = out.cmd;
      x.state = out.state;
      x.elapsed = out.elapsed_s;
      x.bunched = out.bunched;
      x.gap_reset = out.gap_reset;
      x.v_cap = out.v_cap_mps;
      samples.push_back(x);
      last = out.cmd;
      driving_ticks += out.state == PlannerState::Driving ? 1 : 0;
      bunched += out.bunched ? 1 : 0;
      late += dt > 0.05 ? 1 : 0;
      ++total;
    }
    check_windows(samples, g, cfg.logic.rate_limit_fraction, cfg.logic.steer_output_hysteresis_rad,
                  floor_mps, cfg.tick_s, ("scenario " + std::to_string(scenario)).c_str());
    if (::testing::Test::HasFatalFailure()) return;
  }
  // the generator really exercises the claim
  EXPECT_GT(driving_ticks, 1500);
  EXPECT_GT(bunched, 300);
  EXPECT_GT(late, 100);
  EXPECT_EQ(total, 24 * 420);
}

// The node's shape: solve and publish on independent time lines. The publisher keeps its own
// jitter (and bunches), the solver stalls, and a publisher stall as long as a process starved for
// a while.
TEST(PlannerTimingProperties, SlewAllowanceHoldsWhenSolveAndPublishRunOnTheirOwnTimeLines) {
  const VehicleGeometry g = test::binding_geometry();
  int stale_stops = 0, driving_ticks = 0, solves = 0;
  for (int scenario = 0; scenario < 16; ++scenario) {
    Generator gen(31000 + static_cast<std::uint64_t>(scenario));
    PlannerConfig cfg = small_config(gen.uni(0.3, 0.9));
    cfg.logic.start_ticks = scenario % 2 == 0 ? 25 : 3;
    cfg.logic.require_gated_for_start = scenario % 3 != 0;
    const double floor_mps = scenario % 2 == 0 ? 0.23 : 0.03;
    Rig r(cfg, floor_mps);
    Scene scene{gen};
    scene.v = gen.uni(0.0, 0.5);

    struct Event {
      double t;
      bool solve;
    };
    std::vector<Event> events;
    double ts = 5000.0, tp = 5000.003;
    const double end = 5000.0 + 9.0;
    while (ts < end) {
      events.push_back({ts, true});
      const double r1 = gen.u01(gen.rng);
      ts +=
          r1 < 0.8 ? gen.uni(0.017, 0.023) : (r1 < 0.97 ? gen.uni(0.04, 0.09) : gen.uni(0.1, 0.4));
    }
    while (tp < end) {
      events.push_back({tp, false});
      const double r1 = gen.u01(gen.rng);
      tp += r1 < 0.8    ? gen.uni(0.019, 0.021)
            : r1 < 0.92 ? gen.uni(0.0005, 0.009)
            : r1 < 0.99 ? gen.uni(0.03, 0.08)
                        : gen.uni(0.1, 0.3);
    }
    std::stable_sort(events.begin(), events.end(),
                     [](const Event& a, const Event& b) { return a.t < b.t; });
    std::vector<Sample> samples;
    Command last{0.0, 0.0};
    double prev_t = events.front().t;
    for (const Event& e : events) {
      scene.step(e.t - prev_t);
      prev_t = e.t;
      r.v = scene.v;
      if (e.solve) {
        TickInput in = r.input(e.t);
        const LocalTrackData bent = good_track(e.t - 0.1, scene.y_offset);
        in.track = &bent;
        in.gated_speed_mps = 1.0;
        in.gated_steering_rad = last.steering_rad;
        if (gen.u01(gen.rng) < 0.02) in.has_odom = false;
        r.planner.solve(in);
        ++solves;
      } else {
        PublishInput pin = r.pub_input(e.t);
        pin.gated_steering_rad = last.steering_rad;
        const TickOutput out = r.planner.publish(pin);
        Sample x;
        x.t = e.t;
        x.cmd = out.cmd;
        x.state = out.state;
        x.elapsed = out.elapsed_s;
        x.bunched = out.bunched;
        x.gap_reset = out.gap_reset;
        x.v_cap = out.v_cap_mps;
        samples.push_back(x);
        last = out.cmd;
        driving_ticks += out.state == PlannerState::Driving ? 1 : 0;
        if (out.plan_stale) {
          ++stale_stops;
          // a snapshot past the timeout is never published as a moving command
          ASSERT_EQ(out.cmd.speed_mps, 0.0);
          ASSERT_EQ(out.state, PlannerState::Fault);
        }
        if (out.state == PlannerState::Driving || out.state == PlannerState::Recovering) {
          ASSERT_LE(out.plan_age_s, cfg.logic.plan_timeout_s + 1e-9)
              << "a moving state from a plan " << out.plan_age_s << " s old";
        }
      }
    }
    check_windows(samples, g, cfg.logic.rate_limit_fraction, cfg.logic.steer_output_hysteresis_rad,
                  floor_mps, cfg.tick_s,
                  ("two time lines, scenario " + std::to_string(scenario)).c_str());
    if (::testing::Test::HasFatalFailure()) return;
  }
  EXPECT_GT(driving_ticks, 1000);
  EXPECT_GT(stale_stops, 20) << "the generator never starved the publisher of a fresh plan";
  EXPECT_GT(solves, 1000);
}

TEST(PlannerTimingProperties, StartingNeverPublishesASpeedWhateverTheInputsDo) {
  const VehicleGeometry g = test::binding_geometry();
  std::mt19937_64 rng(77);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  int starting = 0;
  for (int scenario = 0; scenario < 30; ++scenario) {
    PlannerConfig cfg = small_config(0.6);
    cfg.logic.require_gated_for_start = u01(rng) < 0.5;
    Rig r(cfg, 0.23);
    double t = 100.0;
    bool left = false;
    for (int k = 0; k < 400; ++k) {
      t += u01(rng) < 0.9 ? 0.02 : 0.06;  // a late tick now and then keeps resetting the count
      r.gated = u01(rng) < 0.5;
      r.odom = u01(rng) < 0.97;
      r.scan = u01(rng) < 0.97;
      const TickOutput out = r.tick(t);
      if (out.state == PlannerState::Starting) {
        ++starting;
        ASSERT_EQ(out.cmd.speed_mps, 0.0) << "scenario " << scenario << " tick " << k;
      }
      if (out.left_starting) left = true;
      ASSERT_LE(out.cmd.steering_rad, g.steering_max_rad);
      ASSERT_GE(out.cmd.steering_rad, g.steering_min_rad);
    }
  }
  EXPECT_GT(starting, 3000);
}
