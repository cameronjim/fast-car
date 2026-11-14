// Thread safety of the solve / publish split: the solve thread and the publish thread run on one
// Planner on a real clock for a few seconds, with stalls injected into the solve thread, while a
// third thread changes the scene the solve reads the way the scan callback does (swapping a
// shared_ptr under a mutex). Meant to run under ThreadSanitizer (cmake -DRACER_PLANNER_TSAN=ON,
// see the PR for the clean output); in a normal build it checks the same invariants as the
// property tests on real timing: finite, within the cap and the steering limits, steps within the
// elapsed time allowance, a moving plan never older than the plan timeout.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "planner_rig.hpp"

using namespace racer_planner;
using namespace racer_planner::rig;
using Clock = std::chrono::steady_clock;

namespace {

double seconds_since(Clock::time_point t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Scene {
  DistanceField field;
  FreespaceField freespace;
  explicit Scene(const VehicleGeometry& g) : freespace(g) {
    const std::vector<PointXY> pts = corridor_points();
    field.build(pts.data(), pts.size());
    freespace.build(field, pts.data(), pts.size(), nullptr);
  }
};

}  // namespace

TEST(PlannerThreads, SolveAndPublishThreadsShareThePlannerWithoutRacesOrBrokenOutput) {
  const VehicleGeometry g = test::binding_geometry();
  PlannerConfig cfg = small_config(0.6);
  cfg.logic.start_ticks = 10;
  cfg.logic.require_gated_for_start = false;
  Planner planner(cfg, dynamics_with_floor(0.23), g);

  std::mutex scene_mutex;
  std::shared_ptr<Scene> scene = std::make_shared<Scene>(g);
  std::atomic<bool> stop{false};
  const Clock::time_point t0 = Clock::now();
  const double base = 5000.0;  // a time that does not start at 0

  // the "scan callback": builds a fresh scene and swaps it in
  std::thread scan_thread([&] {
    while (!stop.load()) {
      auto next = std::make_shared<Scene>(g);
      {
        std::lock_guard<std::mutex> lock(scene_mutex);
        scene = next;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  LocalTrackData track = good_track(0.0);
  std::thread solve_thread([&] {
    int n = 0;
    auto next = Clock::now();
    while (!stop.load()) {
      next += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(next);
      if (n % 97 == 50) std::this_thread::sleep_for(std::chrono::milliseconds(60));  // late solve
      if (n % 211 == 100) std::this_thread::sleep_for(std::chrono::milliseconds(300));  // a stall
      ++n;
      std::shared_ptr<Scene> s;
      {
        std::lock_guard<std::mutex> lock(scene_mutex);
        s = scene;
      }
      const double now = base + seconds_since(t0);
      track.stamp_s = now - 0.1;
      TickInput in;
      in.now_s = now;
      in.has_odom = true;
      in.odom_stamp_s = now;
      in.wheel_speed_mps = 0.3;
      in.has_scan = true;
      in.scan_stamp_s = now - 0.1;
      in.field = &s->field;
      in.freespace = &s->freespace;
      in.track = &track;
      in.has_gated = true;
      in.gated_stamp_s = now;
      in.gated_speed_mps = 1.0;
      planner.solve(in);
    }
  });

  struct Pub {
    double t;
    TickOutput out;
  };
  std::vector<Pub> pubs;
  pubs.reserve(4000);
  std::thread publish_thread([&] {
    auto next = Clock::now();
    while (!stop.load()) {
      next += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(next);
      PublishInput pin;
      pin.now_s = base + seconds_since(t0);
      pin.has_gated = true;
      pin.gated_stamp_s = pin.now_s;
      const TickOutput out = planner.publish(pin);
      pubs.push_back({pin.now_s, out});
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(6000));
  stop.store(true);
  solve_thread.join();
  publish_thread.join();
  scan_thread.join();

  ASSERT_GT(pubs.size(), 150u);
  int driving = 0, stale = 0;
  const double steer_rate = cfg.logic.rate_limit_fraction * g.steering_max_rate_rad_per_s;
  const double accel = cfg.logic.rate_limit_fraction * g.max_acceleration_mps2;
  for (std::size_t i = 0; i < pubs.size(); ++i) {
    const TickOutput& o = pubs[i].out;
    ASSERT_TRUE(std::isfinite(o.cmd.speed_mps) && std::isfinite(o.cmd.steering_rad));
    ASSERT_LE(o.cmd.speed_mps, cfg.logic.v_max_mps + 1e-9);
    ASSERT_GE(o.cmd.steering_rad, g.steering_min_rad);
    ASSERT_LE(o.cmd.steering_rad, g.steering_max_rad);
    if (o.state == PlannerState::Starting) {
      ASSERT_EQ(o.cmd.speed_mps, 0.0);
    }
    if (o.state == PlannerState::Driving) {
      ++driving;
      ASSERT_LE(o.plan_age_s, cfg.logic.plan_timeout_s + 1e-6);
    }
    if (o.plan_stale) {
      ++stale;
      ASSERT_EQ(o.cmd.speed_mps, 0.0);
    }
    if (i > 0) {
      const double dt = pubs[i].t - pubs[i - 1].t;
      ASSERT_LE(std::fabs(o.cmd.steering_rad - pubs[i - 1].out.cmd.steering_rad),
                steer_rate * dt + cfg.logic.steer_output_hysteresis_rad + 1e-9);
      ASSERT_LE(std::fabs(o.cmd.speed_mps) - std::fabs(pubs[i - 1].out.cmd.speed_mps),
                accel * dt + 1e-9);
    }
  }
  EXPECT_GT(driving, 100) << "the planner never got going";
  EXPECT_GE(stale, 1) << "the injected stalls never made a plan stale";
}
