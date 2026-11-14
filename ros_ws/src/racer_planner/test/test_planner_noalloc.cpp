// No heap allocation in the solve and publish paths after the planner is built and warmed up
// (invariant of the 50 Hz path: claude-docs/10-conventions.md). This binary replaces the global
// operator new and counts calls while armed; it is its own executable so no other test sees the
// replacement. Exercises every branch of the output path: on-time ticks, bunched callbacks, late
// ticks and gap resets, a stale plan, STARTING again after a long stop, silence of each input,
// an old scan, garbage stamps.
#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

#include "planner_rig.hpp"

namespace {
std::atomic<long> g_allocations{0};
std::atomic<bool> g_armed{false};

void* counted_alloc(std::size_t n) {
  if (g_armed.load(std::memory_order_relaxed)) g_allocations.fetch_add(1);
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}
}  // namespace

void* operator new(std::size_t n) { return counted_alloc(n); }
void* operator new[](std::size_t n) { return counted_alloc(n); }
void* operator new(std::size_t n, std::align_val_t) { return counted_alloc(n); }
void* operator new[](std::size_t n, std::align_val_t) { return counted_alloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }

using namespace racer_planner;
using namespace racer_planner::rig;

namespace {

struct Armed {
  Armed() { g_armed.store(true); }
  ~Armed() { g_armed.store(false); }
};

}  // namespace

TEST(PlannerNoAlloc, SolveAndPublishNeverAllocateAfterWarmUp) {
  PlannerConfig cfg = small_config(0.6);
  cfg.logic.start_ticks = 10;
  Rig r(cfg, 0.23);
  // the track is built once; only its stamp moves (building one allocates, outside the window)
  LocalTrackData track = good_track(0.0);
  LocalTrackData bent = good_track(0.0, 0.3);
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.1, y});
  DistanceField wall_field;
  wall_field.build(wall.data(), wall.size());
  FreespaceField wall_free(r.g);
  wall_free.build(wall_field, wall.data(), wall.size(), nullptr);

  double t = 1000.0;
  TickOutput out;
  auto run = [&](double now, bool use_wall, bool odom, bool scan, double scan_age,
                 const LocalTrackData* trk, bool bunch_publish) {
    track.stamp_s = now - 0.1;
    bent.stamp_s = now - 0.1;
    TickInput in;
    in.now_s = now;
    in.has_odom = odom;
    in.odom_stamp_s = now;
    in.wheel_speed_mps = use_wall ? 0.0 : 0.3;
    in.has_scan = scan;
    in.scan_stamp_s = now - scan_age;
    in.field = use_wall ? &wall_field : &r.field;
    in.freespace = use_wall ? &wall_free : &r.freespace;
    in.track = trk;
    in.has_gated = true;
    in.gated_stamp_s = now;
    in.gated_speed_mps = 1.0;
    PublishInput pin;
    pin.now_s = now;
    pin.has_gated = true;
    pin.gated_stamp_s = now;
    {
      Armed armed;
      r.planner.solve(in);
      out = r.planner.publish(pin);
      if (bunch_publish) {
        pin.now_s = now + 0.004;
        out = r.planner.publish(pin);  // a callback just after: a republish
      }
    }
  };
  // warm up: leaves STARTING, ramps, drives, a wall, recovery, everything allocates what it needs
  for (int i = 0; i < 400; ++i, t += 0.02) run(t, false, true, true, 0.1, &track, false);
  for (int i = 0; i < 100; ++i, t += 0.02) run(t, true, true, true, 0.1, &track, false);
  for (int i = 0; i < 200; ++i, t += 0.02) run(t, false, true, true, 0.1, &track, false);

  g_allocations.store(0);
  int driving = 0, starting = 0, stale = 0, bunched = 0, other = 0;
  for (int i = 0; i < 1500; ++i) {
    double gap = 0.02;
    if (i % 37 == 0) gap = 0.09;  // a late tick: gap reset
    if (i % 211 == 0) gap = 0.2;  // a stall: stale plan on the publish side next
    t += gap;
    if (i % 211 == 0) {  // the solver stalled: publishes continue from a plan that has gone stale
      PublishInput late;
      late.now_s = t - 0.05;
      late.has_gated = true;
      late.gated_stamp_s = t - 0.05;
      Armed armed;
      out = r.planner.publish(late);
      stale += out.plan_stale ? 1 : 0;
    }
    const bool wall_phase = i % 300 >= 200 && i % 300 < 270;  // 1.4 s of wall every 6 s: BLOCKED
    const bool odom = i % 97 != 0;
    const bool scan = i % 131 != 0;
    const double scan_age = i % 53 == 0 ? 0.45 : 0.1;
    const LocalTrackData* trk = i % 61 == 0 ? nullptr : ((i / 20) % 2 == 0 ? &track : &bent);
    run(t, wall_phase, odom, scan, scan_age, trk, i % 5 == 0);
    switch (out.state) {
      case PlannerState::Driving:
        ++driving;
        break;
      case PlannerState::Starting:
        ++starting;
        break;
      default:
        ++other;
        break;
    }
    bunched += out.bunched ? 1 : 0;
  }
  EXPECT_EQ(g_allocations.load(), 0) << "heap allocations in solve or publish after warm up";
  // the run really went through the branches it is meant to cover
  EXPECT_GT(driving, 500);
  EXPECT_GT(starting, 5);
  EXPECT_GT(bunched, 100);
  EXPECT_GT(other, 20);
  EXPECT_GE(stale, 5);
}

TEST(PlannerNoAlloc, TheCounterWorks) {
  g_allocations.store(0);
  {
    Armed armed;
    auto* p = new int[16];
    delete[] p;
  }
  EXPECT_GE(g_allocations.load(), 1);
}
