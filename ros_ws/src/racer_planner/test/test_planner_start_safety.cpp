// The STARTING pre-turn against the real safety gate (racer_safety's own compiled sources, read
// only, see CMakeLists.txt) when safety_node's cycle stalls during bring-up.
//
// Found 2026-11-17: the L5 canary failed with a rate_limit engagement 0.1 to 0.4 s after the
// planner's first command. safety_node measures dt on its own wall timer; while the perception,
// the lap node and the planner are starting, its cycle stalls and the next one runs a few ms after
// it, with a dt of 4 to 10 ms. The planner's pre-turn of the steering, from the last run's gated
// steering toward the solver's target, moved at 0.45 of the rate limit every tick, so the two or
// three ticks safety had not yet seen were 0.06 to 0.09 rad against an allowance of 3.2 * dt.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gate_safety_rig.hpp"
#include "planner_rig.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::rig;
using namespace racer_planner::test;

namespace {

// Ticks the planner in STARTING at 20 ms with the gated steering 0.30 rad (where safety_node
// still sits from the last run) and feeds the real gate: every cycle evaluates the latest command
// with dt 20 ms, except that `stall_ticks` cycles from `stall_at` do not happen and the next one
// runs `catch_up_dt_s` after the one before them. Returns the number of rate_limit activations.
int rate_limit_activations(PlannerConfig cfg, int stall_at, int stall_ticks, double catch_up_dt_s) {
  Rig r(cfg);
  r.gated_steering = 0.30;
  const racer_safety::SafetyGateLogic safety(safety_limits(true));
  racer_safety::DriveCommand prev_out;  // steering first, speed second
  prev_out.steering_angle_rad = 0.30;
  int clips = 0;
  double t = 1000.0;
  for (int i = 0; i < 24; ++i, t += 0.02) {
    const TickOutput out = r.tick(t);
    if (out.state != PlannerState::Starting) break;
    if (i >= stall_at && i < stall_at + stall_ticks) continue;  // safety_node's cycle is late
    racer_safety::GateInput gi;
    gi.command.speed_mps = out.cmd.speed_mps;
    gi.command.steering_angle_rad = out.cmd.steering_rad;
    gi.drive_raw_age_s = 0.0;
    gi.dt_s = (i == stall_at + stall_ticks) ? catch_up_dt_s : 0.02;
    gi.min_scan_range_m = std::numeric_limits<double>::infinity();
    gi.measured_speed_mps = 0.0;
    const racer_safety::GateResult gr = safety.evaluate(gi, prev_out);
    for (const auto& a : gr.activations) {
      if (a.source == racer_safety::GateSource::kRateLimit) ++clips;
    }
    prev_out = gr.output;
  }
  return clips;
}

}  // namespace

TEST(StartAgainstSafety, TheOldFullRatePreTurnIsClippedByAStalledSafetyCycle) {
  // The behaviour before the fix, by its parameters: no hold, the full 0.45 share. This is the
  // reproduction: the same rig and the same stall give clips.
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_steer_hold_ticks = 0;
  cfg.logic.start_steer_fraction = cfg.logic.rate_limit_fraction;
  int total = 0;
  for (int stall_at = 0; stall_at < 12; ++stall_at) {
    total += rate_limit_activations(cfg, stall_at, 2, 0.008);
  }
  EXPECT_GT(total, 0);
}

TEST(StartAgainstSafety, ThePreTurnStaysInsideSafetyThroughAStallAtAnyTick) {
  const PlannerConfig cfg = small_config(0.5);
  for (int stall_at = 0; stall_at < 20; ++stall_at) {
    for (const double dt : {0.008, 0.02}) {
      EXPECT_EQ(rate_limit_activations(cfg, stall_at, 2, dt), 0)
          << "stall at tick " << stall_at << ", catch up dt " << dt;
    }
  }
}

TEST(StartAgainstSafety, TheSteeringHoldsTheGatedValueUntilTheTicksAreOnTime) {
  const PlannerConfig cfg = small_config(0.5);
  Rig r(cfg);
  r.gated_steering = 0.30;
  double t = 1000.0;
  for (int i = 0; i < cfg.logic.start_steer_hold_ticks; ++i, t += 0.02) {
    const TickOutput out = r.tick(t);
    EXPECT_EQ(out.cmd.steering_rad, 0.30) << "tick " << i;
  }
  // then it moves, at no more than the pre-turn share of the vehicle's rate limit
  const double step = cfg.logic.start_steer_fraction * r.g.steering_max_rate_rad_per_s * 0.02;
  double prev = 0.30;
  bool moved = false;
  for (int i = 0; i < 15; ++i, t += 0.02) {
    const TickOutput out = r.tick(t);
    EXPECT_LE(std::fabs(out.cmd.steering_rad - prev), step + 1e-9) << "tick " << i;
    moved = moved || out.cmd.steering_rad != prev;
    prev = out.cmd.steering_rad;
  }
  EXPECT_TRUE(moved);
}
