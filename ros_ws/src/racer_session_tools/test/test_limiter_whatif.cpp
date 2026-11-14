// Build wiring of the what-if tool: its clip decisions must equal racer_safety's own gate on a
// table of commands. The tool compiles racer_safety's sources by path (the same files), so this
// fails if the wiring ever builds a different gate (other limits, other sources, a stale copy).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_session_tools/whatif_core.hpp"

namespace {

using racer_safety::DriveCommand;
using racer_safety::GateSource;
using racer_session_tools::CycleInput;
using racer_session_tools::Replayer;

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Row {
  DriveCommand previous;  // what the gate output on the previous cycle
  DriveCommand command;
  double dt_s;
};

// racer_safety's own gate on the same input the tool builds: no scan, no odometry.
bool own_gate_clips(const racer_safety::SafetyLimits& limits, const Row& r) {
  const racer_safety::SafetyGateLogic gate(limits);
  racer_safety::GateInput input;
  input.command = r.command;
  input.drive_raw_age_s = 0.0;
  input.dt_s = r.dt_s;
  input.min_scan_range_m = kInf;
  input.min_rear_scan_range_m = kInf;
  const racer_safety::GateResult result = gate.evaluate(input, r.previous);
  for (const auto& a : result.activations) {
    if (a.source == GateSource::kRateLimit) {
      return true;
    }
  }
  return false;
}

// The tool: warm a Replayer to `previous` with a command that is not clipped, then one cycle.
// The warm-up uses an old command age so that it cannot interact: instead it is seeded by
// stepping the same command until the output equals the previous output.
bool tool_clips(const racer_safety::SafetyLimits& limits, const Row& r) {
  Replayer replayer(limits);
  // Walk the output to `previous` in steps small enough never to clip (dt large), then judge.
  CycleInput warm;
  warm.dt_s = 10.0;  // a dt this long allows any change, so the walk is exact in one cycle
  warm.command = r.previous;
  warm.age_s = 0.0;
  replayer.step(warm);  // first cycle uses the nominal period inside the tool's CLI only
  warm.stamp_s = 10.0;
  replayer.step(warm);
  CycleInput in;
  in.stamp_s = 10.0 + r.dt_s;
  in.dt_s = r.dt_s;
  in.command = r.command;
  in.age_s = 0.0;
  return replayer.step(in).rate_limit_clipped;
}

TEST(WhatIfWiring, ClipDecisionsEqualTheGatesOwnOnATable) {
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  const double rate = limits.steering_rate_max_rad_per_s;
  const double acc = limits.max_acceleration_mps2;
  const double nominal = racer_session_tools::nominal_period_s(limits);
  ASSERT_GT(rate, 0.0);
  ASSERT_GT(acc, 0.0);
  const std::vector<Row> table = {
      {{0.0, 0.0}, {0.0, 0.0}, nominal},                              // standing, no command
      {{0.0, 0.0}, {0.1, 0.0}, nominal},                              // small steering step
      {{0.0, 0.0}, {0.4, 0.0}, nominal},                              // large steering step
      {{0.0, 0.0}, {-0.4, 0.0}, nominal},                             // large step, other side
      {{0.4, 0.0}, {0.0, 0.0}, nominal},                              // the start clip of 13.1.3
      {{0.4, 0.0}, {0.0, 0.0}, 0.0068},                               // same, tiny recorded dt
      {{0.0, 0.0}, {0.0, 0.2}, nominal},                              // speed step
      {{0.0, 0.5}, {0.0, 0.5}, nominal},                              // steady
      {{0.0, 0.5}, {0.0, 0.0}, nominal},                              // braking is never clipped
      {{0.0, 0.3}, {0.0, 0.3 + 2.0 * acc * 0.0068}, 0.0068},          // speed step, twice the limit
      {{0.0, 0.3}, {0.0, 0.3 + 0.5 * acc * nominal}, nominal},        // speed step, half the limit
      {{0.0, 0.0}, {2.0 * rate * 0.0068, 0.0}, 0.0068},               // steering, twice the limit
      {{0.0, 0.0}, {0.5 * rate * nominal, 0.0}, nominal},             // steering, half the limit
      {{0.0, 0.0}, {0.5 * rate * 0.0068, 0.0}, 0.0068},               // half the limit at small dt
      {{0.05, 0.4}, {0.15, 0.45}, 0.03},                              // both channels
  };
  int clips = 0;
  for (std::size_t i = 0; i < table.size(); ++i) {
    const bool own = own_gate_clips(limits, table[i]);
    EXPECT_EQ(tool_clips(limits, table[i]), own) << "table row " << i;
    clips += own ? 1 : 0;
  }
  // The table must exercise both outcomes or it proves nothing.
  EXPECT_GT(clips, 3);
  EXPECT_LT(clips, static_cast<int>(table.size()) - 3);
}

TEST(WhatIfWiring, TheRecordedDtChangesTheDecisionAndTheNominalPeriodDoesNot) {
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  const double rate = limits.steering_rate_max_rad_per_s;
  const double nominal = racer_session_tools::nominal_period_s(limits);
  // A steering step of 0.6 of one nominal tick's allowance: fine at the nominal dt, a clip at a
  // dt of a third of it (the bunched tick of 13.1.1).
  const Row row{{0.0, 0.0}, {0.6 * rate * nominal, 0.0}, nominal / 3.0};
  Row nominal_row = row;
  nominal_row.dt_s = nominal;
  EXPECT_TRUE(own_gate_clips(limits, row));
  EXPECT_FALSE(own_gate_clips(limits, nominal_row));
  EXPECT_TRUE(tool_clips(limits, row));
  EXPECT_FALSE(tool_clips(limits, nominal_row));
}

TEST(WhatIfWiring, ARecordedWatchdogGapTripsTheWatchdogAndIsReported) {
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  Replayer replayer(limits);
  CycleInput in;
  in.dt_s = 0.02;
  in.command = DriveCommand{0.0, 0.5};
  in.age_s = 0.01;
  EXPECT_FALSE(replayer.step(in).watchdog_tripped);
  in.stamp_s = 0.02;
  in.age_s = 0.349454;  // the age quoted in the 2026-11-13 watchdog event
  const racer_session_tools::CycleOutput out = replayer.step(in);
  EXPECT_TRUE(out.watchdog_tripped);
  ASSERT_FALSE(out.events.empty());
  EXPECT_EQ(out.events.front().source, "watchdog");
  EXPECT_TRUE(out.events.front().engage);
  EXPECT_EQ(out.events.front().detail.find("/drive_raw stale or age invalid (timeout="), 0u);
}

TEST(WhatIfWiring, ASeededSegmentContinuesFromTheSeedAndForcingReplacesThePreviousOutput) {
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  Replayer replayer(limits);
  replayer.seed(DriveCommand{0.3, 0.5});
  CycleInput in;
  in.dt_s = 0.02;
  in.command = DriveCommand{0.3, 0.5};
  in.age_s = 0.0;
  const racer_session_tools::CycleOutput held = replayer.step(in);
  EXPECT_FALSE(held.rate_limit_clipped);
  EXPECT_DOUBLE_EQ(held.output.steering_angle_rad, 0.3);
  EXPECT_DOUBLE_EQ(held.output.speed_mps, 0.5);
  // Forcing the previous output to standstill makes the same command a clip.
  replayer.force_previous(DriveCommand{0.0, 0.0});
  in.stamp_s = 0.02;
  EXPECT_TRUE(replayer.step(in).rate_limit_clipped);
}

TEST(WhatIfWiring, TheNominalPeriodIsTheGatesControlPeriod) {
  const racer_safety::SafetyLimits limits = racer_session_tools::default_limits();
  EXPECT_DOUBLE_EQ(racer_session_tools::nominal_period_s(limits), limits.control_period_s);
  EXPECT_NEAR(limits.control_period_s, 0.02, 1e-12);
  EXPECT_EQ(limits.watchdog_missed_cycles, 3);
}

}  // namespace
