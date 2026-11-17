// The STARTING pre-turn of main's planner against the real safety gate (racer_safety's own
// compiled sources, read only, see CMakeLists.txt) when safety_node's cycle runs late during
// bring-up.
//
// Found 2026-11-17: the L5 canary fails with a rate_limit engagement 0.1 to 0.4 s after the
// planner's first command. safety_node measures dt on its own wall timer. While the perception,
// the lap node and the planner start, one of its cycles runs late and the next one runs a few ms
// after it, with a dt of 1 to 12 ms. The planner's pre-turn of the steering, from the last run's
// gated steering toward the solver's target, moves at 0.45 of the rate limit every tick, so the
// one tick that cycle sees for the first time is 0.009 of the rate limit's second against an
// allowance of rate * dt.
//
// The model of a stall, per start tick `at` and stall length `n` cycles (1 to 3): the previous
// gate cycle ran on time. The next `n` cycles do not run. The late cycle runs `catch_up_dt_s`
// before the on time position of the cycle after them, with its true dt (the time since the
// previous cycle that ran, which is the stall time minus `catch_up_dt_s`) on the command it can
// see (the one from the tick before). The catch up cycle then runs on its phase with dt
// `catch_up_dt_s` on the newest command. Every other cycle runs on time with dt 20 ms.
//
// The planner is not changed by this test. It must see no rate_limit activation that clipped
// anything: the gate's rate rule has to give a command stream that moves at 0.45 of the limit by
// its own clock room for the late and the catch up cycle.
//
// What counts as a clip. The gate reports rate_limit when its recomputed output (previous output
// plus the granted change) differs from the request in any bit, and that sum can land one ulp off
// a request that is well inside the limit (0.0696 then 0.0120 rad: 0.0696 + (0.0120 - 0.0696) is
// not 0.0120). Such an event clips nothing: the output is the command to 17 digits. That detection
// is not this brief's to change, so the test counts an activation as a clip only when the output
// differs from the command by more than 8 ulps of the larger of the previous output and the
// command, and prints how many it let pass.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>

#include "gate_safety_rig.hpp"
#include "planner_rig.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::rig;
using namespace racer_planner::test;

namespace {

constexpr double kTickS = 0.02;
long g_ulp_events = 0;  // rate_limit activations that clipped nothing (see "What counts as a clip")
constexpr int kTicks = 24;  // the warm start gate leaves STARTING after 25 ticks

racer_safety::GateInput gate_input(const Command& cmd, double age_s, double dt_s) {
  racer_safety::GateInput gi;
  gi.command.speed_mps = cmd.speed_mps;
  gi.command.steering_angle_rad = cmd.steering_rad;
  gi.drive_raw_age_s = age_s;
  gi.dt_s = dt_s;
  gi.min_scan_range_m = std::numeric_limits<double>::infinity();
  gi.measured_speed_mps = 0.0;
  return gi;
}

// Ticks the planner in STARTING at 20 ms with the gated steering 0.30 rad (where safety_node
// still sits from the last run) and feeds the real gate as described above. Returns the number of
// rate_limit activations that clipped; `moved_rad` (if given) receives the largest distance the
// commanded steering got from the gated 0.30.
int rate_limit_activations(int stall_at, int stall_cycles, double catch_up_dt_s,
                           double* moved_rad = nullptr) {
  Rig r(small_config(0.5));
  r.gated_steering = 0.30;
  const racer_safety::SafetyGateLogic safety(safety_limits(true));
  racer_safety::DriveCommand prev_out;
  prev_out.steering_angle_rad = 0.30;
  int clips = 0;
  double moved = 0.0;
  Command previous_cmd;
  double t = 1000.0;
  const auto run_gate = [&](const Command& cmd, double age_s, double dt_s) {
    const racer_safety::GateResult gr = safety.evaluate(gate_input(cmd, age_s, dt_s), prev_out);
    for (const auto& a : gr.activations) {
      if (a.source != racer_safety::GateSource::kRateLimit) continue;
      const double eps = std::numeric_limits<double>::epsilon();
      const double tol =
          8.0 * eps *
          std::fmax(std::fabs(prev_out.steering_angle_rad), std::fabs(cmd.steering_rad));
      if (std::fabs(gr.output.steering_angle_rad - cmd.steering_rad) > tol ||
          gr.output.speed_mps != cmd.speed_mps) {
        ++clips;
      } else {
        ++g_ulp_events;
      }
    }
    prev_out = gr.output;
  };
  for (int i = 0; i < kTicks; ++i, t += kTickS) {
    const TickOutput out = r.tick(t);
    if (out.state != PlannerState::Starting) break;
    moved = std::fmax(moved, std::fabs(out.cmd.steering_rad - 0.30));
    const int resume_at = stall_at + stall_cycles;
    if (i >= stall_at && i < resume_at) {
      // this gate cycle does not run
    } else if (i == resume_at) {
      const double stall_s = (stall_cycles + 1) * kTickS;
      run_gate(previous_cmd, kTickS - catch_up_dt_s, stall_s - catch_up_dt_s);  // the late cycle
      run_gate(out.cmd, 0.0, catch_up_dt_s);                                    // the catch up
    } else {
      run_gate(out.cmd, 0.0, kTickS);
    }
    previous_cmd = out.cmd;
  }
  if (moved_rad != nullptr) *moved_rad = moved;
  return clips;
}

}  // namespace

TEST(StartAgainstSafety, ThePreTurnMovesTheSteeringInStarting) {
  // Guards the test below against passing for the wrong reason: there has to be a pre-turn.
  double moved = 0.0;
  EXPECT_EQ(rate_limit_activations(-10, 0, 0.02, &moved), 0);
  EXPECT_GT(moved, 0.01);
}

TEST(StartAgainstSafety, ALateAndACatchUpSafetyCycleNeverClipThePreTurn) {
  for (int stall_at = 0; stall_at < 22; ++stall_at) {
    for (int stall_cycles = 1; stall_cycles <= 3; ++stall_cycles) {
      for (const double catch_up_dt : {0.001, 0.004, 0.008, 0.012}) {
        EXPECT_EQ(rate_limit_activations(stall_at, stall_cycles, catch_up_dt), 0)
            << "stall at tick " << stall_at << ", " << stall_cycles << " cycles, catch up dt "
            << catch_up_dt;
      }
    }
  }
  std::printf("rate_limit activations that clipped nothing (ulp rounding), let through: %ld\n",
              g_ulp_events);
}
