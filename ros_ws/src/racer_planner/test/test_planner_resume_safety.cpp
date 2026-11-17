// A planner that is driving when safety_node's watchdog trips on a LATE /drive_raw delivery,
// against the real gate (racer_safety's own compiled sources, read only, see CMakeLists.txt).
//
// Found 2026-11-17 in the L5 canary's stall cases (arrival trace, run of the same head on a
// laptop): the planner published on time, 20 ms apart, but one message reached safety_node 58 ms
// after the one before it, so the node's watchdog (60 ms) tripped and its output went to speed 0.
// The planner's own gap was 49 ms, under gap_reset_s (50 ms), so it did not restart its speed ramp
// from 0, and the next command asked for 0.56 m/s from a standstill. The gate allows
// max_acceleration * dt of speed growth, 0.23 m/s on that cycle, and reported rate_limit.
//
// The model: the planner ticks every 20 ms and reads the gated /drive the real gate produced on the
// cycle before. safety_node's cycles run on time, but for `lost` cycles from tick `at` its newest
// /drive_raw is still the one from tick at-1, so its age passes the watchdog timeout. Then the
// newest command arrives. The test counts the gate's rate_limit activations.
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

constexpr double kTickS = 0.02;

struct Outcome {
  int rate_limits = 0;
  int watchdogs = 0;
  double speed_before_mps = 0.0;  // what the planner commanded the tick before the loss
};

Outcome resume_after_late_delivery(int at, int lost) {
  Rig r(small_config(0.5));
  r.gated_steering = 0.0;
  r.gated_speed = 0.0;
  const racer_safety::SafetyGateLogic safety(safety_limits(true));
  racer_safety::DriveCommand prev_out;
  GateCredits credits;
  Outcome o;
  Command visible;
  int last_delivered = -1;
  double t = 1000.0;
  for (int i = 0; i < at + lost + 40; ++i, t += kTickS) {
    const TickOutput out = r.tick(t);
    if (i == at - 1) o.speed_before_mps = out.cmd.speed_mps;
    const bool delivered = i < at || i >= at + lost;
    if (delivered) {
      visible = out.cmd;
      last_delivered = i;
    }
    racer_safety::GateInput gi;
    gi.command.speed_mps = visible.speed_mps;
    gi.command.steering_angle_rad = visible.steering_rad;
    gi.drive_raw_age_s = last_delivered < 0 ? 0.0 : (i - last_delivered) * kTickS;
    gi.dt_s = kTickS;
    gi.min_scan_range_m = std::numeric_limits<double>::infinity();
    gi.measured_speed_mps = prev_out.speed_mps;
    credits.apply(gi);
    const racer_safety::GateResult gr = safety.evaluate(gi, prev_out);
    credits.take(gr);
    for (const auto& a : gr.activations) {
      if (a.source == racer_safety::GateSource::kRateLimit) ++o.rate_limits;
      if (a.source == racer_safety::GateSource::kWatchdog) ++o.watchdogs;
    }
    prev_out = gr.output;
    r.gated_speed = gr.output.speed_mps;  // the planner reads it on its next tick
    r.gated_steering = gr.output.steering_angle_rad;
  }
  return o;
}

}  // namespace

TEST(ResumeAgainstSafety, TheLossReallyTripsTheWatchdogOfADrivingPlanner) {
  // Guards the test below against passing for the wrong reason.
  const Outcome o = resume_after_late_delivery(70, 5);
  EXPECT_GT(o.speed_before_mps, 0.2) << "the planner is not driving at tick 70";
  EXPECT_GE(o.watchdogs, 1);
}

TEST(ResumeAgainstSafety, AWatchdogTripOnALateDeliveryDoesNotMakeTheNextCommandClip) {
  for (const int lost : {4, 5, 6, 8}) {
    for (int at = 60; at < 90; ++at) {
      const Outcome o = resume_after_late_delivery(at, lost);
      EXPECT_EQ(o.rate_limits, 0) << "delivery lost for " << lost << " cycles from tick " << at;
    }
  }
}
