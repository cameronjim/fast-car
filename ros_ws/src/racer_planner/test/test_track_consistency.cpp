// The one frame track bridge (track_consistency.hpp, fix checklist 2026-11-22 item S2): the
// consistency rule (L1), the gate's decisions (L1), a seeded property loop over random track
// sequences (L2) and the planner with the gate in it (one bad frame is bridged; a persistent change
// gets through on the next frame; a wall in the scan still stops the car whatever the track says).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "planner_rig.hpp"
#include "racer_planner/track_consistency.hpp"

using namespace racer_planner;

namespace {

// A track in its own base_link: a straight centreline along x, half width `half` everywhere.
LocalTrackData straight(double stamp, double half = 0.75) {
  LocalTrackData t = rig::good_track(stamp);
  for (int k = 0; k < kTrackStations; ++k) {
    t.half_width_left_m[k] = half;
    t.half_width_right_m[k] = half;
  }
  return t;
}

// The 2026-11-22 outlier, in shape: from station `from` on, the centreline swings `jump_m` to the
// side and the half width drops to `half`.
LocalTrackData outlier(double stamp, double jump_m = 0.45, double half = 0.40, int from = 5) {
  LocalTrackData t = straight(stamp);
  for (int k = from; k < kTrackStations; ++k) {
    t.y_m[k] += jump_m;
    t.half_width_left_m[k] = half;
    t.half_width_right_m[k] = half;
  }
  return t;
}

// The world circle of radius r (centre (0, r), counter clockwise from the origin) sampled at the
// stations, seen from a car at arc length s0 along it, heading along it.
LocalTrackData arc_seen_from(double stamp, double r, double s0) {
  LocalTrackData t = straight(stamp);
  const double th0 = s0 / r;
  const double cx = r * std::sin(th0), cy = r - r * std::cos(th0);  // car position
  for (int k = 0; k < kTrackStations; ++k) {
    const double th = (s0 + k * kTrackStationSpacingM) / r;
    const double wx = r * std::sin(th) - cx, wy = r - r * std::cos(th) - cy;
    t.x_m[k] = std::cos(th0) * wx + std::sin(th0) * wy;
    t.y_m[k] = -std::sin(th0) * wx + std::cos(th0) * wy;
  }
  return t;
}

// Pose of the car at arc length s_ref in the frame of the car at s_new, on the same circle.
Pose2 arc_pose(double r, double s_new, double s_ref) {
  const double a = s_new / r, b = s_ref / r;
  const double dx = r * (std::sin(b) - std::sin(a)), dy = r * (std::cos(a) - std::cos(b));
  Pose2 p;
  p.x = std::cos(a) * dx + std::sin(a) * dy;
  p.y = -std::sin(a) * dx + std::cos(a) * dy;
  p.yaw = b - a;
  return p;
}

const TrackConsistencyParams kP{};

}  // namespace

// -- judge_consistency ---------------------------------------------------------------------------

TEST(TrackConsistency, TheSameTrackIsConsistent) {
  const LocalTrackData t = straight(1.0);
  const ConsistencyResult r = judge_consistency(t, t, Pose2{}, kP);
  EXPECT_TRUE(r.judged);
  EXPECT_TRUE(r.consistent);
  EXPECT_NEAR(r.centre_jump_m, 0.0, 1e-12);
  EXPECT_NEAR(r.half_width_ratio, 1.0, 1e-12);
}

TEST(TrackConsistency, TheOldTrackIsMovedByTheCarsMotionBeforeTheComparison) {
  // On a 0.6 m radius circle (the donut), 0.1 s at 1 m/s apart: 0.1 m and 0.17 rad.
  const double r = 0.6;
  const LocalTrackData ref = arc_seen_from(1.0, r, 0.0);
  const LocalTrackData cand = arc_seen_from(1.1, r, 0.1);
  const ConsistencyResult moved = judge_consistency(cand, ref, arc_pose(r, 0.1, 0.0), kP);
  EXPECT_TRUE(moved.consistent);
  EXPECT_LT(moved.centre_jump_m, 0.01);  // the new stations lie between the old chords' ends
  // With the motion wrong by 0.3 rad the same pair is far apart: what the pose is for.
  Pose2 wrong = arc_pose(r, 0.1, 0.0);
  wrong.yaw += 0.3;
  EXPECT_GT(judge_consistency(cand, ref, wrong, kP).centre_jump_m, 0.15);
}

TEST(TrackConsistency, AJumpAloneOrACollapseAloneIsConsistent) {
  const LocalTrackData ref = straight(1.0);
  const ConsistencyResult jump = judge_consistency(outlier(1.1, 0.45, 0.75), ref, Pose2{}, kP);
  EXPECT_TRUE(jump.judged);
  EXPECT_NEAR(jump.centre_jump_m, 0.45, 1e-9);
  EXPECT_NEAR(jump.half_width_ratio, 1.0, 1e-9);
  EXPECT_TRUE(jump.consistent);
  const ConsistencyResult narrow = judge_consistency(outlier(1.1, 0.0, 0.40), ref, Pose2{}, kP);
  EXPECT_NEAR(narrow.centre_jump_m, 0.0, 1e-9);
  EXPECT_NEAR(narrow.half_width_ratio, 0.40 / 0.75, 1e-9);
  EXPECT_TRUE(narrow.consistent);
}

TEST(TrackConsistency, AJumpWithACollapseIsInconsistent) {
  const ConsistencyResult r = judge_consistency(outlier(1.1), straight(1.0), Pose2{}, kP);
  EXPECT_TRUE(r.judged);
  EXPECT_FALSE(r.consistent);
  EXPECT_NEAR(r.centre_jump_m, 0.45, 1e-9);
  EXPECT_NEAR(r.half_width_ratio, 0.40 / 0.75, 1e-9);
}

TEST(TrackConsistency, BothThresholdsAreStrict) {
  const LocalTrackData ref = straight(1.0);
  // exactly at the thresholds: consistent; just past both: not
  const double at_jump = kP.max_centre_jump_m, at_half = 0.75 * kP.min_half_width_ratio;
  EXPECT_TRUE(judge_consistency(outlier(1.1, at_jump, at_half), ref, Pose2{}, kP).consistent);
  EXPECT_TRUE(
      judge_consistency(outlier(1.1, at_jump + 1e-6, at_half), ref, Pose2{}, kP).consistent);
  EXPECT_TRUE(
      judge_consistency(outlier(1.1, at_jump, at_half - 1e-6), ref, Pose2{}, kP).consistent);
  EXPECT_FALSE(
      judge_consistency(outlier(1.1, at_jump + 1e-6, at_half - 1e-6), ref, Pose2{}, kP).consistent);
}

TEST(TrackConsistency, OnlyTheComparedStationsCount) {
  // A jump and a collapse from station 8 on are beyond stations 0 to 7.
  const ConsistencyResult r =
      judge_consistency(outlier(1.1, 0.6, 0.3, 8), straight(1.0), Pose2{}, kP);
  EXPECT_TRUE(r.consistent);
  EXPECT_NEAR(r.centre_jump_m, 0.0, 1e-9);
  TrackConsistencyParams wide = kP;
  wide.compare_stations = 16;
  EXPECT_FALSE(
      judge_consistency(outlier(1.1, 0.6, 0.3, 8), straight(1.0), Pose2{}, wide).consistent);
}

TEST(TrackConsistency, InvalidOrShortTracksAreNotJudged) {
  const LocalTrackData ref = straight(1.0);
  LocalTrackData nan = outlier(1.1);
  nan.y_m[2] = std::numeric_limits<double>::quiet_NaN();
  LocalTrackData one = outlier(1.1);
  one.valid_count = 1;
  LocalTrackData frame = outlier(1.1);
  frame.frame_id = "laser";
  for (const LocalTrackData* t : {&nan, &one, &frame}) {
    const ConsistencyResult r = judge_consistency(*t, ref, Pose2{}, kP);
    EXPECT_FALSE(r.judged);
    EXPECT_TRUE(r.consistent);
    const ConsistencyResult back = judge_consistency(ref, *t, Pose2{}, kP);
    EXPECT_FALSE(back.judged);
  }
}

// -- TrackGate -----------------------------------------------------------------------------------

namespace {
// select() with the stamps as the clock and no motion (the tracks are already in one frame).
const LocalTrackData* pick(TrackGate& g, const LocalTrackData& t, double now) {
  return g.select(&t, now, Pose2{}, true);
}
}  // namespace

TEST(TrackGate, NoTrackIsNone) {
  TrackGate g;
  EXPECT_EQ(g.select(nullptr, 1.0, Pose2{}, true), nullptr);
  EXPECT_EQ(g.last_pick(), TrackPick::kNone);
}

TEST(TrackGate, OneOutlierFrameIsBridgedForItsLifetimeThenTheNextFrameIsUsed) {
  TrackGate g;
  const LocalTrackData a = straight(1.0), bad = outlier(1.1), b = straight(1.2);
  EXPECT_EQ(pick(g, a, 1.13), &a);
  EXPECT_EQ(g.last_pick(), TrackPick::kNewest);
  const LocalTrackData* used = pick(g, bad, 1.23);
  ASSERT_NE(used, &bad);
  EXPECT_EQ(g.last_pick(), TrackPick::kBridged);
  EXPECT_EQ(used->stamp_s, 1.0);
  EXPECT_EQ(used->y_m, a.y_m);
  EXPECT_FALSE(g.last_result().consistent);
  // the same frame again (the node passes the latest track every tick): still bridged, counted once
  EXPECT_EQ(pick(g, bad, 1.29)->stamp_s, 1.0);
  EXPECT_EQ(g.bridged_frames(), 1u);
  EXPECT_EQ(pick(g, b, 1.33), &b);
  EXPECT_EQ(g.last_pick(), TrackPick::kNewest);
  EXPECT_TRUE(g.last_result().consistent);
}

TEST(TrackGate, NeverTwoFramesInARow_APersistentChangeGetsThroughOnTheNextFrame) {
  TrackGate g;
  const LocalTrackData a = straight(1.0), bad1 = outlier(1.1), bad2 = outlier(1.2),
                       bad3 = outlier(1.3);
  pick(g, a, 1.13);
  EXPECT_NE(pick(g, bad1, 1.23), &bad1);  // bridged
  EXPECT_EQ(pick(g, bad2, 1.33), &bad2);  // the change persisted: used
  EXPECT_EQ(g.last_pick(), TrackPick::kNewest);
  EXPECT_EQ(pick(g, bad3, 1.43), &bad3);  // and now it is the accepted track
  EXPECT_TRUE(g.last_result().consistent);
  EXPECT_EQ(g.bridged_frames(), 1u);
  // back to the wide straight track: inconsistent with bad3 only by width growing, not a collapse
  const LocalTrackData c = straight(1.5);
  EXPECT_EQ(pick(g, c, 1.53), &c);
}

TEST(TrackGate, TheBridgeEndsWhenTheAcceptedTrackIsTooOld) {
  TrackGate g;
  const LocalTrackData a = straight(1.0), bad = outlier(1.1);
  pick(g, a, 1.13);
  EXPECT_NE(pick(g, bad, 1.25), &bad);
  EXPECT_NE(pick(g, bad, 1.0 + kP.max_hold_age_s), &bad);         // at the bound: still bridged
  EXPECT_EQ(pick(g, bad, 1.0 + kP.max_hold_age_s + 1e-6), &bad);  // past it: the newest
  EXPECT_EQ(g.last_pick(), TrackPick::kNewest);
  EXPECT_EQ(pick(g, bad, std::numeric_limits<double>::quiet_NaN()), &bad);
}

TEST(TrackGate, AnOutlierArrivingLateIsNotBridged) {
  TrackGate g;
  const LocalTrackData a = straight(1.0), bad = outlier(1.5);
  pick(g, a, 1.13);
  EXPECT_EQ(pick(g, bad, 1.0 + kP.max_hold_age_s + 0.01), &bad);
}

TEST(TrackGate, InvalidTracksAreNeverBridgedAndBreakTheChain) {
  TrackGate g;
  LocalTrackData a = straight(1.0), nan = outlier(1.1), bad = outlier(1.2);
  nan.x_m[0] = std::numeric_limits<double>::quiet_NaN();
  pick(g, a, 1.13);
  EXPECT_EQ(pick(g, nan, 1.23), &nan);  // used as it is: the planner zeroes the confidence
  EXPECT_FALSE(g.has_accepted());
  EXPECT_EQ(pick(g, bad, 1.33), &bad);  // nothing valid to compare with
  EXPECT_TRUE(g.has_accepted());
}

TEST(TrackGate, NoBridgeWithoutDeadReckoningOrForAnOlderStampOrWhenDisabled) {
  {
    TrackGate g;
    const LocalTrackData a = straight(1.0), bad = outlier(1.1);
    pick(g, a, 1.13);
    EXPECT_EQ(g.select(&bad, 1.23, Pose2{}, false), &bad);  // the motion is not known
  }
  {
    TrackGate g;
    const LocalTrackData a = straight(1.0), bad = outlier(0.9);
    pick(g, a, 1.13);
    EXPECT_EQ(pick(g, bad, 1.23), &bad);  // a stamp that went back
  }
  {
    TrackConsistencyParams off = kP;
    off.enabled = false;
    TrackGate g(off);
    const LocalTrackData a = straight(1.0), bad = outlier(1.1);
    pick(g, a, 1.13);
    EXPECT_EQ(pick(g, bad, 1.23), &bad);
    EXPECT_EQ(g.bridged_frames(), 0u);
  }
}

TEST(TrackGate, ResetForgetsTheAcceptedTrack) {
  TrackGate g;
  const LocalTrackData a = straight(1.0), bad = outlier(1.1);
  pick(g, a, 1.13);
  g.reset();
  EXPECT_FALSE(g.has_accepted());
  EXPECT_EQ(pick(g, bad, 1.23), &bad);
}

// L2: random sequences of tracks (consistent drift, outliers, invalid frames, repeated and late
// stamps, dropouts). Whatever arrives: never two new frames bridged in a row, a bridged track is
// always the last accepted one and never older than max_hold_age_s, and the newest is replaced
// only when it was judged inconsistent.
TEST(TrackGateProperty, RandomSequencesKeepTheBounds) {
  std::mt19937 rng(20261122);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int trial = 0; trial < 200; ++trial) {
    TrackGate g;
    double now = 0.0, stamp = 0.0;
    double accepted_stamp = std::numeric_limits<double>::quiet_NaN();
    int bridged_new_in_row = 0;
    LocalTrackData current = straight(0.0);
    for (int i = 0; i < 300; ++i) {
      now += 0.02;
      const bool new_frame = u(rng) < 0.2;
      if (new_frame) {
        const double kind = u(rng);
        stamp = u(rng) < 0.03 ? stamp - 0.05 : now - 0.05 - 0.1 * u(rng);
        if (kind < 0.55) {
          current = straight(stamp, 0.5 + 0.3 * u(rng));
          for (int k = 0; k < kTrackStations; ++k) current.y_m[k] = 0.1 * (u(rng) - 0.5);
        } else if (kind < 0.9) {
          current = outlier(stamp, 0.6 * u(rng), 0.2 + 0.6 * u(rng), static_cast<int>(u(rng) * 12));
        } else {
          current = outlier(stamp);
          current.half_width_left_m[1] = std::numeric_limits<double>::infinity();
        }
      }
      if (u(rng) < 0.02) continue;  // a tick with no solve
      const bool judged_new = g.is_new_frame(&current);
      const std::uint32_t before = g.bridged_frames();
      const LocalTrackData* used = g.select(&current, now, Pose2{}, u(rng) > 0.05);
      ASSERT_NE(used, nullptr);
      if (judged_new) {
        if (g.bridged_frames() != before) {
          ++bridged_new_in_row;
        } else {
          bridged_new_in_row = 0;
        }
        ASSERT_LE(bridged_new_in_row, 1) << "trial " << trial << " tick " << i;
      }
      if (used != &current) {
        ASSERT_EQ(g.last_pick(), TrackPick::kBridged);
        ASSERT_FALSE(g.last_result().consistent);
        ASSERT_EQ(validate_track(*used), TrackValidity::Ok);
        ASSERT_EQ(used->stamp_s, accepted_stamp);
        ASSERT_LE(now - used->stamp_s, kP.max_hold_age_s);
        ASSERT_LT(used->stamp_s, current.stamp_s);
      } else {
        ASSERT_EQ(g.last_pick(), TrackPick::kNewest);
      }
      if (g.has_accepted()) accepted_stamp = g.accepted_stamp_s();
    }
  }
}

// -- in the planner ------------------------------------------------------------------------------

namespace {

PlannerConfig planner_config() {
  PlannerConfig c = rig::small_config(0.6);
  c.logic.require_gated_for_start = false;
  c.logic.start_ticks = 10;
  return c;
}

// A rig whose track is chosen per tick: `next` replaces the good track the rig would make.
struct TrackRig : rig::Rig {
  explicit TrackRig(PlannerConfig cfg = planner_config()) : rig::Rig(cfg) {}
  TickOutput step(double now, const LocalTrackData* next) {
    TickInput in = input(now);
    if (next != nullptr) {
      track = *next;
      in.track = &track;
    }
    return planner.tick(in);
  }
};

LocalTrackData corridor_outlier(double stamp) {
  LocalTrackData t = rig::good_track(stamp);
  for (int k = 4; k < kTrackStations; ++k) {
    t.y_m[k] = 0.45;  // swings to the wall
    t.half_width_left_m[k] = 0.25;
    t.half_width_right_m[k] = 0.25;
  }
  t.boundary_residual_m = 0.25;  // and its own self-check is poor, as on the car
  return t;
}

}  // namespace

TEST(PlannerTrackBridge, OneBadFrameIsBridgedAndCostsNoConfidence) {
  TrackRig r;
  double t = 1.0;
  TickOutput out;
  for (int i = 0; i < 80; ++i, t += 0.02) out = r.step(t, nullptr);
  ASSERT_EQ(out.state, PlannerState::Driving);
  const double conf_before = out.track_confidence;
  ASSERT_GT(conf_before, 0.99);
  const LocalTrackData bad = corridor_outlier(t - 0.1);
  for (int i = 0; i < 5; ++i, t += 0.02) {  // one track period of the outlier
    out = r.step(t, &bad);
    EXPECT_TRUE(out.track_bridged);
    EXPECT_EQ(out.state, PlannerState::Driving);
    EXPECT_GE(out.track_confidence, conf_before - 1e-9);
    EXPECT_GT(out.cmd.speed_mps, 0.0);
  }
  out = r.step(t, nullptr);  // a good frame again
  EXPECT_FALSE(out.track_bridged);
  EXPECT_EQ(r.planner.bridged_track_frames(), 1u);

  // The same frame without the bridge: the confidence falls at once (the residual).
  PlannerConfig off = planner_config();
  off.consistency.enabled = false;
  TrackRig o(off);
  double t2 = 1.0;
  for (int i = 0; i < 80; ++i, t2 += 0.02) o.step(t2, nullptr);
  const LocalTrackData bad2 = corridor_outlier(t2 - 0.1);
  const TickOutput raw = o.step(t2, &bad2);
  EXPECT_FALSE(raw.track_bridged);
  EXPECT_LT(raw.track_confidence, 0.5 * conf_before);
}

TEST(PlannerTrackBridge, APersistentChangeIsUsedFromTheSecondFrame) {
  TrackRig r;
  double t = 1.0;
  for (int i = 0; i < 80; ++i, t += 0.02) r.step(t, nullptr);
  const LocalTrackData bad1 = corridor_outlier(t - 0.1);
  EXPECT_TRUE(r.step(t, &bad1).track_bridged);
  t += 0.1;
  const LocalTrackData bad2 = corridor_outlier(t - 0.1);
  const TickOutput out = r.step(t, &bad2);
  EXPECT_FALSE(out.track_bridged);
  EXPECT_LT(out.track_confidence, 0.5);  // the narrowed track and its residual are now in force
}

TEST(PlannerTrackBridge, AWallInTheScanStopsTheCarWhileATrackFrameIsBridged) {
  // The car drives the corridor, then a wall appears across it 0.15 m ahead of the bumper in the
  // scan, and the
  // perception sends an outlier frame at the same moment. The bridge keeps the old (clear) track;
  // the collision cost reads the scan, so the planner stops all the same, as it does without the
  // bridge.
  for (const bool bridge : {true, false}) {
    PlannerConfig cfg = planner_config();
    cfg.consistency.enabled = bridge;
    TrackRig r(cfg);
    double t = 1.0;
    TickOutput out;
    for (int i = 0; i < 80; ++i, t += 0.02) out = r.step(t, nullptr);
    ASSERT_EQ(out.state, PlannerState::Driving);
    std::vector<PointXY> walled = rig::corridor_points();
    const double wall_x = r.g.body_max_x_m() + 0.15;
    for (double y = -0.5; y <= 0.5; y += 0.02) walled.push_back({wall_x, y});
    r.set_points(walled);
    const LocalTrackData bad = corridor_outlier(t - 0.1);
    bool stopped = false;
    for (int i = 0; i < 5; ++i, t += 0.02) {
      out = r.step(t, &bad);
      EXPECT_EQ(out.track_bridged, bridge);
      if (out.cmd.speed_mps == 0.0) stopped = true;
    }
    EXPECT_TRUE(stopped) << "bridge " << bridge;
    EXPECT_NE(out.state, PlannerState::Driving) << "bridge " << bridge;
  }
}
