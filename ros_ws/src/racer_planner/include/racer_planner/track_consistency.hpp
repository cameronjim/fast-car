// Frame to frame consistency of the local track (fix checklist 2026-11-22, item S2).
//
// On the car the perception now and then sends one local track that disagrees with the frames on
// either side of it: the centreline a few stations ahead jumps sideways by 0.3 to 0.6 m and the
// half width there collapses (0.39 to 0.52 m against 0.75 m), and the next frame is back to normal.
// The planner followed that one frame, the progress of every plan collapsed and it braked to zero
// for one to six ticks (BLOCKED, below the speed floor).
//
// The rule: a new track is compared with the last accepted track moved into the new track's frame
// by the planner's own dead reckoning. It is inconsistent when BOTH its centreline jumps (some
// station among the first `compare_stations` is further than `max_centre_jump_m` from the
// accepted centreline) AND its half width collapses (the smallest half width over the same
// stations is under `min_half_width_ratio` times the accepted track's). The planner then keeps
// driving on the accepted track for that one frame (TrackGate). Never more than one frame in a row:
// the next new track is accepted whatever it shows, so a change that persists (a real narrowing,
// a real bend) costs at most one track period. Never older than `max_hold_age_s`: past it the
// newest track is used as it is. The collision cost, the clearance cost and the staleness checks
// never read the track (claude-docs/13 section 13.4): a real obstacle in the scan still slows or
// stops the car whatever the track says, and safety_node still gates /drive_raw.
#ifndef RACER_PLANNER_TRACK_CONSISTENCY_HPP_
#define RACER_PLANNER_TRACK_CONSISTENCY_HPP_

#include <cstdint>

#include "racer_planner/local_track.hpp"

namespace racer_planner {

struct TrackConsistencyParams {
  bool enabled = true;
  // Thresholds from the 2026-11-22 bag (4312 track frames while DRIVING; the frames just before
  // the 10 blips that had an outlier: jump 0.30 to 0.51 m, width ratio 0.44 to 0.69; all frames:
  // jump p50 0.05 m, p95 0.30 m). Both must hold.
  double max_centre_jump_m = 0.25;
  double min_half_width_ratio = 0.75;
  int compare_stations = 8;  // stations 0 to 7: 1.4 m, about the 1.5 s horizon at 1 m/s
  // The accepted track may stand in for a newer one only while it is at most this old (s).
  double max_hold_age_s = 0.40;
};

struct ConsistencyResult {
  bool judged = false;            // both tracks had 2 or more usable stations
  bool consistent = true;         // false only when judged and both conditions hold
  double centre_jump_m = 0.0;     // largest distance of a new station from the old centreline
  double half_width_ratio = 1.0;  // smallest new half width over smallest old one
};

// Compares `candidate` with `reference` (the last accepted track). `reference_in_candidate` is
// the pose of the reference track's frame origin in the candidate's frame (a point p of the
// reference becomes R(yaw) p + (x, y)). Both tracks must pass validate_track, else the result is
// not judged (and consistent). Pure function.
ConsistencyResult judge_consistency(const LocalTrackData& candidate,
                                    const LocalTrackData& reference,
                                    const Pose2& reference_in_candidate,
                                    const TrackConsistencyParams& p);

// What TrackGate decided for the newest track.
enum class TrackPick : std::uint8_t {
  kNone = 0,     // no track
  kNewest = 1,   // the newest track is used
  kBridged = 2,  // the newest track is inconsistent: the last accepted one is used instead
};

// Keeps the last accepted track and decides, once per new track frame (a new stamp), whether the
// planner uses it or bridges it with the accepted one. The accepted copy lives in storage reserved
// at construction, so a judgement never allocates (the solve path must not). Solve thread only.
class TrackGate {
 public:
  explicit TrackGate(const TrackConsistencyParams& p = TrackConsistencyParams{});

  // `newest` is the latest track (null when none). `accepted_in_newest` is the pose of the accepted
  // track's frame in the newest track's frame and `covered` says whether the dead reckoning reached
  // back to the accepted track's stamp; both are only read on a new frame. Returns the track the
  // planner should use (null when none).
  const LocalTrackData* select(const LocalTrackData* newest, double now_s,
                               const Pose2& accepted_in_newest, bool covered);

  // The stamp of the accepted track, valid when has_accepted(): what the caller dead-reckons from.
  bool has_accepted() const { return have_accepted_; }
  double accepted_stamp_s() const { return accepted_.stamp_s; }
  // True when `newest` is a frame not judged yet (the caller then supplies the pose).
  bool is_new_frame(const LocalTrackData* newest) const;

  TrackPick last_pick() const { return pick_; }
  const ConsistencyResult& last_result() const { return result_; }
  std::uint32_t bridged_frames() const { return bridged_total_; }
  void reset();

 private:
  TrackConsistencyParams p_;
  LocalTrackData accepted_;
  bool have_accepted_ = false;
  bool have_judged_ = false;
  double judged_stamp_s_ = 0.0;
  bool bridging_ = false;  // the frame being used now is bridged
  int bridged_in_row_ = 0;
  TrackPick pick_ = TrackPick::kNone;
  ConsistencyResult result_;
  std::uint32_t bridged_total_ = 0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_TRACK_CONSISTENCY_HPP_
