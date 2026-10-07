// Reverse escape for gap_follow_node (2026-10-06 night floor finding (b), bags
// 2026-10-06T23-19-41 and 23-25-38). ROS-free and gtest-covered (test/test_reverse_escape.cpp);
// gap_follow_node feeds it once per control cycle and owns the ROS plumbing.
//
// THE PROBLEM. In both lap directions the car ended nose-in to a corner tighter than its 0.8 m
// turning circle (full lock 0.4189 rad on the 0.3302 m wheelbase). safety_node's arc corridor
// braked correctly; then no steering angle gave a clear forward arc (an object 0.2 m ahead and
// 0.16 m off the front corner), so the obstacle latch held for ever. The follower never
// reversed, and until the same night safety_node could not have judged a reverse request.
//
// WHAT THIS DOES. A small state machine, off unless gap_follow_node's `reverse_escape` is true:
//
//   kIdle -> kEscaping   forward-blocked for escape_after_s AND the forward path is blocked
//                        (or kIdle -> kExhausted when escape_max_attempts are used up)
//   kEscaping -> kIdle   escape_distance_m covered (completed) or escape_max_s elapsed
//   kEscaping -> kRetryWait   rear blocked for block_debounce_s (aborted)
//   kRetryWait -> kIdle  escape_retry_after_s elapsed
//   kExhausted -> kIdle  forward progress (see ATTEMPTS)
//
//   * FORWARD-BLOCKED means safety_node is refusing this node's forward request: the node's own
//     request last cycle was > 0 and the gated /drive speed it sees is zero. The gated /drive is
//     the authority (it is what the car is actually told); the node never guesses the gate.
//     Stale /drive (no safety_node) never counts as blocked, so without safety_node in the loop
//     the escape never fires.
//   * THE FORWARD PATH IS BLOCKED is the follower's own view (EscapeInput::forward_path_blocked,
//     computed by the node): its corner override fired, or no steering gives a clear forward arc
//     for escape_probe_distance_m (gap_follow.hpp any_forward_arc_clear). Both are needed: a gate
//     that holds the car while the follower still sees a way through (a person stepping in
//     front, then away) is waited out, not reversed from.
//   * ESCAPING commands escape_speed_mps (negative), its magnitude ramped up at
//     max_acceleration_mps2 like the forward speed, with the steering at escape_steering_rad
//     OPPOSITE to the sign of the follower's wanted forward steering when the escape started.
//     Backing up with the wheels turned the other way swings the nose TOWARD the way the
//     follower was trying to turn (yaw rate = v tan(delta) / L, both signs flipped), i.e. away
//     from the outside of the corner it is nosed into: the second leg of a three-point turn.
//     A zero wanted steering backs straight out.
//   * DISTANCE has no odometry to come from (no /odom yet): it is the integral of the COMMANDED
//     escape speed magnitude over time. ASSUMPTION, stated: the car moves at the commanded
//     speed. It does not, quite: the VESC does nothing below about 0.44 m/s (s_pid_min_erpm), so
//     the first part of the ramp is counted but not driven, and a brake from the gate stops it
//     early. The real escape is therefore SHORTER than escape_distance_m, never longer, and
//     escape_max_s bounds it in time either way.
//   * REAR BLOCKED means safety_node refused the reverse request: the node's own request last
//     cycle was < 0 and the gated /drive speed is not negative (zero), or /drive went stale.
//     It must persist for block_debounce_s before it counts, because the gated /drive lags the
//     request by a cycle or two through safety_node at the start of every escape. Then the
//     escape is ABORTED and the machine waits escape_retry_after_s before it may try again.
//   * ATTEMPTS count escapes started since the car last made forward progress: the gated
//     forward speed integrated over time since the last escape ended reaching
//     escape_distance_m (it drove forward at least as far as it backed). After
//     escape_max_attempts without progress the machine HOLDS (kExhausted): no more escapes, the
//     node drives the follower as usual and safety_node's latch holds the car, exactly the
//     behaviour before this existed, until forward progress resets the count.
//
// Time is the configured control period summed per call (EscapeInput::dt_s), the same "no
// measured dt" rule gap_follow_node's filters follow. Every transition is reported once
// (EscapeOutput::event) so the node can log it at INFO.
//
// SAFETY. This never publishes anything: gap_follow_node publishes /drive_raw only and
// safety_node gates it into /drive like every other command (CLAUDE.md invariant 1). The rear
// corridor in safety_node is what keeps an escape from backing into something; this state
// machine only reacts to safety_node's refusal.
#ifndef RACER_CONTROL_REVERSE_ESCAPE_HPP_
#define RACER_CONTROL_REVERSE_ESCAPE_HPP_

#include <optional>

namespace racer_control {

struct ReverseEscapeConfig {
  double escape_after_s = 1.5;         // forward-blocked this long before an escape may start
  double escape_speed_mps = -0.5;      // < 0; the VESC needs at least about 0.44 m/s
  double escape_steering_rad = 0.0;    // magnitude: steering.max_angle_rad from the binding
  double escape_distance_m = 0.4;      // commanded-speed integral that ends an escape
  double escape_max_s = 2.0;           // time limit of one escape
  double escape_retry_after_s = 3.0;   // wait after an aborted escape
  int escape_max_attempts = 3;         // escapes without forward progress before holding
  double block_debounce_s = 0.1;       // a rear block must persist this long to abort
  double max_acceleration_mps2 = 0.0;  // reverse speed magnitude ramp (<= 0: step)
};

enum class EscapeState { kIdle, kEscaping, kRetryWait, kExhausted };

enum class EscapeEvent {
  kStarted,     // an escape began
  kCompleted,   // escape_distance_m covered
  kTimedOut,    // escape_max_s elapsed first
  kAborted,     // safety_node braked the reverse request (rear corridor)
  kRetryReady,  // escape_retry_after_s elapsed after an abort
  kExhausted,   // escape_max_attempts without forward progress: holding
  kReset,       // forward progress reset the attempt count (also ends kExhausted)
};

struct EscapeInput {
  double dt_s = 0.0;  // the configured control period, 0 on the first cycle
  // What the node published as /drive_raw LAST cycle (the request safety_node judged).
  double requested_speed_mps = 0.0;
  // The latest gated /drive speed while it is fresh; nullopt when stale or never received.
  std::optional<double> gated_speed_mps;
  // The follower's view: corner override fired, or no steering gives a clear forward arc.
  bool forward_path_blocked = false;
  // The follower's wanted forward steering (its sign picks the escape steering).
  double forward_steering_rad = 0.0;
};

struct EscapeOutput {
  bool active = false;  // command speed_mps / steering_rad this cycle instead of the follower
  double speed_mps = 0.0;
  double steering_rad = 0.0;
  std::optional<EscapeEvent> event;  // a transition on this cycle
};

class ReverseEscape {
 public:
  explicit ReverseEscape(const ReverseEscapeConfig& config) : config_(config) {}
  EscapeOutput update(const EscapeInput& input);
  // Back to kIdle with all timers cleared, the attempt count kept (gap_follow_node calls this
  // when it stops publishing on /scan silence, so an escape never resumes after a gap).
  void reset();

  EscapeState state() const { return state_; }
  int attempts() const { return attempts_; }
  // For the transition log lines: the current or last escape's figures.
  double escape_distance_m() const { return distance_m_; }
  double escape_elapsed_s() const { return elapsed_s_; }
  double escape_steering_rad() const { return steering_rad_; }
  double forward_blocked_s() const { return forward_blocked_s_; }
  const ReverseEscapeConfig& config() const { return config_; }

 private:
  void track_forward(const EscapeInput& input, EscapeOutput& out);
  void start(const EscapeInput& input, EscapeOutput& out);
  void leave_escape(EscapeState next, EscapeEvent event, EscapeOutput& out);

  ReverseEscapeConfig config_;
  EscapeState state_ = EscapeState::kIdle;
  int attempts_ = 0;
  double forward_blocked_s_ = 0.0;
  double forward_progress_m_ = 0.0;
  double distance_m_ = 0.0;
  double elapsed_s_ = 0.0;
  double speed_magnitude_mps_ = 0.0;
  double steering_rad_ = 0.0;
  double rear_blocked_s_ = 0.0;
  double retry_wait_s_ = 0.0;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_REVERSE_ESCAPE_HPP_
