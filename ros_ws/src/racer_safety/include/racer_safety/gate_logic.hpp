// racer_safety gate/decision logic (roadmap milestone 1, claude-docs/05-safety.md layer 3).
//
// ROS-free (no rclcpp) so it is gtest-unit-testable with no ROS install
// (claude-docs/12-testing.md L1: "racer_safety gate logic (separated from node plumbing
// precisely so it is testable without ROS)"; claude-docs/10-conventions.md: "Gate/decision
// logic is always separated from node plumbing"). `safety_node` (src/safety_node.cpp) owns
// all ROS plumbing (subscribing /drive_raw and /scan, publishing /drive and
// /safety/events) and calls `SafetyGateLogic::evaluate` once per cycle; nothing here
// allocates on the heap (claude-docs/10-conventions.md: "No heap allocation in the 50 Hz
// control path after init") except for the (small, bounded) `activations` vector returned
// per cycle and the transition records GateEventTracker returns from it, both of which only
// grow when an intervention actually fires.
//
// Design (see also this file's neighboring test/test_gate_logic.cpp for the full
// table-driven pass/marginal/fail/garbage matrix):
//
//   1. Watchdog: if `drive_raw_age_s` has exceeded `watchdog_missed_cycles *
//      control_period_s` (claude-docs/04-architecture.md: "Watchdog: missing /drive_raw for
//      3 cycles -> brake command"), the cycle short-circuits immediately to zero speed with
//      the PREVIOUS cycle's steering angle held -- there is no valid fresh command to reason
//      about further. See "STEERING ON A ZERO-THROTTLE GATE" below for why the steering is
//      held rather than centred.
//   2. Command sanity: a non-finite (NaN/Inf) steering angle or speed in the incoming
//      command is garbage, not a bounds violation -- it short-circuits the same way
//      (claude-docs/05-safety.md: "Fails CLOSED: any internal error -> brake command, not
//      passthrough" -- garbage input is exactly the kind of thing that must not be clamped
//      into something that LOOKS sane and get passed through).
//
// STEERING ON A ZERO-THROTTLE GATE. Every gate in this file that forces the speed to zero
// leaves the steering angle at the last value this logic actually COMMANDED
// (`previous_output.steering_angle_rad`), rather than snapping it to centre. Three reasons,
// in order of weight:
//
//   * Centring is a step input. The steering rate limiter in step 3b exists because the rack
//     cannot be slewed arbitrarily fast and because a step in road-wheel angle at speed is a
//     yaw disturbance. Writing 0.0 straight into the output bypasses that limiter and
//     commands up to full lock of travel in one cycle -- from the one code path whose job is
//     to make the car safer.
//   * Mid-corner, centring straightens a cornering car. On a watchdog trip (the tracker died,
//     the teleop tab closed) the car is still travelling along its current arc, and layer 3
//     cannot stop it (the note on GateResult: zero speed is a coast on this hardware). Holding
//     the arc while the drive current goes to zero keeps it on roughly the path it was on;
//     straightening sends it to the outside of the corner.
//   * Consistency. The TTC gate has always zeroed the speed and left the steering alone
//     (test_gate_logic.cpp's "...BrakesAndZeroesSpeedOnlyKeepsSteering"). The watchdog and
//     sanity paths used to do the opposite. One rule for all of them is one fewer thing to
//     get wrong.
//
// At startup `previous_output` is {0, 0}, so a node that has never commanded anything still
// emits exactly {0, 0} -- the runbook's "verify /drive is neutral with no input" step and its
// L3 test are unaffected.
//   3. Otherwise, in this order:
//      3a. absolute bounds clamp (steering angle, speed) against vehicle_params limits. The
//          result is the REQUESTED command.
//      3b. the obstacle gate (claude-docs/05-safety.md: "TTC braking from /scan"): a TTC
//          brake evaluated on the REQUESTED forward speed, plus a distance floor, sharing one
//          latch. While latched, forward speed is forced to zero. In the warning zone, with
//          no latch, it emits an advisory-only event with no command change. See "THE
//          OBSTACLE GATE AND ITS LATCH" below. Then the same gate mirrored for a REVERSE
//          request on the rear corridor, with its own latch: see "THE REAR OBSTACLE GATE".
//      3c. a rate-limit clamp against the PREVIOUS cycle's output (steering rate, and speed
//          magnitude increase only -- braking toward zero, forward or reverse, is never
//          rate-limited), applied to whatever 3b left, so the rate limit holds on everything
//          that is finally output. See "SPEED RATE LIMIT IN BOTH DIRECTIONS" below.
//      3d. the steering hold: once the obstacle latch has held the output speed at zero for
//          SafetyLimits::obstacle_steering_hold_after_s, the steering output is frozen at
//          the angle it had when the hold started, until the latch releases. See "STEERING
//          HOLD WHILE PARKED ON THE OBSTACLE LATCH" below.
//   4. Covariance gate: STUB. No /pose source exists yet (roadmap task 2.6); see
//      `evaluate_covariance_gate` below. Per this milestone's instructions, the stub must
//      fail SAFE, meaning absent pose input must not disable any of the other gates above --
//      it is evaluated independently and never gates whether steps 1-3 run (see
//      test_gate_logic.cpp's "covariance stub does not disable other gates" cases).
//
// THE OBSTACLE GATE AND ITS LATCH (rewritten 2026-10-06; read before changing step 3b).
//
// The bug. Until 2026-10-06 the order was bounds clamp, then rate limit, then TTC, and the
// TTC check divided the range by the rate-limited OUTPUT speed. On the car that evening
// (wheels off the ground, gap_follow_node requesting a steady ~0.48 m/s, an obstacle ~0.22 m
// ahead, 50 Hz, max_acceleration_mps2 9.51 so the limiter allows +0.19 m/s per cycle) the
// output cycled 0.00 -> 0.19 -> 0.38 -> 0.00 -> ... : TTC braked the output to 0; on the next
// cycle the limiter ramped it from 0 to 0.19 (TTC 1.16 s, passes), then 0.38 (TTC 0.58 s,
// passes), and the next step to 0.57 (TTC 0.39 s) braked again. A 3-cycle limit cycle, 780
// brake/release flips in 286 s, the motor pulsing at about 17 Hz. The gate was judging the
// command it had itself shrunk, not the command it had been asked to pass.
// docs/notes/ttc-limit-cycle-2026-10-06.md has the bag evidence.
//
// The fix, three parts:
//   * TTC is computed from the REQUESTED forward speed (the bounds-clamped input), before
//     the rate limiter, so a request that violates TTC is rejected on every cycle rather
//     than every third. The rate limiter then runs on whatever the obstacle gate leaves, so
//     the output is still rate-limited. This is also the conservative choice: for a forward
//     command the output speed never exceeds the request (the limiter only slows increases,
//     and decreases pass at once), so request-TTC <= output-TTC.
//   * A latch with hysteresis. Once the gate brakes it stays braked (forward speed 0) until
//     the request's TTC EXCEEDS a release threshold above the brake threshold: ttc_warning_s
//     when it is set and larger than ttc_brake_s, otherwise ttc_brake_s *
//     SafetyLimits::ttc_release_hysteresis_factor (>= 2, set in the gate config, not here).
//     Without hysteresis a noisy range near the threshold would still flicker the brake.
//     Release emits a "ttc brake released" note (GateResult::releases) that becomes the
//     detail of the PHASE_RELEASE record.
//   * A distance floor (SafetyLimits::min_forward_clearance_m, vehicle_params
//     limits.min_forward_clearance_m). If the in-path distance is below it and
//     the request is forward at any speed, the gate brakes, on the same latch, and does not
//     release until the range exceeds floor * SafetyLimits::clearance_release_factor. TTC
//     alone cannot do this: a crawl-speed request makes TTC large however close the
//     obstacle is.
//
// Latch details, each one deliberate:
//   * Trip always wins: a cycle whose request trips TTC or the floor is latched whatever the
//     release test says, so a misconfigured release factor can never produce an output pulse.
//   * Only FORWARD speed is held at zero. A zero or reverse request passes (bounds- and
//     rate-limited as usual): backing away from an obstacle is not moving toward it, and it
//     is the one thing an operator needs to be able to do while latched. A non-forward
//     request also clears the TTC half of the release test (its TTC is infinite); the floor
//     half still holds while the obstacle is inside the release clearance.
//   * Garbage range (NaN, zero, negative) never trips, and never RELEASES a latch either:
//     fail closed. +infinity is not garbage, it is "nothing in the path" and counts as
//     clear.
//   * The latch is state, but evaluate() stays a pure function: the previous cycle's latch
//     comes in on GateInput::ttc_brake_latched and the new one goes out on
//     GateResult::ttc_brake_latched, threaded by safety_node exactly like previous_output.
//     The watchdog, command-sanity and internal-fault paths HOLD the latch (they do not run
//     the gate, so they cannot judge a release) and keep reporting its activation so the
//     engagement is not split by an unrelated short-circuit.
//   * The in-path distance itself is computed by forward_sector.hpp. Since the 2026-10-06
//     floor test it is a CORRIDOR, not a wedge: a return counts only if it is ahead of the
//     car and within chassis.width_m / 2 + limits.obstacle_corridor_margin_m of the path
//     (the primary filter), and within +/- limits.ttc_forward_sector_half_angle_rad of the
//     car's +x axis after sensors.lidar.mount_yaw_rad (an outer bound only), invalid returns
//     ignored. Since the late floor test the path is the ARC the REQUESTED steering sweeps
//     (straight only for a straight request) and the distance is the arc length from the
//     LiDAR head, never the slant range. Because that distance depends on the request,
//     safety_node reduces the last /scan on EVERY gate cycle with that cycle's requested
//     steering and passes the result in GateInput::min_scan_range_m: so even while latched,
//     and while the steering hold has frozen the output, the obstacle check is judged on the
//     path the car is being asked to take, which is what lets a latched car release by
//     steering away (bag 2026-10-06T22-12-40_car_teleop). Nothing in this file changed for
//     that: the gate still just reads GateInput::min_scan_range_m, and
//     test_arc_corridor.cpp composes the two exactly the way safety_node does.
//
// THE REAR OBSTACLE GATE (added 2026-10-06 night; read before changing step 3b).
//
// Why. On the night floor test (bags 2026-10-06T23-19-41 and 23-25-38) the car ended nose-in
// to a corner tighter than its turning circle in both lap directions: the forward gate braked
// correctly, no steering gave a clear forward arc, and the latch held forever. The follower
// never reversed, and this gate had no rear check, so nothing would have judged a reverse
// request anyway. gap_follow_node now has a reverse escape (racer_control), and a reverse
// request is judged here exactly like a forward one.
//
// What. GateInput::min_rear_scan_range_m is the in-path distance behind the car along the arc
// of the requested steering, from the rear bumper line (forward_sector.hpp "REAR CORRIDOR";
// safety_node recomputes it every cycle like the forward one). The rear gate is the forward
// gate with the direction flipped: TTC = rear distance / |requested speed| for a reverse
// request (a forward or zero request has infinite rear TTC), the same ttc_brake_s,
// ttc_warning_s, min_forward_clearance_m floor (applied to the rear distance), release
// hysteresis, trip-wins rule and garbage handling. While its latch is set, REVERSE speed is
// held at zero; forward and zero requests pass.
//
// Two latches, not one latch keyed by direction. The forward latch (GateInput/GateResult
// ttc_brake_latched) and the rear latch (reverse_brake_latched) are independent: each trips,
// holds and releases on its own corridor with its own hysteresis, and each only ever zeroes
// motion toward its own side. So the forward latch never blocks a reverse request (backing
// out is exactly what a car parked nose-in needs) and the rear latch never blocks a forward
// one. A single latch keyed by direction would have to drop or re-key the forward latch the
// moment a reverse request arrives and judge the forward release afresh when the request
// turns forward again, losing the hysteresis that exists to stop flicker; and a car boxed in
// front and back needs both held at once. They are also separate engagements on
// /safety/events: the rear gate reports GateSource::kTtcReverse ("ttc_reverse"), so a car
// latched both ways is two interventions, each with its own engage and release record, and
// neither release can be hidden by the other still being engaged.
//
// The steering hold below stays tied to the FORWARD latch only, as before: a car parked by the
// rear latch is one whose reverse request was refused, and the requester (the escape, or an
// operator) holds a steady steering while it waits.
//
// SPEED RATE LIMIT IN BOTH DIRECTIONS (2026-10-06 night, step 3c). The rate limiter used to
// compare signed speeds: any increase was limited and any decrease passed. That is right going
// forward, but backwards it limited BRAKING (-0.5 -> 0 is an increase) and passed any reverse
// ACCELERATION (0 -> -5 is a decrease). With the rear gate a reverse brake must reach the
// output at once, exactly like a forward one, so the limiter now works on the speed's
// magnitude: a change toward zero passes at once in either direction, growth away from zero
// in either direction is limited to max_acceleration_mps2 * dt, and a request that crosses
// zero is a brake to zero (free) followed by growth from zero (limited). Forward behaviour is
// unchanged bit for bit (previous output and request both >= 0).
//
// STEERING HOLD WHILE PARKED ON THE OBSTACLE LATCH (added 2026-10-06 late; read before
// changing step 3d).
//
// Why. A TTC or clearance brake zeroes forward throttle and keeps steering live, and that
// stays true: a car braking at speed must keep steering authority, so for the first part of a
// latched episode the steering passes exactly as it always has (bounds-clamped and
// rate-limited, test_obstacle_gate.cpp's "SteeringIsKeptAndStillRateLimitedDuringALatchedBrake").
// But a car PARKED against an obstacle has nothing to steer for, and on the 2026-10-06 stand
// run with gap_follow_node the owner watched the servo hunt back and forth while the latch
// held the car still, following a steering request that wandered as the planner looked for a
// gap it could not take. So: once the latch has held the output speed at zero for
// SafetyLimits::obstacle_steering_hold_after_s (vehicle_params
// limits.obstacle_steering_hold_after_s, PROVISIONAL 0.5 s), the steering output is frozen at
// the angle it had when the hold started (the previous cycle's output), and stays frozen until
// the latch releases. Braking at speed keeps steering; a car parked against an obstacle stops
// twitching its servo.
//
// Details, each one deliberate:
//   * The timer is the gate's own dt accounting (the same safe_dt the rate limiter uses), not
//     a wall clock: GateInput::obstacle_hold_timer_s comes in, GateResult::
//     obstacle_hold_timer_s goes out, threaded by safety_node like the latch. It is
//     std::nullopt whenever the latch is not holding the output at zero; it starts at 0.0 on
//     the first cycle that does and grows by dt on each further one. The hold engages on the
//     first cycle where it is >= the configured time.
//   * "Held at zero" means the final OUTPUT speed is exactly 0.0 while the latch is set. A
//     reverse request passes while latched (backing away is allowed), the output is then not
//     zero, so the timer resets and the hold lets go: a car moving backwards gets its
//     steering back. Garbage timer input (non-finite or negative) restarts the timer rather
//     than holding on nonsense; that is today's behaviour (steering live), not a new risk.
//   * Zero speed on /drive is a coast (GateResult note below) and safety_node has no /odom, so
//     the hold time is the stand-in for "the car has actually stopped". It must stay longer
//     than the car takes to coast to rest from the speeds it is driven at.
//   * Watchdog, command-sanity and internal-fault short-circuits keep their own behaviour
//     (zero speed, previous steering held). They do not run the gate, so, like the latch,
//     they HOLD the timer as it came in (no advance, no reset) when the latch is set, and keep
//     reporting the hold engagement if it was engaged, so a one-cycle blip does not split it.
//     With no latch they clear the timer and the hold does not apply.
//   * Events: the hold is reported as a (kTtc, kInfo) activation, "steering held while
//     obstacle-latched" with the held angle in the detail, on every cycle it is engaged, so
//     GateEventTracker publishes ONE PHASE_ENGAGE when it engages and one PHASE_RELEASE
//     (detail "steering hold released") when it ends, normally on the same cycle as the
//     latch's own "ttc brake released" record. It shares (kTtc, kInfo) with the TTC warning
//     advisory, which is emitted only while NOT latched, so the two can never be engaged on
//     the same or adjacent cycles: the hold needs the latch, and the latch releases only when
//     the request's TTC is above the warning threshold.
//   * Unset (SafetyLimits default +infinity) never holds. safety_node always sets it from the
//     binding; only tests leave it unset, so the pre-existing suites see no hold.
//
// Every field that would naturally come from `config/vehicle_params.yaml` is threaded in
// through `SafetyLimits`, populated ONLY from the generated vehicle_params C++ binding by
// safety_node.cpp (CLAUDE.md invariant 2: never hand-write a physical constant) -- this
// header and its .cpp never include the generated binding themselves, exactly like
// racer_control's core/tracker_node split.
#ifndef RACER_SAFETY_GATE_LOGIC_HPP_
#define RACER_SAFETY_GATE_LOGIC_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace racer_safety {

// Bounds and tuning thresholds the gate logic enforces. Every physical bound here comes
// from the generated vehicle_params binding (never hand-typed) except `watchdog_missed_cycles`
// and `control_period_s`, which are node-level tuning (the watchdog cadence), not vehicle
// physics. `ttc_warning_s`/`ttc_brake_s` are `std::optional` because
// config/vehicle_params.yaml's `limits.ttc_warning_s`/`limits.ttc_brake_s` are currently
// `null` (not yet tuned -- see that file's comments); when unset, the TTC gate is a documented
// no-op (see evaluate()'s doc comment) rather than inventing an untuned threshold.
struct SafetyLimits {
  double steering_min_rad = 0.0;
  double steering_max_rad = 0.0;
  double steering_rate_min_rad_per_s = 0.0;
  double steering_rate_max_rad_per_s = 0.0;
  double speed_min_mps = 0.0;          // reverse cap, vehicle_params limits.min_velocity_mps (<= 0)
  double speed_max_mps = 0.0;          // vehicle_params limits.global_speed_cap_mps
  double max_acceleration_mps2 = 0.0;  // vehicle_params actuation.max_acceleration_mps2
  std::optional<double> ttc_warning_s;
  std::optional<double> ttc_brake_s;
  // vehicle_params limits.min_forward_clearance_m (schema 0.7.0, required there, so
  // safety_node always sets it). std::nullopt disables the floor; only tests do that, to
  // exercise the TTC half of the gate on its own.
  std::optional<double> min_forward_clearance_m;
  // Hysteresis of the obstacle-gate latch (see "THE OBSTACLE GATE AND ITS LATCH" above).
  // These are gate tuning, not vehicle physics, so they live here rather than in
  // vehicle_params; safety_node does not override them.
  //   * TTC release threshold when ttc_warning_s is unset (or not above ttc_brake_s):
  //     ttc_brake_s * ttc_release_hysteresis_factor. Must be >= 2 (task requirement,
  //     2026-10-06) and is checked by test_gate_logic.cpp. A non-finite value makes the
  //     threshold NaN, which never releases: fail closed.
  //   * Floor release clearance: min_forward_clearance_m * clearance_release_factor.
  double ttc_release_hysteresis_factor = 2.0;
  double clearance_release_factor = 1.5;
  // vehicle_params limits.obstacle_steering_hold_after_s (schema 0.7.3, required there, so
  // safety_node always sets it): how long the obstacle latch must have held the output speed
  // at zero before the steering output is frozen (see "STEERING HOLD WHILE PARKED ON THE
  // OBSTACLE LATCH" above). The default +infinity never holds; only tests rely on it.
  double obstacle_steering_hold_after_s = std::numeric_limits<double>::infinity();
  int watchdog_missed_cycles = 3;
  double control_period_s = 0.02;  // 1 / 50 Hz, claude-docs/04-architecture.md
};

struct DriveCommand {
  double steering_angle_rad = 0.0;
  double speed_mps = 0.0;
};

// Which gate raised a given SafetyEvent -- mirrors racer_msgs/SafetyEvent.msg's `source`
// string field one-to-one (see gate_source_to_string below).
enum class GateSource {
  kWatchdog,
  kCommandSanity,
  kBoundsClamp,
  kRateLimit,
  kTtc,
  kCovariance,
  kInternalFault,
  // The rear obstacle gate (2026-10-06 night, "THE REAR OBSTACLE GATE"): a reverse request
  // judged on the rear corridor. A separate source so it is a separate engagement.
  kTtcReverse,
};

enum class EventSeverity {
  kInfo,
  kWarning,
  kBrake,
};

// Number of enumerators in GateSource / EventSeverity. Used to size GateEventTracker's
// fixed engagement table (below), which indexes on static_cast<std::size_t>(the enumerator)
// -- deliberately arithmetic rather than a switch, so the tracker adds no branches to the
// 100%-branch-coverage gate and no heap allocation to the 50 Hz path. Keep these in step
// with the enums above if an enumerator is ever added.
inline constexpr std::size_t kGateSourceCount = 8;
inline constexpr std::size_t kEventSeverityCount = 3;

// One gate's state for ONE control cycle, as reported by SafetyGateLogic::evaluate(): "this
// gate is engaged right now, at this severity, for this reason". NOT what gets published --
// evaluate() reports this every cycle a gate stays engaged, and GateEventTracker (below)
// turns the per-cycle stream into the transition records that reach /safety/events.
struct GateActivation {
  GateSource source;
  EventSeverity severity;
  std::string detail;
};

// Which end of an engagement a published record marks (mirrors racer_msgs/SafetyEvent.msg's
// PHASE_ENGAGE / PHASE_RELEASE one-to-one).
enum class EventPhase {
  kEngage,
  kRelease,
};

// What safety_node actually publishes on /safety/events: one record when a gate engages and
// one when that engagement releases. Mirrors racer_msgs/SafetyEvent.msg field-for-field
// (minus the stamp, which is ROS plumbing and belongs to the node).
struct SafetyEventRecord {
  GateSource source;
  EventSeverity severity;
  EventPhase phase;
  std::string detail;
  double duration_s = 0.0;  // always 0.0 on kEngage; length of the engagement on kRelease
};

// Everything the gate needs to know about this cycle. All fields are read defensively:
// non-finite/out-of-range sensor or timing values never propagate NaN/Inf into the output
// command or throw -- see evaluate()'s handling of each (test_gate_logic.cpp's "garbage
// input" cases cover every one).
struct GateInput {
  DriveCommand command;          // latest received /drive_raw (or the last cached one, if stale)
  double drive_raw_age_s = 0.0;  // seconds since /drive_raw was last received
  double dt_s = 0.0;             // seconds since evaluate() was last called (for rate limits)
  // Distance to the nearest valid /scan return in the path corridor, along the arc of THIS
  // cycle's requested steering (forward_sector.hpp min_path_distance_m, recomputed by
  // safety_node every cycle from the last scan); +inf if none.
  double min_scan_range_m = 0.0;
  // The obstacle-gate latch as the previous cycle left it (GateResult::ttc_brake_latched).
  // false at startup. See "THE OBSTACLE GATE AND ITS LATCH".
  bool ttc_brake_latched = false;
  // Distance to the nearest valid /scan return in the REAR path corridor, along the reverse
  // arc of THIS cycle's requested steering, from the rear bumper line
  // (forward_sector.hpp min_rear_path_distance_m, recomputed by safety_node every cycle);
  // +inf if none (the default: nothing behind). See "THE REAR OBSTACLE GATE".
  double min_rear_scan_range_m = std::numeric_limits<double>::infinity();
  // The rear gate's latch as the previous cycle left it (GateResult::reverse_brake_latched).
  bool reverse_brake_latched = false;
  // The steering-hold timer as the previous cycle left it (GateResult::obstacle_hold_timer_s):
  // seconds the latch has held the output speed at zero, std::nullopt when it was not. See
  // "STEERING HOLD WHILE PARKED ON THE OBSTACLE LATCH".
  std::optional<double> obstacle_hold_timer_s;
  // Covariance gate stub inputs (TODO roadmap task 2.6: wire from /pose once it exists).
  // `has_pose_input` is always false until then; `pose_covariance_trace` is unused while it
  // is.
  bool has_pose_input = false;
  double pose_covariance_trace = 0.0;
};

// What a "brake" from this layer physically is (read before trusting the word).
//
// This layer has exactly one actuator lever: the `speed` field of the /drive command. A
// "brake" here is `speed = 0`, and nothing more. Downstream, racer_drivers/pwm_output_node
// maps speed 0 to actuation.throttle_pwm_neutral_us, and a VESC in PPM mode reads a neutral
// pulse as ZERO CURRENT -- which is a COAST, not a deceleration. A car that is moving when a
// gate here engages keeps rolling and slows only by drag and drivetrain friction.
//
// So: layer 3 "brake" == "command zero drive current". Whether that becomes real
// deceleration is a property of the VESC's configured PPM control type (layer 2, see
// docs/notes/first-boot-runbook.md's VESC Tool step) or of a future closed-loop vesc_node,
// NOT of this code. The field below is named `zero_throttle` rather than `brake` for exactly
// that reason. `EventSeverity::kBrake` and racer_msgs' SEVERITY_BRAKE keep their names
// because they are a published interface and an evaluation metric; they mean "this gate
// commanded zero", with the same caveat.
struct GateResult {
  DriveCommand output;
  // true iff a gate is holding the throttle at zero this cycle (watchdog, command sanity,
  // or either obstacle-gate latch) rather than passing a clamped command. The forward latch
  // holds only FORWARD throttle and the rear latch only REVERSE throttle, so while one is set
  // a request the other way can still come out non-zero. See the note above: this is a
  // zero-throttle command, not a claim about deceleration.
  bool zero_throttle = false;
  // Which gates are engaged THIS cycle (not what gets published -- see GateActivation and
  // GateEventTracker). A gate that stays engaged appears here every cycle.
  std::vector<GateActivation> activations;
  // The obstacle-gate latch after this cycle; safety_node feeds it back in as next cycle's
  // GateInput::ttc_brake_latched.
  bool ttc_brake_latched = false;
  // The rear gate's latch after this cycle; fed back as GateInput::reverse_brake_latched.
  bool reverse_brake_latched = false;
  // The steering-hold timer after this cycle; safety_node feeds it back in as next cycle's
  // GateInput::obstacle_hold_timer_s.
  std::optional<double> obstacle_hold_timer_s;
  // Why an engagement ended, for gates that know (today only the obstacle gate's "ttc brake
  // released" note). Not an engagement: GateEventTracker::update uses a matching
  // (source, severity) entry here as the detail of that engagement's PHASE_RELEASE record.
  std::vector<GateActivation> releases;
};

// Covariance gate stub (roadmap task 2.6: no /pose source exists yet). Returns the speed-cap
// fraction to apply (1.0 = no derate) and whether the gate is even meaningfully engaged.
// With `has_pose_input == false` this ALWAYS returns {fraction: 1.0, engaged: false} --
// deliberately a no-op, not a guess -- and `evaluate()` never lets this stub's result gate
// whether the watchdog/sanity/bounds/rate/TTC checks above run (fail SAFE: absent pose input
// disables only the covariance derate itself, never the rest of the pipeline).
struct CovarianceGateResult {
  double speed_fraction = 1.0;
  bool engaged = false;
};

CovarianceGateResult evaluate_covariance_gate(bool has_pose_input, double pose_covariance_trace);

// The output a gate produces when it forces the throttle to zero: zero speed, with the
// steering angle from `previous_output` held (see "STEERING ON A ZERO-THROTTLE GATE" above).
// Exposed rather than kept private because safety_node's fail-closed exception path must
// produce the SAME command as the gates do -- two spellings of "the safe output" is exactly
// how the two paths drift apart.
DriveCommand zero_throttle_command(const DriveCommand& previous_output);

std::string gate_source_to_string(GateSource source);

class SafetyGateLogic {
 public:
  explicit SafetyGateLogic(SafetyLimits limits) : limits_(limits) {}

  // `previous_output` is the DriveCommand this same evaluate() call chain produced last
  // cycle (rate limits are relative to what was actually COMMANDED, not to what was
  // requested), seeded to {0, 0} by the node at startup.
  GateResult evaluate(const GateInput& input, const DriveCommand& previous_output) const;

  // Whether a steering-hold timer value means the hold is engaged (>= the configured hold
  // time). Public, with steering_hold_activation, because safety_node's fail-closed exception
  // path must keep reporting an engaged hold exactly the way the gate's own short-circuits do.
  bool steering_hold_engaged(const std::optional<double>& hold_timer_s) const;
  // The (kTtc, kInfo) "steering held while obstacle-latched" activation for a given held angle.
  GateActivation steering_hold_activation(double held_steering_angle_rad) const;

 private:
  DriveCommand clamp_to_bounds(const DriveCommand& cmd) const;
  DriveCommand rate_limit(const DriveCommand& cmd, const DriveCommand& previous_output,
                          double dt_s) const;
  // Step 3b, for one direction: +1.0 forward (range GateInput::min_scan_range_m, latch
  // ttc_brake_latched, source kTtc) or -1.0 reverse (min_rear_scan_range_m,
  // reverse_brake_latched, kTtcReverse). Reads the REQUESTED command, may zero `target`'s
  // speed toward that side, and records the latch, activations and release note in `result`;
  // returns the new latch.
  bool apply_obstacle_gate(double direction, GateSource source, double range_m, bool latched_in,
                           const DriveCommand& requested, DriveCommand& target,
                           GateResult& result) const;
  // TTC the request must exceed to release the latch. Only called when ttc_brake_s is set.
  double ttc_release_threshold_s() const;
  // Step 3d. Advances or clears the hold timer from the final `output` speed and, while the
  // hold is engaged, freezes the steering of both `target` and `output` at the previous
  // output's angle. Records the timer, the activation and the release note in `result`.
  void apply_steering_hold(const GateInput& input, const DriveCommand& previous_output, double dt_s,
                           DriveCommand& target, DriveCommand& output, GateResult& result) const;
  // The watchdog / command-sanity short-circuits: hold both latches and the hold timer as
  // they came in and keep reporting their engagements (see "THE OBSTACLE GATE AND ITS LATCH").
  void hold_obstacle_state(const GateInput& input, const DriveCommand& previous_output,
                           GateResult& result) const;

  SafetyLimits limits_;
};

// Turns SafetyGateLogic::evaluate()'s per-cycle activations into the transition records
// that reach /safety/events (GitHub issue #37: safety_node used to publish one record per
// engaged gate PER CYCLE, so a gate that stayed engaged for 14 s produced ~700 records at
// 50 Hz and the intervention count claude-docs/09-evaluation.md reports measured DURATION,
// not interventions).
//
// Semantics, deliberately simple:
//
//   * One intervention = one ENGAGEMENT of a (source, severity) pair, from the cycle it
//     first appears in `activations` to the cycle it stops appearing. Exactly one kEngage
//     record when it starts and exactly one kRelease record (carrying the duration) when it
//     ends. Nothing at all while it stays engaged -- no periodic "still engaged" record; the
//     interval between the two records IS the sustained state.
//   * (source, severity) rather than source alone is the engagement identity so that an
//     escalation is never silent: a TTC advisory (kTtc/kInfo) that becomes a TTC brake
//     (kTtc/kBrake) releases the advisory engagement and opens a brake engagement, which is
//     exactly what an evaluation counting BRAKE-severity interventions must see.
//   * Gates are independent: several can be engaged at once, each with its own lifecycle.
//   * A gate still engaged when the node exits never gets its release record. That is
//     accepted (the node is gone; there is nobody to publish it) and a bag reader should
//     treat a trailing engage as "engaged until end of bag".
//
// ROS-free and allocation-free apart from the returned vector: the engagement table is a
// fixed array sized by the enum counts above, not a map.
class GateEventTracker {
 public:
  // `now_s` is a monotonic-clock reading in seconds (safety_node passes its steady clock --
  // see that file's CLOCK POLICY); it is only ever used as the two ends of a subtraction,
  // and a non-finite or backwards interval is reported as a 0.0 duration rather than
  // propagating garbage into an evaluation metric.
  //
  // `release_notes` (GateResult::releases) carries a reason for an engagement that ends this
  // cycle; when one matches a releasing (source, severity) its detail is appended to that
  // PHASE_RELEASE record's detail. A note for something that is not releasing is ignored, so
  // a note can never create a record.
  std::vector<SafetyEventRecord> update(const std::vector<GateActivation>& activations,
                                        double now_s,
                                        const std::vector<GateActivation>& release_notes = {});

 private:
  struct Engagement {
    bool engaged = false;
    double engaged_at_s = 0.0;
  };

  std::array<Engagement, kGateSourceCount * kEventSeverityCount> engagements_{};
};

}  // namespace racer_safety

#endif  // RACER_SAFETY_GATE_LOGIC_HPP_
