#include "racer_control/reverse_escape.hpp"

#include <algorithm>
#include <cmath>

namespace racer_control {

namespace {

// A gated speed this close to zero is zero (safety_node publishes an exact 0.0 when it brakes;
// this is a numerical guard, not a vehicle constant).
constexpr double kStoppedSpeedMps = 1e-6;

// Garbage or negative time is no time (the same convention as the filters).
double safe_dt(double dt_s) { return (std::isfinite(dt_s) && dt_s > 0.0) ? dt_s : 0.0; }

}  // namespace

void ReverseEscape::track_forward(const EscapeInput& input, EscapeOutput& out) {
  const double dt = safe_dt(input.dt_s);
  const bool fresh = input.gated_speed_mps.has_value() && std::isfinite(*input.gated_speed_mps);
  // Forward-blocked: this node asked to go forward and safety_node sent zero.
  if (fresh && input.requested_speed_mps > kStoppedSpeedMps &&
      std::abs(*input.gated_speed_mps) <= kStoppedSpeedMps) {
    forward_blocked_s_ += dt;
  } else {
    forward_blocked_s_ = 0.0;
  }
  // Forward progress since the last escape ended, from what the car was actually sent.
  if (fresh && *input.gated_speed_mps > kStoppedSpeedMps) {
    forward_progress_m_ += *input.gated_speed_mps * dt;
    if (forward_progress_m_ >= config_.escape_distance_m && attempts_ > 0) {
      attempts_ = 0;
      if (state_ == EscapeState::kExhausted) {
        state_ = EscapeState::kIdle;
      }
      out.event = EscapeEvent::kReset;
    }
  }
}

void ReverseEscape::start(const EscapeInput& input, EscapeOutput& out) {
  ++attempts_;
  state_ = EscapeState::kEscaping;
  distance_m_ = 0.0;
  elapsed_s_ = 0.0;
  speed_magnitude_mps_ = 0.0;
  rear_blocked_s_ = 0.0;
  // Opposite to the wanted forward steering: the nose swings toward where the follower wanted
  // to go while the car backs up (reverse_escape.hpp).
  const double magnitude = std::abs(config_.escape_steering_rad);
  steering_rad_ = 0.0;
  if (input.forward_steering_rad > 0.0) {
    steering_rad_ = -magnitude;
  } else if (input.forward_steering_rad < 0.0) {
    steering_rad_ = magnitude;
  }
  out.event = EscapeEvent::kStarted;
}

void ReverseEscape::leave_escape(EscapeState next, EscapeEvent event, EscapeOutput& out) {
  state_ = next;
  out.event = event;
  forward_blocked_s_ = 0.0;
  forward_progress_m_ = 0.0;
  rear_blocked_s_ = 0.0;
  retry_wait_s_ = 0.0;
  speed_magnitude_mps_ = 0.0;
}

EscapeOutput ReverseEscape::update(const EscapeInput& input) {
  EscapeOutput out;
  const double dt = safe_dt(input.dt_s);

  if (state_ == EscapeState::kEscaping) {
    elapsed_s_ += dt;
    // Rear blocked: last cycle's request was reverse and safety_node did not pass it (or went
    // silent). Debounced, because /drive lags the request by a cycle or two.
    const bool fresh = input.gated_speed_mps.has_value() && std::isfinite(*input.gated_speed_mps);
    const bool rear_blocked = input.requested_speed_mps < -kStoppedSpeedMps &&
                              (!fresh || *input.gated_speed_mps >= -kStoppedSpeedMps);
    rear_blocked_s_ = rear_blocked ? rear_blocked_s_ + dt : 0.0;
    if (rear_blocked_s_ >= config_.block_debounce_s) {
      leave_escape(EscapeState::kRetryWait, EscapeEvent::kAborted, out);
      return out;
    }
    if (distance_m_ >= config_.escape_distance_m) {
      leave_escape(EscapeState::kIdle, EscapeEvent::kCompleted, out);
      return out;
    }
    if (elapsed_s_ >= config_.escape_max_s) {
      leave_escape(EscapeState::kIdle, EscapeEvent::kTimedOut, out);
      return out;
    }
  } else {
    track_forward(input, out);
    if (state_ == EscapeState::kRetryWait) {
      retry_wait_s_ += dt;
      if (retry_wait_s_ >= config_.escape_retry_after_s) {
        state_ = EscapeState::kIdle;
        if (!out.event) {
          out.event = EscapeEvent::kRetryReady;
        }
      }
      return out;
    }
    if (state_ != EscapeState::kIdle || forward_blocked_s_ < config_.escape_after_s ||
        !input.forward_path_blocked) {
      return out;
    }
    if (attempts_ >= config_.escape_max_attempts) {
      state_ = EscapeState::kExhausted;
      out.event = EscapeEvent::kExhausted;
      return out;
    }
    start(input, out);
  }

  // Escaping: ramp the reverse speed magnitude like the forward speed, integrate the distance
  // from the commanded speed (reverse_escape.hpp DISTANCE).
  const double target = std::abs(config_.escape_speed_mps);
  if (config_.max_acceleration_mps2 > 0.0) {
    speed_magnitude_mps_ =
        std::min(target, speed_magnitude_mps_ + config_.max_acceleration_mps2 * dt);
  } else {
    speed_magnitude_mps_ = target;
  }
  distance_m_ += speed_magnitude_mps_ * dt;
  out.active = true;
  out.speed_mps = -speed_magnitude_mps_;
  out.steering_rad = steering_rad_;
  return out;
}

void ReverseEscape::reset() {
  state_ = EscapeState::kIdle;
  forward_blocked_s_ = 0.0;
  forward_progress_m_ = 0.0;
  distance_m_ = 0.0;
  elapsed_s_ = 0.0;
  speed_magnitude_mps_ = 0.0;
  rear_blocked_s_ = 0.0;
  retry_wait_s_ = 0.0;
}

}  // namespace racer_control
