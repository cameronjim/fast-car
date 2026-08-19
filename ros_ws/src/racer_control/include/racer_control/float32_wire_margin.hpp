// A tiny margin below a physical bound, applied only to a value about to be published on a
// float32 ROS message field (roadmap milestone 3).
//
// Motivation (found empirically while building
// tests/e2e_sim_safety/test_sim_autopilot_e2e.py): a command can legitimately equal a bound
// EXACTLY in double precision (e.g. PurePursuitController::compute_command's own
// std::clamp saturating steering there), and casting a value exactly at a double-precision
// bound to float32 (ackermann_msgs/AckermannDriveStamped's steering_angle/speed fields are
// float32) can round it a few ULPs the wrong way -- which a downstream double-precision
// bounds check (racer_safety::SafetyGateLogic, re-widening the float32 wire value back to
// double) then correctly sees as "outside the bound" given what it actually received, and
// clamps, emitting a /safety/events record for what is really just wire-precision noise, not
// a genuine bounds violation. This is wire-precision headroom, not a second copy of a
// physical constant: it is always far smaller than the bound itself and never changes what
// the controller actually intended.
#ifndef RACER_CONTROL_FLOAT32_WIRE_MARGIN_HPP_
#define RACER_CONTROL_FLOAT32_WIRE_MARGIN_HPP_

namespace racer_control {

// Clamps `value` to [min_bound, max_bound], each drawn in by a small fixed margin, and
// returns the result as a float (the type actually published on the wire). `min_bound`/
// `max_bound` are the real physical bounds (e.g. from vehicle_params) -- this function only
// adds the extra wire-precision margin, it does not invent or duplicate the bound itself.
float clamp_for_float32_publish(double value, double min_bound, double max_bound);

}  // namespace racer_control

#endif  // RACER_CONTROL_FLOAT32_WIRE_MARGIN_HPP_
