// Quantisation-tolerant PWM plausibility window + explicit pass-through clamp -- pure C, no
// Pico SDK, no hardware access, host-testable.
//
// WHY THIS EXISTS (GitHub issue #63, observed on hardware 2026-09-21). The Jetson commanded
// exactly 2000 us on the steering channel (its configured `steering.pwm_max_us`); the mux
// measured 2031 us and `pwm_is_valid_us()` classified it out of range, so an ARMED mux CUT on
// a legal full-left command. The same run measured a commanded 1000 us as 1016 us, which
// happened to land inside the window and passed.
//
// Neither number is a fault. They are the command landing on a coarse pulse-width grid:
// 2031.25 = 130 x 15.625 us and 1015.625 = 65 x 15.625 us, i.e. both measurements are exact
// multiples of a 15.625 us step, and the emitter's own duty granularity (a 20 ms frame split
// 256 ways on the Tegra PWM peripheral = 78.125 us, five of those steps) is a multiple of it
// again. A command that is not itself on the grid CANNOT be emitted or measured on it; the
// nearest grid point is up to a step away, and at the window's edges that step points
// outward, out of an inclusive window with zero tolerance.
//
// The grid is NOT this MCU's timebase. `pico/pwm_capture.c` times edges with `time_us_64()`,
// which is a 1 us hardware timer, so there is no coarser clock here to refine: making the
// capture "measure finer" would change nothing, because the pulse on the wire really is
// 2031 us long. The quantisation is upstream of the connector, in whatever is emitting the
// pulse, and the mux cannot make an emitter produce a width its peripheral cannot express.
// So the window absorbs the grid, and the value that is forwarded is clamped back into the
// configured range -- never 2031 us out to a servo whose calibrated end stop is 2000 us.
#ifndef SAFETY_MUX_PWM_WINDOW_H_
#define SAFETY_MUX_PWM_WINDOW_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// The measured pulse-width grid, microseconds. Both hardware observations on 2026-09-21
// (2031.25 and 1015.625) are exact multiples of this.
#define PWM_WINDOW_CAPTURE_GRID_US 15.625

// How many grid steps of slack the window gets on each side.
//
// The numbers this is chosen from:
//   - 2 steps (31.25 us) is the MINIMUM that fixes the observed defect: 2000 us was measured
//     as 2031.25 us, two steps above the ceiling.
//   - 2.5 steps (39.06 us) is the worst case for nearest-point rounding onto the emitter's
//     own 78.125 us duty step, which is the coarsest grid in the measured chain: half a step
//     is the most a legal command can be displaced by it, in either direction.
//   - 4 steps (62.5 us) is what is used. It is twice the observed error and comfortably past
//     the 39.06 us worst case, so the window is not sitting one rounding decision away from
//     cutting again, and it is a whole number of grid steps rather than a round decimal
//     invented to look tidy.
//   - It is NOT wider, because the window still has to reject garbage. With 62.5 us the
//     steering window becomes 937.5-2062.5 us against a configured 1000-2000 us: 900 us (the
//     issue's example of a genuinely bad low pulse) is rejected with 37.5 us to spare and
//     2450 us is rejected by 387.5 us. Widening to 6 steps would leave 900 us inside by
//     6.25 us, which is the point at which this stops being a tolerance and starts being a
//     different window.
#define PWM_WINDOW_TOLERANCE_GRID_STEPS 4

// 62.5 us. Exactly representable as a double (it is 2^-4 x 1000), so every comparison and
// every boundary case below is deterministic, not epsilon-dependent.
#define PWM_WINDOW_DEFAULT_TOLERANCE_US \
  (PWM_WINDOW_CAPTURE_GRID_US * (double)PWM_WINDOW_TOLERANCE_GRID_STEPS)

// COMPILE-TIME, AND THAT IS A KNOWN GAP -- the same gap as `rc_switch.h`'s
// RC_SWITCH_DEFAULT_HYSTERESIS_US and `pico/pwm_capture.c`'s PWM_CAPTURE_MAX_AGE_US. The
// tolerance describes the MEASUREMENT PATH (emitter peripheral granularity, capture
// resolution), not the vehicle, so it is not a `config/vehicle_params.yaml` physical
// constant and CLAUDE.md invariant 2 does not reach it: there is no mass, wheelbase, gear
// ratio, or unit conversion here. It stays next to the other capture-path constants until a
// scope says what the real grid on this board is, at which point it wants to be measured,
// not conventional.

// Accepts `pulse_us` if it is within the configured window [min_us, max_us] WIDENED by
// `tolerance_us` on each side, and, when it does, writes the value clamped back INTO the
// unwidened window to `*clamped_out_us`.
//
// Returns true (accepted) iff all of:
//   - `min_us` and `max_us` are finite and `min_us <= max_us`. An inverted or non-finite pair
//     is always rejected, exactly as in `pwm_is_valid_us()`: a widened window must not turn a
//     broken calibration into a pass, which would be fail-OPEN.
//   - `pulse_us` is finite.
//   - `min_us - tol <= pulse_us <= max_us + tol`, where `tol` is `tolerance_us` if it is
//     finite and non-negative, and 0 otherwise. A NaN or infinite tolerance degrading to zero
//     is the fail-CLOSED direction (it narrows the window back to the exact configured range,
//     it never opens it); an infinite tolerance would accept every finite pulse.
//
// THE CLAMP IS THE POINT, not a convenience. `*clamped_out_us` is `min_us` below the range,
// `max_us` above it, and `pulse_us` inside it, so a measurement of 2031 us against a
// 1000-2000 us range is forwarded as exactly 2000 us and never as 2031 us. A servo or ESC
// calibrated to a 1000-2000 us range is being told to go past its end stop by anything
// outside it, which is how servos buzz and stall; the whole reason this function returns the
// forwarded value instead of only a verdict is that a caller cannot then forget to clamp.
//
// `clamped_out_us` may be NULL if only the verdict is wanted. On a rejected pulse it is never
// written: there is no meaningful value to forward, and the caller must cut, not substitute.
bool pwm_window_accept_us(double pulse_us, double min_us, double max_us, double tolerance_us,
                          double* clamped_out_us);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_PWM_WINDOW_H_
