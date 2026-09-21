// Host tests for the quantisation-tolerant window + pass-through clamp (pwm_window.h,
// GitHub issue #63). Table-driven, both branches of every comparison in
// logic/src/pwm_window.c exercised. The cases that exist BECAUSE of the hardware observation
// on 2026-09-21 are marked; the rest pin the boundaries and the fail-closed paths.
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/pwm_window.h"

// Shorthands for the numbers the whole suite is about. Grid = 15.625 us, tolerance = 4 grid
// steps = 62.5 us, so the widened steering window is 937.5 - 2062.5 us against a configured
// 1000 - 2000 us. All of these are exact in binary floating point.
#define G PWM_WINDOW_CAPTURE_GRID_US
#define TOL PWM_WINDOW_DEFAULT_TOLERANCE_US
#define MIN_US 1000.0
#define MAX_US 2000.0

typedef struct {
  const char* name;
  double pulse_us;
  double min_us;
  double max_us;
  double tolerance_us;
  bool expect_accept;
  double expect_clamped_us;  // only meaningful when expect_accept
} PwmWindowCase;

static const PwmWindowCase kCases[] = {
    // --- THE DEFECT (issue #63, observed on hardware 2026-09-21) ----------------------
    // Jetson commanded exactly 2000 us (its pwm_max); the mux measured 2031 us and cut.
    {"issue #63: commanded 2000 us measured as 2031 us is ACCEPTED", 2031.0, MIN_US, MAX_US, TOL,
     true, 2000.0},
    {"issue #63: 2031 us is forwarded as 2000 us, never as 2031 us", 2031.0, MIN_US, MAX_US, TOL,
     true, 2000.0},
    {"issue #63: the exact grid point 2031.25 us is accepted and clamped", 2031.25, MIN_US, MAX_US,
     TOL, true, 2000.0},
    // The same run measured a commanded 1000 us as 1016 us. That one already passed (it lands
    // inside the window), and it must keep passing and keep being forwarded as measured.
    {"issue #63: 1000 us measured as 1016 us is accepted (in range, so forwarded as measured)",
     1016.0, MIN_US, MAX_US, TOL, true, 1016.0},
    {"issue #63: the exact grid point 1015.625 us is accepted, in range, unclamped", 1015.625,
     MIN_US, MAX_US, TOL, true, 1015.625},
    // ... and the other rounding direction at the same edge, which the bench happened not to
    // produce: a commanded 1000 us landing one grid step BELOW the floor.
    {"1000 us measured as 984 us (one step low) is accepted and clamped UP to 1000", 984.0, MIN_US,
     MAX_US, TOL, true, 1000.0},
    {"the exact grid point 984.375 us is accepted and clamped up to 1000", 984.375, MIN_US, MAX_US,
     TOL, true, 1000.0},

    // --- Window edges, plus and minus one grid step -----------------------------------
    {"exactly at min is accepted, unclamped", MIN_US, MIN_US, MAX_US, TOL, true, MIN_US},
    {"exactly at max is accepted, unclamped", MAX_US, MIN_US, MAX_US, TOL, true, MAX_US},
    {"one grid step inside min is accepted, unclamped", MIN_US + G, MIN_US, MAX_US, TOL, true,
     MIN_US + G},
    {"one grid step inside max is accepted, unclamped", MAX_US - G, MIN_US, MAX_US, TOL, true,
     MAX_US - G},
    {"one grid step below min is accepted and clamped to min", MIN_US - G, MIN_US, MAX_US, TOL,
     true, MIN_US},
    {"one grid step above max is accepted and clamped to max", MAX_US + G, MIN_US, MAX_US, TOL,
     true, MAX_US},
    {"two grid steps above max (the observed error) is accepted and clamped", MAX_US + (2.0 * G),
     MIN_US, MAX_US, TOL, true, MAX_US},

    // --- The tolerance boundary itself, inclusive, and one step past it ---------------
    {"exactly at the widened floor (min - 4 steps = 937.5) is accepted, clamped to min", 937.5,
     MIN_US, MAX_US, TOL, true, MIN_US},
    {"exactly at the widened ceiling (max + 4 steps = 2062.5) is accepted, clamped to max", 2062.5,
     MIN_US, MAX_US, TOL, true, MAX_US},
    {"one grid step below the widened floor (921.875) is REJECTED", 937.5 - G, MIN_US, MAX_US, TOL,
     false, 0.0},
    {"one grid step above the widened ceiling (2078.125) is REJECTED", 2062.5 + G, MIN_US, MAX_US,
     TOL, false, 0.0},
    {"a hair below the widened floor is REJECTED (the boundary is inclusive, not fuzzy)", 937.4,
     MIN_US, MAX_US, TOL, false, 0.0},
    {"a hair above the widened ceiling is REJECTED", 2062.6, MIN_US, MAX_US, TOL, false, 0.0},

    // --- Garbage still gets rejected. These are the numbers the issue names, plus the ones
    // the existing mux_decision suite uses, so the widened window cannot quietly swallow
    // them. -----------------------------------------------------------------------------
    {"2450 us stays out of range", 2450.0, MIN_US, MAX_US, TOL, false, 0.0},
    {"900 us stays out of range (37.5 us below the widened floor)", 900.0, MIN_US, MAX_US, TOL,
     false, 0.0},
    {"3000 us stays out of range", 3000.0, MIN_US, MAX_US, TOL, false, 0.0},
    {"9999 us stays out of range", 9999.0, MIN_US, MAX_US, TOL, false, 0.0},
    {"0 us stays out of range", 0.0, MIN_US, MAX_US, TOL, false, 0.0},
    {"-1.0, pwm_capture_read_us()'s no-believable-pulse value, is rejected", -1.0, MIN_US, MAX_US,
     TOL, false, 0.0},

    // --- Fail-closed paths: non-finite values and bounds ------------------------------
    {"NaN pulse is rejected", NAN, MIN_US, MAX_US, TOL, false, 0.0},
    {"+Inf pulse is rejected", INFINITY, MIN_US, MAX_US, TOL, false, 0.0},
    {"-Inf pulse is rejected", -INFINITY, MIN_US, MAX_US, TOL, false, 0.0},
    {"NaN min bound is rejected, never valid-by-default", 1500.0, NAN, MAX_US, TOL, false, 0.0},
    {"NaN max bound is rejected, never valid-by-default", 1500.0, MIN_US, NAN, TOL, false, 0.0},
    {"+Inf max bound is rejected (an unbounded channel is a broken calibration)", 1500.0, MIN_US,
     INFINITY, TOL, false, 0.0},
    {"-Inf min bound is rejected", 1500.0, -INFINITY, MAX_US, TOL, false, 0.0},
    {"inverted bounds are rejected even though widening would overlap them", 1010.0, 1020.0, 1000.0,
     TOL, false, 0.0},
    {"wildly inverted bounds are rejected", 1500.0, 2000.0, 1000.0, TOL, false, 0.0},

    // --- The tolerance argument itself -------------------------------------------------
    {"zero tolerance reproduces the old exact window: 2031 us is rejected", 2031.0, MIN_US, MAX_US,
     0.0, false, 0.0},
    {"zero tolerance: exactly at max is still accepted (inclusive)", MAX_US, MIN_US, MAX_US, 0.0,
     true, MAX_US},
    {"a negative tolerance degrades to zero, it never widens or narrows", 2031.0, MIN_US, MAX_US,
     -50.0, false, 0.0},
    {"a NaN tolerance degrades to zero (fail closed), it does not accept everything", 2031.0,
     MIN_US, MAX_US, NAN, false, 0.0},
    {"an infinite tolerance degrades to zero, it does not accept every finite pulse", 9999.0,
     MIN_US, MAX_US, INFINITY, false, 0.0},
    {"an infinite tolerance still accepts an in-range pulse, unclamped", 1500.0, MIN_US, MAX_US,
     INFINITY, true, 1500.0},

    // --- Degenerate ranges --------------------------------------------------------------
    {"equal bounds: the value itself is accepted, unclamped", 1500.0, 1500.0, 1500.0, TOL, true,
     1500.0},
    {"equal bounds: a value within tolerance is accepted and clamped onto the point", 1531.25,
     1500.0, 1500.0, TOL, true, 1500.0},
    {"equal bounds: a value past tolerance is rejected", 1600.0, 1500.0, 1500.0, TOL, false, 0.0},

    // --- The throttle channel uses the same function with its own range. A value that is a
    // legal steering pulse is not automatically a legal throttle pulse. ------------------
    {"a narrower configured range narrows the widened window with it", 1300.0, 1400.0, 1600.0, TOL,
     false, 0.0},
    {"just inside a narrower range's widened floor is accepted and clamped", 1337.5, 1400.0, 1600.0,
     TOL, true, 1400.0},
};

void test_pwm_window_suite(void) {
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); ++i) {
    const PwmWindowCase* c = &kCases[i];

    // Sentinel, not zero: a case that wrongly leaves *clamped untouched must FAIL rather
    // than accidentally match an expectation of 0.0.
    double clamped = -12345.0;
    bool accepted =
        pwm_window_accept_us(c->pulse_us, c->min_us, c->max_us, c->tolerance_us, &clamped);
    CHECK(accepted == c->expect_accept, c->name);
    if (c->expect_accept) {
      CHECK(accepted && clamped == c->expect_clamped_us, c->name);
      // The clamped value is ALWAYS inside the configured range, for every accepted case in
      // the table. This is the invariant the servo/ESC actually depends on.
      CHECK(!accepted || (clamped >= c->min_us && clamped <= c->max_us),
            "accepted pulses are always forwarded inside the configured range");
    } else {
      CHECK(clamped == -12345.0, "a rejected pulse never writes a forwarded value");
    }

    // The verdict must not depend on whether the caller wanted the clamped value.
    bool accepted_no_out =
        pwm_window_accept_us(c->pulse_us, c->min_us, c->max_us, c->tolerance_us, NULL);
    CHECK(accepted_no_out == c->expect_accept, "NULL out-pointer gives the same verdict");
  }

  // The tolerance constants themselves, pinned. If someone widens the window, these fail and
  // they have to come back here and to pwm_window.h's justification, which is the point.
  CHECK(PWM_WINDOW_CAPTURE_GRID_US == 15.625, "capture grid is the measured 15.625 us");
  CHECK(PWM_WINDOW_TOLERANCE_GRID_STEPS == 4, "tolerance is 4 grid steps");
  CHECK(PWM_WINDOW_DEFAULT_TOLERANCE_US == 62.5, "tolerance is 62.5 us");
  CHECK(PWM_WINDOW_DEFAULT_TOLERANCE_US >= 2.0 * PWM_WINDOW_CAPTURE_GRID_US,
        "tolerance covers at least the 2 grid steps the hardware defect needed");
  CHECK(1000.0 - PWM_WINDOW_DEFAULT_TOLERANCE_US > 900.0,
        "the widened floor still rejects a 900 us pulse");
}
