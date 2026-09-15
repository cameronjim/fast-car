#include <math.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/rc_switch.h"

typedef struct {
  const char* name;
  double switch_pwm_us;
  double kill_threshold_us;
  double hysteresis_us;
  double signal_min_us;
  double signal_max_us;
  RcSwitchPosition previous;
  RcSwitchPosition expected;
} RcSwitchCase;

// With hysteresis_us == 0 the dead band is empty and this degrades to exactly the
// `>= threshold` comparison this function had before hysteresis existed. The first block
// below is that original case table, unchanged in expectation, which is what proves the
// no-band path did not change behaviour.
static const RcSwitchCase kCases[] = {
    // --- zero band: the pre-hysteresis behaviour, preserved ------------------------------
    {"no band: below threshold, in valid range -> KILL", 1000.0, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_KILL},
    {"no band: at threshold -> ARMED (inclusive)", 1500.0, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_KILL, RC_SWITCH_ARMED},
    {"no band: above threshold -> ARMED", 2000.0, 1500.0, 0.0, 1000.0, 2000.0, RC_SWITCH_KILL,
     RC_SWITCH_ARMED},
    {"no band: just below threshold -> KILL", 1499.999, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_KILL},
    {"no band: just above threshold -> ARMED", 1500.001, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_KILL, RC_SWITCH_ARMED},
    {"no band: below the receiver's own valid range -> SIGNAL_INVALID, not KILL", 900.0, 1500.0,
     0.0, 1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
    {"no band: above the receiver's own valid range -> SIGNAL_INVALID, not ARMED", 2100.0, 1500.0,
     0.0, 1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
    {"no band: NaN reading -> SIGNAL_INVALID", NAN, 1500.0, 0.0, 1000.0, 2000.0, RC_SWITCH_ARMED,
     RC_SWITCH_SIGNAL_INVALID},
    {"no band: +Inf reading -> SIGNAL_INVALID", INFINITY, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
    {"no band: -Inf reading -> SIGNAL_INVALID", -INFINITY, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
    {"no band: disconnected receiver (0us idle) -> SIGNAL_INVALID", 0.0, 1500.0, 0.0, 1000.0,
     2000.0, RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
    {"no band: never-captured sentinel (-1us) -> SIGNAL_INVALID", -1.0, 1500.0, 0.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},

    // --- 100 us band around a 1500 us threshold: ARM >= 1600, KILL < 1400, hold between ---
    {"band: at the arm edge (1600) -> ARMED", 1600.0, 1500.0, 100.0, 1000.0, 2000.0, RC_SWITCH_KILL,
     RC_SWITCH_ARMED},
    {"band: just above the arm edge -> ARMED", 1600.001, 1500.0, 100.0, 1000.0, 2000.0,
     RC_SWITCH_KILL, RC_SWITCH_ARMED},
    {"band: just below the arm edge, previously KILL -> still KILL", 1599.999, 1500.0, 100.0,
     1000.0, 2000.0, RC_SWITCH_KILL, RC_SWITCH_KILL},
    {"band: at the kill edge (1400) is inside the band, previously ARMED -> holds ARMED", 1400.0,
     1500.0, 100.0, 1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_ARMED},
    {"band: just below the kill edge -> KILL even if previously ARMED", 1399.999, 1500.0, 100.0,
     1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_KILL},
    {"band: exactly the threshold, previously ARMED -> holds ARMED", 1500.0, 1500.0, 100.0, 1000.0,
     2000.0, RC_SWITCH_ARMED, RC_SWITCH_ARMED},
    {"band: exactly the threshold, previously KILL -> holds KILL (no silent arm)", 1500.0, 1500.0,
     100.0, 1000.0, 2000.0, RC_SWITCH_KILL, RC_SWITCH_KILL},
    {"band: in the band with previous SIGNAL_INVALID -> KILL, never ARMED", 1500.0, 1500.0, 100.0,
     1000.0, 2000.0, RC_SWITCH_SIGNAL_INVALID, RC_SWITCH_KILL},
    {"band: in the band but the channel is unreadable -> SIGNAL_INVALID wins over the hold", NAN,
     1500.0, 100.0, 1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},

    // --- degenerate band values ----------------------------------------------------------
    {"NaN band is treated as zero: at threshold -> ARMED", 1500.0, 1500.0, NAN, 1000.0, 2000.0,
     RC_SWITCH_KILL, RC_SWITCH_ARMED},
    {"+Inf band is treated as zero: at threshold -> ARMED", 1500.0, 1500.0, INFINITY, 1000.0,
     2000.0, RC_SWITCH_KILL, RC_SWITCH_ARMED},
    {"negative band is treated as zero: just below threshold -> KILL", 1499.999, 1500.0, -50.0,
     1000.0, 2000.0, RC_SWITCH_ARMED, RC_SWITCH_KILL},
    {"band wider than the whole range: everything holds, previous KILL -> KILL", 1900.0, 1500.0,
     1000.0, 1000.0, 2000.0, RC_SWITCH_KILL, RC_SWITCH_KILL},

    // --- degenerate threshold: must not fail open ---------------------------------------
    {"NaN threshold -> KILL, never a held ARMED", 1900.0, NAN, 100.0, 1000.0, 2000.0,
     RC_SWITCH_ARMED, RC_SWITCH_KILL},
    {"+Inf threshold -> KILL", 1900.0, INFINITY, 100.0, 1000.0, 2000.0, RC_SWITCH_ARMED,
     RC_SWITCH_KILL},
    {"-Inf threshold -> KILL", 1900.0, -INFINITY, 100.0, 1000.0, 2000.0, RC_SWITCH_ARMED,
     RC_SWITCH_KILL},
    {"NaN signal bounds -> SIGNAL_INVALID (the range check runs first)", 1900.0, 1500.0, 100.0, NAN,
     2000.0, RC_SWITCH_ARMED, RC_SWITCH_SIGNAL_INVALID},
};

// The property that actually matters at the bench: sweeping the channel back and forth
// across the threshold inside the dead band must not produce a single arm/kill transition.
static void test_no_flapping_inside_the_band(void) {
  const double threshold = 1500.0;
  const double band = 100.0;
  const double jitter[] = {1500.0, 1499.0, 1501.0, 1450.0, 1550.0, 1500.5, 1499.5};

  RcSwitchPosition position = RC_SWITCH_KILL;
  for (size_t i = 0; i < sizeof(jitter) / sizeof(jitter[0]); ++i) {
    position = rc_switch_read(jitter[i], threshold, band, 1000.0, 2000.0, position);
    CHECK(position == RC_SWITCH_KILL,
          "jitter around the threshold never arms a switch that started KILL");
  }

  position = rc_switch_read(1700.0, threshold, band, 1000.0, 2000.0, position);
  CHECK(position == RC_SWITCH_ARMED, "a real move past the arm edge does arm");

  for (size_t i = 0; i < sizeof(jitter) / sizeof(jitter[0]); ++i) {
    position = rc_switch_read(jitter[i], threshold, band, 1000.0, 2000.0, position);
    CHECK(position == RC_SWITCH_ARMED,
          "jitter around the threshold never kills a switch that started ARMED");
  }

  position = rc_switch_read(1300.0, threshold, band, 1000.0, 2000.0, position);
  CHECK(position == RC_SWITCH_KILL, "a real move past the kill edge does kill");
}

// Coming back from an unreadable channel must go through KILL, not straight to a held ARMED.
static void test_invalid_channel_does_not_hold_armed(void) {
  RcSwitchPosition position = rc_switch_read(1700.0, 1500.0, 100.0, 1000.0, 2000.0, RC_SWITCH_KILL);
  CHECK(position == RC_SWITCH_ARMED, "armed first");

  position = rc_switch_read(-1.0, 1500.0, 100.0, 1000.0, 2000.0, position);
  CHECK(position == RC_SWITCH_SIGNAL_INVALID, "signal lost -> SIGNAL_INVALID");

  position = rc_switch_read(1500.0, 1500.0, 100.0, 1000.0, 2000.0, position);
  CHECK(position == RC_SWITCH_KILL,
        "signal returns inside the dead band -> KILL, not the ARMED it held before the loss");
}

void test_rc_switch_suite(void) {
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); ++i) {
    const RcSwitchCase* c = &kCases[i];
    RcSwitchPosition actual =
        rc_switch_read(c->switch_pwm_us, c->kill_threshold_us, c->hysteresis_us, c->signal_min_us,
                       c->signal_max_us, c->previous);
    CHECK(actual == c->expected, c->name);
  }
  test_no_flapping_inside_the_band();
  test_invalid_channel_does_not_hold_armed();
}
