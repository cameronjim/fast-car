#include <math.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/rc_switch.h"

typedef struct {
  const char* name;
  double switch_pwm_us;
  double kill_threshold_us;
  double signal_min_us;
  double signal_max_us;
  RcSwitchPosition expected;
} RcSwitchCase;

static const RcSwitchCase kCases[] = {
    {"below threshold, in valid range -> KILL", 1000.0, 1500.0, 1000.0, 2000.0,
     RC_SWITCH_KILL},
    {"at threshold -> ARMED (inclusive)", 1500.0, 1500.0, 1000.0, 2000.0, RC_SWITCH_ARMED},
    {"above threshold -> ARMED", 2000.0, 1500.0, 1000.0, 2000.0, RC_SWITCH_ARMED},
    {"just below threshold -> KILL", 1499.999, 1500.0, 1000.0, 2000.0, RC_SWITCH_KILL},
    {"just above threshold -> ARMED", 1500.001, 1500.0, 1000.0, 2000.0, RC_SWITCH_ARMED},
    {"below the receiver's own valid range -> SIGNAL_INVALID, not KILL", 900.0, 1500.0,
     1000.0, 2000.0, RC_SWITCH_SIGNAL_INVALID},
    {"above the receiver's own valid range -> SIGNAL_INVALID, not ARMED", 2100.0, 1500.0,
     1000.0, 2000.0, RC_SWITCH_SIGNAL_INVALID},
    {"NaN reading -> SIGNAL_INVALID", NAN, 1500.0, 1000.0, 2000.0, RC_SWITCH_SIGNAL_INVALID},
    {"disconnected receiver (0us, common failsafe idle value) -> SIGNAL_INVALID", 0.0, 1500.0,
     1000.0, 2000.0, RC_SWITCH_SIGNAL_INVALID},
};

void test_rc_switch_suite(void) {
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); ++i) {
    const RcSwitchCase* c = &kCases[i];
    RcSwitchPosition actual =
        rc_switch_read(c->switch_pwm_us, c->kill_threshold_us, c->signal_min_us,
                       c->signal_max_us);
    CHECK(actual == c->expected, c->name);
  }
}
