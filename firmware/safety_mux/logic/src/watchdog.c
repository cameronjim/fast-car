#include "safety_mux/watchdog.h"

#include <math.h>

bool watchdog_timed_out(double heartbeat_age_s, double timeout_s) {
  // The TIMEOUT is checked before the age. `age >= NaN` is false and `age >= +Inf` is false
  // for every finite age, so a non-finite timeout_s would have made the watchdog never trip
  // -- fail-open, and silently: the mux would look armed and healthy forever with a dead
  // Jetson. A non-positive timeout is equally broken config. Either way the answer is "cut"
  // (claude-docs/05-safety.md fail-closed), and pico/main.c additionally refuses to arm at
  // all on such a value (logic/mux_params.c), so reaching this branch means both guards
  // would have to have been bypassed.
  if (!isfinite(timeout_s) || timeout_s <= 0.0) {
    return true;
  }
  if (!isfinite(heartbeat_age_s)) {
    return true;
  }
  if (heartbeat_age_s < 0.0) {
    return true;
  }
  if (heartbeat_age_s >= timeout_s) {
    return true;
  }
  return false;
}
