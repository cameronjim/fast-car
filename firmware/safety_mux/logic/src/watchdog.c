#include "safety_mux/watchdog.h"

#include <math.h>

bool watchdog_timed_out(double heartbeat_age_s, double timeout_s) {
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
