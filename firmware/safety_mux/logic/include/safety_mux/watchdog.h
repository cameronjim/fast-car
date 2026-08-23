// Pure heartbeat-staleness check -- no Pico SDK, no hardware access, host-testable.
#ifndef SAFETY_MUX_WATCHDOG_H_
#define SAFETY_MUX_WATCHDOG_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Returns true iff the Jetson-side heartbeat has gone stale: `heartbeat_age_s` (seconds
// since the last heartbeat edge was observed) is non-finite, negative (a garbage timestamp
// -- treated as "definitely timed out", the fail-closed direction), or has reached/exceeded
// `timeout_s`. `timeout_s` itself comes from vehicle_params.limits.mux_watchdog_timeout_s
// via mux_params.c, which refuses to start at all if that value is unset -- this function
// trusts its caller on that bound and focuses purely on the age comparison.
bool watchdog_timed_out(double heartbeat_age_s, double timeout_s);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_WATCHDOG_H_
