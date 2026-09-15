#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/watchdog.h"

typedef struct {
  const char* name;
  double heartbeat_age_s;
  double timeout_s;
  bool expected_timed_out;
} WatchdogCase;

static const WatchdogCase kCases[] = {
    {"fresh heartbeat (age 0) is not timed out", 0.0, 0.5, false},
    {"age well under timeout is not timed out", 0.1, 0.5, false},
    {"age just under timeout is not timed out", 0.499999, 0.5, false},
    {"age exactly at timeout IS timed out (>=)", 0.5, 0.5, true},
    {"age just over timeout is timed out", 0.500001, 0.5, true},
    {"age well over timeout is timed out", 5.0, 0.5, true},
    {"negative age (garbage timestamp) is timed out, fail-closed", -0.1, 0.5, true},
    {"NaN age is timed out, fail-closed", NAN, 0.5, true},
    {"+Inf age is timed out", INFINITY, 0.5, true},
    {"-Inf age is timed out (still garbage, still fail-closed)", -INFINITY, 0.5, true},
    // A non-finite or non-positive TIMEOUT used to make `age >= timeout` false forever: a
    // watchdog that never trips, which is the fail-OPEN direction. These pin it shut.
    {"NaN timeout is timed out, not a watchdog that never trips", 0.0, NAN, true},
    {"+Inf timeout is timed out", 0.0, INFINITY, true},
    {"-Inf timeout is timed out", 0.0, -INFINITY, true},
    {"zero timeout is timed out (a zero-length window can never be met)", 0.0, 0.0, true},
    {"negative timeout is timed out", 0.0, -0.5, true},
    {"fresh age with a broken timeout is still timed out", 0.001, NAN, true},
};

void test_watchdog_suite(void) {
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); ++i) {
    const WatchdogCase* c = &kCases[i];
    bool actual = watchdog_timed_out(c->heartbeat_age_s, c->timeout_s);
    CHECK(actual == c->expected_timed_out, c->name);
  }
}
