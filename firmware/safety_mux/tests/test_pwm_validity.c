#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/pwm_validity.h"

typedef struct {
  const char* name;
  double pulse_us;
  double min_us;
  double max_us;
  bool expected;
} PwmValidityCase;

static const PwmValidityCase kCases[] = {
    {"below min is invalid", 999.0, 1000.0, 2000.0, false},
    {"at min is valid (inclusive)", 1000.0, 1000.0, 2000.0, true},
    {"mid-range is valid", 1500.0, 1000.0, 2000.0, true},
    {"at max is valid (inclusive)", 2000.0, 1000.0, 2000.0, true},
    {"above max is invalid", 2001.0, 1000.0, 2000.0, false},
    {"NaN is invalid regardless of bounds", NAN, 1000.0, 2000.0, false},
    {"+Inf is invalid", INFINITY, 1000.0, 2000.0, false},
    {"-Inf is invalid", -INFINITY, 1000.0, 2000.0, false},
    {"zero is invalid outside a realistic PWM range", 0.0, 1000.0, 2000.0, false},
    {"negative is invalid", -500.0, 1000.0, 2000.0, false},
    {"inverted bounds (min > max) is always invalid, not a crash", 1500.0, 2000.0, 1000.0,
     false},
    {"degenerate equal bounds: value equals both is valid", 1500.0, 1500.0, 1500.0, true},
    {"degenerate equal bounds: value off by epsilon is invalid", 1500.001, 1500.0, 1500.0,
     false},
};

void test_pwm_validity_suite(void) {
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); ++i) {
    const PwmValidityCase* c = &kCases[i];
    bool actual = pwm_is_valid_us(c->pulse_us, c->min_us, c->max_us);
    CHECK(actual == c->expected, c->name);
  }
}
