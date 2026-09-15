#include <math.h>
#include <stddef.h>
#include <string.h>

#include "framework.h"
#include "safety_mux/mux_params.h"

static RawMuxField set_field(double value) {
  RawMuxField f;
  f.value = value;
  f.is_set = true;
  return f;
}

static RawMuxField unset_field(void) {
  RawMuxField f;
  f.value = 0.0;
  f.is_set = false;
  return f;
}

// A fully-set fixture -- every field present with an arbitrary-but-plausible value. Each
// negative case below starts from a fresh copy of this and changes exactly one field,
// isolating that field as the single variable under test.
static RawMuxParamFields all_set_fixture(void) {
  RawMuxParamFields raw;
  raw.steering_pwm_min_us = set_field(1000.0);
  raw.steering_pwm_max_us = set_field(2000.0);
  raw.steering_pwm_neutral_us = set_field(1500.0);
  raw.throttle_pwm_min_us = set_field(1000.0);
  raw.throttle_pwm_max_us = set_field(2000.0);
  raw.throttle_pwm_neutral_us = set_field(1500.0);
  raw.watchdog_timeout_s = set_field(0.5);
  raw.kill_switch_threshold_us = set_field(1500.0);
  raw.kill_switch_hysteresis_us = set_field(100.0);
  raw.rc_signal_min_us = set_field(1000.0);
  raw.rc_signal_max_us = set_field(2000.0);
  return raw;
}

// Every field, in declaration order: the order is load-bearing, because
// mux_params_from_raw() reports the FIRST problem it finds and pico/main.c prints that name.
typedef RawMuxField* (*FieldPicker)(RawMuxParamFields* raw);

static RawMuxField* pick_steering_min(RawMuxParamFields* r) { return &r->steering_pwm_min_us; }
static RawMuxField* pick_steering_max(RawMuxParamFields* r) { return &r->steering_pwm_max_us; }
static RawMuxField* pick_steering_neutral(RawMuxParamFields* r) {
  return &r->steering_pwm_neutral_us;
}
static RawMuxField* pick_throttle_min(RawMuxParamFields* r) { return &r->throttle_pwm_min_us; }
static RawMuxField* pick_throttle_max(RawMuxParamFields* r) { return &r->throttle_pwm_max_us; }
static RawMuxField* pick_throttle_neutral(RawMuxParamFields* r) {
  return &r->throttle_pwm_neutral_us;
}
static RawMuxField* pick_watchdog(RawMuxParamFields* r) { return &r->watchdog_timeout_s; }
static RawMuxField* pick_threshold(RawMuxParamFields* r) { return &r->kill_switch_threshold_us; }
static RawMuxField* pick_hysteresis(RawMuxParamFields* r) { return &r->kill_switch_hysteresis_us; }
static RawMuxField* pick_rc_min(RawMuxParamFields* r) { return &r->rc_signal_min_us; }
static RawMuxField* pick_rc_max(RawMuxParamFields* r) { return &r->rc_signal_max_us; }

typedef struct {
  FieldPicker pick;
  const char* name;
} FieldCase;

static const FieldCase kFields[] = {
    {pick_steering_min, "steering_pwm_min_us"},
    {pick_steering_max, "steering_pwm_max_us"},
    {pick_steering_neutral, "steering_pwm_neutral_us"},
    {pick_throttle_min, "throttle_pwm_min_us"},
    {pick_throttle_max, "throttle_pwm_max_us"},
    {pick_throttle_neutral, "throttle_pwm_neutral_us"},
    {pick_watchdog, "watchdog_timeout_s"},
    {pick_threshold, "kill_switch_threshold_us"},
    {pick_hysteresis, "kill_switch_hysteresis_us"},
    {pick_rc_min, "rc_signal_min_us"},
    {pick_rc_max, "rc_signal_max_us"},
};

// One structural (set, finite, but unusable) case per check in mux_params.c.
typedef struct {
  const char* name;
  FieldPicker pick;
  double value;
  const char* expected_field;
} RangeCase;

static const RangeCase kRangeCases[] = {
    {"steering neutral below its own min", pick_steering_neutral, 900.0, "steering_pwm_neutral_us"},
    {"steering neutral above its own max", pick_steering_neutral, 2100.0,
     "steering_pwm_neutral_us"},
    {"inverted steering range (min above max) is caught via the neutral check", pick_steering_min,
     2500.0, "steering_pwm_neutral_us"},
    {"throttle neutral below its own min", pick_throttle_neutral, 900.0, "throttle_pwm_neutral_us"},
    {"throttle neutral above its own max -- the creep-on-cut config", pick_throttle_neutral, 2100.0,
     "throttle_pwm_neutral_us"},
    {"zero watchdog timeout", pick_watchdog, 0.0, "watchdog_timeout_s"},
    {"negative watchdog timeout", pick_watchdog, -0.5, "watchdog_timeout_s"},
    {"negative kill-switch hysteresis", pick_hysteresis, -1.0, "kill_switch_hysteresis_us"},
    {"inverted receiver range", pick_rc_min, 2500.0, "rc_signal_max_us"},
    {"degenerate receiver range (min == max)", pick_rc_min, 2000.0, "rc_signal_max_us"},
    {"kill threshold below the receiver's range", pick_threshold, 900.0,
     "kill_switch_threshold_us"},
    {"kill threshold above the receiver's range", pick_threshold, 2100.0,
     "kill_switch_threshold_us"},
};

void test_mux_params_suite(void) {
  // Happy path: every field set -> ok, and every value carried through unchanged.
  {
    RawMuxParamFields raw = all_set_fixture();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(result.ok, "all fields set -> ok");
    CHECK(result.problem == MUX_PARAM_PROBLEM_NONE, "ok result reports no problem");
    CHECK(result.missing_field == NULL, "ok result names no field");
    CHECK(result.params.steering_pwm_min_us == 1000.0, "steering_pwm_min_us carried through");
    CHECK(result.params.steering_pwm_max_us == 2000.0, "steering_pwm_max_us carried through");
    CHECK(result.params.steering_pwm_neutral_us == 1500.0,
          "steering_pwm_neutral_us carried through");
    CHECK(result.params.throttle_pwm_min_us == 1000.0, "throttle_pwm_min_us carried through");
    CHECK(result.params.throttle_pwm_max_us == 2000.0, "throttle_pwm_max_us carried through");
    CHECK(result.params.throttle_pwm_neutral_us == 1500.0,
          "throttle_pwm_neutral_us carried through");
    CHECK(result.params.watchdog_timeout_s == 0.5, "watchdog_timeout_s carried through");
    CHECK(result.params.kill_switch_threshold_us == 1500.0,
          "kill_switch_threshold_us carried through");
    CHECK(result.params.kill_switch_hysteresis_us == 100.0,
          "kill_switch_hysteresis_us carried through");
    CHECK(result.params.rc_signal_min_us == 1000.0, "rc_signal_min_us carried through");
    CHECK(result.params.rc_signal_max_us == 2000.0, "rc_signal_max_us carried through");
  }

  // Zero hysteresis is a legitimate configuration (it degrades to the plain threshold), so
  // the range check must not reject it.
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.kill_switch_hysteresis_us = set_field(0.0);
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(result.ok, "zero hysteresis is accepted, not refused");
  }

  // One case per field, three ways: missing, NaN, +-Inf. Each must refuse and name exactly
  // that field.
  for (size_t i = 0; i < sizeof(kFields) / sizeof(kFields[0]); ++i) {
    {
      RawMuxParamFields raw = all_set_fixture();
      *kFields[i].pick(&raw) = unset_field();
      MuxParamsResult result = mux_params_from_raw(raw);
      CHECK(!result.ok, kFields[i].name);
      CHECK(result.problem == MUX_PARAM_PROBLEM_MISSING, kFields[i].name);
      CHECK(strcmp(result.missing_field, kFields[i].name) == 0, kFields[i].name);
    }
    {
      RawMuxParamFields raw = all_set_fixture();
      *kFields[i].pick(&raw) = set_field(NAN);
      MuxParamsResult result = mux_params_from_raw(raw);
      CHECK(!result.ok, kFields[i].name);
      CHECK(result.problem == MUX_PARAM_PROBLEM_NOT_FINITE, kFields[i].name);
      CHECK(strcmp(result.missing_field, kFields[i].name) == 0, kFields[i].name);
    }
    {
      RawMuxParamFields raw = all_set_fixture();
      *kFields[i].pick(&raw) = set_field(INFINITY);
      MuxParamsResult result = mux_params_from_raw(raw);
      CHECK(!result.ok, kFields[i].name);
      CHECK(result.problem == MUX_PARAM_PROBLEM_NOT_FINITE, kFields[i].name);
      CHECK(strcmp(result.missing_field, kFields[i].name) == 0, kFields[i].name);
    }
    {
      RawMuxParamFields raw = all_set_fixture();
      *kFields[i].pick(&raw) = set_field(-INFINITY);
      MuxParamsResult result = mux_params_from_raw(raw);
      CHECK(!result.ok, kFields[i].name);
      CHECK(result.problem == MUX_PARAM_PROBLEM_NOT_FINITE, kFields[i].name);
      CHECK(strcmp(result.missing_field, kFields[i].name) == 0, kFields[i].name);
    }
  }

  // Structural refusals.
  for (size_t i = 0; i < sizeof(kRangeCases) / sizeof(kRangeCases[0]); ++i) {
    const RangeCase* c = &kRangeCases[i];
    RawMuxParamFields raw = all_set_fixture();
    *c->pick(&raw) = set_field(c->value);
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, c->name);
    CHECK(result.problem == MUX_PARAM_PROBLEM_RANGE, c->name);
    CHECK(strcmp(result.missing_field, c->expected_field) == 0, c->name);
  }

  // Missing beats non-finite beats structural, and earlier-declared beats later: a startup
  // failure message always names something true rather than a red herring further down.
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.throttle_pwm_neutral_us = unset_field();
    raw.rc_signal_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "two missing fields -> still refused");
    CHECK(strcmp(result.missing_field, "throttle_pwm_neutral_us") == 0,
          "two missing fields -> earlier-declared field is named first");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.steering_pwm_neutral_us = set_field(9000.0);  // structural problem, earlier field
    raw.rc_signal_max_us = unset_field();             // missing, later field
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing + structural -> refused");
    CHECK(result.problem == MUX_PARAM_PROBLEM_MISSING,
          "a missing field is reported before a structural problem, whatever the order");
    CHECK(strcmp(result.missing_field, "rc_signal_max_us") == 0,
          "missing + structural -> the missing field is named");
  }

  // Nothing set at all -> the very first declared field is named.
  {
    RawMuxParamFields raw;
    for (size_t i = 0; i < sizeof(kFields) / sizeof(kFields[0]); ++i) {
      *kFields[i].pick(&raw) = unset_field();
    }
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "nothing set -> refused");
    CHECK(strcmp(result.missing_field, "steering_pwm_min_us") == 0,
          "nothing set -> first-declared field is named");
  }
}
