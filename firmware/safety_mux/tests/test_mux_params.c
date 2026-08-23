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
// "one field missing" test below starts from a fresh copy of this and clears exactly one
// field, isolating that field as the single variable under test.
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
  raw.rc_signal_min_us = set_field(1000.0);
  raw.rc_signal_max_us = set_field(2000.0);
  return raw;
}

void test_mux_params_suite(void) {
  // Happy path: every field set -> ok, and every value carried through unchanged.
  {
    RawMuxParamFields raw = all_set_fixture();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(result.ok, "all fields set -> ok");
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
    CHECK(result.params.rc_signal_min_us == 1000.0, "rc_signal_min_us carried through");
    CHECK(result.params.rc_signal_max_us == 2000.0, "rc_signal_max_us carried through");
  }

  // One test per field: clearing exactly that field refuses with exactly that field named.
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.steering_pwm_min_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing steering_pwm_min_us -> refused");
    CHECK(strcmp(result.missing_field, "steering_pwm_min_us") == 0,
          "missing steering_pwm_min_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.steering_pwm_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing steering_pwm_max_us -> refused");
    CHECK(strcmp(result.missing_field, "steering_pwm_max_us") == 0,
          "missing steering_pwm_max_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.steering_pwm_neutral_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing steering_pwm_neutral_us -> refused");
    CHECK(strcmp(result.missing_field, "steering_pwm_neutral_us") == 0,
          "missing steering_pwm_neutral_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.throttle_pwm_min_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing throttle_pwm_min_us -> refused");
    CHECK(strcmp(result.missing_field, "throttle_pwm_min_us") == 0,
          "missing throttle_pwm_min_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.throttle_pwm_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing throttle_pwm_max_us -> refused");
    CHECK(strcmp(result.missing_field, "throttle_pwm_max_us") == 0,
          "missing throttle_pwm_max_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.throttle_pwm_neutral_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing throttle_pwm_neutral_us -> refused");
    CHECK(strcmp(result.missing_field, "throttle_pwm_neutral_us") == 0,
          "missing throttle_pwm_neutral_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.watchdog_timeout_s = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing watchdog_timeout_s -> refused");
    CHECK(strcmp(result.missing_field, "watchdog_timeout_s") == 0,
          "missing watchdog_timeout_s -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.kill_switch_threshold_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing kill_switch_threshold_us -> refused");
    CHECK(strcmp(result.missing_field, "kill_switch_threshold_us") == 0,
          "missing kill_switch_threshold_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.rc_signal_min_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing rc_signal_min_us -> refused");
    CHECK(strcmp(result.missing_field, "rc_signal_min_us") == 0,
          "missing rc_signal_min_us -> named correctly");
  }
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.rc_signal_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "missing rc_signal_max_us -> refused");
    CHECK(strcmp(result.missing_field, "rc_signal_max_us") == 0,
          "missing rc_signal_max_us -> named correctly");
  }

  // Two fields missing at once: the FIRST one in declaration order is reported, so a startup
  // failure message always names something true (never a red herring further down the list).
  {
    RawMuxParamFields raw = all_set_fixture();
    raw.throttle_pwm_neutral_us = unset_field();
    raw.rc_signal_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "two missing fields -> still refused");
    CHECK(strcmp(result.missing_field, "throttle_pwm_neutral_us") == 0,
          "two missing fields -> earlier-declared field is named first");
  }

  // Nothing set at all -> the very first declared field is named.
  {
    RawMuxParamFields raw;
    raw.steering_pwm_min_us = unset_field();
    raw.steering_pwm_max_us = unset_field();
    raw.steering_pwm_neutral_us = unset_field();
    raw.throttle_pwm_min_us = unset_field();
    raw.throttle_pwm_max_us = unset_field();
    raw.throttle_pwm_neutral_us = unset_field();
    raw.watchdog_timeout_s = unset_field();
    raw.kill_switch_threshold_us = unset_field();
    raw.rc_signal_min_us = unset_field();
    raw.rc_signal_max_us = unset_field();
    MuxParamsResult result = mux_params_from_raw(raw);
    CHECK(!result.ok, "nothing set -> refused");
    CHECK(strcmp(result.missing_field, "steering_pwm_min_us") == 0,
          "nothing set -> first-declared field is named");
  }
}
