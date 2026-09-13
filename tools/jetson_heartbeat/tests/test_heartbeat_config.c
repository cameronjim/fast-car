#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "framework.h"
#include "jetson_heartbeat/config.h"

static HeartbeatParseStatus parse(int argc, char* const* argv, HeartbeatConfig* out, char* err,
                                  size_t err_len) {
  return heartbeat_parse_args(argc, argv, out, err, err_len);
}

static void test_defaults_with_no_args(void) {
  char* argv[] = {(char*)"racer-heartbeat"};
  HeartbeatConfig cfg;
  char err[256] = {0};
  HeartbeatParseStatus status = parse(1, argv, &cfg, err, sizeof(err));

  CHECK(status == kHeartbeatParseOk, "no args parses OK");
  CHECK(strcmp(cfg.chip_name, JETSON_HEARTBEAT_DEFAULT_CHIP) == 0, "default chip");
  CHECK(cfg.line_offset == JETSON_HEARTBEAT_DEFAULT_LINE, "default line");
  CHECK(cfg.rate_hz == JETSON_HEARTBEAT_DEFAULT_RATE_HZ, "default rate");
}

static void test_overrides_all_three(void) {
  char* argv[] = {
      (char*)"racer-heartbeat", (char*)"--chip", (char*)"gpiochip1", (char*)"--line", (char*)"7",
      (char*)"--rate-hz",       (char*)"100"};
  HeartbeatConfig cfg;
  char err[256] = {0};
  HeartbeatParseStatus status = parse(7, argv, &cfg, err, sizeof(err));

  CHECK(status == kHeartbeatParseOk, "overrides parse OK");
  CHECK(strcmp(cfg.chip_name, "gpiochip1") == 0, "chip overridden");
  CHECK(cfg.line_offset == 7, "line overridden");
  CHECK(cfg.rate_hz == 100.0, "rate overridden");
}

static void test_help_flags(void) {
  {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"-h"};
    HeartbeatConfig cfg;
    char err[256] = {0};
    CHECK(parse(2, argv, &cfg, err, sizeof(err)) == kHeartbeatParseHelp, "-h is help");
  }
  {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--help"};
    HeartbeatConfig cfg;
    char err[256] = {0};
    CHECK(parse(2, argv, &cfg, err, sizeof(err)) == kHeartbeatParseHelp, "--help is help");
  }
}

static void test_unknown_flag_errors(void) {
  char* argv[] = {(char*)"racer-heartbeat", (char*)"--bogus"};
  HeartbeatConfig cfg;
  char err[256] = {0};
  HeartbeatParseStatus status = parse(2, argv, &cfg, err, sizeof(err));
  CHECK(status == kHeartbeatParseError, "unknown flag is an error");
  CHECK(err[0] != '\0', "unknown flag sets an error message");
}

static void test_missing_values_error(void) {
  const char* flags[] = {"--chip", "--line", "--rate-hz"};
  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i) {
    char* argv[] = {(char*)"racer-heartbeat", (char*)flags[i]};
    HeartbeatConfig cfg;
    char err[256] = {0};
    HeartbeatParseStatus status = parse(2, argv, &cfg, err, sizeof(err));
    CHECK(status == kHeartbeatParseError, flags[i]);
  }
}

static void test_bad_line_values_error(void) {
  const char* bad_lines[] = {"abc", "-1", "3.5", ""};
  for (size_t i = 0; i < sizeof(bad_lines) / sizeof(bad_lines[0]); ++i) {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--line", (char*)bad_lines[i]};
    HeartbeatConfig cfg;
    char err[256] = {0};
    HeartbeatParseStatus status = parse(3, argv, &cfg, err, sizeof(err));
    CHECK(status == kHeartbeatParseError, bad_lines[i]);
  }
}

static void test_bad_rate_values_error(void) {
  const char* bad_rates[] = {"abc", "0", "-5", "-0.0001"};
  for (size_t i = 0; i < sizeof(bad_rates) / sizeof(bad_rates[0]); ++i) {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--rate-hz", (char*)bad_rates[i]};
    HeartbeatConfig cfg;
    char err[256] = {0};
    HeartbeatParseStatus status = parse(3, argv, &cfg, err, sizeof(err));
    CHECK(status == kHeartbeatParseError, bad_rates[i]);
  }
}

typedef struct {
  const char* name;
  double rate_hz;
  bool expected_ok;
  int64_t expected_half_period_ns;  // only checked when expected_ok
} HalfPeriodCase;

static const HalfPeriodCase kHalfPeriodCases[] = {
    {"50 Hz -> 10 ms half-period (edge every 10 ms)", 50.0, true, 10000000},
    {"100 Hz -> 5 ms half-period", 100.0, true, 5000000},
    {"1 Hz -> 500 ms half-period", 1.0, true, 500000000},
    {"zero rate is rejected", 0.0, false, 0},
    {"negative rate is rejected", -50.0, false, 0},
    {"NaN rate is rejected", NAN, false, 0},
    {"+Inf rate is rejected", INFINITY, false, 0},
    {"-Inf rate is rejected", -INFINITY, false, 0},
};

static void test_half_period_ns(void) {
  for (size_t i = 0; i < sizeof(kHalfPeriodCases) / sizeof(kHalfPeriodCases[0]); ++i) {
    const HalfPeriodCase* c = &kHalfPeriodCases[i];
    bool ok = false;
    int64_t half_period_ns = heartbeat_half_period_ns(c->rate_hz, &ok);
    CHECK(ok == c->expected_ok, c->name);
    if (c->expected_ok) {
      CHECK(half_period_ns == c->expected_half_period_ns, c->name);
    }
  }
}

void test_heartbeat_config_suite(void) {
  test_defaults_with_no_args();
  test_overrides_all_three();
  test_help_flags();
  test_unknown_flag_errors();
  test_missing_values_error();
  test_bad_line_values_error();
  test_bad_rate_values_error();
  test_half_period_ns();
}
