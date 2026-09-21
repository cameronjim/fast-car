#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
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
  // "-18446744073709551615" is the one that mattered: strtoul negates it modulo ULONG_MAX+1
  // to exactly 1, with no error and no overflow, so without an explicit sign rejection it
  // parsed as line 1. "-1" is the same class and happens to be caught by the 32-bit cap on a
  // 64-bit host, but not on a 32-bit one.
  const char* bad_lines[] = {"abc", "-1",   "3.5", "",  "-18446744073709551615", "+7", " 7",
                             "7x",  "0x10", "1e3", "-0"};
  for (size_t i = 0; i < sizeof(bad_lines) / sizeof(bad_lines[0]); ++i) {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--line", (char*)bad_lines[i]};
    HeartbeatConfig cfg;
    char err[256] = {0};
    HeartbeatParseStatus status = parse(3, argv, &cfg, err, sizeof(err));
    CHECK(status == kHeartbeatParseError, bad_lines[i]);
  }
}

static void test_bad_rate_values_error(void) {
  const char* bad_rates[] = {"abc", "0", "-5", "-0.0001", "", "inf", "nan", "-inf", "50x", "1e9"};
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

// The ceiling exists so the toggle loop cannot be turned into a spin; the value just under
// it must still be accepted, or the bound is a regression rather than a guard.
static void test_rate_ceiling(void) {
  {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--rate-hz", (char*)"10000"};
    HeartbeatConfig cfg;
    char err[256] = {0};
    CHECK(parse(3, argv, &cfg, err, sizeof(err)) == kHeartbeatParseOk,
          "the ceiling itself is accepted");
    CHECK(cfg.rate_hz == JETSON_HEARTBEAT_MAX_RATE_HZ, "ceiling rate carried through");
  }
  {
    char* argv[] = {(char*)"racer-heartbeat", (char*)"--rate-hz", (char*)"10000.001"};
    HeartbeatConfig cfg;
    char err[256] = {0};
    CHECK(parse(3, argv, &cfg, err, sizeof(err)) == kHeartbeatParseError,
          "just over the ceiling is rejected");
    CHECK(err[0] != '\0', "over-ceiling rate sets an error message");
  }
}

static void test_empty_chip_errors(void) {
  char* argv[] = {(char*)"racer-heartbeat", (char*)"--chip", (char*)""};
  HeartbeatConfig cfg;
  char err[256] = {0};
  CHECK(parse(3, argv, &cfg, err, sizeof(err)) == kHeartbeatParseError,
        "an empty --chip is an error, not a chip named \"\"");
}

// The arguments EnvironmentFile= substitution into the systemd unit's ExecStart produces,
// once ${RACER_HB_CHIP}/${RACER_HB_LINE}/${RACER_HB_RATE_HZ} are expanded from
// racer-heartbeat.default's shipped values, must parse to exactly the pin mapping
// tools/jetson_heartbeat/README.md documents. If someone edits one without the other, this
// fails rather than the Jetson quietly toggling a different line. (Consistency between
// racer-heartbeat.default's literal contents and these compiled-in defaults is
// test_default_env_file_matches_compiled_defaults(), below.)
static void test_installed_service_arguments(void) {
  char* argv[] = {
      (char*)"racer-heartbeat", (char*)"--chip", (char*)"gpiochip0", (char*)"--line", (char*)"144",
      (char*)"--rate-hz",       (char*)"50"};
  HeartbeatConfig cfg;
  char err[256] = {0};
  CHECK(parse(7, argv, &cfg, err, sizeof(err)) == kHeartbeatParseOk,
        "racer-heartbeat.default's shipped values, once substituted into ExecStart, parse");
  CHECK(strcmp(cfg.chip_name, "gpiochip0") == 0, "service args -> gpiochip0");
  CHECK(cfg.line_offset == 144, "service args -> line 144 (header pin 7, PAC.06)");
  CHECK(cfg.rate_hz == 50.0, "service args -> 50 Hz");

  bool ok = false;
  int64_t half_period_ns = heartbeat_half_period_ns(cfg.rate_hz, &ok);
  CHECK(ok, "service args -> a usable half-period");
  CHECK(half_period_ns == 10000000,
        "service args -> an edge every 10 ms, 10 per the mux's 0.1 s watchdog window");
}

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

// Tiny KEY=VALUE reader for racer-heartbeat.default: no quoting, no expansion, just what the
// systemd EnvironmentFile= parser and this test both need to agree on. Lines that are blank,
// pure whitespace, or start with '#' (after skipping leading whitespace) are comments; every
// other line must be NAME=VALUE with no spaces around '=', which is what
// systemd.exec(5)'s EnvironmentFile= documents and what racer-heartbeat.default is written
// to. Returns true and writes *out (NUL-terminated, truncated to out_len - 1 if needed) if
// `key` was found.
static bool read_default_env_value(const char* path, const char* key, char* out,
                                   size_t out_len) {
  FILE* f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  char line[256];
  size_t key_len = strlen(key);
  bool found = false;
  while (fgets(line, sizeof(line), f) != NULL) {
    char* p = line;
    while (*p == ' ' || *p == '\t') {
      ++p;
    }
    if (*p == '#' || *p == '\n' || *p == '\0') {
      continue;
    }
    if (strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
      char* value = p + key_len + 1;
      size_t value_len = strlen(value);
      while (value_len > 0 &&
             (value[value_len - 1] == '\n' || value[value_len - 1] == '\r')) {
        value[--value_len] = '\0';
      }
      snprintf(out, out_len, "%s", value);
      found = true;
      break;
    }
  }
  fclose(f);
  return found;
}

// racer-heartbeat.default (installed to /etc/default/racer-heartbeat by `make install`, read
// by systemd/racer-heartbeat.service's EnvironmentFile=) ships the SAME chip/line/rate as
// this binary's own compiled-in defaults, so that a fresh install (env file present, unedited)
// and a bare invocation with no flags (env file absent, per systemd's "-" prefix on
// EnvironmentFile=) behave identically. If someone edits one without the other, this fails
// rather than the two silently drifting apart. Run from tools/jetson_heartbeat/ (both `make
// test` and .github/scripts/jetson_heartbeat_host_tests.sh do this), so the path below is
// relative to that directory, not to this test file.
static void test_default_env_file_matches_compiled_defaults(void) {
  const char* path = "racer-heartbeat.default";
  char value[64];

  CHECK(read_default_env_value(path, "RACER_HB_CHIP", value, sizeof(value)),
        "racer-heartbeat.default has RACER_HB_CHIP");
  CHECK(strcmp(value, JETSON_HEARTBEAT_DEFAULT_CHIP) == 0,
        "RACER_HB_CHIP matches the compiled-in default chip");

  CHECK(read_default_env_value(path, "RACER_HB_LINE", value, sizeof(value)),
        "racer-heartbeat.default has RACER_HB_LINE");
  char expected_line[32];
  snprintf(expected_line, sizeof(expected_line), "%u", JETSON_HEARTBEAT_DEFAULT_LINE);
  CHECK(strcmp(value, expected_line) == 0,
        "RACER_HB_LINE matches the compiled-in default line");

  CHECK(read_default_env_value(path, "RACER_HB_RATE_HZ", value, sizeof(value)),
        "racer-heartbeat.default has RACER_HB_RATE_HZ");
  CHECK(strtod(value, NULL) == JETSON_HEARTBEAT_DEFAULT_RATE_HZ,
        "RACER_HB_RATE_HZ matches the compiled-in default rate");
}

void test_heartbeat_config_suite(void) {
  test_defaults_with_no_args();
  test_overrides_all_three();
  test_help_flags();
  test_unknown_flag_errors();
  test_missing_values_error();
  test_bad_line_values_error();
  test_bad_rate_values_error();
  test_rate_ceiling();
  test_empty_chip_errors();
  test_installed_service_arguments();
  test_half_period_ns();
  test_default_env_file_matches_compiled_defaults();
}
