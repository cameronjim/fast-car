#include "jetson_heartbeat/config.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_error(char* err_buf, size_t err_buf_len, const char* msg) {
  if (err_buf == NULL || err_buf_len == 0) {
    return;
  }
  snprintf(err_buf, err_buf_len, "%s", msg);
}

// Parses a base-10 unsigned integer strictly: no sign, no trailing garbage, no empty
// string. Returns true and writes *out on success.
static bool parse_uint(const char* text, unsigned int* out) {
  if (text == NULL || text[0] == '\0') {
    return false;
  }
  // strtoul skips leading whitespace and ACCEPTS a leading '-', returning the negation
  // modulo ULONG_MAX+1 with no error. "no sign" has to be enforced here or it is not
  // enforced at all: on a 64-bit host a negative usually wraps above the 32-bit cap below
  // and is rejected by accident, but "-18446744073709551615" wraps to exactly 1 and would
  // have parsed as line 1, and on a 32-bit host "--line -1" would have parsed as line
  // 4294967295. Requiring the first character to be a digit rejects the sign, the leading
  // whitespace, and the empty string in one comparison.
  if (text[0] < '0' || text[0] > '9') {
    return false;
  }
  errno = 0;
  char* end = NULL;
  unsigned long value = strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  if (value > 0xFFFFFFFFul) {
    return false;
  }
  *out = (unsigned int)value;
  return true;
}

// Parses a base-10 double strictly: no trailing garbage, no empty string.
static bool parse_double(const char* text, double* out) {
  if (text == NULL || text[0] == '\0') {
    return false;
  }
  errno = 0;
  char* end = NULL;
  double value = strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *out = value;
  return true;
}

HeartbeatParseStatus heartbeat_parse_args(int argc, char* const* argv, HeartbeatConfig* out,
                                          char* err_buf, size_t err_buf_len) {
  if (out == NULL) {
    set_error(err_buf, err_buf_len, "internal error: out is NULL");
    return kHeartbeatParseError;
  }

  out->chip_name = JETSON_HEARTBEAT_DEFAULT_CHIP;
  out->line_offset = JETSON_HEARTBEAT_DEFAULT_LINE;
  out->rate_hz = JETSON_HEARTBEAT_DEFAULT_RATE_HZ;

  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];

    if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
      return kHeartbeatParseHelp;
    }

    if (strcmp(arg, "--chip") == 0) {
      if (i + 1 >= argc) {
        set_error(err_buf, err_buf_len, "--chip requires a value, e.g. --chip gpiochip0");
        return kHeartbeatParseError;
      }
      out->chip_name = argv[++i];
      if (out->chip_name[0] == '\0') {
        set_error(err_buf, err_buf_len, "--chip must not be empty, e.g. --chip gpiochip0");
        return kHeartbeatParseError;
      }
      continue;
    }

    if (strcmp(arg, "--line") == 0) {
      if (i + 1 >= argc) {
        set_error(err_buf, err_buf_len, "--line requires a value, e.g. --line 144");
        return kHeartbeatParseError;
      }
      unsigned int line = 0;
      if (!parse_uint(argv[++i], &line)) {
        set_error(err_buf, err_buf_len, "--line must be a non-negative integer GPIO line offset");
        return kHeartbeatParseError;
      }
      out->line_offset = line;
      continue;
    }

    if (strcmp(arg, "--rate-hz") == 0) {
      if (i + 1 >= argc) {
        set_error(err_buf, err_buf_len, "--rate-hz requires a value, e.g. --rate-hz 50");
        return kHeartbeatParseError;
      }
      double rate = 0.0;
      if (!parse_double(argv[++i], &rate)) {
        set_error(err_buf, err_buf_len, "--rate-hz must be a number");
        return kHeartbeatParseError;
      }
      out->rate_hz = rate;
      continue;
    }

    char msg[256];
    snprintf(msg, sizeof(msg), "unrecognized argument '%s' (see --help)", arg);
    set_error(err_buf, err_buf_len, msg);
    return kHeartbeatParseError;
  }

  if (!(out->rate_hz > 0.0) || !isfinite(out->rate_hz)) {
    char msg[128];
    snprintf(msg, sizeof(msg), "--rate-hz must be finite and > 0 (got %g)", out->rate_hz);
    set_error(err_buf, err_buf_len, msg);
    return kHeartbeatParseError;
  }
  // An upper bound, because there is no rate this fast that is a heartbeat and there IS a
  // rate this fast that is a fault: at 1e9 Hz the half-period rounds to a single nanosecond
  // and the toggle loop stops sleeping, pinning a core and flooding the GPIO chardev on a
  // machine whose job is to run the control stack. The mux only needs edges comfortably
  // inside its 0.1 s window (README.md "Requirements"); anything above a few kHz is a typo
  // or a unit mistake, not a tuning choice.
  if (out->rate_hz > JETSON_HEARTBEAT_MAX_RATE_HZ) {
    char msg[128];
    snprintf(msg, sizeof(msg), "--rate-hz must be <= %g (got %g)",
             (double)JETSON_HEARTBEAT_MAX_RATE_HZ, out->rate_hz);
    set_error(err_buf, err_buf_len, msg);
    return kHeartbeatParseError;
  }

  return kHeartbeatParseOk;
}

int64_t heartbeat_half_period_ns(double rate_hz, bool* ok) {
  if (ok == NULL) {
    return 0;
  }
  if (!isfinite(rate_hz) || rate_hz <= 0.0) {
    *ok = false;
    return 0;
  }
  double half_period_s = 1.0 / (2.0 * rate_hz);
  double half_period_ns = half_period_s * 1e9;
  if (!isfinite(half_period_ns) || half_period_ns <= 0.0) {
    *ok = false;
    return 0;
  }
  *ok = true;
  return (int64_t)(half_period_ns + 0.5);
}
