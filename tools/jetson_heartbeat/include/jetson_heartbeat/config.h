// Pure argument-parsing and rate/period math for racer-heartbeat -- no libgpiod, no file or
// GPIO I/O, host-testable with plain gcc (see ../tests/). The GPIO side (src/main.c) is not
// unit-tested for the same reason firmware/safety_mux/pico/heartbeat_input.h is not: real
// GPIO access has no host equivalent.
#ifndef JETSON_HEARTBEAT_CONFIG_H_
#define JETSON_HEARTBEAT_CONFIG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Defaults. These are this standalone process's OWN operating parameters (which GPIO line
// to toggle, how fast) -- not a vehicle physical constant, so CLAUDE.md invariant 2 (all
// physical constants live in config/vehicle_params.yaml) does not apply to them. The pin
// mapping is recorded in README.md and cross-checked there against the mux firmware's
// pinout table; it does not flow through tools/gen_params.py because it is a fact about
// this Jetson's 40-pin header + the mux wiring harness, not about the vehicle.
//
// gpiochip0 line 144 ("PAC.06" per gpioinfo) == Jetson 40-pin header physical pin 7, as
// determined 2026-09-12 by reading this exact carrier board's live pinmux device tree via
// NVIDIA's own /opt/nvidia/jetson-io tooling (not assumed from a generic pinout diagram).
// See README.md "GPIO mapping" for the full derivation and the toggling verification.
#define JETSON_HEARTBEAT_DEFAULT_CHIP "gpiochip0"
#define JETSON_HEARTBEAT_DEFAULT_LINE 144u
#define JETSON_HEARTBEAT_DEFAULT_RATE_HZ 50.0

// Sanity ceiling on --rate-hz. Not a physical constant and not a tuning knob: anything above
// this is a typo or a units mistake (see heartbeat_parse_args), and accepting it turns the
// toggle loop into a spin that starves the rest of the Jetson.
#define JETSON_HEARTBEAT_MAX_RATE_HZ 10000.0

// mux_watchdog_timeout_s (config/vehicle_params.yaml, currently 0.1 s PROVISIONAL) is
// deliberately NOT read here at runtime. Two reasons, both load-bearing:
//   1. Reading it would mean parsing YAML/JSON on the critical path of a process whose only
//      job is to prove liveness independent of every other software stack on this Jetson
//      (claude-docs/05-safety.md); the generated vehicle_params binding depends on
//      tools/gen_params.py's Python/jsonschema/yaml toolchain, which is exactly the kind of
//      dependency this process exists to not need.
//   2. The comparison against the timeout happens on the MCU side
//      (firmware/safety_mux/logic/src/watchdog.c), which DOES load that field from its own
//      generated C binding. This process's only obligation is to toggle comfortably faster
//      than that timeout (default 50 Hz => an edge every 10 ms against a 100 ms timeout,
//      roughly 10 edges per window); the operator picks --rate-hz with that margin in mind
//      when the timeout changes, and README.md states the margin explicitly so it is not
//      silently invalidated by a future timeout edit.
typedef struct {
  const char* chip_name;     // e.g. "gpiochip0"
  unsigned int line_offset;  // e.g. 144
  double rate_hz;            // toggle rate in Hz, must be finite and > 0
} HeartbeatConfig;

typedef enum {
  kHeartbeatParseOk = 0,
  kHeartbeatParseHelp,   // -h/--help was given; caller should print usage and exit 0
  kHeartbeatParseError,  // bad argument; err_buf holds a human-readable reason
} HeartbeatParseStatus;

// Parses argv[1..argc-1] into *out, which starts from the defaults above and is overridden
// field-by-field by any of: --chip NAME, --line N, --rate-hz F, -h/--help. Pure function:
// no I/O, no global state, safe to call repeatedly in tests. On kHeartbeatParseError,
// err_buf (a caller-owned buffer of err_buf_len bytes) holds a NUL-terminated message.
HeartbeatParseStatus heartbeat_parse_args(int argc, char* const* argv, HeartbeatConfig* out,
                                          char* err_buf, size_t err_buf_len);

// Converts a toggle rate in Hz to the half-period in nanoseconds: the time the line spends
// in each of the high/low state before flipping (an edge every half-period, matching the
// square-wave design in firmware/safety_mux/pico/heartbeat_input.h's comment). Returns the
// half-period and sets *ok = true for any finite rate_hz > 0; returns 0 and sets *ok = false
// otherwise (non-positive, NaN, +-Inf) -- the caller must treat *ok = false as a hard error,
// never substitute a default silently (that defaulting already happened in
// heartbeat_parse_args, once, at the boundary).
int64_t heartbeat_half_period_ns(double rate_hz, bool* ok);

#ifdef __cplusplus
}
#endif

#endif  // JETSON_HEARTBEAT_CONFIG_H_
