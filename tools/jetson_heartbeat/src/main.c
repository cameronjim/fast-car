// _POSIX_C_SOURCE must be set before any system header is included: -std=c11 alone hides
// clock_gettime/clock_nanosleep/CLOCK_MONOTONIC/TIMER_ABSTIME (POSIX.1-2001+ extensions, not
// part of plain C11), and this program's whole point is those two calls (see "Design
// constraints" below), so pin the feature-test macro instead of reaching for -std=gnu11.
#define _POSIX_C_SOURCE 200809L

// racer-heartbeat: the Jetson-side half of the layer-1 safety mux's heartbeat watchdog
// (firmware/safety_mux/pico/heartbeat_input.h). Toggles one GPIO line at a fixed rate,
// deliberately independent of ROS, the network stack, and the desktop session -- its only
// job is to prove this Jetson's low-level I/O is alive and being serviced. See
// tools/jetson_heartbeat/README.md for the pin mapping and how that was verified.
//
// Design constraints (see README.md "Requirements" and CLAUDE.md):
//   - fixed-rate square-wave toggle, rate configurable via --rate-hz, default 50 Hz
//   - no heap allocation in the toggle loop, no drift accumulation: absolute deadlines via
//     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...), not a relative sleep() each edge
//   - any error is a nonzero exit with a message on stderr, never a silent stop
#include <errno.h>
#include <gpiod.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "jetson_heartbeat/config.h"

static volatile sig_atomic_t g_stop = 0;

static void handle_stop_signal(int signum) {
  (void)signum;
  g_stop = 1;
}

static void print_usage(const char* argv0) {
  fprintf(stderr,
          "usage: %s [--chip NAME] [--line N] [--rate-hz F]\n"
          "\n"
          "Toggles a GPIO line at a fixed rate as the Jetson-side heartbeat for the\n"
          "layer-1 safety mux watchdog. Runs until SIGTERM/SIGINT.\n"
          "\n"
          "  --chip NAME    gpiochip device name (default: %s)\n"
          "  --line N       GPIO line offset on that chip (default: %u)\n"
          "  --rate-hz F    toggle rate in Hz; an edge every 1/(2F) seconds "
          "(default: %.1f)\n",
          argv0, JETSON_HEARTBEAT_DEFAULT_CHIP, JETSON_HEARTBEAT_DEFAULT_LINE,
          JETSON_HEARTBEAT_DEFAULT_RATE_HZ);
}

static void add_ns(struct timespec* ts, int64_t ns) {
  ts->tv_nsec += ns;
  while (ts->tv_nsec >= 1000000000L) {
    ts->tv_nsec -= 1000000000L;
    ts->tv_sec += 1;
  }
}

int main(int argc, char** argv) {
  HeartbeatConfig cfg;
  char err_buf[256];
  HeartbeatParseStatus status = heartbeat_parse_args(argc, argv, &cfg, err_buf, sizeof(err_buf));
  if (status == kHeartbeatParseHelp) {
    print_usage(argv[0]);
    return 0;
  }
  if (status != kHeartbeatParseOk) {
    fprintf(stderr, "racer-heartbeat: %s\n", err_buf);
    print_usage(argv[0]);
    return 1;
  }

  bool ok = false;
  int64_t half_period_ns = heartbeat_half_period_ns(cfg.rate_hz, &ok);
  if (!ok) {
    fprintf(stderr, "racer-heartbeat: invalid --rate-hz %g\n", cfg.rate_hz);
    return 1;
  }

  struct gpiod_chip* chip = gpiod_chip_open_by_name(cfg.chip_name);
  if (chip == NULL) {
    fprintf(stderr, "racer-heartbeat: cannot open gpiochip '%s': %s\n", cfg.chip_name,
            strerror(errno));
    return 1;
  }

  struct gpiod_line* line = gpiod_chip_get_line(chip, cfg.line_offset);
  if (line == NULL) {
    fprintf(stderr, "racer-heartbeat: cannot get line %u on '%s': %s\n", cfg.line_offset,
            cfg.chip_name, strerror(errno));
    gpiod_chip_close(chip);
    return 1;
  }

  if (gpiod_line_request_output(line, "racer-heartbeat", 0) < 0) {
    fprintf(stderr,
            "racer-heartbeat: cannot request line %u on '%s' as output (already claimed by "
            "another process?): %s\n",
            cfg.line_offset, cfg.chip_name, strerror(errno));
    gpiod_chip_close(chip);
    return 1;
  }

  if (signal(SIGTERM, handle_stop_signal) == SIG_ERR ||
      signal(SIGINT, handle_stop_signal) == SIG_ERR) {
    fprintf(stderr, "racer-heartbeat: cannot install signal handlers: %s\n", strerror(errno));
    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 1;
  }

  struct timespec deadline;
  if (clock_gettime(CLOCK_MONOTONIC, &deadline) != 0) {
    fprintf(stderr, "racer-heartbeat: clock_gettime failed: %s\n", strerror(errno));
    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 1;
  }

  fprintf(stderr,
          "racer-heartbeat: toggling %s line %u at %.3f Hz (edge every %lld ns), consumer "
          "'racer-heartbeat'\n",
          cfg.chip_name, cfg.line_offset, cfg.rate_hz, (long long)half_period_ns);

  int value = 0;
  while (!g_stop) {
    if (gpiod_line_set_value(line, value) < 0) {
      fprintf(stderr, "racer-heartbeat: gpiod_line_set_value failed: %s\n", strerror(errno));
      gpiod_line_release(line);
      gpiod_chip_close(chip);
      return 1;
    }
    value = value ? 0 : 1;

    add_ns(&deadline, half_period_ns);

    int rc;
    do {
      rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    } while (rc == EINTR && !g_stop);
    if (rc != 0 && rc != EINTR) {
      fprintf(stderr, "racer-heartbeat: clock_nanosleep failed: %s\n", strerror(rc));
      gpiod_line_release(line);
      gpiod_chip_close(chip);
      return 1;
    }
  }

  // Clean shutdown (SIGTERM from systemd, or SIGINT from a terminal). We deliberately do not
  // try to leave the line at a particular level: the mux watchdog treats "no edges" as a cut
  // regardless of which level toggling stopped at (firmware/safety_mux/README.md) -- that is
  // the intended fail-safe, not a defect to work around here.
  fprintf(stderr, "racer-heartbeat: stopping on signal, releasing line %u\n", cfg.line_offset);
  gpiod_line_release(line);
  gpiod_chip_close(chip);
  return 0;
}
