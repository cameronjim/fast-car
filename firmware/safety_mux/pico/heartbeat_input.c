// UNVERIFIED ON HARDWARE. See heartbeat_input.h and firmware/safety_mux/README.md.
#include "heartbeat_input.h"

#include <math.h>
#include <stdbool.h>

#include "gpio_irq_dispatch.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Written only in interrupt context, read only under a disabled-interrupt section below.
// `volatile` alone is NOT enough for the timestamp: a uint64_t is two 32-bit accesses on a
// Cortex-M0+, so an edge landing between them can hand the reader a mix of the old high word
// and the new low word -- a timestamp roughly 71 minutes in the past, i.e. a spurious
// watchdog cut. Reading both under save_and_disable_interrupts() removes that window.
static uint64_t g_last_edge_us = 0;
static bool g_edge_seen = false;

// Interrupt context: short and allocation-free. gpio_irq_dispatch only routes edges on the
// GPIO this handler was registered for, so both arguments are known and unused here.
static void heartbeat_irq_handler(uint gpio, uint32_t events) {
  (void)gpio;
  (void)events;
  g_last_edge_us = time_us_64();
  g_edge_seen = true;
}

void heartbeat_input_init(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_down(gpio);
  // Registered through gpio_irq_dispatch, NOT with gpio_set_irq_enabled_with_callback():
  // that SDK call installs THE single per-core GPIO callback and would silently evict
  // pwm_capture.c's handler for the whole bank. It used to be called right here, and it did
  // exactly that. See gpio_irq_dispatch.h. If registration fails (handler table full), no
  // edge is ever recorded, heartbeat_input_age_s() keeps returning +Inf, and
  // watchdog_timed_out() reads that as timed out: fail-closed, never a false "alive".
  (void)gpio_irq_dispatch_register(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                   &heartbeat_irq_handler);
}

double heartbeat_input_age_s(void) {
  uint32_t irq_state = save_and_disable_interrupts();
  bool edge_seen = g_edge_seen;
  uint64_t last_us = g_last_edge_us;
  restore_interrupts(irq_state);

  if (!edge_seen) {
    return INFINITY;
  }
  uint64_t now_us = time_us_64();
  if (now_us < last_us) {
    // time_us_64() wrapping is not reachable on any realistic session length (it wraps after
    // ~584,000 years), but treating it as "unknown, so timed out" is the fail-closed answer
    // if it somehow ever were.
    return INFINITY;
  }
  return (double)(now_us - last_us) / 1e6;
}
