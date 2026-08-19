// UNVERIFIED ON HARDWARE. See heartbeat_input.h and firmware/safety_mux/README.md.
#include "heartbeat_input.h"

#include <math.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

static volatile uint64_t g_last_edge_us = 0;
static volatile bool g_edge_seen = false;

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
  gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
                                     &heartbeat_irq_handler);
}

double heartbeat_input_age_s(void) {
  if (!g_edge_seen) {
    return INFINITY;
  }
  uint64_t now_us = time_us_64();
  uint64_t last_us = g_last_edge_us;  // single volatile read; good enough for a coarse age
  if (now_us < last_us) {
    // time_us_64() wrapping is not reachable on any realistic session length (it wraps after
    // ~584,000 years), but treating it as "unknown, so timed out" is the fail-closed answer
    // if it somehow ever were.
    return INFINITY;
  }
  return (double)(now_us - last_us) / 1e6;
}
