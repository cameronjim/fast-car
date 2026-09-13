// UNVERIFIED ON HARDWARE. See gpio_irq_dispatch.h (which carries the full explanation of the
// single-callback-per-core constraint this file exists to contain) and
// firmware/safety_mux/README.md.
#include "gpio_irq_dispatch.h"

#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

// One slot per GPIO this firmware takes interrupts on: the RC kill switch, the two Jetson
// PWM inputs, and the Jetson heartbeat (pico/main.c's pin defines) -- four, with headroom.
// Raise this if more interrupt-driven inputs are added; gpio_irq_dispatch_register() returns
// false rather than dropping a registration silently.
#define GPIO_IRQ_DISPATCH_MAX_HANDLERS 8

typedef struct {
  uint gpio;
  GpioIrqDispatchHandler handler;  // NULL == free slot
} DispatchSlot;

static DispatchSlot g_slots[GPIO_IRQ_DISPATCH_MAX_HANDLERS];
static bool g_callback_installed = false;

// THE single shared per-core GPIO callback. Everything about why this is the only place in
// firmware/safety_mux/pico/ allowed to be installed as one is in gpio_irq_dispatch.h; the
// short version is that the SDK keeps one callback per core and a second installer silently
// evicts the first, which is a real bug this firmware already had.
//
// Interrupt context: no allocation, no printf, no blocking. A linear scan over at most
// GPIO_IRQ_DISPATCH_MAX_HANDLERS slots is deliberately the whole implementation.
static void gpio_irq_dispatch_callback(uint gpio, uint32_t events) {
  for (int i = 0; i < GPIO_IRQ_DISPATCH_MAX_HANDLERS; ++i) {
    if (g_slots[i].handler != NULL && g_slots[i].gpio == gpio) {
      g_slots[i].handler(gpio, events);
      return;
    }
  }
  // An edge on a GPIO nobody registered: ignore it. Doing nothing is the fail-safe answer --
  // every consumer of this dispatch treats "no event" as stale/invalid, never as valid.
}

bool gpio_irq_dispatch_register(uint gpio, uint32_t event_mask, GpioIrqDispatchHandler handler) {
  if (handler == NULL) {
    return false;
  }

  DispatchSlot* slot = NULL;
  for (int i = 0; i < GPIO_IRQ_DISPATCH_MAX_HANDLERS; ++i) {
    if (g_slots[i].handler != NULL && g_slots[i].gpio == gpio) {
      slot = &g_slots[i];  // re-registration of the same GPIO: replace its handler
      break;
    }
    if (g_slots[i].handler == NULL && slot == NULL) {
      slot = &g_slots[i];  // remember the first free slot, keep scanning for an exact match
    }
  }
  if (slot == NULL) {
    return false;
  }

  slot->gpio = gpio;
  slot->handler = handler;

  if (!g_callback_installed) {
    // gpio_set_irq_callback() + irq_set_enabled(IO_IRQ_BANK0) rather than
    // gpio_set_irq_enabled_with_callback(): identical effect, but it keeps the
    // one-callback-per-core write visibly in this file, which is the point of this module.
    gpio_set_irq_callback(gpio_irq_dispatch_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    g_callback_installed = true;
  }
  gpio_set_irq_enabled(gpio, event_mask, true);
  return true;
}
