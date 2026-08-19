// UNVERIFIED ON HARDWARE. See power_cutoff.h and firmware/safety_mux/README.md.
#include "power_cutoff.h"

#include "hardware/gpio.h"

void power_cutoff_init(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
  gpio_put(gpio, false);  // power cut until explicitly enabled
}

void power_cutoff_set_enabled(uint gpio, bool enabled) { gpio_put(gpio, enabled); }
