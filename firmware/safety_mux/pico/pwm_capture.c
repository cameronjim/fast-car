// UNVERIFIED ON HARDWARE. See pwm_capture.h and firmware/safety_mux/README.md.
#include "pwm_capture.h"

#include <stdbool.h>
#include <stddef.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define PWM_CAPTURE_MAX_CHANNELS 8

typedef struct {
  uint gpio;
  bool in_use;
  uint64_t rise_us;
  volatile double last_pulse_us;  // -1.0 until the first complete pulse is captured
} PwmCaptureChannel;

static PwmCaptureChannel g_channels[PWM_CAPTURE_MAX_CHANNELS];
static bool g_callback_installed = false;

static PwmCaptureChannel* find_channel(uint gpio) {
  for (int i = 0; i < PWM_CAPTURE_MAX_CHANNELS; ++i) {
    if (g_channels[i].in_use && g_channels[i].gpio == gpio) {
      return &g_channels[i];
    }
  }
  return NULL;
}

static void pwm_capture_irq_handler(uint gpio, uint32_t events) {
  PwmCaptureChannel* ch = find_channel(gpio);
  if (ch == NULL) {
    return;
  }
  uint64_t now_us = time_us_64();
  if (events & GPIO_IRQ_EDGE_RISE) {
    ch->rise_us = now_us;
  } else if (events & GPIO_IRQ_EDGE_FALL) {
    if (ch->rise_us != 0) {
      ch->last_pulse_us = (double)(now_us - ch->rise_us);
    }
  }
}

void pwm_capture_init_channel(uint gpio) {
  PwmCaptureChannel* slot = NULL;
  for (int i = 0; i < PWM_CAPTURE_MAX_CHANNELS; ++i) {
    if (!g_channels[i].in_use) {
      slot = &g_channels[i];
      break;
    }
  }
  // A draft assumption, not a defended invariant: three channels (RC kill switch, Jetson
  // steering, Jetson throttle) are wired per firmware/safety_mux/README.md's pinout table,
  // well under PWM_CAPTURE_MAX_CHANNELS. Silently doing nothing on overflow rather than a
  // hard fault matches this function's void signature; a bench build that actually needs
  // more channels than this constant should raise it, not rely on silent overflow.
  if (slot == NULL) {
    return;
  }

  slot->gpio = gpio;
  slot->in_use = true;
  slot->rise_us = 0;
  slot->last_pulse_us = -1.0;

  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_down(gpio);  // idle/disconnected reads low, not floating

  if (!g_callback_installed) {
    gpio_set_irq_callback(pwm_capture_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    g_callback_installed = true;
  }
  gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

double pwm_capture_read_us(uint gpio) {
  PwmCaptureChannel* ch = find_channel(gpio);
  if (ch == NULL) {
    return -1.0;
  }
  return ch->last_pulse_us;
}
