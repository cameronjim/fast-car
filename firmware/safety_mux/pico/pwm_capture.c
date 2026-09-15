// UNVERIFIED ON HARDWARE. See pwm_capture.h and firmware/safety_mux/README.md.
#include "pwm_capture.h"

#include <stdbool.h>
#include <stddef.h>

#include "gpio_irq_dispatch.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "safety_mux/pwm_validity.h"

#define PWM_CAPTURE_MAX_CHANNELS 8

// A pulse is only believed if its width is inside this band. It is deliberately MUCH wider
// than any vehicle_params PWM range: this is not a second copy of the mux's validity check
// (logic/pwm_validity.c, driven by config/vehicle_params.yaml, is the only thing that
// decides whether a pulse is a legal COMMAND). It only rejects widths no RC/servo source can
// physically produce -- a nanosecond noise spike on the level shifter, or a "pulse" longer
// than a frame -- so that electrical noise cannot overwrite a good reading and drop the mux
// into a cut for a frame. A channel receiving nothing but noise still cuts, because an
// implausible pulse does not refresh the freshness timestamp below either.
#define PWM_CAPTURE_MIN_PLAUSIBLE_US 200.0
#define PWM_CAPTURE_MAX_PLAUSIBLE_US 5000.0

// How long a captured width stays believable with no new pulse behind it. At the 50 Hz frame
// rate every source here runs at (hobby servo/ESC convention; the Jetson side is
// ros_ws/src/racer_drivers/pwm_output_node's output_rate_hz, default 50 Hz) a pulse arrives
// every 20 ms, so this is three missed frames.
//
// This is what makes a STUCK input fail closed. Without it, a line that goes high and stays
// high (a shorted level shifter, a receiver that latches, a Jetson PWM peripheral left at
// 100% duty) produces no further falling edges, and the last width captured before the fault
// would have been reported as a fresh, valid command forever.
//
// COMPILE-TIME, AND THAT IS A KNOWN GAP -- like rc_switch.h's hysteresis, this wants to be a
// vehicle_params field once the frame rate is settled on the bench. See
// docs/notes/firmware-review-2026-09-14.md's subject-to-change list.
#define PWM_CAPTURE_MAX_AGE_US 60000u

typedef struct {
  uint gpio;
  bool in_use;
  // Written only in interrupt context, read only under a disabled-interrupt section (see
  // pwm_capture_read_us). On this core an interrupt cannot preempt a disabled-interrupt
  // section, so the pair below is always read as one consistent sample -- `volatile` alone
  // would NOT give that: a double and a uint64_t are each two 32-bit accesses on a Cortex-M0+
  // and the main loop could read half of one pulse and half of the next.
  uint64_t rise_us;
  bool have_rise;
  uint32_t last_pulse_us;      // 0 until the first plausible complete pulse is captured
  uint64_t last_pulse_end_us;  // when that pulse's falling edge arrived
  bool have_pulse;
} PwmCaptureChannel;

static PwmCaptureChannel g_channels[PWM_CAPTURE_MAX_CHANNELS];

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
    ch->have_rise = true;
    return;
  }
  if (events & GPIO_IRQ_EDGE_FALL) {
    // A falling edge with no rising edge behind it (the first edge after boot, or a rising
    // edge lost to a missed interrupt) measures nothing. Drop it rather than measuring from
    // a stale rise, which would report a width spanning several frames.
    if (!ch->have_rise) {
      return;
    }
    ch->have_rise = false;
    double width_us = (double)(now_us - ch->rise_us);
    if (!pwm_is_valid_us(width_us, PWM_CAPTURE_MIN_PLAUSIBLE_US, PWM_CAPTURE_MAX_PLAUSIBLE_US)) {
      return;  // noise: neither recorded nor counted as freshness
    }
    ch->last_pulse_us = (uint32_t)width_us;
    ch->last_pulse_end_us = now_us;
    ch->have_pulse = true;
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
  slot->rise_us = 0;
  slot->have_rise = false;
  slot->last_pulse_us = 0;
  slot->last_pulse_end_us = 0;
  slot->have_pulse = false;
  slot->in_use = true;  // set last: the IRQ handler finds channels by this flag

  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_down(gpio);  // idle/disconnected reads low, not floating

  // Registered through gpio_irq_dispatch, NOT with gpio_set_irq_callback() /
  // gpio_set_irq_enabled_with_callback() directly: the SDK keeps ONE GPIO callback per core,
  // so installing one here would silently evict heartbeat_input.c's (and vice versa). See
  // gpio_irq_dispatch.h for the full explanation -- that exact collision was a real bug in
  // this file. If registration fails (handler table full), this channel simply never records
  // an edge, so pwm_capture_read_us() keeps returning -1.0 and the mux treats it as invalid:
  // the same fail-safe direction as a disconnected input, never a false "valid" reading.
  (void)gpio_irq_dispatch_register(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                   pwm_capture_irq_handler);
}

double pwm_capture_read_us(uint gpio) {
  PwmCaptureChannel* ch = find_channel(gpio);
  if (ch == NULL) {
    return -1.0;
  }

  // Read the width and the time it was captured as ONE sample. Interrupts off for a handful
  // of instructions; the capture handler is the only writer and it runs on this core.
  uint32_t irq_state = save_and_disable_interrupts();
  bool have_pulse = ch->have_pulse;
  uint32_t width_us = ch->last_pulse_us;
  uint64_t end_us = ch->last_pulse_end_us;
  restore_interrupts(irq_state);

  if (!have_pulse) {
    return -1.0;
  }
  uint64_t now_us = time_us_64();
  if (now_us < end_us || (now_us - end_us) > (uint64_t)PWM_CAPTURE_MAX_AGE_US) {
    return -1.0;  // stuck, disconnected, or slower than the frame rate: stale is not valid
  }
  return (double)width_us;
}
