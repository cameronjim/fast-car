// UNVERIFIED ON HARDWARE. See pwm_output.h and firmware/safety_mux/README.md.
#include "pwm_output.h"

#include <math.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// 50 Hz frame (20000 us period), the standard hobby servo/ESC rate. wrap=39999 with a clkdiv
// chosen so one PWM counter tick = 0.5 us gives convenient, exact microsecond-to-tick math
// (level = pulse_us * 2) without a floating-point divide at runtime.
#define PWM_OUTPUT_WRAP 39999
#define PWM_OUTPUT_US_TO_LEVEL(us) ((uint16_t)((us)*2.0))

void pwm_output_init_safe(uint gpio) {
  // Called in the FIRST lines of main(), before params are even read. An RP2040 comes out of
  // reset with its GPIOs as high-impedance inputs: a floating servo signal line can twitch
  // the servo, and a floating ESC signal line is exactly the condition some ESCs arm on. So
  // the very first thing that happens to these pins is being driven, LOW, as plain outputs.
  //
  // Low means "no pulses", which is not the same as the mux's neutral output and is not a
  // substitute for it (claude-docs/05-safety.md: a cut drives a configured neutral, never
  // merely an absent signal). It is the correct state for the handful of milliseconds BEFORE
  // a neutral is known: no servo command and no ESC arming sequence, from a defined level.
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
  gpio_put(gpio, false);
}

void pwm_output_init_channel(uint gpio, double initial_pulse_us) {
  uint slice = pwm_gpio_to_slice_num(gpio);

  // clk_sys is 125 MHz on a stock RP2040; div=62.5 -> 2 MHz counter -> 0.5 us/tick.
  // PICO_SDK note (unverified): confirm clk_sys on the actual bench board before trusting
  // this constant -- if the board runs an overclocked or otherwise non-default clk_sys, this
  // divider must be recomputed from clock_get_hz(clk_sys), not hardcoded.
  pwm_set_clkdiv(slice, 62.5f);
  pwm_set_wrap(slice, PWM_OUTPUT_WRAP);

  // Level BEFORE the pin is handed to the PWM peripheral, and before the slice runs. The
  // compare level resets to 0, so configuring the pin first would put a frame or more of 0%
  // duty on the wire before the first pwm_output_set_us() call -- a gap between "the pin is
  // a PWM output" and "the PWM output says neutral". There is no such gap this way.
  pwm_output_set_us(gpio, initial_pulse_us);
  pwm_set_enabled(slice, true);
  gpio_set_function(gpio, GPIO_FUNC_PWM);
}

void pwm_output_set_us(uint gpio, double pulse_us) {
  uint slice = pwm_gpio_to_slice_num(gpio);
  uint channel = pwm_gpio_to_channel(gpio);

  // mux_decide() only ever returns a passed-through pulse that pwm_is_valid_us() accepted or
  // a configured neutral that mux_params_from_raw() accepted, so a value outside the frame
  // should be unreachable. Clamp anyway: the cast in PWM_OUTPUT_US_TO_LEVEL is undefined for
  // NaN and wraps for anything past 32767.5 us, and "undefined" on the pin that drives the
  // ESC is not a risk worth carrying to save two comparisons at 200 Hz.
  double level_us = pulse_us;
  if (!isfinite(level_us) || level_us < 0.0) {
    level_us = 0.0;  // no pulse: the same defined, non-arming state as pwm_output_init_safe
  }
  // The counter wraps at PWM_OUTPUT_WRAP, so a level above it is permanently high rather
  // than a pulse. Half a tick under the wrap is the longest expressible pulse.
  const double max_us = (double)PWM_OUTPUT_WRAP / 2.0;
  if (level_us > max_us) {
    level_us = max_us;
  }
  pwm_set_chan_level(slice, channel, PWM_OUTPUT_US_TO_LEVEL(level_us));
}
