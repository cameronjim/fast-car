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
#define PWM_OUTPUT_CLKDIV 62.5f

// The three numbers above are one piece of arithmetic split across three constants, and the
// only thing tying them together used to be a comment. These assertions tie them together at
// build time instead. They emit no code -- the .uf2 is byte-for-byte unchanged by them -- and
// they exist because the 2026-09-20 bench session raised "is the emitted frame actually
// 50 Hz / 1500 us?" and there was no way to answer it from the source alone.
//
// What they pin, and what they deliberately do NOT:
//
//   - DUTY CYCLE is level / (TOP + 1), and nothing else. The RP2040 counter counts 0..TOP and
//     wraps (pico-sdk 2.1.0 hardware_pwm/include/hardware/pwm.h), so the period is TOP+1
//     counts and the output is high for `level` of them. That ratio does not contain clk_sys
//     or the divider anywhere, so NO divider or system-clock error can change the DC average
//     a multimeter reads. It can only change the FREQUENCY.
//   - FREQUENCY is clk_sys / (divider * (TOP + 1)), and that one does depend on clk_sys. The
//     62.5 divider is only correct at 125 MHz. That was a silent assumption in a comment;
//     the assertion below makes a different clk_sys a build failure instead of a servo that
//     mysteriously ignores its signal.
_Static_assert(SYS_CLK_HZ == 125000000u,
               "pwm_output.c's 62.5 clock divider assumes a 125 MHz clk_sys. At any other "
               "system clock the 50 Hz frame rate is wrong (the duty cycle is not: that is "
               "level/(TOP+1) and is clock-independent). Recompute PWM_OUTPUT_CLKDIV from "
               "clock_get_hz(clk_sys) rather than changing this assertion.");
_Static_assert(PWM_OUTPUT_WRAP + 1 == 40000,
               "The frame is TOP+1 counts. 40000 counts at 0.5 us/count is the 20 ms "
               "(50 Hz) hobby servo/ESC frame.");
_Static_assert(PWM_OUTPUT_US_TO_LEVEL(1500.0) == 3000,
               "One count must be 0.5 us for PWM_OUTPUT_US_TO_LEVEL's `* 2` to be a "
               "microsecond-to-count conversion: 1500 us must be 3000 counts.");
_Static_assert(PWM_OUTPUT_US_TO_LEVEL(1500.0) * 8 == (PWM_OUTPUT_WRAP + 1) * 3 / 5,
               "A 1500 us pulse in a 20 ms frame is 7.5 percent duty, i.e. 3000/40000. This "
               "is the number a DC multimeter on the output pin measures (7.5 percent of the "
               "logic level, about 0.25 V at 3.3 V); if this assertion and the meter ever "
               "disagree, the fault is downstream of this file.");

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
  pwm_set_clkdiv(slice, PWM_OUTPUT_CLKDIV);
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

#ifdef SAFETY_MUX_DIAG
// DIAGNOSTIC BUILD ONLY. See pwm_output.h's PwmOutputDiag comment. Read-only.
PwmOutputDiag pwm_output_diag(uint gpio) {
  uint slice = pwm_gpio_to_slice_num(gpio);
  uint channel = pwm_gpio_to_channel(gpio);
  uint32_t div = pwm_hw->slice[slice].div;
  uint32_t cc = pwm_hw->slice[slice].cc;

  PwmOutputDiag d;
  d.slice = slice;
  d.channel = channel;
  d.top = (uint16_t)pwm_hw->slice[slice].top;
  d.div_int = (uint8_t)((div & PWM_CH0_DIV_INT_BITS) >> PWM_CH0_DIV_INT_LSB);
  d.div_frac = (uint8_t)((div & PWM_CH0_DIV_FRAC_BITS) >> PWM_CH0_DIV_FRAC_LSB);
  d.level = (uint16_t)(channel ? ((cc & PWM_CH0_CC_B_BITS) >> PWM_CH0_CC_B_LSB)
                               : ((cc & PWM_CH0_CC_A_BITS) >> PWM_CH0_CC_A_LSB));
  d.enabled = (pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS) != 0u;
  d.pin_is_pwm = (gpio_get_function(gpio) == GPIO_FUNC_PWM);
  d.clk_sys_hz = (uint32_t)clock_get_hz(clk_sys);
  return d;
}
#endif  // SAFETY_MUX_DIAG
