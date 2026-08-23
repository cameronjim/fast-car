// UNVERIFIED ON HARDWARE. See pwm_output.h and firmware/safety_mux/README.md.
#include "pwm_output.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// 50 Hz frame (20000 us period), the standard hobby servo/ESC rate. wrap=39999 with a clkdiv
// chosen so one PWM counter tick = 0.5 us gives convenient, exact microsecond-to-tick math
// (level = pulse_us * 2) without a floating-point divide at runtime.
#define PWM_OUTPUT_WRAP 39999
#define PWM_OUTPUT_US_TO_LEVEL(us) ((uint16_t)((us) * 2.0))

void pwm_output_init_channel(uint gpio) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(gpio);

  // clk_sys is 125 MHz on a stock RP2040; div=62.5 -> 2 MHz counter -> 0.5 us/tick.
  // PICO_SDK note (unverified): confirm clk_sys on the actual bench board before trusting
  // this constant -- if the board runs an overclocked or otherwise non-default clk_sys, this
  // divider must be recomputed from clock_get_hz(clk_sys), not hardcoded.
  pwm_set_clkdiv(slice, 62.5f);
  pwm_set_wrap(slice, PWM_OUTPUT_WRAP);
  pwm_set_enabled(slice, true);
}

void pwm_output_set_us(uint gpio, double pulse_us) {
  uint slice = pwm_gpio_to_slice_num(gpio);
  uint channel = pwm_gpio_to_channel(gpio);
  pwm_set_chan_level(slice, channel, PWM_OUTPUT_US_TO_LEVEL(pulse_us));
}
