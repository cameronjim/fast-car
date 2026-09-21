// DIAGNOSTIC BUILD ONLY. See diag_report.h and docs/notes/mux-diagnostic-build.md.
#include "diag_report.h"

#ifdef SAFETY_MUX_DIAG

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pwm_capture.h"
#include "pwm_output.h"
#include "safety_mux/pwm_validity.h"
#include "safety_mux/rc_switch.h"
#include "safety_mux/watchdog.h"

#define DIAG_REPORT_PERIOD_US 500000u  // about twice a second
#define DIAG_LINE_MAX 1024

static DiagGpioMap g_gpios;
static MuxParams g_params;
static uint64_t g_next_report_us = 0;
static bool g_was_connected = false;
static uint32_t g_seq = 0;

// Microseconds as a plain integer for printing. -1 stands for "no believable reading", which
// is exactly the value pwm_capture_read_us() itself uses for that case.
static int32_t us_int(double us) {
  if (!isfinite(us) || us < 0.0) {
    return -1;
  }
  return (int32_t)(us + 0.5);
}

// One capture channel, rendered as "<width>us <status>". The status is the point of this
// whole build: pwm_capture_read_us() returns -1.0 for three different faults and the mux
// treats all three identically, but a human needs to tell "the wire is dead" from "the pulse
// is the wrong width".
static void format_channel(char* out, size_t out_len, uint gpio, double reading_us,
                           double range_min_us, double range_max_us) {
  PwmCaptureDiag d = pwm_capture_diag(gpio);
  if (!d.registered) {
    snprintf(out, out_len, "gp%u=?? NOT_REGISTERED", gpio);
    return;
  }
  if (!d.have_pulse) {
    snprintf(out, out_len,
             "gp%u=--- NO_EDGES(no plausible pulse since boot; line dead/unplugged/noise)", gpio);
    return;
  }
  if (d.stale) {
    snprintf(out, out_len, "gp%u=%luus STALE(last edge %lums ago, window %lums; stuck or stopped)",
             gpio, (unsigned long)d.last_pulse_us, (unsigned long)(d.age_us / 1000u),
             (unsigned long)(d.max_age_us / 1000u));
    return;
  }
  if (!pwm_is_valid_us(reading_us, range_min_us, range_max_us)) {
    snprintf(out, out_len, "gp%u=%ldus OUT_OF_RANGE(allowed %ld-%ld)", gpio,
             (long)us_int(reading_us), (long)us_int(range_min_us), (long)us_int(range_max_us));
    return;
  }
  snprintf(out, out_len, "gp%u=%ldus FRESH(%lums, in %ld-%ld)", gpio, (long)us_int(reading_us),
           (unsigned long)(d.age_us / 1000u), (long)us_int(range_min_us),
           (long)us_int(range_max_us));
}

// Renders what the PWM hardware is ACTUALLY emitting on one output pin, derived from the
// peripheral registers rather than from the value that was commanded. Duty is level/(TOP+1)
// and contains no clock term at all, so it is what a DC multimeter on the pin measures; the
// frame period and frequency do contain clk_sys, which is read rather than assumed. All
// integer arithmetic: no float formatting on a Cortex-M0+, and no rounding surprises.
static void format_pwm_reg(char* out, size_t out_len, const char* name, uint gpio) {
  PwmOutputDiag d = pwm_output_diag(gpio);
  uint32_t div_x16 = ((uint32_t)d.div_int * 16u) + d.div_frac;  // divider in 1/16ths
  uint32_t frame_counts = (uint32_t)d.top + 1u;
  if (div_x16 == 0u || d.clk_sys_hz == 0u) {
    snprintf(out, out_len, "%s gp%u slice%u.%c top=%u div=0 level=%u BAD_DIVIDER", name, gpio,
             d.slice, d.channel ? 'B' : 'A', (unsigned)d.top, (unsigned)d.level);
    return;
  }
  // tick = div_x16 / (16 * clk_sys) seconds. Scaled to tenths of a microsecond.
  uint64_t pulse_us_x10 =
      ((uint64_t)d.level * div_x16 * 10000000ull) / (16ull * (uint64_t)d.clk_sys_hz);
  uint64_t frame_us_x10 =
      ((uint64_t)frame_counts * div_x16 * 10000000ull) / (16ull * (uint64_t)d.clk_sys_hz);
  uint64_t hz_x100 = (16ull * (uint64_t)d.clk_sys_hz * 100ull) / ((uint64_t)div_x16 * frame_counts);
  uint32_t duty_x100 = (uint32_t)(((uint64_t)d.level * 10000ull) / frame_counts);
  uint32_t div_x1000 = (div_x16 * 1000u) / 16u;

  snprintf(out, out_len,
           "%s gp%u slice%u.%c top=%u div=%lu.%03lu level=%u en=%d pinfn=%s -> pulse=%lu.%lu"
           "us frame=%lu.%luus %lu.%02luHz duty=%lu.%02lu%%",
           name, gpio, d.slice, d.channel ? 'B' : 'A', (unsigned)d.top,
           (unsigned long)(div_x1000 / 1000u), (unsigned long)(div_x1000 % 1000u),
           (unsigned)d.level, d.enabled ? 1 : 0, d.pin_is_pwm ? "PWM" : "NOT_PWM",
           (unsigned long)(pulse_us_x10 / 10u), (unsigned long)(pulse_us_x10 % 10u),
           (unsigned long)(frame_us_x10 / 10u), (unsigned long)(frame_us_x10 % 10u),
           (unsigned long)(hz_x100 / 100u), (unsigned long)(hz_x100 % 100u),
           (unsigned long)(duty_x100 / 100u), (unsigned long)(duty_x100 % 100u));
}

static const char* switch_text(RcSwitchPosition p) {
  switch (p) {
    case RC_SWITCH_ARMED:
      return "ARMED";
    case RC_SWITCH_KILL:
      return "KILLED";
    case RC_SWITCH_SIGNAL_INVALID:
      return "UNREADABLE";
  }
  return "?";
}

static const char* reason_text(MuxCutReason r) {
  switch (r) {
    case MUX_REASON_NORMAL:
      return "NORMAL";
    case MUX_REASON_RC_KILL_SWITCH:
      return "1:RC_KILL_SWITCH";
    case MUX_REASON_RC_SIGNAL_INVALID:
      return "1:RC_SIGNAL_INVALID";
    case MUX_REASON_WATCHDOG_TIMEOUT:
      return "2:WATCHDOG_TIMEOUT";
    case MUX_REASON_STEERING_PWM_INVALID:
      return "3:STEERING_PWM_INVALID";
    case MUX_REASON_THROTTLE_PWM_INVALID:
      return "4:THROTTLE_PWM_INVALID";
  }
  return "?";
}

// Printed once each time a USB host attaches, so whoever just opened the terminal can see
// the pin map and the thresholds the lines below are being compared against.
static void print_banner(void) {
  printf(
      "\n=== safety_mux DIAGNOSTIC build (printing only; decision logic identical to the "
      "shipping build) ===\n");
  printf(
      "pins: kill=gp%u steer=gp%u throttle=gp%u heartbeat=gp%u | out servo=gp%u esc=gp%u "
      "cutoff=gp%u\n",
      g_gpios.kill_gpio, g_gpios.steering_gpio, g_gpios.throttle_gpio, g_gpios.heartbeat_gpio,
      g_gpios.servo_gpio, g_gpios.esc_gpio, g_gpios.cutoff_gpio);
  printf(
      "params: steer %ld-%ld (neutral %ld) | throttle %ld-%ld (neutral %ld) | watchdog %ldms | "
      "kill thr %ldus +-%ldus | rc range %ld-%ld\n",
      (long)us_int(g_params.steering_pwm_min_us), (long)us_int(g_params.steering_pwm_max_us),
      (long)us_int(g_params.steering_pwm_neutral_us), (long)us_int(g_params.throttle_pwm_min_us),
      (long)us_int(g_params.throttle_pwm_max_us), (long)us_int(g_params.throttle_pwm_neutral_us),
      (long)(g_params.watchdog_timeout_s * 1000.0 + 0.5),
      (long)us_int(g_params.kill_switch_threshold_us),
      (long)us_int(g_params.kill_switch_hysteresis_us), (long)us_int(g_params.rc_signal_min_us),
      (long)us_int(g_params.rc_signal_max_us));
  printf("cut priority: 1 rc kill/unreadable, 2 heartbeat watchdog, 3 steering, 4 throttle\n");
}

void diag_report_init(const DiagGpioMap* gpios, const MuxParams* params) {
  g_gpios = *gpios;
  g_params = *params;
  g_next_report_us = time_us_64();
  g_was_connected = false;
  g_seq = 0;
}

void diag_report_tick(const MuxInput* input, const MuxOutput* output) {
  uint64_t now_us = time_us_64();
  if (now_us < g_next_report_us) {
    return;
  }
  g_next_report_us = now_us + DIAG_REPORT_PERIOD_US;

  // Nothing is printed unless a host actually has the CDC port open. With no host,
  // stdio_usb_out_chars() would return immediately anyway (SDK 2.1.0), but checking here
  // also skips the snprintf work entirely, so an unattended board pays nothing.
  bool connected = stdio_usb_connected();
  if (!connected) {
    g_was_connected = false;
    return;
  }
  if (!g_was_connected) {
    g_was_connected = true;
    print_banner();
  }

  // static, not stack: the RP2040's default main stack is small (PICO_STACK_SIZE, 2 KiB) and
  // these buffers together are most of it. diag_report_tick() is called from exactly one
  // place, the main loop, never from interrupt context, so there is no reentrancy to worry
  // about.
  static char kill_s[128];
  static char steer_s[128];
  static char thr_s[128];
  format_channel(kill_s, sizeof(kill_s), g_gpios.kill_gpio, input->rc_kill_switch_pwm_us,
                 g_params.rc_signal_min_us, g_params.rc_signal_max_us);
  format_channel(steer_s, sizeof(steer_s), g_gpios.steering_gpio, input->jetson_steering_pwm_us,
                 g_params.steering_pwm_min_us, g_params.steering_pwm_max_us);
  format_channel(thr_s, sizeof(thr_s), g_gpios.throttle_gpio, input->jetson_throttle_pwm_us,
                 g_params.throttle_pwm_min_us, g_params.throttle_pwm_max_us);

  // Heartbeat. +Inf means no edge has ever been seen (heartbeat_input_age_s()).
  static char hb_s[96];
  double age_s = input->jetson_heartbeat_age_s;
  // The watchdog verdict is the LOGIC's, not a second implementation of it: watchdog_timed_out()
  // is the same pure function mux_decide() uses, called here with the same two numbers.
  bool hb_timed_out = watchdog_timed_out(age_s, g_params.watchdog_timeout_s);
  if (!isfinite(age_s)) {
    snprintf(hb_s, sizeof(hb_s), "gp%u=NO_EDGES_EVER (timeout %ldms) TIMED_OUT",
             g_gpios.heartbeat_gpio, (long)(g_params.watchdog_timeout_s * 1000.0 + 0.5));
  } else {
    snprintf(hb_s, sizeof(hb_s), "gp%u age=%ldms (timeout %ldms) %s", g_gpios.heartbeat_gpio,
             (long)(age_s * 1000.0 + 0.5), (long)(g_params.watchdog_timeout_s * 1000.0 + 0.5),
             hb_timed_out ? "TIMED_OUT" : "OK");
  }

  // Kill-switch decode. output->switch_position is the position the decision ACTUALLY used
  // this cycle, not a recomputation, so this field can never disagree with the decision.
  long arm_edge =
      (long)us_int(g_params.kill_switch_threshold_us + g_params.kill_switch_hysteresis_us);
  long kill_edge =
      (long)us_int(g_params.kill_switch_threshold_us - g_params.kill_switch_hysteresis_us);

  bool cutoff_high = gpio_get_out_level(g_gpios.cutoff_gpio);

  static char line[DIAG_LINE_MAX];
  snprintf(line, sizeof(line),
           "[%lu t=%lu.%03lus] KILL %s -> %s (arm>=%ldus kill<%ldus) | HB %s | STEER %s | THR %s "
           "| DECISION=%s reason=%s | OUT servo gp%u=%ldus esc gp%u=%ldus cutoff gp%u=%s",
           (unsigned long)g_seq++, (unsigned long)(now_us / 1000000u),
           (unsigned long)((now_us / 1000u) % 1000u), kill_s, switch_text(output->switch_position),
           arm_edge, kill_edge, hb_s, steer_s, thr_s, output->cut ? "CUT" : "PASS",
           reason_text(output->reason), g_gpios.servo_gpio, (long)us_int(output->steering_out_us),
           g_gpios.esc_gpio, (long)us_int(output->throttle_out_us), g_gpios.cutoff_gpio,
           cutoff_high ? "HIGH(power enabled)" : "LOW(power cut)");
  printf("%s\n", line);

  // Second line: what the PWM hardware is actually emitting, straight from the registers.
  // The line above says what was COMMANDED; this one says what is on the wire. They should
  // agree, and when they do not, the arithmetic here says which of pulse width, frame rate,
  // or pin routing is wrong, without needing a scope.
  static char servo_reg[224];
  static char esc_reg[224];
  format_pwm_reg(servo_reg, sizeof(servo_reg), "servo", g_gpios.servo_gpio);
  format_pwm_reg(esc_reg, sizeof(esc_reg), "esc", g_gpios.esc_gpio);
  printf("    PWMREG clk_sys=%luHz | %s | %s\n",
         (unsigned long)pwm_output_diag(g_gpios.servo_gpio).clk_sys_hz, servo_reg, esc_reg);
}

#endif  // SAFETY_MUX_DIAG
