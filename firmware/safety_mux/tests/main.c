#include <stdio.h>

#include "framework.h"

int g_test_failures = 0;

void test_pwm_validity_suite(void);
void test_watchdog_suite(void);
void test_rc_switch_suite(void);
void test_mux_params_suite(void);
void test_mux_decision_suite(void);

int main(void) {
  RUN_SUITE(test_pwm_validity_suite);
  RUN_SUITE(test_watchdog_suite);
  RUN_SUITE(test_rc_switch_suite);
  RUN_SUITE(test_mux_params_suite);
  RUN_SUITE(test_mux_decision_suite);

  if (g_test_failures > 0) {
    printf("\n%d assertion(s) FAILED.\n", g_test_failures);
    return 1;
  }
  printf("\nAll assertions passed.\n");
  return 0;
}
