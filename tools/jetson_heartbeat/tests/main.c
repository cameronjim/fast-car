#include <stdio.h>

#include "framework.h"

int g_test_failures = 0;
int g_test_assertions = 0;

void test_heartbeat_config_suite(void);

int main(void) {
  RUN_SUITE(test_heartbeat_config_suite);

  if (g_test_failures > 0) {
    printf("\n%d of %d assertion(s) FAILED.\n", g_test_failures, g_test_assertions);
    return 1;
  }
  printf("\nAll %d assertions passed.\n", g_test_assertions);
  return 0;
}
