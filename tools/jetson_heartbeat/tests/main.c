#include <stdio.h>

#include "framework.h"

int g_test_failures = 0;

void test_heartbeat_config_suite(void);

int main(void) {
  RUN_SUITE(test_heartbeat_config_suite);

  if (g_test_failures > 0) {
    printf("\n%d assertion(s) FAILED.\n", g_test_failures);
    return 1;
  }
  printf("\nAll assertions passed.\n");
  return 0;
}
