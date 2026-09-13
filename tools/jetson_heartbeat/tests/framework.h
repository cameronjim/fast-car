// Minimal, dependency-free table-driven test framework, copied in spirit from
// firmware/safety_mux/tests/framework.h so the two host-test trees read the same way.
#ifndef JETSON_HEARTBEAT_TEST_FRAMEWORK_H_
#define JETSON_HEARTBEAT_TEST_FRAMEWORK_H_

#include <stdio.h>

// Defined once in tests/main.c; every suite links against and increments this same global.
extern int g_test_failures;

#define CHECK(condition, description)                                                    \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      printf("  FAIL: %s (%s:%d): %s\n", (description), __FILE__, __LINE__, #condition); \
      g_test_failures++;                                                                 \
    }                                                                                    \
  } while (0)

#define RUN_SUITE(suite_fn)          \
  do {                               \
    printf("== %s ==\n", #suite_fn); \
    suite_fn();                      \
  } while (0)

#endif  // JETSON_HEARTBEAT_TEST_FRAMEWORK_H_
