// Minimal, dependency-free table-driven test framework. Deliberately not gtest/Unity/etc --
// CLAUDE.md: "Keep it dead simple and readable; simplicity is the safety argument" -- this
// whole test tree compiles with plain gcc, nothing to vendor or audit. See
// .github/scripts/safety_mux_host_tests.sh for how CI builds and runs it.
#ifndef SAFETY_MUX_TEST_FRAMEWORK_H_
#define SAFETY_MUX_TEST_FRAMEWORK_H_

#include <stdio.h>

// Defined once in tests/main.c; every suite links against and increments this same global.
extern int g_test_failures;
// Every CHECK counts itself, so a run reports how many assertions actually executed rather
// than only how many failed -- a suite that silently stopped running is otherwise
// indistinguishable from a suite that passed.
extern int g_test_assertions;

#define CHECK(condition, description)                                                    \
  do {                                                                                   \
    g_test_assertions++;                                                                 \
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

#endif  // SAFETY_MUX_TEST_FRAMEWORK_H_
