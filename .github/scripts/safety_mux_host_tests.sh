#!/usr/bin/env bash
# firmware/safety_mux host-logic tests (roadmap task 1.3 DRAFT; claude-docs/05-safety.md
# layer 1; claude-docs/12-testing.md L1: safety decision logic is table-driven, every
# branch). Compiles and runs firmware/safety_mux/logic/ + firmware/safety_mux/tests/ with
# plain gcc on ubuntu-latest -- no Pico SDK, no arm-none-eabi cross-compiler, no external
# test framework (tests/framework.h is a ~20-line macro pair, not a vendored dependency).
#
# This is the ONLY thing CI does for firmware/safety_mux (see that directory's README.md for
# why): the Pico SDK glue in firmware/safety_mux/pico/ is NOT built or tested here. Standing
# up an RP2040 cross-compile toolchain (arm-none-eabi-gcc) plus a FetchContent'd pico-sdk in
# CI is real infrastructure this milestone chose not to take on, given the actual safety
# claim rests entirely on the DECISION LOGIC below being correct -- not on whether the Pico
# glue happens to compile. A cross-compile CI job is a reasonable future addition once this
# firmware is closer to bench-tested than drafted; see the roadmap 1.3 note in
# claude-docs/01-roadmap.md.
#
# -Wall -Wextra -Werror -Wpedantic: this is safety decision logic (claude-docs/05-safety.md);
# an unused-variable or implicit-conversion warning here is exactly the kind of thing that
# should fail the build, not scroll by.
#
# No coverage gate is enforced here (contrast racer_safety's 100%-branch gcovr gate in
# .github/scripts/racer_safety_coverage.sh) -- tests/test_*.c's table-driven cases were
# manually audited against every `if`/comparison in logic/src/*.c to confirm both branch
# outcomes are exercised (see each suite's case table), but wiring up gcovr for a firmware
# tree this small, still in draft, felt like process for its own sake. Promote this to an
# enforced gate (mirroring racer_safety_coverage.sh) once firmware/safety_mux's logic is
# larger or closer to a bench-verified state.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"/firmware/safety_mux

if [ ! -d logic/src ]; then
  echo "NOTICE: firmware/safety_mux/logic does not exist yet. Nothing to test yet. Passing."
  exit 0
fi

BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT

echo "Compiling firmware/safety_mux logic + host tests (gcc, -Wall -Wextra -Werror -Wpedantic)..."
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic \
  -I logic/include \
  -I tests \
  -o "$BUILD_DIR/safety_mux_tests" \
  logic/src/*.c \
  tests/*.c

echo "Running firmware/safety_mux host tests..."
"$BUILD_DIR/safety_mux_tests"
