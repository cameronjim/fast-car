#!/usr/bin/env bash
# tools/jetson_heartbeat host-logic tests (roadmap task 1.3, layer-1 safety mux heartbeat).
# Compiles and runs the pure argument-parsing / rate-to-period math in
# tools/jetson_heartbeat/src/config.c against tools/jetson_heartbeat/tests/ with plain gcc on
# ubuntu-latest -- no libgpiod, no real GPIO access, no cross-compiler. Mirrors
# .github/scripts/safety_mux_host_tests.sh's pattern and rationale: the part of this tool
# that can be wrong in a way a compiler won't catch (line/rate parsing, half-period math) is
# host-testable and tested here; the GPIO toggle loop in src/main.c is real hardware access
# with no host equivalent and is verified on the Jetson itself instead (see
# tools/jetson_heartbeat/README.md "Verification").
#
# -Wall -Wextra -Werror -Wpedantic: this feeds the layer-1 safety mux's watchdog input, so an
# unused-variable or implicit-conversion warning here should fail the build, not scroll by.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"/tools/jetson_heartbeat

if [ ! -f src/config.c ]; then
  echo "NOTICE: tools/jetson_heartbeat/src/config.c does not exist yet. Nothing to test yet. Passing."
  exit 0
fi

BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT

echo "Compiling tools/jetson_heartbeat host tests (gcc, -Wall -Wextra -Werror -Wpedantic)..."
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic \
  -I include \
  -I tests \
  -o "$BUILD_DIR/jetson_heartbeat_tests" \
  src/config.c \
  tests/*.c

echo "Running tools/jetson_heartbeat host tests..."
"$BUILD_DIR/jetson_heartbeat_tests"
