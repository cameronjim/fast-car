# firmware/safety_mux Pico SDK build. DRAFTED, NOT BUILT ANYWHERE YET (no arm-none-eabi-gcc
# or Pico SDK toolchain is available in this repo's dev containers or in CI -- see README.md
# and .github/scripts/safety_mux_host_tests.sh's header comment for why CI only builds and
# runs logic/ + tests/ with plain host gcc, not this file). This is written from the Pico
# SDK's documented CMake integration pattern, for a human with the toolchain installed
# (Desktop B, claude-docs/03-environments.md) to build and correct once real hardware exists.
#
# Usage (once arm-none-eabi-gcc + a Pico SDK checkout or this FetchContent are available):
#   cmake -S firmware/safety_mux -B firmware/safety_mux/build
#   cmake --build firmware/safety_mux/build
#
# pico-sdk pinned to release tag 2.1.0 by FetchContent (rather than requiring a pre-existing
# PICO_SDK_PATH checkout) so this is reproducible from this file alone, mirroring every other
# pinned dependency in this repo (docker/*/Dockerfile base images, sim/racer_gym's f1tenth_gym
# commit). Bump deliberately, not silently, if a newer SDK release is wanted.
cmake_minimum_required(VERSION 3.13)

include(FetchContent)
FetchContent_Declare(
  pico_sdk
  GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
  GIT_TAG        2.1.0
)
FetchContent_MakeAvailable(pico_sdk)
set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
include(${pico_sdk_SOURCE_DIR}/pico_sdk_init.cmake)

project(safety_mux C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# --- Regenerate vehicle_params bindings at build time (never committed, claude-docs/06-
# vehicle-params.md rule 3 / claude-docs/10-conventions.md) ----------------------------
set(GENERATED_PARAMS_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_custom_command(
  OUTPUT ${GENERATED_PARAMS_DIR}/vehicle_params_generated.h
  COMMAND python3 ${REPO_ROOT}/tools/gen_params.py
          --params ${REPO_ROOT}/config/vehicle_params.yaml
          --schema ${REPO_ROOT}/config/vehicle_params.schema.json
          --out-dir ${GENERATED_PARAMS_DIR}
  DEPENDS ${REPO_ROOT}/config/vehicle_params.yaml ${REPO_ROOT}/config/vehicle_params.schema.json
          ${REPO_ROOT}/tools/gen_params.py
  COMMENT "Regenerating vehicle_params C binding (tools/gen_params.py)"
)
add_custom_target(vehicle_params_generated DEPENDS ${GENERATED_PARAMS_DIR}/vehicle_params_generated.h)

# --- Pure decision logic (also built and tested independently on the host -- see
# .github/scripts/safety_mux_host_tests.sh; this target exists so the SAME source files are
# what actually ships, not a hand-synced copy) ------------------------------------------
add_library(safety_mux_logic STATIC
  logic/src/pwm_validity.c
  logic/src/watchdog.c
  logic/src/rc_switch.c
  logic/src/mux_params.c
  logic/src/mux_decision.c
)
target_include_directories(safety_mux_logic PUBLIC logic/include)

# --- Pico SDK glue (UNVERIFIED ON HARDWARE, see README.md) ------------------------------
add_executable(safety_mux_firmware
  pico/main.c
  pico/pwm_capture.c
  pico/pwm_output.c
  pico/heartbeat_input.c
  pico/power_cutoff.c
)
add_dependencies(safety_mux_firmware vehicle_params_generated)
target_include_directories(safety_mux_firmware PRIVATE pico ${GENERATED_PARAMS_DIR})
target_link_libraries(safety_mux_firmware
  safety_mux_logic
  pico_stdlib
  hardware_gpio
  hardware_pwm
  hardware_clocks
)

# USB serial for the fault-halt printf in pico/main.c; PROPOSED, not bench-decided (see
# README.md) -- a UART could be used instead with no change to the decision-logic side.
pico_enable_stdio_usb(safety_mux_firmware 1)
pico_enable_stdio_uart(safety_mux_firmware 0)

pico_add_extra_outputs(safety_mux_firmware)
