cmake_minimum_required(VERSION 3.8)
project(racer_session_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# -- racer_safety's OWN sources, compiled read only ---------------------------------------------
# The what-if tool runs the gate exactly as safety_node runs it, so it needs the gate sources.
# racer_safety_core is not exported as a library (that is an owner decision, claude-docs/13
# section 14.9 item 2), so the same .cpp files are compiled here, by path, the way racer_planner's
# gate tests do. Nothing under racer_safety is edited and production code does not link this.
# A missing sibling package fails the configure step: no silent skip.
set(RACER_SAFETY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../racer_safety")
foreach(f src/forward_sector.cpp src/gate_logic.cpp src/gate_logic_formatting.cpp
          include/racer_safety/forward_sector.hpp include/racer_safety/gate_logic.hpp)
  if(NOT EXISTS "${RACER_SAFETY_DIR}/${f}")
    message(FATAL_ERROR "racer_session_tools needs ${RACER_SAFETY_DIR}/${f} (racer_safety sibling package)")
  endif()
endforeach()
add_library(racer_session_tools_gate STATIC
  "${RACER_SAFETY_DIR}/src/forward_sector.cpp"
  "${RACER_SAFETY_DIR}/src/gate_logic.cpp"
  "${RACER_SAFETY_DIR}/src/gate_logic_formatting.cpp"
)
target_include_directories(racer_session_tools_gate PUBLIC "${RACER_SAFETY_DIR}/include")

# -- Generated vehicle_params binding, same recipe as racer_safety (never committed) -------------
get_filename_component(RACER_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(RACER_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
find_program(RACER_UV_EXECUTABLE uv REQUIRED)
add_custom_command(
  OUTPUT "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
  # PYTHONPATH is unset for the same reason as in racer_safety and racer_planner.
  COMMAND "${CMAKE_COMMAND}" -E env --unset=PYTHONPATH
          "${RACER_UV_EXECUTABLE}" run --project "${RACER_REPO_ROOT}/tools" python
          "${RACER_REPO_ROOT}/tools/gen_params.py"
          --params "${RACER_REPO_ROOT}/config/vehicle_params.yaml"
          --schema "${RACER_REPO_ROOT}/config/vehicle_params.schema.json"
          --out-dir "${RACER_GENERATED_DIR}"
  DEPENDS
    "${RACER_REPO_ROOT}/config/vehicle_params.yaml"
    "${RACER_REPO_ROOT}/config/vehicle_params.schema.json"
    "${RACER_REPO_ROOT}/tools/gen_params.py"
  COMMENT "racer_session_tools: generating vehicle_params C++ binding (never committed)"
  VERBATIM
)
add_custom_target(racer_session_tools_generate_vehicle_params
  DEPENDS "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
)

# -- ROS-free replay core and the CLI ------------------------------------------------------------
add_library(racer_session_tools_core STATIC src/whatif_core.cpp)
add_dependencies(racer_session_tools_core racer_session_tools_generate_vehicle_params)
target_include_directories(racer_session_tools_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  "${RACER_GENERATED_DIR}"
)
target_link_libraries(racer_session_tools_core PUBLIC racer_session_tools_gate)

add_executable(limiter_whatif_cli src/limiter_whatif_cli.cpp)
target_link_libraries(limiter_whatif_cli racer_session_tools_core)

install(TARGETS limiter_whatif_cli DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_limiter_whatif test/test_limiter_whatif.cpp TIMEOUT 120)
  target_link_libraries(test_limiter_whatif racer_session_tools_core)

  # The tool binary on a committed stream cut from 2026-11-13T22-20-18 (run R2): with the recorded
  # dt it must raise the rate_limit events the bag recorded (test/check_limiter_reproduction.py).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_limiter_reproduction
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/check_limiter_reproduction.py"
            --cli "$<TARGET_FILE:limiter_whatif_cli>"
            --fixture-dir "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
endif()

ament_package()
