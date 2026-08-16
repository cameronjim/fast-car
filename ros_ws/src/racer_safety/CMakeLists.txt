cmake_minimum_required(VERSION 3.8)
project(racer_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# claude-docs/12-testing.md: "envelope/ and racer_safety gate at 100% branch on decision
# logic". OFF by default (instrumentation overhead is pointless for a normal build); the
# dedicated racer-safety-coverage CI job (.github/workflows/ci.yml, .github/scripts/
# racer_safety_coverage.sh) turns this ON via -DRACER_SAFETY_COVERAGE=ON, builds ONLY this
# package, and runs gcovr scoped to gate_logic.cpp/.hpp with --fail-under-branch 100.
option(RACER_SAFETY_COVERAGE "Build racer_safety_core with --coverage instrumentation" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(racer_msgs REQUIRED)

# -- ROS-free core library (claude-docs/10-conventions.md: gate/decision logic separated
#    from node plumbing) -------------------------------------------------------------------
add_library(racer_safety_core
  src/gate_logic.cpp
  src/gate_logic_formatting.cpp
)
target_include_directories(racer_safety_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
if(RACER_SAFETY_COVERAGE)
  target_compile_options(racer_safety_core PUBLIC --coverage -O0 -g)
  target_link_options(racer_safety_core PUBLIC --coverage)
endif()

# -- Generated vehicle_params bindings (same pattern as racer_control/CMakeLists.txt; see
#    that file's comment for the full rationale). Only safety_node.cpp includes the generated
#    header; racer_safety_core (and its gtest suite) never does. --------------------------
get_filename_component(RACER_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(RACER_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
find_program(RACER_UV_EXECUTABLE uv REQUIRED)
add_custom_command(
  OUTPUT "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
  COMMAND "${RACER_UV_EXECUTABLE}" run --project "${RACER_REPO_ROOT}/tools" python
          "${RACER_REPO_ROOT}/tools/gen_params.py"
          --params "${RACER_REPO_ROOT}/config/vehicle_params.yaml"
          --schema "${RACER_REPO_ROOT}/config/vehicle_params.schema.json"
          --out-dir "${RACER_GENERATED_DIR}"
  DEPENDS
    "${RACER_REPO_ROOT}/config/vehicle_params.yaml"
    "${RACER_REPO_ROOT}/config/vehicle_params.schema.json"
    "${RACER_REPO_ROOT}/tools/gen_params.py"
  COMMENT "racer_safety: generating vehicle_params C++ binding (never committed, see claude-docs/06-vehicle-params.md)"
  VERBATIM
)
add_custom_target(racer_safety_generate_vehicle_params
  DEPENDS "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
)

# -- safety_node executable ----------------------------------------------------------------
add_executable(safety_node src/safety_node.cpp)
add_dependencies(safety_node racer_safety_generate_vehicle_params)
target_include_directories(safety_node PRIVATE "${RACER_GENERATED_DIR}")
target_link_libraries(safety_node racer_safety_core)
ament_target_dependencies(safety_node rclcpp rcl_interfaces ackermann_msgs sensor_msgs racer_msgs)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_racer_safety_core
    test/test_gate_logic.cpp
  )
  target_link_libraries(test_racer_safety_core racer_safety_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_node_launch.py
    TARGET test_safety_node_launch
    TIMEOUT 120
  )
endif()

ament_package()
