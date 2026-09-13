cmake_minimum_required(VERSION 3.8)
project(racer_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ackermann_msgs REQUIRED)

# -- ROS-free core library (claude-docs/10-conventions.md: decision logic separated from node
#    plumbing; same shape as racer_safety's racer_safety_core) --------------------------------
add_library(racer_drivers_pwm_core
  src/pwm_mapping.cpp
  src/pwm_sink.cpp
  src/pwm_output_driver.cpp
)
target_include_directories(racer_drivers_pwm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# -- Generated vehicle_params bindings (same pattern as racer_safety/CMakeLists.txt; see that
#    file's comment for the full rationale). Only pwm_output_node.cpp includes the generated
#    header; racer_drivers_pwm_core (and its gtest suite) never does. --------------------------
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
  COMMENT "racer_drivers: generating vehicle_params C++ binding (never committed, see claude-docs/06-vehicle-params.md)"
  VERBATIM
)
add_custom_target(racer_drivers_generate_vehicle_params
  DEPENDS "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
)

# -- pwm_output_node executable ---------------------------------------------------------------
add_executable(pwm_output_node src/pwm_output_node.cpp)
add_dependencies(pwm_output_node racer_drivers_generate_vehicle_params)
target_include_directories(pwm_output_node PRIVATE "${RACER_GENERATED_DIR}")
target_link_libraries(pwm_output_node racer_drivers_pwm_core)
ament_target_dependencies(pwm_output_node rclcpp rcl_interfaces ackermann_msgs)

install(TARGETS pwm_output_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_racer_drivers_pwm_core
    test/test_pwm_mapping.cpp
  )
  target_link_libraries(test_racer_drivers_pwm_core racer_drivers_pwm_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pwm_output_node_launch.py
    TARGET test_pwm_output_node_launch
    TIMEOUT 120
  )
endif()

ament_package()
