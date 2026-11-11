cmake_minimum_required(VERSION 3.8)
project(racer_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# The solver is the 50 Hz hot path and colcon builds with no build type by default, which would
# be -O0. Default to an optimised build; an explicit CMAKE_BUILD_TYPE still wins.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(racer_pnp_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# -- ROS-free core: no rclcpp, no generated vehicle_params header (it takes a VehicleGeometry) --
add_library(racer_planner_core
  src/dynamics_model.cpp
  src/sha256.cpp
  src/bundle_loader.cpp
  src/local_track.cpp
  src/distance_field.cpp
  src/footprint.cpp
  src/ego_motion.cpp
  src/mppi.cpp
  src/planner_logic.cpp
  src/planner.cpp
)
target_include_directories(racer_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(racer_planner_core PUBLIC yaml-cpp)

# -- Generated vehicle_params binding, same recipe as racer_control (never committed) -----------
get_filename_component(RACER_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(RACER_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
find_program(RACER_UV_EXECUTABLE uv REQUIRED)
add_custom_command(
  OUTPUT "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
  # PYTHONPATH is unset for the same reason as in racer_control: an inherited one reaches the
  # tools/ venv's interpreter ahead of its own site-packages.
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
  COMMENT "racer_planner: generating vehicle_params C++ binding (never committed)"
  VERBATIM
)
add_custom_target(racer_planner_generate_vehicle_params
  DEPENDS "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
)

# -- mppi_planner_node: publishes /drive_raw only, never /drive ---------------------------------
add_executable(mppi_planner_node src/mppi_planner_node.cpp)
add_dependencies(mppi_planner_node racer_planner_generate_vehicle_params)
target_include_directories(mppi_planner_node PRIVATE "${RACER_GENERATED_DIR}" src)
target_link_libraries(mppi_planner_node racer_planner_core)
ament_target_dependencies(mppi_planner_node rclcpp rcl_interfaces sensor_msgs nav_msgs
  ackermann_msgs geometry_msgs racer_pnp_msgs)

# -- mppi_bench: solve time on a synthetic corridor ----------------------------------------------
add_executable(mppi_bench src/mppi_bench.cpp)
add_dependencies(mppi_bench racer_planner_generate_vehicle_params)
target_include_directories(mppi_bench PRIVATE "${RACER_GENERATED_DIR}" src)
target_link_libraries(mppi_bench racer_planner_core)

install(TARGETS mppi_planner_node mppi_bench DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_racer_planner_core
    test/test_dynamics_model.cpp
    test/test_sha256.cpp
    test/test_bundle_loader.cpp
    test/test_local_track.cpp
    test/test_distance_field.cpp
    test/test_footprint.cpp
    test/test_ego_motion.cpp
    test/test_rng.cpp
    test/test_mppi_costs.cpp
    test/test_planner_logic.cpp
    test/test_planner_properties.cpp
    TIMEOUT 120
  )
  add_dependencies(test_racer_planner_core racer_planner_generate_vehicle_params)
  target_include_directories(test_racer_planner_core PRIVATE "${RACER_GENERATED_DIR}" src test)
  target_link_libraries(test_racer_planner_core racer_planner_core)
  target_compile_definitions(test_racer_planner_core PRIVATE
    RACER_PLANNER_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    RACER_PLANNER_REPO_ROOT="${RACER_REPO_ROOT}"
  )

  # Closed loop on synthetic worlds with an oracle track (the L5 stand-in, no ROS).
  ament_add_gtest(test_planner_closed_loop test/test_planner_closed_loop.cpp TIMEOUT 120)
  add_dependencies(test_planner_closed_loop racer_planner_generate_vehicle_params)
  target_include_directories(test_planner_closed_loop PRIVATE "${RACER_GENERATED_DIR}" src test)
  target_link_libraries(test_planner_closed_loop racer_planner_core)
  target_compile_definitions(test_planner_closed_loop PRIVATE
    RACER_PLANNER_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(
    NAME test_mppi_bench_smoke
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/run_bench_smoke.py"
            --bench "$<TARGET_FILE:mppi_bench>"
            --repo-root "${RACER_REPO_ROOT}"
  )

  find_package(launch_testing_ament_cmake REQUIRED)
  foreach(t test_planner_node_launch test_planner_node_silence_launch test_planner_node_refusal_launch)
    add_launch_test(test/${t}.py TARGET ${t} TIMEOUT 180)
  endforeach()
endif()

ament_package()
