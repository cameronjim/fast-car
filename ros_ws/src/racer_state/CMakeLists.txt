cmake_minimum_required(VERSION 3.8)
project(racer_state)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# -- ROS-free core (claude-docs/10-conventions.md: logic separated from node plumbing) ------
add_library(racer_state_core
  src/servo_model.cpp
  src/wheel_twist.cpp
)
target_include_directories(racer_state_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# -- Generated vehicle_params binding: same recipe as racer_control's CMakeLists.txt (see the
#    comment there for why uv runs tools/gen_params.py and why PYTHONPATH is unset). Only the
#    node includes the generated header; the core and its gtests never do. -----------------
get_filename_component(RACER_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(RACER_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
find_program(RACER_UV_EXECUTABLE uv REQUIRED)
add_custom_command(
  OUTPUT "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
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
  COMMENT "racer_state: generating vehicle_params C++ binding (never committed, see claude-docs/06-vehicle-params.md)"
  VERBATIM
)
add_custom_target(racer_state_generate_vehicle_params
  DEPENDS "${RACER_GENERATED_DIR}/vehicle_params_generated.hpp"
)

# -- wheel_twist_node: /odom/wheel + /drive -> /odom/wheel_twist for robot_localization -----
add_executable(wheel_twist_node src/wheel_twist_node.cpp)
add_dependencies(wheel_twist_node racer_state_generate_vehicle_params)
target_include_directories(wheel_twist_node PRIVATE "${RACER_GENERATED_DIR}")
target_link_libraries(wheel_twist_node racer_state_core)
ament_target_dependencies(wheel_twist_node rclcpp rcl_interfaces nav_msgs geometry_msgs
  ackermann_msgs)

install(TARGETS wheel_twist_node
  DESTINATION lib/${PROJECT_NAME}
)

# EKF, slam_toolbox and AMCL parameter files, read by racer_bringup's odometry, mapping and
# localization launch files.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # L1: kinematics, sign conventions, servo lag and rate limit, reverse, zero speed, stale and
  # non-finite inputs.
  ament_add_gtest(test_racer_state_core
    test/test_servo_model.cpp
    test/test_wheel_twist.cpp
  )
  target_link_libraries(test_racer_state_core racer_state_core)

  # L3: the node on mocked topics (nominal, /drive silence, /odom/wheel silence, QoS, exit).
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_wheel_twist_node_launch.py
    TARGET test_wheel_twist_node_launch
    TIMEOUT 120
  )

  # L1: the shipped EKF / slam_toolbox / AMCL configs say what the launch files and the note
  # rely on (frames, topics, two_d_mode, the IMU slot left empty).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_state_configs test/test_state_configs.py)
endif()

ament_package()
