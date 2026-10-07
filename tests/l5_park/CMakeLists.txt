cmake_minimum_required(VERSION 3.8)
project(l5_park)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # park_node finds a pocket in a walled corridor and parks in it, safety_node in the loop.
  add_launch_test(test_park_canary.py
    TARGET test_park_canary
    TIMEOUT 240
  )
endif()

ament_package()
