cmake_minimum_required(VERSION 3.8)
project(l5_reactive_lap)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test_gap_follow_lap_canary.py
    TARGET test_gap_follow_lap_canary
    TIMEOUT 180
  )
endif()

ament_package()
