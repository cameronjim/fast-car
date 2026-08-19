cmake_minimum_required(VERSION 3.8)
project(l5_tracker_lap)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test_tracker_lap_canary.py
    TARGET test_tracker_lap_canary
    TIMEOUT 120
  )
  add_launch_test(test_tracker_lap_canary_oschersleben.py
    TARGET test_tracker_lap_canary_oschersleben
    TIMEOUT 180
  )
endif()

ament_package()
