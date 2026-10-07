cmake_minimum_required(VERSION 3.8)
project(l5_reactive_lap)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test_gap_follow_lap_canary.py
    TARGET test_gap_follow_lap_canary
    TIMEOUT 180
  )
  # The same lap clockwise (bridge_node reverse_direction), 2026-10-06 night.
  add_launch_test(test_gap_follow_lap_canary_cw.py
    TARGET test_gap_follow_lap_canary_cw
    TIMEOUT 180
  )
  # A corner tighter than the turning circle, safety_node in the loop: the reverse escape must
  # fire and the car must still finish the lap.
  add_launch_test(test_gap_follow_escape_canary.py
    TARGET test_gap_follow_escape_canary
    TIMEOUT 300
  )
endif()

ament_package()
