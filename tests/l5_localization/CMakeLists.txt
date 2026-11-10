cmake_minimum_required(VERSION 3.8)
project(l5_localization)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # slam_toolbox maps a living-room lane track; the saved map must match /sim/map.
  add_launch_test(test_mapping_canary.py
    TARGET test_mapping_canary
    TIMEOUT 300
  )
  # tracker_node laps gym_oval at the car's speed cap on the AMCL pose; pose error vs ground truth.
  add_launch_test(test_localization_canary_oval.py
    TARGET test_localization_canary_oval
    TIMEOUT 300
  )
  # The same on the living-room lane track.
  add_launch_test(test_localization_canary_lane.py
    TARGET test_localization_canary_lane
    TIMEOUT 300
  )
endif()

ament_package()
