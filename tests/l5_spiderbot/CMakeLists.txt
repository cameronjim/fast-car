cmake_minimum_required(VERSION 3.8)
project(l5_spiderbot)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # The spiderbot patrols gym_oval through the real safety node, stops for a synthetic person,
  # faces them, alerts, holds and resumes.
  add_launch_test(test_spiderbot_patrol.py
    TARGET test_spiderbot_patrol
    TIMEOUT 240
  )
endif()

ament_package()
