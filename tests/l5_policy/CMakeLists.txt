cmake_minimum_required(VERSION 3.8)
project(l5_policy)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # tracker_node (handoff) + policy_node + safety_node on a stadium, both directions, against
  # the tracker alone: two runs each, about a minute apiece. Skipped without torch.
  ament_add_pytest_test(test_policy_canary test_policy_canary.py
    TIMEOUT 900
  )
endif()

ament_package()
