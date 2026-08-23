cmake_minimum_required(VERSION 3.8)
project(e2e_sim_safety)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test_sim_safety_e2e.py
    TARGET test_sim_safety_e2e
    TIMEOUT 120
  )
  add_launch_test(test_sim_autopilot_e2e.py
    TARGET test_sim_autopilot_e2e
    TIMEOUT 120
  )
  add_launch_test(test_sim_autopilot_e2e_oschersleben.py
    TARGET test_sim_autopilot_e2e_oschersleben
    TIMEOUT 120
  )
endif()

ament_package()
