cmake_minimum_required(VERSION 3.8)
project(racer_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # L3-style launch test for car_teleop.launch.py (claude-docs/12-testing.md L3): brings up
  # safety_node + pwm_output_node against a fake sysfs tree and asserts /drive is neutral and
  # both PWM channels sit at the calibrated neutral with no input. The sim launch files are
  # covered by tests/e2e_sim_safety and tests/l5_tracker_lap instead, which need f1tenth_gym
  # in the workspace and therefore cannot live here.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_car_teleop_launch.py
    TARGET test_car_teleop_launch
    TIMEOUT 180
  )
  # Logging half of the same launch file (GitHub issue #64, roadmap 1.6, CLAUDE.md invariant
  # 5): asserts the rosbag recorder and rail_voltage_node come up BY DEFAULT and that the
  # closed bag contains every topic a drive must be reconstructable from. Separate file, not
  # extra cases in the test above, because it needs its own launch configuration (a live
  # teleop source, a fake INA3221 tree, a temp bag root) and its own DDS domain.
  add_launch_test(test/test_car_teleop_bag_launch.py
    TARGET test_car_teleop_bag_launch
    TIMEOUT 240
  )
endif()

ament_package()
