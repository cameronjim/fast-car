cmake_minimum_required(VERSION 3.8)
project(l5_pnp_laps)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The Plug-and-Play driver with the lap memory (start-scan lap source, D v0) laps the tuning
  # stadium on the sim's car-like plant through the real command path: system lap count and
  # lap times against ground truth, no wall contact, the cap rises, one publisher each on
  # /drive_raw and /drive; plus a half-lap case where the gate must not close. The decision
  # functions D-S0 to D-S2 of the pre-registration are tested without ROS.
  ament_add_pytest_test(test_pnp_laps_canary test_pnp_laps_canary.py
    TIMEOUT 1800
  )
  ament_add_pytest_test(test_lap_battery test_lap_battery.py
    TIMEOUT 120
  )
  # Battery B4 of the v0.3 pre-registration (brief 30): the decision functions, the constants
  # file check, the command builder against the runner's own parser, and the run machinery with
  # a fake runner. Not a battery run: the battery is not a CI job.
  ament_add_pytest_test(test_lap_battery_b4 test_lap_battery_b4.py
    TIMEOUT 300
  )
  # Battery B5 and measurement M-A2 of the v0.4 pre-registration (brief 40): the D4 decision
  # functions, the M-A2 reading, the command builders, the B4 gate and freeze check, the run
  # machinery with a fake runner, and the innovation gate counter. Not a battery run.
  ament_add_pytest_test(test_lap_battery_b5 test_lap_battery_b5.py
    TIMEOUT 300
  )
  ament_add_pytest_test(test_lap_node_counting test_lap_node_counting.py
    TIMEOUT 120
  )
endif()

ament_package()
