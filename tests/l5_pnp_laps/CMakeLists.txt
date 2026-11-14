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
endif()

ament_package()
