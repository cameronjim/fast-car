cmake_minimum_required(VERSION 3.8)
project(l5_pnp)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The Plug-and-Play driver (perception + MPPI planner, no map, no pose) laps tracks it never
  # saw through the real command path (track_family_runner.py, driver pnp): three laps at
  # 0.6 m/s on a stadium both ways and a rounded rectangle (geometric backend) and on a stadium
  # with the released net bundle. Four runs of about a minute each.
  ament_add_pytest_test(test_pnp_canary test_pnp_canary.py
    TIMEOUT 1500
  )
endif()

ament_package()
