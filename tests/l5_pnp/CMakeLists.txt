cmake_minimum_required(VERSION 3.8)
project(l5_pnp)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The Plug-and-Play driver (perception + MPPI planner, no map, no pose) laps tracks it never
  # saw through the real command path (track_family_runner.py, driver pnp): three laps at
  # 0.6 m/s on a stadium both ways and a rounded rectangle (geometric backend) and on a stadium
  # with the released net bundle (four runs of about a minute each), and the same driver on a
  # car-like plant (speed dead zone, steering lag): five more runs, and the planner timing cases
  # (stadium lane 1.0 m both ways at 0.6 and 1.0 m/s, with and without stalls injected into the
  # planner's solve thread): eight more runs. RACER_L5_PNP_FULL=1 adds the full matrix.
  ament_add_pytest_test(test_pnp_canary test_pnp_canary.py
    TIMEOUT 4800
  )
endif()

ament_package()
