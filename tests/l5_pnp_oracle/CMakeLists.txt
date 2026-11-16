cmake_minimum_required(VERSION 3.8)
project(l5_pnp_oracle)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # TEST ONLY. The oracle track (ground truth LocalTrack) is never installed: nothing here is
  # installed, and the runner starts oracle_track_node.py by path. L1 geometry, the guard that the
  # car's launch cannot start it, stamp and timing parity with track_perception_node, and the
  # obstacle development case lapped with the geometric backend and with the oracle.
  ament_add_pytest_test(test_oracle_core test_oracle_core.py TIMEOUT 120)
  ament_add_pytest_test(test_oracle_guard test_oracle_guard.py TIMEOUT 120)
  ament_add_pytest_test(test_oracle_parity test_oracle_parity.py TIMEOUT 300)
  ament_add_pytest_test(test_oracle_canary test_oracle_canary.py TIMEOUT 1800)
endif()

ament_package()
