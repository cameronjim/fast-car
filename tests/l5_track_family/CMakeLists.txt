cmake_minimum_required(VERSION 3.8)
project(l5_track_family)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The raceline tracker laps two family tracks in both directions (track_family_runner.py: the
  # real command path, the car as measured, the raceline made from the track's own map by
  # tools/track_session). Four runs of about a minute each.
  ament_add_pytest_test(test_track_family_canary test_track_family_canary.py
    TIMEOUT 900
  )
  # gap_follow (lane profile, lookahead 0.8, outside entry 0.25) on two thin-island stadiums,
  # both directions: the case the tracker cannot do. Four runs of about a minute each.
  ament_add_pytest_test(test_gap_follow_family_canary test_gap_follow_family_canary.py
    TIMEOUT 900
  )
  # The tracker on the LOCALISED pose (AMCL + EKF, sensor errors on): the configuration of the
  # real car, both directions on two 1 m lane tracks.
  ament_add_pytest_test(test_tracker_localised_canary test_tracker_localised_canary.py
    TIMEOUT 1200
  )
endif()

ament_package()
