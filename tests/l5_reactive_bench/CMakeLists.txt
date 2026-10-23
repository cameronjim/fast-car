cmake_minimum_required(VERSION 3.8)
project(l5_reactive_bench)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The runner end to end, short: one real run of the stack on a synthetic track, checking the
  # JSON result's shape. The metric maths has its own pure tests (test_bench_metrics.py, run by
  # tests/l5_reactive_bench's own pyproject in the python-tests job).
  ament_add_pytest_test(test_reactive_bench_smoke test_reactive_bench_smoke.py
    TIMEOUT 240
  )
endif()

ament_package()
