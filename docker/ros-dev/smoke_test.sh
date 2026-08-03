#!/usr/bin/env bash
# Toolchain smoke test for the ros-dev image (roadmap task 0.4).
#
# Proves what claude-docs/03-environments.md and 01-roadmap.md require of
# this image: it can actually build and test ROS 2 packages, not just have
# `ros2`/`colcon` binaries present. Since ros_ws/src has no real packages
# yet, this creates a throwaway ament_cmake package (C++/gtest) and a
# throwaway ament_python package (Python/pytest) in a scratch workspace,
# runs `colcon build` + `colcon test` on both, and fails the whole script
# (nonzero exit) unless colcon reports every test passed.
#
# Run as the container's default CMD (see Dockerfile) so `docker run
# ros-dev:ci` alone is the proof, matching the sim-cpu/train-cuda pattern.
set -euo pipefail

fail() {
    echo "SMOKE TEST FAILED: $*" >&2
    exit 1
}
trap 'fail "unexpected error at line $LINENO"' ERR

# ROS's setup.bash references variables that are legitimately unset before
# sourcing it (e.g. AMENT_TRACE_SETUP_FILES), which trips `set -u` -- the
# standard ROS + `set -u` workaround (same as .github/scripts/ros_build_test.sh).
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

command -v ros2 >/dev/null 2>&1 || fail "ros2 not found on PATH"
command -v colcon >/dev/null 2>&1 || fail "colcon not found on PATH"
ros2 --help >/dev/null 2>&1 || fail "'ros2 --help' did not run cleanly"
colcon --help >/dev/null 2>&1 || fail "'colcon --help' did not run cleanly"
echo "ros2 and colcon are present and runnable."

WORKSPACE="$(mktemp -d)"
trap 'rm -rf "$WORKSPACE"' EXIT
trap 'fail "unexpected error at line $LINENO"' ERR

mkdir -p "$WORKSPACE/src/smoke_cpp/test" "$WORKSPACE/src/smoke_py/smoke_py" \
         "$WORKSPACE/src/smoke_py/test" "$WORKSPACE/src/smoke_py/resource"

# --- throwaway ament_cmake package (C++ / gtest) ---------------------------
cat > "$WORKSPACE/src/smoke_cpp/package.xml" <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>smoke_cpp</name>
  <version>0.0.0</version>
  <description>Throwaway ament_cmake package built by docker/ros-dev/smoke_test.sh</description>
  <maintainer email="dev@example.com">ros-dev smoke test</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > "$WORKSPACE/src/smoke_cpp/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(smoke_cpp)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_smoke_cpp test/test_smoke.cpp)
endif()

ament_package()
EOF

cat > "$WORKSPACE/src/smoke_cpp/test/test_smoke.cpp" <<'EOF'
#include <gtest/gtest.h>

TEST(SmokeCpp, BasicArithmetic) {
  EXPECT_EQ(1 + 1, 2);
}
EOF

# --- throwaway ament_python package (Python / pytest) ----------------------
cat > "$WORKSPACE/src/smoke_py/package.xml" <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>smoke_py</name>
  <version>0.0.0</version>
  <description>Throwaway ament_python package built by docker/ros-dev/smoke_test.sh</description>
  <maintainer email="dev@example.com">ros-dev smoke test</maintainer>
  <license>Apache-2.0</license>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > "$WORKSPACE/src/smoke_py/setup.py" <<'EOF'
from setuptools import find_packages, setup

setup(
    name="smoke_py",
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/smoke_py"]),
        ("share/smoke_py", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros-dev smoke test",
    maintainer_email="dev@example.com",
    description="Throwaway ament_python package built by docker/ros-dev/smoke_test.sh",
    license="Apache-2.0",
    tests_require=["pytest"],
)
EOF

touch "$WORKSPACE/src/smoke_py/resource/smoke_py"
touch "$WORKSPACE/src/smoke_py/smoke_py/__init__.py"

cat > "$WORKSPACE/src/smoke_py/smoke_py/mathy.py" <<'EOF'
def add(a, b):
    return a + b
EOF

cat > "$WORKSPACE/src/smoke_py/test/test_mathy.py" <<'EOF'
from smoke_py.mathy import add


def test_add():
    assert add(2, 3) == 5
EOF

echo "Building throwaway ament_cmake + ament_python packages..."
(
    cd "$WORKSPACE"
    colcon build --symlink-install
    colcon test
    colcon test-result --verbose
)

echo "ros-dev smoke test PASSED: colcon built and tested a throwaway ament_cmake (gtest) and ament_python (pytest) package pair."
