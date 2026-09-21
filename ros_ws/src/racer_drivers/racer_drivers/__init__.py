"""Python side of racer_drivers (claude-docs/02-repo-layout.md: on-vehicle drivers).

racer_drivers is a hybrid ament_cmake package: the command-path drivers are C++ (CLAUDE.md
"control-critical code in C++"), and this Python subpackage holds the NON-control-critical
telemetry drivers -- code that reads a sensor and publishes it, and can never move the car.
rail_voltage.py / rail_voltage_node.py (roadmap 1.6, CLAUDE.md invariant 5) are the first.
"""
