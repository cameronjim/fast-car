#!/usr/bin/env python3
"""Console entry point for racer_drivers' rail_voltage_node.

racer_drivers is an ament_cmake package (its command-path drivers are C++), so it has no
setup.py console_scripts section the way racer_tools does. CMakeLists.txt installs this file
into lib/racer_drivers as `rail_voltage_node`, which is what makes
`ros2 run racer_drivers rail_voltage_node` and launch's Node(executable=...) work.
"""

from racer_drivers.rail_voltage_node import main

if __name__ == "__main__":
    main()
