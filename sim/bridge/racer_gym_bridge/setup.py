from setuptools import find_packages, setup

package_name = "racer_gym_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description=(
        "ROS 2 Humble bridge between the pinned f1tenth_gym simulator and the racer "
        "topics (roadmap task 0.5)."
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = racer_gym_bridge.bridge_node:main",
        ],
    },
)
