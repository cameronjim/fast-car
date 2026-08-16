from setuptools import find_packages, setup

package_name = "racer_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description=(
        "Teleop, bag utilities, timing-gate reader (roadmap milestone 1: keyboard_teleop_node)."
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop_node = racer_tools.keyboard_teleop_node:main",
        ],
    },
)
