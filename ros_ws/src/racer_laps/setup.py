from setuptools import find_packages, setup

package_name = "racer_laps"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description="Lap memory: start-scan lap gate, lap records, station set and cap schedule.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lap_memory_node = racer_laps.lap_memory_node:main",
        ],
    },
)
