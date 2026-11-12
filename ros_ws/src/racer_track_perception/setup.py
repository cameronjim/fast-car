from setuptools import find_packages, setup

package_name = "racer_track_perception"

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
    description="LiDAR scan to local track: numpy net and geometric corridor tracer.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "track_perception_node = racer_track_perception.track_perception_node:main",
        ],
    },
)
