from glob import glob

from setuptools import find_packages, setup

package_name = "racer_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description="Roadmap 5.1 camera perception: YOLO detector, LiDAR fusion, person target.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = racer_perception.detector_node:main",
            "fusion_node = racer_perception.fusion_node:main",
        ],
    },
)
