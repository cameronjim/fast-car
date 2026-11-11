from glob import glob

from setuptools import find_packages, setup

package_name = "racer_apps"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description="Roadmap phase 5 applications on the racer: spiderbot patrol (5.3).",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spiderbot_node = racer_apps.spiderbot.spiderbot_node:main",
            "fake_detector_node = racer_apps.spiderbot.fake_detector_node:main",
        ],
    },
)
