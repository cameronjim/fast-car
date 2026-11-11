from setuptools import find_packages, setup

package_name = "racer_policy"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    package_data={package_name: ["contract.schema.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="cameronjim",
    maintainer_email="64621038+cameronjim@users.noreply.github.com",
    description=(
        "Deployment-contract loader (ROS-free) and the policy_node that runs a learned "
        "residual on top of the tracker (roadmap S.5 and 5.2 scaffolding)."
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_node = racer_policy.policy_node:main",
        ],
    },
)
