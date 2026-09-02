import os
from glob import glob
from setuptools import setup

package_name = 'reactive_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Autonomous Racing',
    maintainer_email='noreply@example.com',
    description='Classical reactive driving for the F1TENTH platform: LiDAR disparity-based '
                'gap following, wall following, vision path following, and an independent '
                'safety/AEB node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = reactive_control.safety_node:main',
            'gap_follow_node = reactive_control.gap_follow_node:main',
            'wall_follow_node = reactive_control.wall_follow_node:main',
            'cv_node = reactive_control.cv_node:main',
        ],
    },
)
