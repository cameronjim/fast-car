import os
from glob import glob
from setuptools import setup

package_name = 'learned_control'

setup(
    name=package_name,
    version='0.1.0',
    package_dir={
        package_name: 'nodes',
        package_name + '.bc': 'bc',
        package_name + '.sac': 'sac',
    },
    packages=[package_name, package_name + '.bc', package_name + '.sac'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Model weights and scalers (needed at runtime by inference nodes)
        (os.path.join('share', package_name, 'bc'), glob(os.path.join('bc', '*.pth'))),
        (os.path.join('share', package_name, 'sac'), glob(os.path.join('sac', '*.pth'))),
        (os.path.join('share', package_name, 'processed'),
            glob(os.path.join('processed', '*.npz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Autonomous Racing',
    maintainer_email='noreply@example.com',
    description='Learning-based driving for the F1TENTH platform: behavioural cloning '
                'and Soft Actor-Critic policies with an independent LiDAR safety layer.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = learned_control.safety_node:main',
            'bc_demo_node = learned_control.bc_demo_node:main',
            'sac_train_node = learned_control.sac_train_node:main',
            'sac_demo_node = learned_control.sac_demo_node:main',
        ],
    },
)
