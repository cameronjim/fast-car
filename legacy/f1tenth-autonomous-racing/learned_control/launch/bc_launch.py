"""brings up the safety node and the behavioural cloning inference node."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

_SHARE = get_package_share_directory('learned_control')


def generate_launch_description() -> LaunchDescription:
    sim = LaunchConfiguration('sim')
    odom_topic = PythonExpression(["'/ego_racecar/odom' if '", sim, "' == 'true' else '/odom'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true for the F1TENTH Gym simulator (/ego_racecar/odom), '
                        'false for the physical car (/odom)'),
        Node(
            package='learned_control',
            executable='safety_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('learned_control'), 'config', 'safety_params.yaml']),
                {'odom_topic': odom_topic},
            ],
        ),
        Node(
            package='learned_control',
            executable='bc_demo_node',
            output='screen',
            parameters=[{
                'model_path': os.path.join(_SHARE, 'bc', 'bc_model.pth'),
                'scalers_path': os.path.join(_SHARE, 'processed', 'scalers.npz'),
                'max_speed': 1.0,
                'min_speed': 0.7,
            }],
        ),
    ])
