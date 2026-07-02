"""brings up the safety node and the rl agent node on an exported policy plus its contract."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('learned_control')
    sim = LaunchConfiguration('sim')
    odom_topic = PythonExpression(["'/ego_racecar/odom' if '", sim, "' == 'true' else '/odom'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true for the F1TENTH Gym simulator (/ego_racecar/odom), '
                        'false for the physical car (/odom)'),
        DeclareLaunchArgument(
            'policy_path', default_value=os.path.join(share, 'policies', 'policy.pt'),
            description='torchscript policy written by gym_training export_policy'),
        DeclareLaunchArgument(
            'obs_config_path', default_value=os.path.join(share, 'policies', 'obs_config.json'),
            description='the deploy contract exported next to that policy'),
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
            executable='rl_agent_node',
            output='screen',
            parameters=[{
                'policy_path': LaunchConfiguration('policy_path'),
                'obs_config_path': LaunchConfiguration('obs_config_path'),
                'odom_topic': odom_topic,
            }],
        ),
    ])
