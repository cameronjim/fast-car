"""
Wall-following launch.

Brings up the safety/AEB node and the reactive wall-following controller.

Use sim:=false to run on the physical car (uses /odom instead of
/ego_racecar/odom).

Launch:
    ros2 launch reactive_control wall_follow_launch.py
    ros2 launch reactive_control wall_follow_launch.py sim:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for wall following.

    Returns:
        The launch description.
    """
    sim = LaunchConfiguration('sim')
    odom_topic = PythonExpression(["'/ego_racecar/odom' if '", sim, "' == 'true' else '/odom'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true for the F1TENTH Gym simulator (/ego_racecar/odom), '
                        'false for the physical car (/odom)'),
        Node(
            package='reactive_control',
            executable='safety_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('reactive_control'), 'config', 'safety_params.yaml']),
                {'odom_topic': odom_topic},
            ],
        ),
        Node(
            package='reactive_control',
            executable='wall_follow_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('reactive_control'), 'config', 'wall_follow_params.yaml']),
                {'odom_topic': odom_topic},
            ],
        ),
    ])
