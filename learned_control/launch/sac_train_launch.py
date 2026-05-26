"""
SAC training launch (simulator).

Runs online SAC training together with the safety node. Training is intended for
the F1TENTH Gym simulator, since it resets the car on a crash. The sim argument
defaults to true; pass sim:=false only if you have wired up odometry on a
physical setup.

Launch:
    ros2 launch learned_control sac_train_launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Get the share directory and source directory
_SHARE = get_package_share_directory('learned_control')
_SRC = os.path.join(os.path.expanduser('~'), 'f1tenth_ws', 'src', 'learned_control')


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for SAC training.

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
        # Safety node
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
        # SAC training node
        Node(
            package='learned_control',
            executable='sac_train_node',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'bc_weights_path': os.path.join(_SHARE, 'bc', 'bc_model.pth'),
                'scalers_path': os.path.join(_SHARE, 'processed', 'scalers.npz'),
                # Start from the BC policy, then begin SAC updates after warmup.
                'initial_checkpoint_path': os.path.join(_SRC, 'sac', 'sac_checkpoint_best.pth'),
                'checkpoint_path': os.path.join(_SRC, 'sac', 'sac_checkpoint.pth'),
                'log_path': os.path.join(_SRC, 'sac', 'training_log.csv'),
                'max_speed': 1.2,
                'min_speed': 0.7,
                'deterministic': False,
                'resume_training': False,
                'lr_actor': 1e-4,
                'lr_critic': 3e-4,
                'warmup_steps': 4000,
                'learning_starts': 4000,
                'actor_learning_starts': 12000,
                'bc_reg_weight': 5.0,
                'bc_reg_decay_steps': 150000,
            }],
        ),
    ])
