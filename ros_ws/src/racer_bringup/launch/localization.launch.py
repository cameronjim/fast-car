"""localization.launch.py -- AMCL on a saved map, on top of the EKF odometry.

    odometry.launch.py (odometry:=true)   -> /odom, TF odom -> base_link
    nav2_map_server map_server            -> /map (latched), from the map:= yaml
    nav2_amcl amcl                        -> TF map -> odom, /pose (amcl_pose remapped)
    nav2_lifecycle_manager                -> configures and activates the two above

/pose is geometry_msgs/PoseWithCovarianceStamped (claude-docs/04-architecture.md), published
whenever the filter updates (every 0.1 m or 0.1 rad of travel). A map-frame pose at the EKF's
rate is TF map -> base_link, which is what tracker_node uses (pose_frame:=map).

map is REQUIRED: a slam_toolbox yaml from mapping.launch.py (the launch refuses without one).
The initial pose (initial_pose_x/y/yaw, map frame) defaults to the map origin, which is where
the car stood when mapping started: put the car back on that spot facing the same way, or give
the pose here, or set it later from Foxglove's pose estimate (/initialpose). AMCL replaces the
F1TENTH particle filter the architecture doc names; see docs/notes/localization-2026-11-03.md.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_ODOMETRY_LAUNCH_FILE = str(Path(__file__).resolve().parent / "odometry.launch.py")


def _require_map(context):
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        raise RuntimeError(
            "localization.launch.py: map:=<path to a map yaml> is required (save one with "
            "mapping.launch.py, see docs/notes/localization-2026-11-03.md)"
        )
    if not Path(map_yaml).is_file():
        raise RuntimeError(f"localization.launch.py: map yaml not found: {map_yaml}")
    return []


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Map yaml to localise on (slam_toolbox save_map output). Required.",
        ),
        DeclareLaunchArgument(
            "odometry",
            default_value="true",
            description="Also start odometry.launch.py (wheel_twist_node + ekf_node).",
        ),
        DeclareLaunchArgument(
            "amcl_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("racer_state"), "config", "amcl.yaml"]
            ),
            description="AMCL parameters (racer_state/config/amcl.yaml).",
        ),
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="true",
            description="Start AMCL at initial_pose_* (true) or wait for /initialpose (false).",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0", description="m, map frame"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0", description="m, map frame"),
        DeclareLaunchArgument(
            "initial_pose_yaw", default_value="0.0", description="rad, map frame, CCW positive"
        ),
    ]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description=(
            "Run the nodes on /clock (the sim bridge's sim_time mode, GitHub issue 156). "
            "false everywhere on the car."
        ),
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_ODOMETRY_LAUNCH_FILE),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        condition=IfCondition(LaunchConfiguration("odometry")),
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map"), "use_sim_time": use_sim_time}],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            LaunchConfiguration("amcl_params_file"),
            {
                "use_sim_time": use_sim_time,
                "set_initial_pose": ParameterValue(
                    LaunchConfiguration("set_initial_pose"), value_type=bool
                ),
                "initial_pose.x": ParameterValue(
                    LaunchConfiguration("initial_pose_x"), value_type=float
                ),
                "initial_pose.y": ParameterValue(
                    LaunchConfiguration("initial_pose_y"), value_type=float
                ),
                "initial_pose.yaw": ParameterValue(
                    LaunchConfiguration("initial_pose_yaw"), value_type=float
                ),
            },
        ],
        remappings=[("amcl_pose", "/pose")],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server", "amcl"],
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return LaunchDescription(
        [
            *args,
            use_sim_time_arg,
            OpaqueFunction(function=_require_map),
            odometry_launch,
            map_server,
            amcl,
            lifecycle_manager,
        ]
    )
