"""Launch the LIMO autonomy patrol node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the patrol node."""
    pkg_share = FindPackageShare('limo_autonomy')
    default_mission_file = PathJoinSubstitution([pkg_share, 'config', 'sim_patrol.yaml'])

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission_file,
        description='Path to the mission YAML configuration file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )

    set_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='false',
        description='Whether to set the initial pose before starting patrol.',
    )

    extra_args = [
        DeclareLaunchArgument('localizer', default_value='amcl',
                              description="amcl | slam_toolbox (lifecycle olmayan yerelleştirici)."),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('goal_timeout_s', default_value='180.0'),
    ]

    patrol_node = Node(
        package='limo_autonomy',
        executable='patrol',
        name='patrol',
        output='screen',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'localizer': LaunchConfiguration('localizer'),
            'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
            'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
            'initial_yaw_deg': ParameterValue(LaunchConfiguration('initial_yaw_deg'), value_type=float),
            'goal_timeout_s': ParameterValue(LaunchConfiguration('goal_timeout_s'), value_type=float),
        }],
    )

    return LaunchDescription([
        mission_file_arg,
        use_sim_time_arg,
        set_initial_pose_arg,
        *extra_args,
        patrol_node,
    ])
