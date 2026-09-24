"""Nav2 yığını — LIMO, ROS 2 Humble.

Upstream limo_ros2/foxy'deki limo_navigation.launch.py + limo_localization.launch.py
yerine geçer. Oradaki üç şey Humble'da çalışmıyordu:

  * ``from nav2_common.launch import Node``  → Foxy'de kaldırılmıştı, ImportError verir
  * ``node_name=``                           → Dashing/Eloquent imzası, karşılığı ``name=``
  * ``package='nav2_recoveries'``            → Humble'da paket adı nav2_behaviors

Ayrıca upstream'de map_server lifecycle listesinde olmasına rağmen hiçbir launch
dosyası onu başlatmıyordu ve velocity_smoother / collision_monitor hiç yoktu.

Hız zinciri (Humble nav2_bringup ile aynı, sonuna collision_monitor eklenmiş):

    controller_server ──cmd_vel_nav──> velocity_smoother ──cmd_vel_smoothed──>
    collision_monitor ──cmd_vel──> limo_base
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('limo_bringup')

    mode = LaunchConfiguration('mode')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_localization = LaunchConfiguration('use_localization')
    log_level = LaunchConfiguration('log_level')
    nav_to_pose_bt = LaunchConfiguration('nav_to_pose_bt')
    nav_through_poses_bt = LaunchConfiguration('nav_through_poses_bt')

    # /tf ve /tf_static'i göreli isme çevir ki namespace uygulanabilsin.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'default_nav_to_pose_bt_xml': nav_to_pose_bt,
        'default_nav_through_poses_bt_xml': nav_through_poses_bt,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = ['controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                        'collision_monitor']

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Üst düzey ad alanı'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo saatini kullan'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Lifecycle düğümlerini otomatik başlat'),

        DeclareLaunchArgument(
            'mode', default_value='ackermann',
            choices=['ackermann', 'diff'],
            description='Sürüş modu. Parametre dosyasını ve davranış ağaçlarını seçer.'),

        DeclareLaunchArgument(
            'params_file',
            default_value=[bringup_dir, '/param/nav2_', mode, '.yaml'],
            description='Nav2 parametre dosyası. Varsayılan olarak mode argümanından türer.'),

        # Ackermann profilinde behavior_server "spin" eklentisini yüklemiyor
        # (araç yerinde dönemez). Humble'ın varsayılan ağaçları Spin çağırdığı için
        # bt_navigator "Action server spin not available" verip yığını durdururdu;
        # bu yüzden Ackermann'a Spin'siz kendi ağaçları veriliyor.
        DeclareLaunchArgument(
            'nav_to_pose_bt',
            default_value=[bringup_dir, '/behavior_trees/navigate_to_pose_', mode, '.xml'],
            description='navigate_to_pose davranış ağacı'),

        DeclareLaunchArgument(
            'nav_through_poses_bt',
            default_value=[bringup_dir, '/behavior_trees/navigate_through_poses_', mode, '.xml'],
            description='navigate_through_poses davranış ağacı'),

        DeclareLaunchArgument(
            'map', default_value='',
            description='Harita yaml dosyası. Boşsa map_server boş harita bekler '
                        '(SLAM ile kullanılıyorsa use_localization:=false verin).'),

        DeclareLaunchArgument(
            'use_localization', default_value='true',
            description='map_server + amcl başlatılsın mı. SLAM kullanırken false.'),

        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Günlük seviyesi'),

        # ---------------- Lokalizasyon ----------------

        Node(
            condition=IfCondition(use_localization),
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=IfCondition(use_localization),
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=IfCondition(use_localization),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': localization_nodes}]),

        # ---------------- Navigasyon ----------------

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # Foxy'de recoveries_server idi.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # cmd_vel_nav dinler, cmd_vel_smoothed yayınlar.
        # Resmi nav2_bringup burada cmd_vel_smoothed'ı cmd_vel'e remap eder; biz
        # etmiyoruz çünkü araya collision_monitor giriyor.
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        # Zincirin sonu. Nav2'den bağımsız çalışır: engel yakınsa cmd_vel'i keser.
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': navigation_nodes}]),
    ])
