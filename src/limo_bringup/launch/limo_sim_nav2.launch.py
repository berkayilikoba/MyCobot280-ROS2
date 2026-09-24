"""Gazebo + SLAM + Nav2 — tek komutluk simülasyon ortamı.

Robot olmadan Nav2 ayarlarını denemek için. Üç parçayı birleştirir:

  1. Gazebo simülasyonu:
     - use_arm:=true (varsayılan)  -> limo_description/gazebo_limo_mycobot.launch.py (MyCobot robot kollu LIMO)
     - use_arm:=false              -> limo_car/ackermann_gazebo.launch.py (Ackermann LIMO)
  2. slam_toolbox (online async)   -> canlı harita, map -> odom TF'i
  3. limo_bringup/limo_nav2.launch.py -> Nav2 yığını (use_localization:=false,
                                        çünkü map -> odom'u SLAM veriyor)

Kullanım:
    # Kollu robot (MyCobot 280) + 4WD diferansiyel Nav2:
    ros2 launch limo_bringup limo_sim_nav2.launch.py

    # Kolsuz Ackermann modeli ile çalıştırmak için:
    ros2 launch limo_bringup limo_sim_nav2.launch.py use_arm:=false mode:=ackermann
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('limo_bringup')
    limo_car_dir = get_package_share_directory('limo_car')
    limo_desc_dir = get_package_share_directory('limo_description')

    mode = LaunchConfiguration('mode')
    use_arm = LaunchConfiguration('use_arm')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    localization = LaunchConfiguration('localization')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav_to_pose_bt = LaunchConfiguration('nav_to_pose_bt')
    is_slam = PythonExpression(["'", localization, "' == 'slam'"])
    is_amcl = PythonExpression(["'", localization, "' == 'amcl'"])
    slam_params = os.path.join(bringup_dir, 'param', 'slam_toolbox_limo.yaml')

    nav2_rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    # Robot kolu (MyCobot 280) takılı 4WD Diferansiyel Gazebo Simülasyonu
    gazebo_arm = GroupAction(
        condition=IfCondition(use_arm),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(limo_desc_dir, 'launch', 'gazebo_limo_mycobot.launch.py')
                ),
                launch_arguments={
                    'mode': mode,
                    'use_sim_time': 'true',
                    'gui': 'false',
                    'headless': headless,
                    'use_rviz': use_rviz,
                    'rviz_config_file': nav2_rviz_config,
                }.items()
            )
        ]
    )

    # Standart Kolsuz Ackermann Gazebo Simülasyonu
    gazebo_ackermann = GroupAction(
        condition=UnlessCondition(use_arm),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(limo_car_dir, 'launch', 'ackermann_gazebo.launch.py')
                )
            )
        ]
    )

    # Gazebo ayağa kalkıp /scan yayınlamaya başlamadan SLAM açılırsa ilk taramaları
    # kaçırıp TF ağacını geç kurar; Nav2 de o sırada costmap'i boş görür.
    slam = TimerAction(
        period=8.0,
        condition=IfCondition(is_slam),
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True}])])

    nav2 = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'limo_nav2.launch.py')),
            launch_arguments={
                'mode': mode,
                # Kapsam sızıntısına karşı ikinci savunma: bu üçü açıkça
                # geçilmezse dışarıdaki boş değerleri miras alırlar.
                'params_file': nav2_params_file,
                'nav_to_pose_bt': nav_to_pose_bt,
                'nav_through_poses_bt': [bringup_dir, '/behavior_trees/navigate_through_poses_', mode, '.xml'],
                'use_sim_time': 'true',
                # localization:=slam -> map->odom'u slam_toolbox yayınlar, amcl KAPALI
                # (ikisi birden açılırsa aynı dönüşümü iki düğüm yayınlar, TF bozulur).
                # localization:=amcl -> map_server + amcl, bilinen harita.
                'use_localization': is_amcl,
                'map': map_yaml,
            }.items())])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_arm', default_value='true',
            description='Robot kolu (MyCobot 280) entegreli modeli kullan (varsayılan: true)'),
        DeclareLaunchArgument(
            'mode', default_value='diff',
            choices=['ackermann', 'diff'],
            description='Sürüş modu. Nav2 parametre dosyasını ve davranış ağaçlarını seçer. '
                        'Kollu model 4 tekerlek diferansiyel (diff) sürüşlüdür.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Gazebo GUI (gzclient) kapalı çalıştır (varsayılan: false)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='RViz2 başlat (varsayılan: true)'),
        DeclareLaunchArgument(
            'localization', default_value='slam',
            choices=['slam', 'amcl'],
            description='slam: slam_toolbox canlı harita. amcl: map argümanındaki bilinen '
                        'harita + AMCL (ölçüm koşuları için; bkz. scripts/world_to_map.py).'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup_dir, 'maps', 'sim_world.yaml'),
            description='localization:=amcl iken yüklenecek harita'),
        # Ad bilinçli olarak params_file DEĞİL: gzserver.launch.py aynı adlı bir
        # LaunchConfiguration okuyor; dış kapsamdaki değer gzserver'a sızar.
        DeclareLaunchArgument(
            'nav2_params_file', default_value=[bringup_dir, '/param/nav2_', mode, '.yaml'],
            description='Nav2 parametre dosyası (varsayılan: mode argümanından türer). '
                        'Parametre deneyleri için farklı bir dosya verilebilir.'),
        DeclareLaunchArgument(
            'nav_to_pose_bt',
            default_value=[bringup_dir, '/behavior_trees/navigate_to_pose_', mode, '.xml'],
            description='NavigateToPose davranış ağacı (varsayılan: mode argümanından türer).'),
        gazebo_arm,
        gazebo_ackermann,
        slam,
        nav2,
    ])
