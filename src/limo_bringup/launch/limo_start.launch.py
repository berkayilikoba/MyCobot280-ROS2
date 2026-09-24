"""LIMO donanım bringup — ROS 2 Humble.

Robottaki ROS 1 ``limo_start.launch`` dosyasının birebir karşılığı. Oradaki saha
ayarları buraya taşındı; upstream limo_ros2/foxy sürümü bunların hiçbirini
içermiyordu:

  * seri port ``ttyTHS0``  (upstream ``ttyTHS1``)
  * LiDAR EAI T-mini Pro   (upstream YDLIDAR X2L — bu cihazla çalışmaz)
  * ``base_link -> laser_link`` dönüşümü, yaw ``-0.15 rad`` (≈ -8.6°) montaj kalibrasyonlu.
    ROS 1'deki konumsal argümanların sırası x y z YAW pitch roll'dur;
    tf2_echo ile ölçüldü: RPY [0.000, 0.000, -0.150]
    (upstream ROS 2 launch'ında laser dönüşümü hiç yayınlanmıyordu; costmap
    ``/scan``'i dönüştüremediği için engel katmanı boş kalırdı)
  * ``base_link -> camera_link`` dönüşümü
  * ``pub_odom_tf = false`` + EKF — TF'i robot_localization yayınlar

Gereksinim: ``ydlidar_ros2_driver`` (humble dalı) workspace'te derlenmiş olmalı.
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    (önce YDLidar-SDK kurulmalı)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('limo_bringup')

    port_name = LaunchConfiguration('port_name')
    use_ekf = LaunchConfiguration('use_ekf')
    use_lidar = LaunchConfiguration('use_lidar')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ydlidar_params = os.path.join(bringup_dir, 'param', 'ydlidar_tmini.yaml')
    ekf_params = os.path.join(bringup_dir, 'param', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name', default_value='ttyTHS0',
            description='Taban seri portu. Robotta ttyTHS0; upstream ttyTHS1 yazıyordu. '
                        'udev kuralı varsa ttylimo de olabilir.'),

        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='robot_localization EKF çalışsın mı. true ise limo_base '
                        'TF yayınlamaz (pub_odom_tf=false) — iki yayıncı çakışmasın.'),

        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='T-mini Pro sürücüsü başlatılsın mı'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo saatini kullan'),

        # ---------------- Taban sürücüsü ----------------

        Node(
            package='limo_base',
            executable='limo_base',
            name='limo_base_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port_name': port_name,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom',
                # EKF açıkken sürücü TF yayınlamamalı. Robottaki ROS 1 kurulumu da
                # tam olarak bunu yapıyor (limo_start.launch: pub_odom_tf=false).
                'pub_odom_tf': False,
                'control_rate': 50,
                'use_sim_time': use_sim_time,
            }]),

        # ---------------- LiDAR ----------------

        Node(
            condition=IfCondition(use_lidar),
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[ydlidar_params, {'use_sim_time': use_sim_time}]),

        # ---------------- Sensör dönüşümleri ----------------
        # Değerler robottaki ROS 1 limo_start.launch'tan. URDF'teki karşılıkları
        # farklı (laser z = -0.034, yaw kalibrasyonu yok) — sahada kalibre edilmiş olan bu.

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_link',
            arguments=['--x', '0.105', '--y', '0.0', '--z', '0.08',
                       '--yaw', '-0.15',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_link',
            arguments=['--x', '0.105', '--z', '0.1',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),

        # ---------------- Odometri füzyonu ----------------

        Node(
            condition=IfCondition(use_ekf),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}]),
    ])
