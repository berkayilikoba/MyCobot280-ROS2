import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'limo_description'
    robot_name_in_model = 'limo_mycobot'
    rviz_config_file_path = 'rviz/urdf.rviz'
    urdf_file_path = 'urdf/limo_mycobot.xacro'
    world_file_path = 'worlds/neighborhood.world'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.15'
    spawn_yaw_val = '0.00'

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_mycobot = FindPackageShare(package='mycobot_description').find('mycobot_description')
    
    try:
        pkg_limo_car = FindPackageShare(package='limo_car').find('limo_car')
        world_path = os.path.join(pkg_limo_car, 'worlds', 'empty_world.model')
        if not os.path.isfile(world_path):
            world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')
    except Exception:
        world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

    # Disable remote model downloads to prevent hangs
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    # Configure GAZEBO_MODEL_PATH so Gazebo can resolve all meshes
    gazebo_model_paths = [
        os.path.join(pkg_share, gazebo_models_path),
        os.path.dirname(pkg_share),
        os.path.dirname(pkg_mycobot),
        '/usr/share/gazebo-11/models',
        os.path.expanduser('~/.gazebo/models'),
    ]
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(gazebo_model_paths) + ((':' + current_model_path) if current_model_path else '')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    kol_kilit = LaunchConfiguration('kol_kilit')

    # Declare the launch arguments
    declare_mode_cmd = DeclareLaunchArgument(
        name='mode',
        default_value='diff',
        choices=['diff', 'ackermann'],
        description='Drive mode: diff or ackermann'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_kol_kilit_cmd = DeclareLaunchArgument(
        name='kol_kilit',
        default_value='true',
        description='true: myCobot eklemleri fixed (simde kolu tutan kontrolcü yok, yoksa kol çöküp lidarı kapatır)'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    # Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # value_type=str şart: yoksa launch_ros URDF'i YAML olarak ayrıştırmaya çalışır ve
            # xacro'nun taşıdığı bir XML yorumundaki 'kelime:' bile launch'ı düşürür
            # ("mapping values are not allowed here"). display_*.launch.py'ler de böyle.
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_model, ' mode:=', mode, ' kol_kilit:=', kol_kilit]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # Publish the joint states of the robot
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file]
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(["'", use_simulator, "'.lower() == 'true' and '", headless, "'.lower() != 'true'"]))
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-package_to_model',
            '-timeout', '60.0',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_kol_kilit_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_rviz_cmd)
    # Gazebo ackermann_drive angular.z'yi direksiyon açısı sayar; Nav2/gerçek sürücü yaw hızı
    # gönderir. Adaptör /cmd_vel'i gerçek sürücüyle aynı formülle cmd_vel_steer'e çevirir.
    ld.add_action(Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ackermann'"])),
        package='limo_kinematics', executable='ackermann_cmd_adapter',
        name='ackermann_cmd_adapter', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]))
    # Diff: planar_move komutu anında uygular, gerçek taban ise gecikmeli/ağır tepki verir. Taban modeli
    # /cmd_vel'i ölçülmüş tepkiyle cmd_vel_taban'a çevirir (limo_kinematics/taban_modeli.py).
    ld.add_action(Node(
        condition=IfCondition(PythonExpression(["'", mode, "' != 'ackermann'"])),
        package='limo_kinematics', executable='taban_modeli',
        name='taban_modeli', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]))

    return ld
