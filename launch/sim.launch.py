import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
CUBE_XYZ = (0.20, 0.0, 0.02)     # cartesianMove_gz.py içindeki CUBE ile AYNI olmalı
CUBE_SIZE = 0.04
CUBE_SDF_PATH = '/tmp/mycobot_cube.sdf'


def build_urdf():
    desc = get_package_share_directory('mycobot_description')
    src = os.path.join(desc, 'urdf', 'mycobot_280_m5', 'mycobot_280_m5_adaptive_gripper.urdf')
    ctrl_yaml = os.path.join(get_package_share_directory('mycobot_280_moveit2'),
                             'config', 'ros2_controllers.yaml')
    robot = ET.parse(src).getroot()

    # package:// -> file://
    for el in robot.iter():
        fn = el.get('filename')
        if fn and fn.startswith('package://'):
            pkg, rest = fn[len('package://'):].split('/', 1)
            el.set('filename', 'file://' + os.path.join(get_package_share_directory(pkg), rest))

    # gripper eklemlerini sabitle (açık pozda rijit)
    for j in robot.findall('joint'):
        if j.get('name', '').startswith('gripper'):
            j.set('type', 'fixed')
            for tag in ('mimic', 'axis', 'limit', 'dynamics'):
                for e in j.findall(tag):
                    j.remove(e)

    # --- STABİL MOD: çarpışmaları kaldır, yerçekimini kapat ---
    for link in robot.findall('link'):
        for col in link.findall('collision'):
            link.remove(col)
        for ine in link.findall('inertial'):
            link.remove(ine)
        m = '2.0' if link.get('name') == 'g_base' else '0.2'
        ine = ET.Element('inertial')
        ET.SubElement(ine, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ET.SubElement(ine, 'mass', {'value': m})
        ET.SubElement(ine, 'inertia', {'ixx': '1e-3', 'iyy': '1e-3', 'izz': '1e-3',
                                       'ixy': '0', 'ixz': '0', 'iyz': '0'})
        link.insert(0, ine)
        if link.get('name') != 'world':
            g = ET.SubElement(robot, 'gazebo', {'reference': link.get('name')})
            ET.SubElement(g, 'gravity').text = 'false'
            ET.SubElement(g, 'self_collide').text = 'false'

    # eklemlere sönümleme (titremeyi azaltır)
    for j in robot.findall('joint'):
        if j.get('type') in ('revolute', 'continuous'):
            for d in j.findall('dynamics'):
                j.remove(d)
            ET.SubElement(j, 'dynamics', {'damping': '0.5', 'friction': '0.05'})

    # world -> g_base sabit eklemi
    ET.SubElement(robot, 'link', {'name': 'world'})
    wj = ET.SubElement(robot, 'joint', {'name': 'world_to_g_base', 'type': 'fixed'})
    ET.SubElement(wj, 'parent', {'link': 'world'})
    ET.SubElement(wj, 'child', {'link': 'g_base'})
    ET.SubElement(wj, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})

    # ros2_control + gazebo eklentisi
    rc = ET.SubElement(robot, 'ros2_control', {'name': 'GazeboSystem', 'type': 'system'})
    ET.SubElement(ET.SubElement(rc, 'hardware'), 'plugin').text = 'gazebo_ros2_control/GazeboSystem'
    for name in ARM_JOINTS:
        je = ET.SubElement(rc, 'joint', {'name': name})
        ET.SubElement(je, 'command_interface', {'name': 'position'})
        ET.SubElement(je, 'state_interface', {'name': 'position'})
        ET.SubElement(je, 'state_interface', {'name': 'velocity'})
    gz = ET.SubElement(robot, 'gazebo')
    pl = ET.SubElement(gz, 'plugin', {'filename': 'libgazebo_ros2_control.so',
                                      'name': 'gazebo_ros2_control'})
    ET.SubElement(pl, 'parameters').text = ctrl_yaml

    return ET.tostring(robot, encoding='unicode')


def cube_sdf():
    s = CUBE_SIZE
    return f"""<?xml version='1.0'?>
<sdf version='1.6'><model name='cube'><link name='link'><gravity>0</gravity><kinematic>true</kinematic><velocity_decay><linear>5.0</linear><angular>5.0</angular></velocity_decay>
<inertial><mass>0.05</mass><inertia><ixx>1.3e-5</ixx><iyy>1.3e-5</iyy><izz>1.3e-5</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='c'><geometry><box><size>{s} {s} {s}</size></box></geometry>
<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
<visual name='v'><geometry><box><size>{s} {s} {s}</size></box></geometry>
<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
</link></model></sdf>"""


def generate_launch_description():
    # küp SDF'sini dosyaya yaz (spawn_entity.py -string desteklemiyor)
    with open(CUBE_SDF_PATH, 'w') as f:
        f.write(cube_sdf())

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(get_package_share_directory('mycobot_pick_place_gz'), 'worlds', 'world_state.world')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': build_urdf(), 'use_sim_time': True}])

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                       arguments=['-topic', 'robot_description', '-entity', 'mycobot'])

    spawn_cube = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                      arguments=['-entity', 'cube', '-file', CUBE_SDF_PATH,
                                 '-x', str(CUBE_XYZ[0]), '-y', str(CUBE_XYZ[1]),
                                 '-z', str(CUBE_XYZ[2])])

    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    arm = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['arm_group_controller', '--controller-manager', '/controller_manager'])

    mc = MoveItConfigsBuilder('firefighter', package_name='mycobot_280_moveit2').to_moveit_configs()
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[mc.to_dict(), {
                          'use_sim_time': True,
                          'allow_trajectory_execution': True,
                          'publish_planning_scene': True,
                          'publish_geometry_updates': True,
                          'publish_state_updates': True,
                          'publish_transforms_updates': True,
                          'monitor_dynamics': False}])

    return LaunchDescription([
        gazebo, rsp, spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb, spawn_cube])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        move_group,
    ])
