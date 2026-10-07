#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, get_action_names_and_types
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.parameter import Parameter

# ================= AYARLAR =================
GROUP = 'arm_group'
BASE = 'g_base'
EE_LINK = 'joint6_flange'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']

CUBE = (0.20, 0.0, 0.02)     # x ileri, y yan, z yukarı (m), g_base çerçevesinde     # küp merkezi (m), BASE çerçevesinde
TOOL_LEN = 0.10              # flanştan gripper ucuna mesafe (m) -> kendi gripper'ına göre ayarla
APPROACH_H = 0.06            # küpün üstündeki ara nokta yüksekliği (m)
VMAX = 0.6                   # rad/s, hız sınırı (yavaş = güvenli)
MIN_DT = 0.02
CONTROLLER_ACTION = None     # örn: '/arm_group_controller/follow_joint_trajectory'; None = otomatik bul
# Flanş eksenleri: tool ekseni = -y_flanş varsayımı. Bu yönelim aracı aşağı çevirir.
TOOL_DOWN = (0.0, 1.0, 0.0, 0.0)
# ===========================================


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def qaxis(axis, deg):
    h = math.radians(deg) / 2.0
    s, c = math.sin(h), math.cos(h)
    return {'x': (s, 0, 0, c), 'y': (0, s, 0, c), 'z': (0, 0, s, c)}[axis]


def candidate_orientations():
    """Aşağı bakan yönelim + farklı eğim/dönüş açıları."""
    out = []
    for tilt in (0, 15, -15, 30, -30, 45):
        for roll in (0, 90, 180, 270):
            q = qmul(qaxis('y', tilt), qmul(qaxis('z', roll), TOOL_DOWN))
            out.append(((tilt, roll), q))
    return out


class PickApproach(Node):
    def __init__(self):
        super().__init__('pick_approach_node',
                 parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_subscription(JointState, 'joint_states', self.js_cb, 10)
        self.cart_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.ik_cli = self.create_client(GetPositionIK, 'compute_ik')
        self.joints = {}
        self.create_timer(0.2, self.publish_marker)
        self.action_cli = None

    def js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = BASE
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'pick_place_cube', 0
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = CUBE
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.a = 1.0, 1.0
        self.marker_pub.publish(m)

    # ---------- yardımcılar ----------
    def wait_future(self, fut, timeout=10.0):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def robot_state(self, joint_dict):
        rs = RobotState()
        rs.joint_state.name = list(joint_dict.keys())
        rs.joint_state.position = list(joint_dict.values())
        return rs

    def make_pose(self, xyz, q):
        p = PoseStamped()
        p.header.frame_id = BASE
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = q
        return p

    def find_action(self):
        if CONTROLLER_ACTION:
            return CONTROLLER_ACTION
        for name, types in get_action_names_and_types(self):
            if 'control_msgs/action/FollowJointTrajectory' in types:
                return name
        return None

    def solve_ik(self, pose, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state = self.robot_state(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        res = self.wait_future(self.ik_cli.call_async(req))
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        if not all(j in sol for j in ARM_JOINTS):
            return None
        return {j: sol[j] for j in ARM_JOINTS}

    def cartesian(self, start_joints, goal_pose):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state = self.robot_state(start_joints)
        req.group_name = GROUP
        req.link_name = EE_LINK
        req.waypoints = [goal_pose.pose]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        return self.wait_future(self.cart_cli.call_async(req))

    def retime(self, names, positions_list):
        """Cartesian path zamansız gelir: her nokta için süre hesapla."""
        pts, t, prev = [], 0.0, None
        for pos in positions_list:
            if prev is not None:
                dq = max(abs(a - b) for a, b in zip(pos, prev))
                t += max(dq / VMAX, MIN_DT)
            prev = pos
            pts.append((list(pos), t))
        return pts

    def execute(self, names, timed_points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for pos, t in timed_points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        gh = self.wait_future(self.action_cli.send_goal_async(goal))
        if gh is None or not gh.accepted:
            self.get_logger().error('Kontrolcü hedefi reddetti.')
            return False
        res = self.wait_future(gh.get_result_async(), timeout=120.0)
        ok = res is not None and res.result.error_code == 0
        self.get_logger().info('Hareket tamamlandı.' if ok else f'Hareket hatası: {res.result.error_string if res else "timeout"}')
        return ok

    # ---------- ana akış ----------
    def run(self):
        for cli, nm in ((self.cart_cli, 'compute_cartesian_path'), (self.ik_cli, 'compute_ik')):
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'{nm} yok. move_group çalışıyor mu?')
                return

        act = self.find_action()
        if act is None:
            self.get_logger().error("FollowJointTrajectory action'ı bulunamadı. `ros2 action list` bak, "
                                    'CONTROLLER_ACTION değişkenini doldur.')
            return
        self.get_logger().info(f'Kontrolcü action: {act}')
        self.action_cli = ActionClient(self, FollowJointTrajectory, act)
        if not self.action_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action sunucusu yanıt vermiyor.')
            return

        # mevcut eklem durumu
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(j in self.joints for j in ARM_JOINTS):
                break
        else:
            self.get_logger().error('joint_states alınamadı.')
            return
        current = dict(self.joints)

        grasp_xyz = (CUBE[0], CUBE[1], CUBE[2] + TOOL_LEN)
        appr_xyz = (CUBE[0], CUBE[1], CUBE[2] + TOOL_LEN + APPROACH_H)

        # yönelim adaylarını dene: IK + düz çizgi inişi
        chosen = None
        for (tilt, roll), q in candidate_orientations():
            ready = self.solve_ik(self.make_pose(appr_xyz, q), current)
            if ready is None:
                continue
            res = self.cartesian({**current, **ready}, self.make_pose(grasp_xyz, q))
            if res and res.fraction >= 0.99 and res.solution.joint_trajectory.points:
                chosen = (tilt, roll, q, ready, res)
                break
            self.get_logger().info(f'Aday (eğim={tilt}, dönüş={roll}) IK ok, iniş %{(res.fraction*100 if res else 0):.0f}')

        if chosen is None:
            self.get_logger().error('Hiçbir yönelimle küpe ulaşılamadı. CUBE konumunu robota yaklaştır '
                                    'veya TOOL_LEN değerini küçült.')
            return
        tilt, roll, q, ready, res = chosen
        self.get_logger().info(f'Seçilen yönelim: eğim={tilt}°, dönüş={roll}°')

        # 1) hazır poza eklem uzayında git
        dq = max(abs(ready[j] - current[j]) for j in ARM_JOINTS)
        t = max(dq / VMAX, 1.0)
        self.get_logger().info('Hazır poza gidiliyor...')
        if not self.execute(ARM_JOINTS, [([ready[j] for j in ARM_JOINTS], t)]):
            return

        # 2) düz çizgiyle iniş
        jt = res.solution.joint_trajectory
        timed = self.retime(jt.joint_names, [p.positions for p in jt.points])
        self.get_logger().info(f'Küpe iniliyor ({len(timed)} nokta)...')
        self.execute(list(jt.joint_names), timed)


def main():
    rclpy.init()
    node = PickApproach()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()