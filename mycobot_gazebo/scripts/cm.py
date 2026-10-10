#!/usr/bin/env python3
import math
import random
import time

import rclpy
import tf2_ros
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
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity


# ================= AYARLAR =================
GROUP = 'arm_group'
BASE = 'g_base'
EE_LINK = 'joint6_flange'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']

# --- Döngü ---
NUM_CYCLES = 5                   # kaç küp taşınsın (0 = Ctrl+C'ye kadar sonsuz)
HOME_JOINTS = [0.0] * 6          # home pozisyonu (rad), ARM_JOINTS sırasıyla. None = script başladığındaki poz
KEEP_PLACED_CUBES = True         # True: bırakılan küpler sahnede kalır, yer kalmazsa otomatik temizlenir
DELETE_LAUNCH_CUBE = True        # launch dosyasının başta koyduğu 'cube' modelini sil

# --- Rastgele konumlar (g_base çerçevesi, KUTUPSAL örnekleme). Hem alma hem bırakma için geçerli ---
# Açı: 0° = +x (robotun önü), +90° = +y (sol), 180° = robotun tam arkası.
# Arka tarafa da küp düşmesi için geniş aralık verildi. Joint1 sınırı (~±168°) yüzünden
# tam 180° civarı ulaşılamaz; bu yüzden ±160° ile sınırlandı. (-180, 180) yaparsan
# ulaşılamayan konumlar IK tarafından elenir ama daha çok deneme gerekir.
ANGLE_RANGE_DEG = (-160.0, 160.0)
MIN_RADIUS = 0.12                # tabana yatay uzaklık alt sınırı (m)
MAX_RADIUS = 0.25                # tabana yatay uzaklık üst sınırı (m)
CUBE_Z = 0.02                    # küp merkezi yüksekliği (küp 4 cm, zeminde)
MIN_DIST_BETWEEN = 0.06          # küpler (alma, bırakma, sahnedekiler) arası en az mesafe
MAX_TRIES = 20                   # ulaşılabilir konum bulmak için en çok kaç örnek denensin
RANDOM_SEED = None               # sabit sayı verirsen hep aynı konumlar gelir

CUBE_SIZE = 0.04
TOOL_LEN = 0.10              # flanştan gripper ucuna mesafe (m)
APPROACH_H = 0.06            # küpün üstündeki ara nokta yüksekliği (m)
LIFT_H = 0.08                # kaldırma / bırakma öncesi yükseklik (m)
VMAX = 0.6                   # rad/s, hız sınırı
MIN_DT = 0.02

CONTROLLER_ACTION = None     # None = otomatik bul
GZ_SET_STATE_SRV = '/gazebo/set_entity_state'
GZ_SPAWN_SRV = '/spawn_entity'
GZ_DELETE_SRV = '/delete_entity'
CUBE_SMOOTH = 0.4            # 0-1 arası; küçük = daha yumuşak ama biraz gecikmeli

# Flanş eksenleri: tool ekseni = -y_flanş varsayımı. Bu yönelim aracı aşağı çevirir.
TOOL_DOWN = (0.0, 1.0, 0.0, 0.0)

CUBE_COLORS = ['1 0 0 1', '0 0.6 1 1', '0 0.8 0.2 1', '1 0.8 0 1', '0.8 0.2 1 1']
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


def sample_position(avoid):
    """Robotun etrafında (arka dahil) alan-düzgün dağılımlı rastgele konum.
    'avoid' listesindeki noktalardan en az MIN_DIST_BETWEEN uzakta olur."""
    r2_min, r2_max = MIN_RADIUS ** 2, MAX_RADIUS ** 2
    for _ in range(2000):
        r = math.sqrt(random.uniform(r2_min, r2_max))
        ang = math.radians(random.uniform(*ANGLE_RANGE_DEG))
        x = r * math.cos(ang)
        y = r * math.sin(ang)
        if any(math.hypot(x - a[0], y - a[1]) < MIN_DIST_BETWEEN for a in avoid):
            continue
        return (round(x, 3), round(y, 3), CUBE_Z)
    raise RuntimeError('Uygun rastgele konum bulunamadı.')


def cube_sdf(name, color):
    s = CUBE_SIZE
    return f"""<?xml version='1.0'?>
<sdf version='1.6'><model name='{name}'><link name='link'>
<gravity>0</gravity><kinematic>true</kinematic>
<inertial><mass>0.05</mass><inertia><ixx>1.3e-5</ixx><iyy>1.3e-5</iyy><izz>1.3e-5</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='c'><geometry><box><size>{s} {s} {s}</size></box></geometry></collision>
<visual name='v'><geometry><box><size>{s} {s} {s}</size></box></geometry>
<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
</link></model></sdf>"""


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_subscription(JointState, 'joint_states', self.js_cb, 10)
        self.cart_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.ik_cli = self.create_client(GetPositionIK, 'compute_ik')
        self.gz_cli = self.create_client(SetEntityState, GZ_SET_STATE_SRV)
        self.spawn_cli = self.create_client(SpawnEntity, GZ_SPAWN_SRV)
        self.del_cli = self.create_client(DeleteEntity, GZ_DELETE_SRV)
        self.action_cli = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joints = {}
        self.home = None
        self.run_id = int(time.time()) % 100000

        self.attached = False          # True iken küp gripper ile birlikte hareket eder
        self.cube_name = None          # şu an işlenen Gazebo küpünün adı
        self.cube_active = False       # RViz'de aktif küp gösterilsin mi
        self.cube_pos = [0.20, 0.0, CUBE_Z]
        self.placed = []               # bırakılıp sahnede kalan küpler: [(isim, xyz), ...]
        self._max_marker_id = 0

        self._warned_tf = False
        self._warned_gz = False
        self._gz_fail_count = 0
        self.create_timer(0.02, self.update_cube)

    def js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    # ---------- küp takibi ----------
    def update_cube(self):
        """Tutulurken küp konumunu flanştan hesapla; RViz marker + Gazebo küpünü güncelle."""
        if self.attached:
            try:
                t = self.tf_buffer.lookup_transform(BASE, EE_LINK, rclpy.time.Time())
                tgt = [t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z - TOOL_LEN]
                a = CUBE_SMOOTH
                self.cube_pos = [c + a * (g - c) for c, g in zip(self.cube_pos, tgt)]
            except Exception as e:
                if not self._warned_tf:
                    self.get_logger().warn(f'TF alınamadı ({BASE} -> {EE_LINK}): {e}')
                    self._warned_tf = True
            self.sync_gz_cube()
        self.publish_markers()

    def _marker(self, mid, xyz, action):
        m = Marker()
        m.header.frame_id = BASE
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns, m.id = 'pick_place_cube', mid
        m.type, m.action = Marker.CUBE, action
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = CUBE_SIZE
        m.color.r, m.color.a = 1.0, 1.0
        return m

    def publish_markers(self):
        self.marker_pub.publish(self._marker(
            0, self.cube_pos, Marker.ADD if self.cube_active else Marker.DELETE))
        for i, (_, xyz) in enumerate(self.placed, start=1):
            self.marker_pub.publish(self._marker(i, xyz, Marker.ADD))
        for i in range(len(self.placed) + 1, self._max_marker_id + 1):
            self.marker_pub.publish(self._marker(i, (0, 0, 0), Marker.DELETE))
        self._max_marker_id = len(self.placed)

    def sync_gz_cube(self):
        """Gazebo'daki aktif küp modelini cube_pos konumuna ışınlar."""
        if self.cube_name is None:
            return
        if not self.gz_cli.service_is_ready():
            if not self._warned_gz:
                self.get_logger().warn(f'{GZ_SET_STATE_SRV} servisi hazır değil, Gazebo küpü taşınamıyor.')
                self._warned_gz = True
            return
        req = SetEntityState.Request()
        req.state.name = self.cube_name
        req.state.reference_frame = 'world'
        req.state.pose.position.x = float(self.cube_pos[0])
        req.state.pose.position.y = float(self.cube_pos[1])
        req.state.pose.position.z = float(self.cube_pos[2])
        req.state.pose.orientation.w = 1.0
        fut = self.gz_cli.call_async(req)
        fut.add_done_callback(self._gz_done)

    def _gz_done(self, fut):
        try:
            r = fut.result()
        except Exception as e:
            r = None
            err = str(e)
        else:
            err = getattr(r, 'status_message', '')
        if r is None or not r.success:
            self._gz_fail_count += 1
            if self._gz_fail_count <= 3:
                self.get_logger().warn(f"set_entity_state başarısız (küp '{self.cube_name}'): {err}")

    def flush_spin(self, n=10):
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------- Gazebo: spawn / delete ----------
    def spawn_cube(self, name, xyz):
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{GZ_SPAWN_SRV} servisi yok.')
            return False
        req = SpawnEntity.Request()
        req.name = name
        req.xml = cube_sdf(name, random.choice(CUBE_COLORS))
        req.initial_pose.position.x = float(xyz[0])
        req.initial_pose.position.y = float(xyz[1])
        req.initial_pose.position.z = float(xyz[2])
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'
        res = self.wait_future(self.spawn_cli.call_async(req), timeout=15.0)
        if res is None or not res.success:
            self.get_logger().error(f"Küp spawn edilemedi: {getattr(res, 'status_message', 'timeout')}")
            return False
        return True

    def delete_cube(self, name):
        if not self.del_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'{GZ_DELETE_SRV} servisi yok, {name} silinemedi.')
            return False
        req = DeleteEntity.Request()
        req.name = name
        res = self.wait_future(self.del_cli.call_async(req), timeout=10.0)
        return res is not None and res.success

    def clear_placed(self):
        for name, _ in self.placed:
            self.delete_cube(name)
        self.placed = []

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
                if 'gripper' in name.lower() or 'hand' in name.lower():
                    continue
                return name
        return None

    def current_joints(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        return dict(self.joints)

    def _ik_once(self, pose, seed):
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

    def solve_ik(self, pose, seed):
        """Önce mevcut seed ile dener. Başarısız olursa (özellikle robotun arkasındaki
        hedeflerde) taban eklemini hedefe doğru çevirmiş alternatif seed'lerle tekrar dener."""
        sol = self._ik_once(pose, seed)
        if sol is not None:
            return sol
        px, py = pose.pose.position.x, pose.pose.position.y
        yaw = math.atan2(py, px)
        base_joint = ARM_JOINTS[0]
        for guess in (yaw, yaw - math.copysign(math.pi, yaw) if yaw != 0 else math.pi):
            alt = dict(seed)
            alt[base_joint] = guess
            sol = self._ik_once(pose, alt)
            if sol is not None:
                return sol
        return None

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
        """Cartesian path zamansız gelir: her nokta için süre ve hız hesapla."""
        pts, t, prev = [], 0.0, None
        for pos in positions_list:
            if prev is not None:
                dq = max(abs(a - b) for a, b in zip(pos, prev))
                t += max(dq / VMAX, MIN_DT)
            prev = pos
            pts.append([list(pos), t, None])
        n = len(pts)
        for i in range(n):
            nj = len(pts[i][0])
            if i == 0 or i == n - 1:
                pts[i][2] = [0.0] * nj
            else:
                dt = pts[i + 1][1] - pts[i - 1][1]
                if dt > 1e-6:
                    pts[i][2] = [(b - a) / dt for a, b in zip(pts[i - 1][0], pts[i + 1][0])]
                else:
                    pts[i][2] = [0.0] * nj
        return [(p_, t_, v_) for p_, t_, v_ in pts]

    def execute(self, names, timed_points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for item in timed_points:
            pos, t = item[0], item[1]
            vel = item[2] if len(item) > 2 else ([0.0] * len(pos) if len(timed_points) == 1 else None)
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            if vel is not None:
                pt.velocities = [float(x) for x in vel]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        gh = self.wait_future(self.action_cli.send_goal_async(goal))
        if gh is None or not gh.accepted:
            self.get_logger().error('Kontrolcü hedefi reddetti.')
            return False
        res = self.wait_future(gh.get_result_async(), timeout=120.0)
        ok = res is not None and res.result.error_code == 0
        self.get_logger().info(
            'Hareket tamamlandı.' if ok
            else f'Hareket hatası: {res.result.error_string if res else "timeout"}')
        return ok

    # ---------- hareket primitifleri ----------
    def move_joints(self, target):
        """Eklem uzayında hedef eklem sözlüğüne git."""
        cur = self.current_joints()
        dq = max(abs(target[j] - cur[j]) for j in ARM_JOINTS)
        t = max(dq / VMAX, 1.0)
        return self.execute(ARM_JOINTS, [([target[j] for j in ARM_JOINTS], t)])

    def move_line(self, xyz, q):
        """Mevcut konumdan xyz'ye düz çizgi (Cartesian)."""
        cur = self.current_joints()
        res = self.cartesian(cur, self.make_pose(xyz, q))
        if not res or res.fraction < 0.99 or not res.solution.joint_trajectory.points:
            self.get_logger().error(
                f'Düz hareket başarısız (%{(res.fraction * 100 if res else 0):.0f})')
            return False
        jt = res.solution.joint_trajectory
        timed = self.retime(jt.joint_names, [p.positions for p in jt.points])
        return self.execute(list(jt.joint_names), timed)

    def run_cartesian(self, res):
        jt = res.solution.joint_trajectory
        timed = self.retime(jt.joint_names, [p.positions for p in jt.points])
        return self.execute(list(jt.joint_names), timed)

    def go_home(self):
        cur = self.current_joints()
        if max(abs(self.home[j] - cur[j]) for j in ARM_JOINTS) < 0.01:
            return True
        self.get_logger().info('Home pozisyonuna dönülüyor...')
        return self.move_joints(self.home)

    # ---------- planlama ----------
    def plan_grasp(self, cube, current):
        """Küp için çalışan bir yönelim: (tilt, roll, q, ready, res) ya da None."""
        grasp_xyz = (cube[0], cube[1], cube[2] + TOOL_LEN)
        appr_xyz = (cube[0], cube[1], cube[2] + TOOL_LEN + APPROACH_H)
        for (tilt, roll), q in candidate_orientations():
            ready = self.solve_ik(self.make_pose(appr_xyz, q), current)
            if ready is None:
                continue
            res = self.cartesian({**current, **ready}, self.make_pose(grasp_xyz, q))
            if res and res.fraction >= 0.99 and res.solution.joint_trajectory.points:
                return (tilt, roll, q, ready, res)
        return None

    def plan_place(self, place, q_pick, seed):
        """Bırakma noktası için (q, ready, res) ya da None. Önce alma yönelimi denenir."""
        above = (place[0], place[1], place[2] + TOOL_LEN + LIFT_H)
        grasp = (place[0], place[1], place[2] + TOOL_LEN)
        for q in [q_pick] + [c[1] for c in candidate_orientations()]:
            ready = self.solve_ik(self.make_pose(above, q), seed)
            if ready is None:
                continue
            res = self.cartesian({**seed, **ready}, self.make_pose(grasp, q))
            if res and res.fraction >= 0.99 and res.solution.joint_trajectory.points:
                return (q, ready, res)
        return None

    def sample_free(self, extra=()):
        """Sahnedeki küplerden kaçınan konum; yer kalmadıysa bırakılmış küpleri temizler."""
        for _ in range(2):
            avoid = [p[1] for p in self.placed] + list(extra)
            try:
                return sample_position(avoid)
            except RuntimeError:
                self.get_logger().info('Boş yer kalmadı, sahnedeki küpler temizleniyor...')
                self.clear_placed()
        return None

    def plan_cycle(self, current):
        """Alma + bırakma konumlarını rastgele seç ve ikisinin de ulaşılabilir olduğunu doğrula."""
        for attempt in range(1, MAX_TRIES + 1):
            cube = self.sample_free()
            if cube is None:
                return None
            ang = math.degrees(math.atan2(cube[1], cube[0]))
            self.get_logger().info(
                f'[{attempt}/{MAX_TRIES}] Küp adayı: x={cube[0]:.3f}, y={cube[1]:.3f} (açı {ang:.0f}°)')
            pick_plan = self.plan_grasp(cube, current)
            if pick_plan is None:
                self.get_logger().info('Bu küp konumuna ulaşılamıyor, yenisi deneniyor...')
                continue
            _, _, q_pick, ready_pick, _ = pick_plan
            seed = {**current, **ready_pick}
            for pa in range(1, MAX_TRIES + 1):
                place = self.sample_free(extra=[cube])
                if place is None:
                    return None
                pang = math.degrees(math.atan2(place[1], place[0]))
                self.get_logger().info(
                    f'   [{pa}/{MAX_TRIES}] Bırakma adayı: x={place[0]:.3f}, y={place[1]:.3f} (açı {pang:.0f}°)')
                place_plan = self.plan_place(place, q_pick, seed)
                if place_plan is not None:
                    return cube, pick_plan, place, place_plan
                self.get_logger().info('   Bu bırakma noktasına ulaşılamıyor, yenisi deneniyor...')
            self.get_logger().info('Bırakma noktası bulunamadı, yeni küp konumu deneniyor...')
        return None

    # ---------- tek döngü ----------
    def pick_and_place_once(self, cycle):
        current = self.current_joints()
        plan = self.plan_cycle(current)
        if plan is None:
            self.get_logger().error(f'{MAX_TRIES} denemede ulaşılabilir alma/bırakma konumu bulunamadı. '
                                    'ANGLE_RANGE_DEG / MIN_RADIUS / MAX_RADIUS aralıklarını daralt.')
            return False
        cube, pick_plan, place, place_plan = plan
        tilt, roll, q, ready, res = pick_plan
        q_pl, ready_pl, res_pl = place_plan
        self.get_logger().info(f'Küp: ({cube[0]:.3f}, {cube[1]:.3f}) -> Bırakma: ({place[0]:.3f}, {place[1]:.3f}), '
                               f'yönelim eğim={tilt}°, dönüş={roll}°')

        # yeni küpü spawn et
        name = f'cube_{self.run_id}_{cycle}'
        self.get_logger().info(f'Yeni küp spawn ediliyor: {name}')
        if not self.spawn_cube(name, cube):
            return False
        self.cube_name = name
        self.cube_pos = list(cube)
        self.cube_active = True
        self.attached = False
        self.flush_spin(5)

        # 1) hazır poza git
        self.get_logger().info('Hazır poza gidiliyor...')
        if not self.move_joints(ready):
            return False

        # 2) düz çizgiyle iniş
        self.get_logger().info('Küpe iniliyor...')
        if not self.run_cartesian(res):
            return False

        # 3) tut (yazılımla bağla)
        self.get_logger().info('Küp tutuldu (yazılımla bağlandı).')
        self.attached = True

        # 4) kaldır
        self.get_logger().info('Küp kaldırılıyor...')
        if not self.move_line((cube[0], cube[1], cube[2] + TOOL_LEN + LIFT_H), q):
            return False

        # 5) bırakma noktasının üstüne git
        self.get_logger().info('Bırakma noktasına gidiliyor...')
        if not self.move_joints(ready_pl):
            return False

        # 6) in
        self.get_logger().info('Bırakma noktasına iniliyor...')
        if not self.run_cartesian(res_pl):
            return False

        # 7) bırak
        self.get_logger().info('Küp bırakılıyor...')
        self.attached = False
        self.cube_pos = list(place)
        self.sync_gz_cube()
        self.flush_spin(10)
        if KEEP_PLACED_CUBES:
            self.placed.append((name, tuple(place)))
        else:
            self.delete_cube(name)
        self.cube_active = False
        self.cube_name = None

        # 8) geri çekil
        self.get_logger().info('Geri çekiliyor...')
        self.move_line((place[0], place[1], place[2] + TOOL_LEN + LIFT_H), q_pl)
        return True

    # ---------- ana akış ----------
    def run(self):
        if RANDOM_SEED is not None:
            random.seed(RANDOM_SEED)

        for cli, nm in ((self.cart_cli, 'compute_cartesian_path'), (self.ik_cli, 'compute_ik')):
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'{nm} yok. move_group çalışıyor mu?')
                return

        for cli, nm in ((self.gz_cli, GZ_SET_STATE_SRV), (self.spawn_cli, GZ_SPAWN_SRV)):
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'{nm} yok. Gazebo ve world_state.world açık mı?')
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

        # home pozisyonu
        if HOME_JOINTS is not None:
            self.home = dict(zip(ARM_JOINTS, HOME_JOINTS))
        else:
            self.home = {j: self.joints[j] for j in ARM_JOINTS}
        self.get_logger().info('Home: ' + ', '.join(f'{self.home[j]:.2f}' for j in ARM_JOINTS))

        # launch'taki başlangıç küpünü kaldır
        if DELETE_LAUNCH_CUBE and self.delete_cube('cube'):
            self.get_logger().info("Launch'taki 'cube' modeli silindi.")

        if not self.go_home():
            return

        cycle = 0
        while NUM_CYCLES == 0 or cycle < NUM_CYCLES:
            cycle += 1
            total = 'sonsuz' if NUM_CYCLES == 0 else str(NUM_CYCLES)
            self.get_logger().info(f'===== Döngü {cycle}/{total} =====')
            if not self.pick_and_place_once(cycle):
                self.get_logger().error('Döngü başarısız, duruyor.')
                break
            if not self.go_home():
                break
            self.get_logger().info(f'Döngü {cycle} tamamlandı.')
        else:
            self.get_logger().info('Tüm döngüler tamamlandı.')


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run()
        rclpy.spin(node)   # marker yayını devam etsin
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()