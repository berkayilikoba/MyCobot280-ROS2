"""Autonomous patrol node wrapping Nav2 BasicNavigator and MissionPlanner."""

import math
import sys
from typing import List, Optional

from geometry_msgs.msg import PoseStamped, Quaternion
from limo_msgs.msg import LimoStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.signals import SignalHandlerOptions

from limo_autonomy.mission import MissionPlanner, Waypoint, load_mission


def yaw_deg_to_quaternion(yaw_deg: float) -> Quaternion:
    """Convert yaw angle in degrees to a geometry_msgs Quaternion."""
    rad = math.radians(yaw_deg)
    half = rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def create_pose_stamped(node: BasicNavigator, wp: Waypoint) -> PoseStamped:
    """Create a PoseStamped message in the 'map' frame from a Waypoint."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(wp.x)
    pose.pose.position.y = float(wp.y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_deg_to_quaternion(wp.yaw_deg)
    return pose


class PatrolNode(BasicNavigator):
    """ROS 2 wrapper node executing waypoint patrol missions."""

    def __init__(self, node_name: str = 'patrol') -> None:
        """Initialize the patrol node and declare ROS parameters."""
        super().__init__(node_name=node_name)

        self.declare_parameter('mission_file', '')
        self.declare_parameter('goal_timeout_s', 180.0)
        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('localizer', 'amcl')
        self.declare_parameter('min_battery_voltage', 0.0)

        self.planner: Optional[MissionPlanner] = None
        self._battery_low_warned = False
        self._pending_battery_low = False

        self._battery_sub = self.create_subscription(
            LimoStatus,
            '/limo_status',
            self._battery_callback,
            10,
        )

    def _battery_callback(self, msg: LimoStatus) -> None:
        """Monitor battery voltage and request return home once if below threshold."""
        min_voltage = float(self.get_parameter('min_battery_voltage').value)
        if min_voltage > 0.0 and msg.battery_voltage < min_voltage and not self._battery_low_warned:
            self._battery_low_warned = True
            self.get_logger().warn(
                f"Battery voltage ({msg.battery_voltage:.2f}V) below threshold ({min_voltage:.2f}V). "
                "Requesting return home."
            )
            if self.planner is not None:
                self.planner.request_return_home('battery_low')
            else:
                # Görev henüz yüklenmediyse isteği kaybetme; run() planlayıcıyı kurunca uygular.
                self._pending_battery_low = True

    def run(self) -> None:
        """Execute the patrol mission loop to completion."""
        mission_file = str(self.get_parameter('mission_file').value)
        if not mission_file:
            self.get_logger().error("Parameter 'mission_file' is required.")
            return

        try:
            cfg = load_mission(mission_file)
            self.planner = MissionPlanner(**cfg)
        except Exception as err:
            self.get_logger().error(f"Failed to load mission file '{mission_file}': {err}")
            return
        if self._pending_battery_low:
            self.planner.request_return_home('battery_low')

        set_init_pose = bool(self.get_parameter('set_initial_pose').value)
        if set_init_pose:
            init_x = float(self.get_parameter('initial_x').value)
            init_y = float(self.get_parameter('initial_y').value)
            init_yaw = float(self.get_parameter('initial_yaw_deg').value)
            initial_wp = Waypoint('initial_pose', init_x, init_y, init_yaw)
            self.get_logger().info(f"Setting initial pose: x={init_x}, y={init_y}, yaw={init_yaw}°")
            self.setInitialPose(create_pose_stamped(self, initial_wp))

        localizer = str(self.get_parameter('localizer').value)
        self.get_logger().info(f"Waiting until Nav2 is active with localizer '{localizer}'...")
        if localizer in ('amcl', 'robot_localization'):
            self.waitUntilNav2Active(localizer=localizer)
        else:
            # Humble slam_toolbox (2.6.x) lifecycle düğümü değil: get_state servisi yok,
            # waitUntilNav2Active sonsuza kadar bekler. Yalnızca bt_navigator'ı bekle.
            self._waitForNodeToActivate('bt_navigator')

        goal_timeout_s = float(self.get_parameter('goal_timeout_s').value)

        action = self.planner.start()
        kind, wp = action

        while kind != 'done':
            if wp is None:
                break

            goal_pose = create_pose_stamped(self, wp)
            start_time = self.get_clock().now()

            if not self.goToPose(goal_pose):
                outcome = 'failed'
                elapsed = 0.0
            else:
                timed_out = False
                while not self.isTaskComplete():
                    now = self.get_clock().now()
                    elapsed = max(0.0, (now - start_time).nanoseconds / 1e9)
                    if goal_timeout_s > 0.0 and elapsed > goal_timeout_s:
                        self.get_logger().warn(
                            f"Goal '{wp.name}' exceeded timeout of {goal_timeout_s:.1f}s. Canceling task."
                        )
                        self.cancelTask()
                        timed_out = True
                        break

                elapsed = max(0.0, (self.get_clock().now() - start_time).nanoseconds / 1e9)
                if timed_out:
                    outcome = 'timeout'
                else:
                    task_res = self.getResult()
                    if task_res == TaskResult.SUCCEEDED:
                        outcome = 'succeeded'
                    elif task_res == TaskResult.CANCELED:
                        outcome = 'canceled'
                    else:
                        outcome = 'failed'

            self.get_logger().info(f"Waypoint '{wp.name}': outcome={outcome}, elapsed={elapsed:.2f}s")
            action = self.planner.report(outcome)
            kind, wp = action

        self.get_logger().info(f"Patrol mission finished. Stats: {self.planner.stats()}")


def main(args: Optional[List[str]] = None) -> None:
    """Main entry point for patrol executable."""
    # rclpy'nin kendi SIGINT işleyicisi bağlamı hemen kapatıyor (ölçüldü: sonrasında rclpy.ok() False);
    # o zaman aktif Nav2 hedefi iptal edilemez. Python'un varsayılan işleyicisi KeyboardInterrupt atar
    # ve bağlam açık kalır.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PatrolNode()
    completed = False
    try:
        node.run()
        completed = True
    except KeyboardInterrupt:
        node.get_logger().info("Patrol interrupted by user.")
    finally:
        if not completed:
            # Düğüm yarıda kalırsa aktif Nav2 hedefi sürmesin (robot son hedefe gitmeye devam ederdi).
            # SIGINT bir C uzantısının içine denk gelince KeyboardInterrupt yerine TypeError olarak
            # yükselebiliyor; bu yüzden istisna türünden bağımsız, her yarım kalışta iptal edilir.
            try:
                if rclpy.ok():
                    node.cancelTask()
            except BaseException as err:  # iptal en iyi çaba; bağlam kapanmış olabilir
                print(f"patrol: cancelTask failed: {err!r}", file=sys.stderr)
        try:
            node.destroy_node()
        except BaseException:
            pass  # launch ikinci SIGINT gönderebilir
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)
