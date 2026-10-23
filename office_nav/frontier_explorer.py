"""Autonomous LIDAR mapping: drive to map frontiers with Nav2 until the office is fully mapped.

Start-up: spin 360 deg on the spot (Nav2 `spin` behaviour) so SLAM gets a full first view.

Loop (1 Hz):
  1. read the latest SLAM map (/map) and the robot pose (TF map -> base_footprint)
  2. find frontiers (free cells bordering unknown space), cluster them, pick the best one
  3. send it to Nav2's `navigate_to_pose` action
  4. unreachable / timed-out goals are blacklisted; goals whose frontier has already been
     seen are pre-empted so the robot does not drive to a spot it has already mapped
  5. if nothing is reachable, spin again and wait for the map to update (up to `max_spins`)
  6. when no frontiers remain: save the map via SLAM Toolbox and (optionally) drive home

Run via:  ros2 launch office_nav mapping.launch.py explore:=true
"""
import math
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from office_nav.frontiers import (choose_frontier, explored_area_m2, find_frontiers,
                                  frontier_mask)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FrontierExplorer(Node):
    IDLE, SPINNING, NAVIGATING, FINISHING, DONE = (
        'IDLE', 'SPINNING', 'NAVIGATING', 'FINISHING', 'DONE')

    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size', 8)         # cells (8 x 5 cm = 0.4 m)
        self.declare_parameter('goal_timeout', 45.0)           # s
        self.declare_parameter('blacklist_radius', 0.4)        # m
        self.declare_parameter('min_goal_distance', 0.35)      # m
        self.declare_parameter('empty_checks_to_finish', 10)   # consecutive 1 s ticks, no frontier
        self.declare_parameter('min_explore_time', 30.0)       # s; never declare 'done' before this
        self.declare_parameter('max_spins', 3)                 # 360 deg spins when stuck
        self.declare_parameter('return_home', True)
        self.declare_parameter('map_save_path', os.path.expanduser('~/office_nav_maps/office_map'))

        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_size = self.get_parameter('min_frontier_size').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.empty_checks_to_finish = self.get_parameter('empty_checks_to_finish').value
        self.min_explore_time = self.get_parameter('min_explore_time').value
        self.max_spins = self.get_parameter('max_spins').value
        self.return_home = self.get_parameter('return_home').value
        self.map_save_path = os.path.expanduser(self.get_parameter('map_save_path').value)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'explore/frontiers', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.save_client = self.create_client(SaveMap, 'slam_toolbox/save_map')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.state = self.IDLE
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_at = None
        self.preempted = False
        self.cancel_requested = False
        self.blacklist = []
        self.empty_checks = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.spins = 0
        self.started_at = time.monotonic()

        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready -- starting frontier exploration.')
        self.timer = self.create_timer(1.0, self._tick)
        self.started_spin = False

    # ------------------------------------------------------------------ inputs
    def _on_map(self, msg):
        self.map = msg

    def _grid(self):
        info = self.map.info
        return np.array(self.map.data, dtype=np.int8).reshape(info.height, info.width)

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map.header.frame_id, self.robot_frame, Time(),
                                                timeout=Duration(seconds=0.2))
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f'No robot pose yet: {e}', throttle_duration_sec=5.0)
            return None

    # -------------------------------------------------------------------- loop
    def _tick(self):
        if self.state in (self.FINISHING, self.DONE, self.SPINNING) or self.map is None:
            return
        pose = self._robot_xy()
        if pose is None:
            return
        if not self.started_spin:
            self.started_spin = True
            self._spin('Initial 360 deg spin so the LIDAR sees the whole room')
            return
        info = self.map.info
        grid = self._grid()
        frontiers = find_frontiers(grid, info.resolution, info.origin.position.x,
                                   info.origin.position.y, min_size=self.min_size)
        self._publish_markers(frontiers)

        if self.state == self.NAVIGATING:
            if self.cancel_requested:
                return  # waiting for Nav2 to confirm the cancel
            elapsed = time.monotonic() - self.goal_sent_at
            if elapsed > self.goal_timeout:
                self.get_logger().warn(f'Goal timed out after {elapsed:.0f}s -- blacklisting it.')
                self.blacklist.append(self.goal_xy)
                self._cancel()
            elif elapsed > 3.0 and not self._frontier_near(grid, *self.goal_xy, radius=0.5):
                self.get_logger().info('Frontier already mapped on the way -- picking the next one.')
                self.preempted = True
                self._cancel()
            return

        target = choose_frontier(frontiers, *pose, blacklist=self.blacklist,
                                 blacklist_radius=self.blacklist_radius,
                                 min_distance=self.min_goal_distance)
        if target is None:
            self.empty_checks += 1
            area = explored_area_m2(grid, info.resolution)
            self.get_logger().info(
                f'No reachable frontier yet ({len(frontiers)} found, {area:.1f} m^2 mapped) '
                f'-- check {self.empty_checks}/{self.empty_checks_to_finish}',
                throttle_duration_sec=3.0)
            if self.empty_checks >= 3 and self.spins < self.max_spins:
                self.empty_checks = 0
                self._spin('Spinning to refresh the map')
            elif (self.empty_checks >= self.empty_checks_to_finish
                  and time.monotonic() - self.started_at >= self.min_explore_time):
                self._finish()
            return
        self.empty_checks = 0
        self.get_logger().info(
            f'Frontier -> ({target.x:.2f}, {target.y:.2f}), {target.length_m:.2f} m long, '
            f'{len(frontiers)} frontiers left')
        self._send_goal(target.x, target.y)

    def _frontier_near(self, grid, x, y, radius):
        """True if any frontier cell lies within `radius` m of (x, y)."""
        info = self.map.info
        res = info.resolution
        c = int((x - info.origin.position.x) / res)
        r = int((y - info.origin.position.y) / res)
        k = int(radius / res)
        r0, r1 = max(r - k, 0), min(r + k + 1, grid.shape[0])
        c0, c1 = max(c - k, 0), min(c + k + 1, grid.shape[1])
        if r0 >= r1 or c0 >= c1:
            return False
        return bool(frontier_mask(grid[r0:r1, c0:c1]).any())

    def _spin(self, why):
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 spin behaviour not available -- skipping spin.')
            self.spins = self.max_spins
            return
        self.spins += 1
        self.state = self.SPINNING
        self.get_logger().info(f'{why} (spin {self.spins}/{self.max_spins}).')
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = Duration(seconds=30.0).to_msg()
        self.spin_client.send_goal_async(goal).add_done_callback(self._on_spin_response)

    def _on_spin_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Spin rejected.')
            self.state = self.IDLE
            return
        handle.get_result_async().add_done_callback(lambda _: self._spin_done())

    def _spin_done(self):
        self.empty_checks = 0
        if self.state == self.SPINNING:
            self.state = self.IDLE

    # -------------------------------------------------------------- navigation
    def _send_goal(self, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        self.goal_xy = (x, y)
        self.goal_sent_at = time.monotonic()
        self.preempted = False
        self.cancel_requested = False
        if self.state != self.FINISHING:
            self.state = self.NAVIGATING
        self.nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal -- blacklisting it.')
            self.blacklist.append(self.goal_xy)
            self._goal_over()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            # Don't come back to the same spot if a sliver of frontier survives there.
            self.blacklist.append(self.goal_xy)
        elif not self.preempted:
            self.goals_failed += 1
            if self.goal_xy not in self.blacklist:
                self.blacklist.append(self.goal_xy)
            self.get_logger().warn(f'Goal ended with status {status} -- blacklisted.')
        self._goal_over()

    def _goal_over(self):
        self.goal_handle = None
        if self.state == self.FINISHING:
            self.state = self.DONE
            self.get_logger().info('Back at the start pose. Exploration complete.')
        else:
            self.state = self.IDLE

    def _cancel(self):
        if self.goal_handle is not None:
            self.cancel_requested = True
            self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------ finish
    def _finish(self):
        self.state = self.FINISHING
        minutes = (time.monotonic() - self.started_at) / 60.0
        area = explored_area_m2(self._grid(), self.map.info.resolution)
        self.get_logger().info(
            f'No frontiers left: office mapped in {minutes:.1f} min, {area:.1f} m^2 of free space, '
            f'{self.goals_reached} goals reached, {self.goals_failed} failed.')
        self._save_map()
        if self.return_home:
            self.get_logger().info('Returning to the start pose (0, 0).')
            self._send_goal(0.0, 0.0)
        else:
            self.state = self.DONE

    def _save_map(self):
        os.makedirs(os.path.dirname(self.map_save_path) or '.', exist_ok=True)
        if not self.save_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('slam_toolbox/save_map not available. Save it manually:\n'
                                    f'  ros2 run nav2_map_server map_saver_cli -f {self.map_save_path}')
            return
        req = SaveMap.Request()
        req.name = String(data=self.map_save_path)
        self.save_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'Map saved to {self.map_save_path}.yaml/.pgm' if f.result().result == 0
                else f'Map save failed (code {f.result().result})'))

    # ------------------------------------------------------------------- RViz
    def _publish_markers(self, frontiers):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map.header.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'frontiers', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y = f.x, f.y
            m.pose.orientation.w = 1.0
            s = min(0.15 + 0.05 * f.length_m, 0.5)
            m.scale.x = m.scale.y = m.scale.z = s
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.0, 0.9
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
