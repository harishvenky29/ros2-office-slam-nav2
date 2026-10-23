"""Office patrol: visit named waypoints on the saved map with Nav2, loop, and log a report.

    ros2 run office_nav waypoint_patrol --ros-args \
        -p use_sim_time:=true \
        -p waypoints_file:=$(ros2 pkg prefix office_nav)/share/office_nav/config/office_waypoints.yaml \
        -p loops:=2

Parameters
  waypoints_file  YAML with `initial_pose` and a `waypoints` list (see config/office_waypoints.yaml)
  loops           number of patrol loops, 0 = forever                       (default 1)
  dwell_time      seconds to pause at each waypoint ("inspection")         (default 2.0)
  goal_timeout    seconds before a waypoint is abandoned and skipped        (default 90.0)
  set_initial_pose  publish the initial pose to AMCL before starting       (default true)
  report_file     CSV report of every waypoint attempt  (default ~/office_nav_maps/patrol_report.csv)
"""
import csv
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def load_route(path):
    with open(path) as f:
        route = yaml.safe_load(f)
    if not route or not route.get('waypoints'):
        raise ValueError(f'{path} has no waypoints')
    return route.get('initial_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}), route['waypoints']


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator(node_name='waypoint_patrol')
    nav.declare_parameter('waypoints_file', '')
    nav.declare_parameter('loops', 1)
    nav.declare_parameter('dwell_time', 2.0)
    nav.declare_parameter('goal_timeout', 90.0)
    nav.declare_parameter('set_initial_pose', True)
    nav.declare_parameter('report_file', os.path.expanduser('~/office_nav_maps/patrol_report.csv'))

    path = nav.get_parameter('waypoints_file').value
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('office_nav'), 'config', 'office_waypoints.yaml')
    loops = int(nav.get_parameter('loops').value)
    dwell = float(nav.get_parameter('dwell_time').value)
    timeout = float(nav.get_parameter('goal_timeout').value)
    report_file = os.path.expanduser(nav.get_parameter('report_file').value)
    initial, waypoints = load_route(path)
    log = nav.get_logger()

    if nav.get_parameter('set_initial_pose').value:
        nav.setInitialPose(make_pose(nav, initial['x'], initial['y'], initial.get('yaw', 0.0)))
    log.info('Waiting for Nav2 (AMCL + navigation) to become active...')
    nav.waitUntilNav2Active()
    log.info(f'Patrolling {len(waypoints)} waypoints from {path}, '
             f'{"forever" if loops == 0 else f"{loops} loop(s)"}.')

    os.makedirs(os.path.dirname(report_file) or '.', exist_ok=True)
    rows, loop = [], 0
    try:
        while rclpy.ok() and (loops == 0 or loop < loops):
            loop += 1
            for wp in waypoints:
                nav.goToPose(make_pose(nav, wp['x'], wp['y'], wp.get('yaw', 0.0)))
                started = time.monotonic()
                while not nav.isTaskComplete():
                    fb = nav.getFeedback()
                    if fb and Duration.from_msg(fb.navigation_time) > Duration(seconds=timeout):
                        log.warn(f'{wp["name"]}: timed out after {timeout:.0f}s, skipping')
                        nav.cancelTask()
                    time.sleep(0.2)
                took = time.monotonic() - started
                result = nav.getResult()
                outcome = {TaskResult.SUCCEEDED: 'reached', TaskResult.CANCELED: 'skipped',
                           TaskResult.FAILED: 'failed'}.get(result, str(result))
                log.info(f'[loop {loop}] {wp["name"]:<15} {outcome:<8} in {took:5.1f}s')
                rows.append({'loop': loop, 'waypoint': wp['name'], 'x': wp['x'], 'y': wp['y'],
                             'result': outcome, 'seconds': round(took, 1)})
                if result == TaskResult.SUCCEEDED and dwell > 0:
                    time.sleep(dwell)
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        if rows:
            with open(report_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            ok = sum(r['result'] == 'reached' for r in rows)
            log.info(f'Patrol done: {ok}/{len(rows)} waypoints reached '
                     f'({100.0 * ok / len(rows):.0f}%). Report: {report_file}')
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
