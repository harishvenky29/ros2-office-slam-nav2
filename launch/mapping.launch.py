"""Map the office with the LIDAR using SLAM Toolbox.

Manual mapping (drive with the keyboard):
    ros2 launch office_nav mapping.launch.py
    ros2 run turtlebot3_teleop teleop_keyboard          # second terminal

Autonomous mapping (Nav2 + frontier explorer drive the robot, map is auto-saved):
    ros2 launch office_nav mapping.launch.py explore:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_office = get_package_share_directory('office_nav')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    explore = LaunchConfiguration('explore')
    rviz = LaunchConfiguration('rviz')
    map_save_path = LaunchConfiguration('map_save_path')

    slam_params = os.path.join(pkg_office, 'config', 'slam_toolbox_office.yaml')
    nav2_params = os.path.join(pkg_office, 'config', 'nav2_params.yaml')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_office, 'launch', 'office_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'gui': LaunchConfiguration('gui')}.items())

    # SLAM Toolbox is a lifecycle node on Jazzy: configure, then activate.
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time,
                                  'use_lifecycle_manager': False}])
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[LogInfo(msg='[office_nav] activating SLAM Toolbox'),
                  EmitEvent(event=ChangeState(
                      lifecycle_node_matcher=matches_action(slam),
                      transition_id=Transition.TRANSITION_ACTIVATE))]))

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    # Autonomous exploration: Nav2 (no AMCL / map_server -- SLAM provides map->odom)
    exploration = GroupAction(condition=IfCondition(explore), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params,
                              'autostart': 'true'}.items()),
        # Give Gazebo, SLAM and Nav2 time to come up before exploring.
        TimerAction(period=15.0, actions=[Node(
            package='office_nav', executable='frontier_explorer', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'map_save_path': map_save_path}])]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false',
                              description='true = explore autonomously with Nav2 + frontier_explorer'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('map_save_path',
                              default_value=os.path.expanduser('~/office_nav_maps/office_map'),
                              description='Where frontier_explorer saves the finished map (no extension)'),
        world, slam, slam_configure, slam_activate, rviz_node, exploration,
    ])
