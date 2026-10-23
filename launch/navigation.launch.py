"""Navigate the office on a saved map with Nav2 (AMCL localisation), optionally patrolling.

    ros2 launch office_nav navigation.launch.py                   # click "Nav2 Goal" in RViz
    ros2 launch office_nav navigation.launch.py patrol:=true      # run the waypoint patrol
    ros2 launch office_nav navigation.launch.py map:=/path/to/office_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_office = get_package_share_directory('office_nav')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_office, 'launch', 'office_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'gui': LaunchConfiguration('gui')}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_office, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    patrol = TimerAction(period=10.0, actions=[Node(
        package='office_nav', executable='waypoint_patrol', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'waypoints_file': os.path.join(pkg_office, 'config', 'office_waypoints.yaml'),
                     'loops': LaunchConfiguration('loops')}],
        condition=IfCondition(LaunchConfiguration('patrol')))])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/office_nav_maps/office_map.yaml'),
                              description='Saved map from the mapping step'),
        DeclareLaunchArgument('patrol', default_value='false'),
        DeclareLaunchArgument('loops', default_value='1', description='Patrol loops; 0 = forever'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        world, nav2, rviz, patrol,
    ])
