"""Gazebo Harmonic: small office world + TurtleBot3 (Burger by default) with its LIDAR.

    ros2 launch office_nav office_world.launch.py
    ros2 launch office_nav office_world.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # TurtleBot3's own launch files read this env var; default to the Burger (LDS-01 LIDAR).
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    pkg_office = get_package_share_directory('office_nav')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_gz = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg_office, 'worlds', 'small_office.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Spawn at the origin so the SLAM `map` frame lines up with the world frame.
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_tb3, 'models')),

        # Gazebo server (physics + sensors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v2 ', world],
                              'on_exit_shutdown': 'true'}.items()),
        # Gazebo GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        # Spawns the robot and bridges /scan, /odom, /tf, /cmd_vel, /clock to ROS 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])
