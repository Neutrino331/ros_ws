import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    params = {'respawn': True, 'type': 'serial-imu'}
    imu = Node(
        package='serial-imu',
        executable='serial-imu',
        output='screen',
        parameters=[params]
    )
    return LaunchDescription([
        imu
    ])