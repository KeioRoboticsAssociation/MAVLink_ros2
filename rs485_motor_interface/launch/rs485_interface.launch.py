#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('rs485_motor_interface').find('rs485_motor_interface')

    # Default config file path
    default_config_file = PathJoinSubstitution([
        FindPackageShare('rs485_motor_interface'),
        'config',
        'motor_config.yaml'
    ])

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to motor configuration YAML file'
    )

    polling_rate_arg = DeclareLaunchArgument(
        'polling_rate_hz',
        default_value='10.0',
        description='Motor state polling rate in Hz'
    )

    # RS485 Motor Interface Node
    rs485_motor_node = Node(
        package='rs485_motor_interface',
        executable='rs485_motor_node',
        name='rs485_motor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'polling_rate_hz': LaunchConfiguration('polling_rate_hz'),
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        polling_rate_arg,
        rs485_motor_node,
    ])
