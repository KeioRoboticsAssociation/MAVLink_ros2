#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MAVLink Wizard"""

    # Launch arguments
    auto_scan_arg = DeclareLaunchArgument(
        'auto_scan',
        default_value='true',
        description='Automatically scan for devices on startup'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Configuration file to load on startup'
    )

    # Get package directories
    mavlink_wizard_dir = get_package_share_directory('mavlink_wizard')

    # Configuration file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('mavlink_wizard'),
        'config',
        'wizard_config.yaml'
    ])

    # NOTE: MAVLink interface should be launched separately
    # For UART: ros2 launch stm32_mavlink_uart stm32_interface.launch.py serial_port:=<port> baudrate:=115200
    # For UDP:  ros2 launch stm32_mavlink_udp stm32_udp.launch.py
    # The wizard will connect to whichever interface is running and publishing on the standard topics

    # MAVLink Wizard GUI Node
    mavlink_wizard_gui_node = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution([
            FindPackagePrefix('mavlink_wizard'),
            'lib',
            'mavlink_wizard',
            'mavlink_wizard_gui.py'
        ])],
        name='mavlink_wizard_gui',
        output='screen'
    )

    return LaunchDescription([
        auto_scan_arg,
        config_file_arg,
        mavlink_wizard_gui_node,
    ])