#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'local_host',
            default_value='0.0.0.0',
            description='Local IP address to bind to (0.0.0.0 for all interfaces)'
        ),
        DeclareLaunchArgument(
            'local_port',
            default_value='14550',
            description='Local UDP port to bind to'
        ),
        DeclareLaunchArgument(
            'remote_host',
            default_value='192.168.1.100',
            description='Remote IP address to send to'
        ),
        DeclareLaunchArgument(
            'remote_port',
            default_value='14551',
            description='Remote UDP port to send to'
        ),
        DeclareLaunchArgument(
            'is_server_mode',
            default_value='true',
            description='Server mode: auto-detect remote address from incoming packets'
        ),
        DeclareLaunchArgument(
            'system_id',
            default_value='255',
            description='MAVLink system ID'
        ),
        DeclareLaunchArgument(
            'component_id',
            default_value='1',
            description='MAVLink component ID'
        ),
        DeclareLaunchArgument(
            'target_system_id',
            default_value='1',
            description='Target MAVLink system ID'
        ),
        DeclareLaunchArgument(
            'target_component_id',
            default_value='1',
            description='Target MAVLink component ID'
        ),

        # Launch the UDP MAVLink node
        Node(
            package='stm32_mavlink_udp',
            executable='mavlink_udp_node',
            name='mavlink_udp_node',
            output='screen',
            parameters=[{
                'local_host': LaunchConfiguration('local_host'),
                'local_port': LaunchConfiguration('local_port'),
                'remote_host': LaunchConfiguration('remote_host'),
                'remote_port': LaunchConfiguration('remote_port'),
                'is_server_mode': LaunchConfiguration('is_server_mode'),
                'system_id': LaunchConfiguration('system_id'),
                'component_id': LaunchConfiguration('component_id'),
                'target_system_id': LaunchConfiguration('target_system_id'),
                'target_component_id': LaunchConfiguration('target_component_id'),
            }],
            remappings=[
                # Remap topics to match serial interface
                ('/servo/command', '/servo/command'),
                ('/servo/states', '/servo/states'),
                ('/encoder/states', '/encoder/states'),
                ('/robomaster/motor_command', '/robomaster/motor_command'),
                ('/robomaster/motor_state', '/robomaster/motor_state'),
                ('/dcmotor/command', '/dcmotor/command'),
                ('/dcmotor/state', '/dcmotor/state'),
            ]
        )
    ])