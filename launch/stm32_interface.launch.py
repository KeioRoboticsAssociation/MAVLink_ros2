from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('stm32_mavlink_interface')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'serial_config.yaml')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for MAVLink communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    
    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='255',
        description='MAVLink system ID for this node'
    )
    
    target_system_id_arg = DeclareLaunchArgument(
        'target_system_id',
        default_value='1',
        description='Target STM32 MAVLink system ID'
    )
    
    # MAVLink serial node
    mavlink_node = Node(
        package='stm32_mavlink_interface',
        executable='mavlink_serial_node',
        name='mavlink_serial_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'system_id': LaunchConfiguration('system_id'),
            'target_system_id': LaunchConfiguration('target_system_id'),
        }],
        remappings=[
            ('diagnostics', '/diagnostics'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        system_id_arg,
        target_system_id_arg,
        mavlink_node,
    ])
