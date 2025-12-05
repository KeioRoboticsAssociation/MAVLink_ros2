#!/usr/bin/env python3
"""
RS485 Parameter Writer
======================

Example script demonstrating how to write parameters and save to flash
using RS485 motors (MAVLink messages 12009/12010/12011).

Usage:
    # Write PID gain
    ros2 run mavlink_wizard rs485_param_writer.py --motor-id 30 --param vel_pid_kp --value 2.5

    # Write operating mode
    ros2 run mavlink_wizard rs485_param_writer.py --motor-id 30 --param operating_mode --value 1

    # Flash save
    ros2 run mavlink_wizard rs485_param_writer.py --motor-id 30 --flash-save
"""

import rclpy
from rclcpp.node import Node
from stm32_mavlink_msgs.msg import RS485WriteRequest, RS485WriteResponse, RS485FlashSaveRequest
from stm32_mavlink_msgs.srv import WriteRS485Param, FlashSaveRS485
import struct
import argparse


class RS485ParamWriter(Node):
    """Node for writing RS485 motor parameters"""

    # Control table addresses with types
    WRITE_PARAMS = {
        # EEPROM parameters (need flash_save)
        'operating_mode': (0x20, 1, 'B'),  # uint8: 0=vel, 1=pos, 2=duty
        'motor_direction': (0x21, 1, 'B'),  # uint8: 0=normal, 1=reverse
        'vel_pid_kp': (0x24, 4, '<f'),  # float
        'vel_pid_ki': (0x28, 4, '<f'),  # float
        'vel_pid_kd': (0x2C, 4, '<f'),  # float
        'vel_pid_mode': (0x30, 1, 'B'),  # uint8
        'pos_pid_kp': (0x34, 4, '<f'),  # float
        'pos_pid_ki': (0x38, 4, '<f'),  # float
        'pos_pid_kd': (0x3C, 4, '<f'),  # float
        'pos_pid_mode': (0x40, 1, 'B'),  # uint8
        'max_pwm_duty': (0x44, 4, '<f'),  # float
        'max_velocity': (0x48, 4, '<f'),  # float
        'max_acceleration': (0x4C, 4, '<f'),  # float
        'max_position': (0x50, 4, '<i'),  # int32
        'min_position': (0x54, 4, '<i'),  # int32
        'encoder_resolution': (0x58, 2, '<H'),  # uint16

        # RAM parameters (no flash_save needed)
        'torque_enable': (0x80, 1, 'B'),  # uint8: 0=disable, 1=enable
        'goal_velocity': (0x84, 4, '<f'),  # float (RPS)
        'goal_position': (0x88, 4, '<i'),  # int32 (counts)
        'goal_duty': (0x8C, 4, '<f'),  # float (-1.0 to 1.0)
    }

    def __init__(self):
        super().__init__('rs485_param_writer')

        # Publishers
        self.write_request_pub = self.create_publisher(
            RS485WriteRequest, '/rs485motor/write_request', 10)
        self.flash_save_request_pub = self.create_publisher(
            RS485FlashSaveRequest, '/rs485motor/flash_save_request', 10)

        # Subscribers
        self.write_response_sub = self.create_subscription(
            RS485WriteResponse, '/rs485motor/write_response',
            self.write_response_callback, 10)

        # Service clients
        self.write_client = self.create_client(
            WriteRS485Param, '/rs485motor/write_param')
        self.flash_save_client = self.create_client(
            FlashSaveRS485, '/rs485motor/flash_save')

        self.pending_write_response = None
        self.get_logger().info('RS485 Parameter Writer initialized')

    def write_response_callback(self, msg):
        """Handle incoming write responses (async interface)"""
        self.pending_write_response = msg
        self.print_write_response(msg)

    def print_write_response(self, msg):
        """Print formatted write response"""
        self.get_logger().info(
            f'Write Response from motor {msg.motor_id}, address 0x{msg.address:04X}:')

        if msg.status == 0:
            self.get_logger().info(f'  ✓ Successfully wrote {msg.length} bytes')
        else:
            self.get_logger().error(
                f'  ✗ Write failed: status={msg.status}, rs485_error={msg.rs485_error}')

    def write_param_sync(self, motor_id, address, length, data):
        """Write parameter using service interface (sync)"""
        if not self.write_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /rs485motor/write_param not available')
            return None

        request = WriteRS485Param.Request()
        request.motor_id = motor_id
        request.address = address
        request.length = length
        request.data = data

        future = self.write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ {response.message}')
                return response
            else:
                self.get_logger().error(f'✗ {response.message}')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None

    def write_named_param(self, motor_id, param_name, value):
        """Write a named parameter"""
        if param_name not in self.WRITE_PARAMS:
            self.get_logger().error(f'Unknown parameter: {param_name}')
            self.get_logger().info('Available parameters:')
            for name in self.WRITE_PARAMS.keys():
                self.get_logger().info(f'  - {name}')
            return None

        address, length, fmt = self.WRITE_PARAMS[param_name]

        # Pack value based on format
        try:
            data_bytes = struct.pack(fmt, value)
            data = list(data_bytes) + [0] * (64 - len(data_bytes))
        except struct.error as e:
            self.get_logger().error(f'Failed to pack value: {e}')
            return None

        self.get_logger().info(
            f'Writing {param_name} = {value} to motor {motor_id} (addr=0x{address:04X}, len={length})')
        return self.write_param_sync(motor_id, address, length, data)

    def flash_save_sync(self, motor_id):
        """Save configuration to flash using service interface (sync)"""
        if not self.flash_save_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /rs485motor/flash_save not available')
            return None

        request = FlashSaveRS485.Request()
        request.motor_id = motor_id

        future = self.flash_save_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ Configuration saved to flash! {response.message}')
                return response
            else:
                self.get_logger().error(f'✗ Flash save failed: {response.message}')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None


def main():
    parser = argparse.ArgumentParser(
        description='Write parameters to RS485 motors and save to flash')
    parser.add_argument('--motor-id', type=int, required=True,
                       help='Motor ID (30-49)')
    parser.add_argument('--param', type=str,
                       help='Named parameter to write (e.g., vel_pid_kp)')
    parser.add_argument('--value', type=float,
                       help='Value to write')
    parser.add_argument('--flash-save', action='store_true',
                       help='Save configuration to flash')
    parser.add_argument('--list-params', action='store_true',
                       help='List available writable parameters')

    args = parser.parse_args()

    rclpy.init()
    node = RS485ParamWriter()

    if args.list_params:
        print('\nAvailable writable parameters:')
        print('=' * 60)
        print('\nEEPROM (need flash_save to persist):')
        eeprom_params = ['operating_mode', 'motor_direction', 'vel_pid_kp',
                        'vel_pid_ki', 'vel_pid_kd', 'vel_pid_mode',
                        'pos_pid_kp', 'pos_pid_ki', 'pos_pid_kd', 'pos_pid_mode',
                        'max_pwm_duty', 'max_velocity', 'max_acceleration',
                        'max_position', 'min_position', 'encoder_resolution']
        for name in eeprom_params:
            addr, length, _ = node.WRITE_PARAMS[name]
            print(f'  {name:25s}  0x{addr:04X}  {length:2d} bytes')

        print('\nRAM (no flash_save needed):')
        ram_params = ['torque_enable', 'goal_velocity', 'goal_position', 'goal_duty']
        for name in ram_params:
            addr, length, _ = node.WRITE_PARAMS[name]
            print(f'  {name:25s}  0x{addr:04X}  {length:2d} bytes')
        print('=' * 60)
        rclpy.shutdown()
        return

    try:
        if args.flash_save:
            # Flash save only
            node.flash_save_sync(args.motor_id)
        elif args.param and args.value is not None:
            # Write parameter
            result = node.write_named_param(args.motor_id, args.param, args.value)
            if result:
                print(f'\n✓ Successfully wrote {args.param} = {args.value}')
                print('TIP: Use --flash-save to persist changes across power cycles')
        else:
            parser.print_help()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
