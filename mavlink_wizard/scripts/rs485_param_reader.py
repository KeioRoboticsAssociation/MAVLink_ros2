#!/usr/bin/env python3
"""
RS485 Parameter Reader
======================

Example script demonstrating how to read parameters from RS485 motors
using the on-demand read functionality (MAVLink messages 12007/12008).

Usage:
    # Read present velocity from motor 30
    ros2 run mavlink_wizard rs485_param_reader.py --motor-id 30 --address 0x90 --length 4

    # Read all RAM parameters (29 bytes) in one request
    ros2 run mavlink_wizard rs485_param_reader.py --motor-id 30 --address 0x80 --length 29

    # Using the service interface (synchronous)
    ros2 service call /rs485motor/read_param stm32_mavlink_msgs/srv/ReadRS485Param \
        "{motor_id: 30, address: 144, length: 4}"
"""

import rclpy
from rclpy.node import Node
from stm32_mavlink_msgs.msg import RS485ReadRequest, RS485ReadResponse
from stm32_mavlink_msgs.srv import ReadRS485Param
import struct
import argparse


class RS485ParamReader(Node):
    """Node for reading RS485 motor parameters"""

    # Control table addresses
    ADDRESSES = {
        # Global parameters
        'model_number': (0x00, 2, '<H'),
        'firmware_version': (0x02, 1, 'B'),
        'device_id': (0x03, 1, 'B'),
        'baud_rate': (0x04, 1, 'B'),

        # EEPROM parameters
        'operating_mode': (0x20, 1, 'B'),
        'motor_direction': (0x21, 1, 'B'),
        'vel_pid_kp': (0x24, 4, '<f'),
        'vel_pid_ki': (0x28, 4, '<f'),
        'vel_pid_kd': (0x2C, 4, '<f'),
        'pos_pid_kp': (0x34, 4, '<f'),
        'pos_pid_ki': (0x38, 4, '<f'),
        'pos_pid_kd': (0x3C, 4, '<f'),
        'max_pwm_duty': (0x44, 4, '<f'),
        'max_velocity': (0x48, 4, '<f'),

        # RAM parameters
        'torque_enable': (0x80, 1, 'B'),
        'goal_velocity': (0x84, 4, '<f'),
        'goal_position': (0x88, 4, '<i'),
        'goal_duty': (0x8C, 4, '<f'),
        'present_velocity': (0x90, 4, '<f'),
        'present_position': (0x94, 4, '<i'),
        'present_duty': (0x98, 4, '<f'),
        'hw_error_status': (0x9C, 1, 'B'),
    }

    def __init__(self):
        super().__init__('rs485_param_reader')

        # Publishers and subscribers for message interface
        self.request_pub = self.create_publisher(
            RS485ReadRequest, '/rs485motor/read_request', 10)
        self.response_sub = self.create_subscription(
            RS485ReadResponse, '/rs485motor/read_response',
            self.response_callback, 10)

        # Service client for synchronous interface
        self.read_client = self.create_client(
            ReadRS485Param, '/rs485motor/read_param')

        self.pending_response = None
        self.get_logger().info('RS485 Parameter Reader initialized')

    def response_callback(self, msg):
        """Handle incoming read responses (async interface)"""
        self.pending_response = msg
        self.print_response(msg)

    def print_response(self, msg):
        """Print formatted response"""
        self.get_logger().info(
            f'Response from motor {msg.motor_id}, address 0x{msg.address:04X}:')

        if msg.status == 0:
            # Success - print data
            data_hex = ' '.join([f'{b:02X}' for b in msg.data[:msg.length]])
            self.get_logger().info(f'  Data ({msg.length} bytes): {data_hex}')

            # Try to interpret based on known addresses
            for name, (addr, length, fmt) in self.ADDRESSES.items():
                if addr == msg.address and length == msg.length:
                    try:
                        value = struct.unpack(fmt, bytes(msg.data[:msg.length]))[0]
                        self.get_logger().info(f'  Interpreted as {name}: {value}')
                    except:
                        pass
        else:
            self.get_logger().error(
                f'  Read failed: status={msg.status}, rs485_error={msg.rs485_error}')

    async def read_param_async(self, motor_id, address, length):
        """Read parameter using topic interface (async)"""
        req = RS485ReadRequest()
        req.motor_id = motor_id
        req.address = address
        req.length = length

        self.pending_response = None
        self.request_pub.publish(req)
        self.get_logger().info(
            f'Sent read request: motor_id={motor_id}, address=0x{address:04X}, length={length}')

        # Wait for response (timeout 1 second)
        timeout = 1.0
        elapsed = 0.0
        rate = self.create_rate(10)  # 10 Hz

        while elapsed < timeout and self.pending_response is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1

        if self.pending_response is None:
            self.get_logger().error('Timeout waiting for response')
            return None

        return self.pending_response

    def read_param_sync(self, motor_id, address, length):
        """Read parameter using service interface (sync)"""
        if not self.read_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /rs485motor/read_param not available')
            return None

        request = ReadRS485Param.Request()
        request.motor_id = motor_id
        request.address = address
        request.length = length

        future = self.read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Read successful: {response.message}')
                # Create fake RS485ReadResponse for print_response
                msg = RS485ReadResponse()
                msg.motor_id = motor_id
                msg.address = address
                msg.length = length
                msg.data = response.data
                msg.status = response.status
                msg.rs485_error = response.rs485_error
                self.print_response(msg)
                return response
            else:
                self.get_logger().error(f'Read failed: {response.message}')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None

    def read_named_param(self, motor_id, param_name):
        """Read a named parameter"""
        if param_name not in self.ADDRESSES:
            self.get_logger().error(f'Unknown parameter: {param_name}')
            self.get_logger().info('Available parameters:')
            for name in self.ADDRESSES.keys():
                self.get_logger().info(f'  - {name}')
            return None

        address, length, _ = self.ADDRESSES[param_name]
        return self.read_param_sync(motor_id, address, length)


def main():
    parser = argparse.ArgumentParser(
        description='Read parameters from RS485 motors')
    parser.add_argument('--motor-id', type=int, required=True,
                       help='Motor ID (30-49)')
    parser.add_argument('--address', type=lambda x: int(x, 0),
                       help='Control table address (e.g., 0x90)')
    parser.add_argument('--length', type=int,
                       help='Number of bytes to read (1-64)')
    parser.add_argument('--param', type=str,
                       help='Named parameter to read (e.g., present_velocity)')
    parser.add_argument('--method', choices=['sync', 'async'], default='sync',
                       help='Read method: sync (service) or async (topic)')
    parser.add_argument('--list-params', action='store_true',
                       help='List available named parameters')

    args = parser.parse_args()

    rclpy.init()
    node = RS485ParamReader()

    if args.list_params:
        print('\nAvailable named parameters:')
        print('=' * 60)
        for name, (addr, length, _) in sorted(node.ADDRESSES.items()):
            print(f'  {name:25s}  0x{addr:04X}  {length:2d} bytes')
        print('=' * 60)
        rclpy.shutdown()
        return

    try:
        if args.param:
            # Read named parameter
            node.read_named_param(args.motor_id, args.param)
        elif args.address is not None and args.length is not None:
            # Read raw address
            if args.method == 'sync':
                node.read_param_sync(args.motor_id, args.address, args.length)
            else:
                import asyncio
                asyncio.run(node.read_param_async(args.motor_id, args.address, args.length))
        else:
            parser.print_help()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
