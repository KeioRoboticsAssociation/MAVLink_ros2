# STM32 MAVLink ROS2 System

A comprehensive ROS2 Jazzy workspace for MAVLink-based STM32 device communication, providing both serial (UART) and network (UDP) interfaces with a professional GUI management tool.

## Overview

This system enables bidirectional communication between ROS2 and STM32 microcontrollers using the MAVLink protocol. It supports multiple device types including servos, encoders, RoboMaster motors (CAN-based), and DC motors with advanced control features.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Applications Layer                          │
│  ┌──────────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │ mavlink_wizard   │  │   seiretu    │  │  Custom Apps       │   │
│  │ (GUI Manager)    │  │ (Robot Ctrl) │  │  (Your Code)       │   │
│  └──────────────────┘  └──────────────┘  └────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ▲ ▼
                    ROS2 Topics & Services
                              ▲ ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    Communication Interfaces                         │
│  ┌──────────────────────────────┐  ┌──────────────────────────┐   │
│  │  stm32_mavlink_uart          │  │  stm32_mavlink_udp       │   │
│  │  (Serial/UART Interface)     │  │  (Network/UDP Interface) │   │
│  │  - /dev/ttyUSB*, ttyACM*     │  │  - WiFi/Ethernet         │   │
│  │  - 115200 baud default       │  │  - Port 14550 default    │   │
│  └──────────────────────────────┘  └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ▲ ▼
                         MAVLink Protocol
                              ▲ ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                              │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │   Servos    │  │  DC Motors   │  │  Encoders    │              │
│  │  (16 max)   │  │  (Position,  │  │  (16 max)    │              │
│  │             │  │   Velocity,  │  │              │              │
│  │             │  │   Current)   │  │              │              │
│  └─────────────┘  └──────────────┘  └──────────────┘              │
│                                                                     │
│  ┌─────────────────────────────────────────────────┐              │
│  │         RoboMaster Motors (CAN Bus)             │              │
│  │  GM6020, M3508, etc. (8 motors max)            │              │
│  └─────────────────────────────────────────────────┘              │
│                                                                     │
│                      STM32F446RE                                    │
└─────────────────────────────────────────────────────────────────────┘
```

## Package Structure

### Core Communication Packages

#### 1. **stm32_mavlink_uart** - Serial/UART Interface
- Direct serial communication with STM32 devices via UART
- Default configuration: 115200 baud, 8N1
- Supports USB-to-Serial (ttyUSB*) and USB CDC (ttyACM*) devices
- Real-time telemetry at 10Hz
- See [stm32_mavlink_uart/README.md](stm32_mavlink_uart/README.md) for details

#### 2. **stm32_mavlink_udp** - Network/UDP Interface
- Wireless communication over WiFi/Ethernet
- Server mode (auto-detect remote) and client mode (fixed remote)
- Compatible with ESP32, Raspberry Pi, and network-enabled microcontrollers
- Drop-in replacement for serial interface - same topics/services
- Default port: 14550
- See [stm32_mavlink_udp/README.md](stm32_mavlink_udp/README.md) for details

#### 3. **mavlink_SDK** - Shared MAVLink Headers
- Unified MAVLink C library v2 headers
- Includes standard messages (common, minimal, standard)
- Custom message support (robomaster, robomaster_motor)
- Shared by both UART and UDP interfaces
- **Not built by ROS2** (contains COLCON_IGNORE)

### Application Layer

#### 4. **mavlink_wizard** - Device Management GUI
- PyQt5-based GUI similar to Dynamixel Wizard
- Features:
  - Automatic device discovery
  - Real-time monitoring and plotting
  - Parameter configuration with validation
  - Calibration wizards for servos and encoders
  - Firmware management
- See [mavlink_wizard/README.md](mavlink_wizard/README.md) for details

## Supported Devices

### Servo Motors
- **Count**: Up to 16 servos
- **Control**: Angle-based positioning (-180° to +180°)
- **Features**: Configurable limits, speed control, enable/disable
- **Topics**: `/servo/command`, `/servo/states`

### DC Motors
- **Motor ID**: 10 (dedicated)
- **Control Modes**:
  - Position control (rad)
  - Velocity control (rad/s)
  - Current control (A)
  - Duty-to-position control (duty cycle + target position)
- **Advanced Features**:
  - Cascade PID control (separate speed and position loops)
  - Configurable limits and safety monitoring
  - Real-time temperature and current monitoring
- **Topics**: `/dcmotor/command`, `/dcmotor/state`

### RoboMaster Motors (CAN)
- **Count**: Up to 8 motors
- **Supported Models**: GM6020, M3508, etc.
- **Control Modes**: Current, velocity, position
- **Communication**: CAN bus via STM32
- **Custom MAVLink**: Message ID 180-183
- **Topics**: `/robomaster/motor_command`, `/robomaster/motor_state`

### Encoders
- **Count**: Up to 16 encoders
- **Data**: Position (rad), velocity (rad/s)
- **Features**: Position reset, configuration management
- **Topics**: `/encoder/states`

## Quick Start

### Installation

```bash
# Clone the workspace (if not already done)
cd ~/ros2_jazzy
source /opt/ros/jazzy/setup.bash

# Install dependencies
sudo apt update
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-sensor-msgs \
                 ros-jazzy-geometry-msgs python3-pyqt5

# Build all packages
colcon build --packages-select stm32_mavlink_uart stm32_mavlink_udp mavlink_wizard
source install/setup.bash

# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER
# Log out and log back in for group changes to take effect
```

### Basic Usage

#### Serial (UART) Communication

```bash
# Launch UART interface with auto-detected serial port
ros2 launch stm32_mavlink_uart stm32_interface.launch.py

# Launch with specific serial port
ros2 launch stm32_mavlink_uart stm32_interface.launch.py \
    serial_port:=/dev/ttyUSB0 baud_rate:=115200

# Launch with GUI manager
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

#### Network (UDP) Communication

```bash
# Launch UDP interface in server mode (auto-detect remote)
ros2 launch stm32_mavlink_udp stm32_udp.launch.py

# Connect to ESP32 WiFi module
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    remote_host:=192.168.4.1 remote_port:=14550 is_server_mode:=false

# Launch with GUI manager (UDP backend)
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

### Command Examples

#### Servo Control
```bash
# Move servo 1 to 45 degrees
ros2 topic pub /servo/command stm32_mavlink_uart/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 45.0, enabled: true}" --once

# Monitor servo states
ros2 topic echo /servo/states
```

#### DC Motor Control
```bash
# Position control: move to 1.57 radians (90 degrees)
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 0, target_value: 1.57, enabled: true}" --once

# Velocity control: 5 rad/s
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 1, target_value: 5.0, enabled: true}" --once

# Duty-to-position: 80% duty to reach 180 degrees
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 3, target_value: 0.8, target_position_rad: 3.14, enabled: true}" --once
```

#### RoboMaster Motor Control
```bash
# Velocity control: 10 RPS
ros2 topic pub /robomaster/motor_command stm32_mavlink_uart/msg/RobomasterMotorCommand \
    "{motor_id: 1, control_mode: 1, target_velocity_rps: 10.0, enabled: true}" --once

# Monitor motor state
ros2 topic echo /robomaster/motor_state
```

## ROS2 Topics & Services

### Common Topics (Both UART and UDP)

**Published:**
- `/servo/states` - Servo positions and status
- `/encoder/states` - Encoder positions and velocities
- `/robomaster/motor_state` - RoboMaster motor feedback
- `/dcmotor/state` - DC motor status and telemetry
- `/diagnostics` - System diagnostics

**Subscribed:**
- `/servo/command` - Servo control commands
- `/robomaster/motor_command` - RoboMaster motor commands
- `/dcmotor/command` - DC motor commands

### Services

**Servo:**
- `/servo/set_config` - Configure servo parameters

**Encoder:**
- `/encoder/set_config` - Configure encoder parameters
- `/encoder/reset_position` - Reset encoder position to zero

**DC Motor:**
- `/set_dcmotor_config` - Configure PID parameters and limits
- `/get_dcmotor_config` - Retrieve current configuration

**RoboMaster:**
- `/set_robomaster_motor_config` - Configure motor parameters
- `/get_robomaster_motor_config` - Retrieve current configuration

## Configuration Files

### UART Interface
- `stm32_mavlink_uart/config/serial_config.yaml`
  - Serial port, baud rate, timeout settings
  - MAVLink system/component IDs

### UDP Interface
- `stm32_mavlink_udp/config/udp_config.yaml`
  - Network addresses and ports
  - Server/client mode settings
  - MAVLink system/component IDs

### GUI Manager
- `mavlink_wizard/config/wizard_config.yaml`
  - GUI layout and appearance
  - Device parameter definitions
  - Plotting and visualization settings

## Development

### Building Individual Packages

```bash
# Build only UART interface
colcon build --packages-select stm32_mavlink_uart

# Build only UDP interface
colcon build --packages-select stm32_mavlink_udp

# Build GUI manager
colcon build --packages-select mavlink_wizard

# Build all with dependencies
colcon build --packages-up-to mavlink_wizard
```

### Adding Custom Applications

Your custom application should depend on either `stm32_mavlink_uart` or `stm32_mavlink_udp`:

```xml
<!-- package.xml -->
<depend>stm32_mavlink_uart</depend>  <!-- For serial communication -->
<!-- OR -->
<depend>stm32_mavlink_udp</depend>   <!-- For network communication -->
```

```python
# Python example
from stm32_mavlink_uart.msg import ServoCommand, DCMotorCommand

# Create publishers
servo_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
motor_pub = self.create_publisher(DCMotorCommand, '/dcmotor/command', 10)
```

## Hardware Setup

### STM32 Firmware
- Compatible STM32F446RE firmware located at: `/home/imanoob/ilias2026_ws/hal_ws/f446re`
- Firmware must implement:
  - MAVLink v2 protocol parser
  - Device-specific message handlers
  - CAN bus communication for RoboMaster motors
  - Timer-based telemetry (100ms / 10Hz)

### Wiring Requirements
- **UART**: TX/RX connected to USB-to-Serial adapter or STM32 USB CDC
- **CAN**: CAN_H/CAN_L connected to RoboMaster motor bus
- **Servos**: PWM outputs connected to servo signal pins
- **DC Motors**: Motor driver H-bridge connected to STM32 PWM/GPIO
- **Encoders**: Quadrature encoder signals connected to STM32 timer inputs

### Network Setup (UDP)
- ESP32 WiFi bridge or direct Ethernet connection
- Configure ESP32 to bridge UART ↔ UDP (MAVLink transparent mode)
- Default port: 14550 (MAVLink standard)

## Troubleshooting

### Serial Port Issues

```bash
# Check available serial ports
ls /dev/tty* | grep -E "USB|ACM"

# Check permissions
ls -la /dev/ttyUSB0

# Add user to dialout group (requires logout)
sudo usermod -a -G dialout $USER

# Test serial port
screen /dev/ttyUSB0 115200
```

### Network Issues

```bash
# Test UDP connectivity
nc -u 192.168.4.1 14550

# Check firewall
sudo ufw status
sudo ufw allow 14550/udp

# Monitor network traffic
sudo tcpdump -i any port 14550
```

### Device Not Found

1. **Check hardware connection**: LED indicators, power supply
2. **Verify firmware**: Ensure STM32 is flashed with compatible firmware
3. **Check ROS2 topics**: `ros2 topic list | grep servo`
4. **Monitor diagnostics**: `ros2 topic echo /diagnostics`
5. **Check MAVLink communication**: Enable debug logging in launch files

### Build Errors

```bash
# Clean build
rm -rf build/ install/ log/
colcon build

# Build with verbose output
colcon build --event-handlers console_direct+

# Check dependencies
rosdep check --from-paths src --ignore-src
```

## Performance Notes

- **Telemetry Rate**: 10Hz (100ms) for all devices
- **Servo Update Rate**: Matches telemetry at 10Hz
- **DC Motor PID**: Configurable loop rates (typically 100Hz on STM32)
- **Network Latency**: Add 10-50ms for WiFi, 1-5ms for Ethernet
- **CAN Bus**: 1Mbps for RoboMaster motors

## License

MIT License

## Contributing

When contributing to this project:
1. Follow ROS2 naming conventions
2. Use MAVLink SDK from `mavlink_SDK/` directory
3. Update both UART and UDP interfaces if adding new features
4. Test with both serial and network communication
5. Update relevant README files

## Support & Documentation

- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **MAVLink Protocol**: https://mavlink.io/en/
- **STM32 HAL**: https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html
- **RoboMaster SDK**: https://github.com/RoboMaster

## Related Packages

- **seiretu**: Robot control application using this MAVLink system
- **dynamixel_controller**: Alternative servo system for Dynamixel motors
- **rogilink_flex**: RogiLink sensor communication system

---

**Version**: 1.0.0
**ROS2 Distribution**: Jazzy
**Last Updated**: 2025-10-03
