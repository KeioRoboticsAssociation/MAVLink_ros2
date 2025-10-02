# STM32 MAVLink UDP Interface

ROS2 package providing UDP communication interface for STM32 devices using the MAVLink protocol. This package enables wireless communication with STM32-based systems including servo control, motor management, and sensor data acquisition over WiFi/Ethernet networks.

## Features

- **UDP Communication**: Server and client modes for flexible network configurations
- **Complete Device Support**: Servos, encoders, RoboMaster motors, and DC motors
- **Independent Package**: Self-contained with own message/service definitions and MAVLink headers
- **Drop-in Replacement**: Compatible with existing serial-based applications
- **Network Flexibility**: Configurable IP addresses, ports, and connection modes

## Architecture

The UDP package mirrors the serial interface architecture but communicates over UDP sockets instead of UART:

```
Network Device ←→ [UDP Socket] ←→ stm32_mavlink_udp ←→ [ROS2 Topics/Services] ←→ Applications
      ↓                              ↓                                           ↓
   WiFi/Ethernet                Device Control                              mavlink_wizard
  ESP32, Pi, etc.             (Servos, Motors,                              seiretu (GUI)
                               Encoders)                                    rogilink_flex
```

## Supported Hardware

- **ESP32 with WiFi/Ethernet**: Wireless STM32 communication
- **Raspberry Pi**: Network-connected STM32 systems
- **Ethernet-enabled microcontrollers**: Direct network communication
- **WiFi modules**: Bridge between UART and UDP
- **Simulation environments**: SITL testing and development

## Dependencies

- ROS2 Jazzy
- Standard ROS2 packages: `rclcpp`, `std_msgs`, `sensor_msgs`, `diagnostic_msgs`
- Network socket support (built-in Linux)

## Installation

1. **Clone to workspace**:
```bash
cd ~/ros2_jazzy/src
# Package should already be present
```

2. **Install dependencies**:
```bash
cd ~/ros2_jazzy
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package**:
```bash
colcon build --packages-select stm32_mavlink_udp
source install/setup.bash
```

## Usage

### Basic Launch

**Server Mode (Auto-detect remote)**:
```bash
# Default: bind to all interfaces on port 14550
ros2 launch stm32_mavlink_udp stm32_udp.launch.py
```

**Client Mode (Fixed remote)**:
```bash
# Connect to specific device
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    is_server_mode:=false remote_host:=192.168.1.100
```

### Network Configurations

**ESP32 WiFi Module**:
```bash
# ESP32 in AP mode (default IP: 192.168.4.1)
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    remote_host:=192.168.4.1 remote_port:=14550 is_server_mode:=false
```

**Custom Network Settings**:
```bash
# Specific local and remote configuration
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    local_host:=192.168.1.10 local_port:=14550 \
    remote_host:=192.168.1.100 remote_port:=14551
```

**Localhost Testing**:
```bash
# Test without hardware
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    local_host:=127.0.0.1 remote_host:=127.0.0.1
```

**SITL Simulation**:
```bash
# Connect to simulation environment
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    local_host:=127.0.0.1 local_port:=14550 \
    remote_host:=127.0.0.1 remote_port:=14560 is_server_mode:=false
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `local_host` | `0.0.0.0` | Local IP address to bind to (0.0.0.0 for all interfaces) |
| `local_port` | `14550` | Local UDP port to bind to |
| `remote_host` | `192.168.1.100` | Remote IP address to send to |
| `remote_port` | `14551` | Remote UDP port to send to |
| `is_server_mode` | `true` | Auto-detect remote address from incoming packets |
| `system_id` | `255` | MAVLink system ID |
| `component_id` | `1` | MAVLink component ID |
| `target_system_id` | `1` | Target MAVLink system ID |
| `target_component_id` | `1` | Target MAVLink component ID |

## ROS2 Topics

### Published Topics

- `/servo/states` - Servo position and status information
- `/encoder/states` - Encoder position data
- `/robomaster/motor_state` - RoboMaster motor feedback
- `/dcmotor/state` - DC motor status and telemetry
- `/diagnostics` - System diagnostic information

### Subscribed Topics

- `/servo/command` - Servo control commands
- `/robomaster/motor_command` - RoboMaster motor control
- `/dcmotor/command` - DC motor control commands

## Device Control Examples

### Servo Control
```bash
# Move servo 1 to 45 degrees
ros2 topic pub /servo/command stm32_mavlink_udp/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 45.0, enabled: true}" --once
```

### RoboMaster Motor Control
```bash
# Set motor 1 to 10 RPS velocity
ros2 topic pub /robomaster/motor_command stm32_mavlink_udp/msg/RobomasterMotorCommand \
    "{motor_id: 1, control_mode: 1, target_velocity_rps: 10.0, enabled: true}" --once
```

### DC Motor Control
```bash
# Position control: move to 1.57 radians (90 degrees)
ros2 topic pub /dcmotor/command stm32_mavlink_udp/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 0, target_value: 1.57, enabled: true}" --once

# Duty-to-position control: 80% duty cycle to reach 180 degrees
ros2 topic pub /dcmotor/command stm32_mavlink_udp/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 3, target_value: 0.8, target_position_rad: 3.14, enabled: true}" --once
```

## ROS2 Services

### Configuration Services

- `/set_servo_config` - Configure servo parameters
- `/set_dcmotor_config` - Configure DC motor PID parameters
- `/get_dcmotor_config` - Retrieve DC motor configuration
- `/set_robomaster_motor_config` - Configure RoboMaster motor settings
- `/get_robomaster_motor_config` - Retrieve RoboMaster motor configuration

### Service Examples

```bash
# Configure DC motor PID parameters
ros2 service call /set_dcmotor_config stm32_mavlink_udp/srv/SetDCMotorConfig \
    "{config: {speed_kp: 1.0, speed_ki: 0.1, position_kp: 2.0, max_current: 5.0}}"
```

## Message Types

### Core Messages

- **ServoCommand**: Control servo position and enable state
- **ServoState**: Servo position, status, and initialization state
- **RobomasterMotorCommand**: RoboMaster motor control (current/velocity/position)
- **RobomasterMotorState**: Motor feedback including position, velocity, current, temperature
- **DCMotorCommand**: DC motor control with multiple modes
- **DCMotorState**: Comprehensive DC motor status and diagnostics
- **EncoderState**: Encoder position and velocity data

### Device Status Codes

**Servo Status**:
- `0`: OK
- `1`: NOT_INITIALIZED
- `2`: ERROR

**DC Motor Status**:
- `0`: OK
- `1`: ERROR
- `2`: OVERHEAT
- `3`: OVERCURRENT
- `4`: TIMEOUT

## Network Setup

### ESP32 WiFi Bridge Example

1. **ESP32 Code**: Configure ESP32 to bridge UART ↔ UDP
2. **STM32 Connection**: Connect STM32 UART to ESP32
3. **ROS2 Launch**: Use ESP32's IP address as remote_host

```bash
# ESP32 at 192.168.4.1 (AP mode)
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    remote_host:=192.168.4.1 is_server_mode:=false
```

### Raspberry Pi Setup

1. **Network Connection**: Connect Pi and STM32 via Ethernet/WiFi
2. **MAVLink Bridge**: Run MAVLink UDP bridge on Pi
3. **ROS2 Connection**: Connect to Pi's IP address

```bash
# Pi at 192.168.1.100
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    remote_host:=192.168.1.100 remote_port:=14550
```

## Troubleshooting

### Common Issues

**Connection Failed**:
```bash
# Check network connectivity
ping 192.168.1.100

# Check firewall settings
sudo ufw allow 14550/udp
```

**No Data Received**:
- Verify remote device is sending to correct IP/port
- Check server vs client mode configuration
- Ensure MAVLink protocol compatibility

**Topic Not Publishing**:
```bash
# Check if node is running
ros2 node list | grep mavlink_udp

# Monitor raw network traffic
sudo tcpdump -i any port 14550

# Check ROS2 topic activity
ros2 topic hz /servo/states
```

### Debug Mode

Enable detailed logging:
```bash
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    --ros-args --log-level debug
```

## Integration with Existing Applications

### MAVLink Wizard Compatibility

The UDP package is compatible with MAVLink Wizard:
```bash
# Start UDP interface
ros2 launch stm32_mavlink_udp stm32_udp.launch.py

# Launch wizard (in another terminal)
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

### Seiretu Integration

Seiretu robot control works seamlessly:
```bash
# Start UDP interface for hardware
ros2 launch stm32_mavlink_udp stm32_udp.launch.py remote_host:=192.168.4.1

# Start seiretu GUI
ros2 launch seiretu seiretu_gui.launch.py
```

## Performance Considerations

- **Latency**: UDP typically has lower latency than serial for wireless links
- **Reliability**: Implement application-level acknowledgments for critical commands
- **Bandwidth**: MAVLink is efficient, but monitor network usage for high-frequency data
- **Packet Loss**: Server mode helps with dynamic IP scenarios

## Configuration Files

### Default Configuration (`config/udp_config.yaml`)

```yaml
mavlink_udp_node:
  ros__parameters:
    local_host: "0.0.0.0"
    local_port: 14550
    remote_host: "192.168.1.100"
    remote_port: 14551
    is_server_mode: true
    system_id: 255
    component_id: 1
    target_system_id: 1
    target_component_id: 1
```

### Custom Configuration

Create custom config files for different deployment scenarios:
```bash
# Use custom configuration
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    --ros-args --params-file /path/to/custom_config.yaml
```

## Security Considerations

- **Network Security**: Use VPN or secure networks for production deployments
- **Access Control**: Configure firewall rules to limit access
- **Authentication**: Consider implementing application-level authentication
- **Encryption**: Use VPN or application-level encryption for sensitive data

## Development and Testing

### Unit Testing
```bash
# Run package tests
colcon test --packages-select stm32_mavlink_udp

# Check test results
colcon test-result --all
```

### Integration Testing
```bash
# Test with localhost loopback
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    local_host:=127.0.0.1 remote_host:=127.0.0.1

# Send test commands
ros2 topic pub /servo/command stm32_mavlink_udp/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 0.0, enabled: true}" --once
```

## License

MIT License - See parent workspace for full license details.

## Contributing

This package is part of the larger STM32 MAVLink ROS2 workspace. Contributions should maintain compatibility with the serial interface package while extending UDP-specific functionality.

## Support

For issues related to:
- **UDP networking**: Check network configuration and firewall settings
- **MAVLink protocol**: Verify message compatibility and protocol versions
- **ROS2 integration**: Check topic/service interfaces and message types
- **Hardware setup**: Refer to device-specific documentation for network bridge configuration