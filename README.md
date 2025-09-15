# STM32 MAVLink Interface for ROS2

ROS2 package for communicating with STM32 devices using MAVLink protocol over UART.

## Features

- MAVLink v2 protocol support
- Servo motor control (up to 16 servos)
- RoboMaster motor control (CAN-based GM6020, etc.)
- Encoder feedback (up to 16 encoders)
- Custom MAVLink message support (IDs 180-183)
- Configuration management via ROS2 services
- Diagnostic monitoring
- Real-time telemetry at 10Hz

## Installation

### Prerequisites

```bash
# Install ROS2 dependencies (adjust for your ROS2 distribution)
sudo apt update
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs

# Clone MAVLink headers (if not already included)
cd ~/ros2_jazzy/src/stm32_mavlink_interface/include
git clone https://github.com/mavlink/c_library_v2.git mavlink
```

### Build

```bash
cd ~/ros2_jazzy
colcon build --packages-select stm32_mavlink_interface --symlink-install
source install/setup.bash
```

## Usage

### Launch with default settings

```bash
ros2 launch stm32_mavlink_interface stm32_interface.launch.py
```

### Launch with custom serial port

```bash
# For USB-to-Serial devices (most common)
ros2 launch stm32_mavlink_interface stm32_interface.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200

# For CDC/ACM devices (STM32 with USB CDC)
ros2 launch stm32_mavlink_interface stm32_interface.launch.py \
    serial_port:=/dev/ttyACM0 \
    baudrate:=115200
```

### Monitor topics

```bash
# View servo states
ros2 topic echo /servo/states

# View encoder states  
ros2 topic echo /encoder/states

# Send servo command
ros2 topic pub /servo/command stm32_mavlink_interface/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 45.0, enable: true}"
```

### Services

```bash
# Set servo configuration
ros2 service call /servo/set_config stm32_mavlink_interface/srv/SetServoConfig \
    "{servo_id: 1, angle_min_deg: -90.0, angle_max_deg: 90.0}"

# Reset encoder position
ros2 service call /encoder/reset_position std_srvs/srv/Trigger
```

## Topics

### Published Topics

- `/servo/states` (ServoState): Current servo positions and status
- `/encoder/states` (EncoderState): Current encoder positions
- `/diagnostics` (DiagnosticStatus): System health status

### Subscribed Topics

- `/servo/command` (ServoCommand): Servo position commands

## Services

- `/servo/set_config` (SetServoConfig): Configure servo parameters
- `/encoder/set_config` (SetEncoderConfig): Configure encoder parameters
- `/encoder/reset_position` (Trigger): Reset encoder position

## Configuration

Edit `config/serial_config.yaml` to modify default parameters.

## Troubleshooting

### Permission denied on serial port

```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Find serial port

```bash
ls /dev/tty* | grep -E "USB|ACM"
```

### Check MAVLink communication

```bash
# Monitor heartbeat
ros2 topic hz /diagnostics
```

## License

MIT License