# STM32 MAVLink Interface - Usage Guide

## Quick Start

### 1. Hardware Setup
Connect your STM32 device via USB or USB-to-Serial adapter:
- **USB CDC devices**: Usually appear as `/dev/ttyACM0`, `/dev/ttyACM1`, etc.
- **USB-to-Serial adapters**: Usually appear as `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.

Check available ports:
```bash
ls /dev/tty* | grep -E "USB|ACM"
```

### 2. Launch the Interface

#### Default launch (tries /dev/ttyUSB0):
```bash
ros2 launch stm32_mavlink_interface stm32_interface.launch.py
```

#### Custom serial port:
```bash
# For USB-to-Serial devices
ros2 launch stm32_mavlink_interface stm32_interface.launch.py serial_port:=/dev/ttyUSB0

# For STM32 USB CDC devices
ros2 launch stm32_mavlink_interface stm32_interface.launch.py serial_port:=/dev/ttyACM0
```

#### Custom baud rate:
```bash
ros2 launch stm32_mavlink_interface stm32_interface.launch.py baudrate:=230400
```

### 3. Verify Connection

Check if heartbeats are being received:
```bash
ros2 topic hz /diagnostics
```

Monitor MAVLink communication:
```bash
ros2 topic echo /diagnostics
```

## Device Control

### Servo Motors

#### Send servo commands:
```bash
# Move servo 1 to 45 degrees
ros2 topic pub /servo/command stm32_mavlink_interface/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 45.0, enable: true}" --once

# Disable servo 2
ros2 topic pub /servo/command stm32_mavlink_interface/msg/ServoCommand \
    "{servo_id: 2, enable: false}" --once
```

#### Monitor servo states:
```bash
ros2 topic echo /servo/states
```

#### Configure servo parameters:
```bash
ros2 service call /servo/set_config stm32_mavlink_interface/srv/SetServoConfig \
    "{servo_id: 1, angle_min_deg: -90.0, angle_max_deg: 90.0, pulse_min_us: 1000, pulse_max_us: 2000}"
```

### RoboMaster Motors

#### Send motor commands:
```bash
# Position control for motor 1
ros2 topic pub /robomaster/motor/command stm32_mavlink_interface/msg/RobomasterMotorCommand \
    "{motor_id: 1, control_mode: 1, target_position: 1.57, enable: true}" --once

# Velocity control for motor 2
ros2 topic pub /robomaster/motor/command stm32_mavlink_interface/msg/RobomasterMotorCommand \
    "{motor_id: 2, control_mode: 2, target_velocity: 100.0, enable: true}" --once
```

#### Monitor motor states:
```bash
ros2 topic echo /robomaster/motor/states
```

#### Configure motor parameters:
```bash
ros2 service call /robomaster/motor/set_config stm32_mavlink_interface/srv/SetRobomasterMotorConfig \
    "{motor_id: 1, kp: 0.8, ki: 0.1, kd: 0.05, max_velocity: 500.0, max_current: 8000}"
```

### Encoders

#### Monitor encoder data:
```bash
ros2 topic echo /encoder/states
```

#### Reset encoder position:
```bash
ros2 service call /encoder/reset_position std_srvs/srv/Trigger
```

#### Configure encoder settings:
```bash
ros2 service call /encoder/set_config stm32_mavlink_interface/srv/SetEncoderConfig \
    "{encoder_id: 1, cpr: 1024, direction: 1, zero_offset: 0.0}"
```

## Troubleshooting

### Connection Issues

1. **Permission denied on serial port**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **Device not found**:
   ```bash
   # Check if device is connected
   dmesg | tail

   # List all USB devices
   lsusb
   ```

3. **Wrong baud rate**:
   - Default is 115200, common alternatives: 9600, 57600, 230400
   - Check STM32 firmware configuration

### Communication Issues

1. **No heartbeat received**:
   - Check serial port and baud rate
   - Verify STM32 firmware is running
   - Check cable connections

2. **Partial communication**:
   - Check for electromagnetic interference
   - Try different USB port/cable
   - Verify ground connections

3. **High error rate**:
   - Reduce baud rate
   - Check for loose connections
   - Verify MAVLink protocol version compatibility

### Configuration Issues

1. **Parameters not persisting**:
   - Check if STM32 firmware supports parameter storage
   - Verify flash memory is not write-protected

2. **Servo/motor not responding**:
   - Check enable flag in commands
   - Verify device ID matches firmware configuration
   - Check power supply to devices

## Advanced Usage

### Custom MAVLink Messages

The interface supports custom RoboMaster messages (IDs 180-183):
- 180: Motor control commands
- 181: Motor status feedback
- 182: Motor configuration
- 183: Motor telemetry

### Multiple Device Support

To use multiple STM32 devices:
```bash
# Device 1 on /dev/ttyUSB0
ros2 launch stm32_mavlink_interface stm32_interface.launch.py \
    serial_port:=/dev/ttyUSB0 system_id:=1

# Device 2 on /dev/ttyUSB1
ros2 launch stm32_mavlink_interface stm32_interface.launch.py \
    serial_port:=/dev/ttyUSB1 system_id:=2
```

### Integration with MAVLink Wizard

For a graphical interface:
```bash
ros2 launch mavlink_wizard mavlink_wizard.launch.py serial_port:=/dev/ttyUSB0
```

## Performance Tuning

### Reduce Latency
- Use higher baud rates (230400, 460800)
- Reduce telemetry rates in config file
- Use real-time kernel if needed

### Increase Throughput
- Batch multiple commands
- Optimize message frequency
- Use appropriate QoS settings

### Power Management
- Disable unused servos/motors
- Use appropriate current limits
- Monitor system diagnostics

ros2 topic pub /robomaster/motor_command stm32_mavlink_interface/msg/RobomasterMotorCommand "{motor_id: 5, control_mode: 2, target_position_rad: 1.5708, enabled: true}" --once