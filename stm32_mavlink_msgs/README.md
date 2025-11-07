# stm32_mavlink_msgs

**Shared ROS2 Message and Service Definitions for STM32 MAVLink Communication**

This package provides unified message and service definitions used by all STM32 MAVLink packages in this workspace. By centralizing these definitions, we ensure consistency across different transport layers (UART, UDP) and applications.

## Purpose

This package serves as the **single source of truth** for all ROS2 message and service interfaces related to STM32 MAVLink communication. It is a dependency for:

- **stm32_mavlink_uart**: Serial/UART communication interface
- **stm32_mavlink_udp**: Network/UDP communication interface
- **mavlink_wizard**: GUI management and configuration tool
- **Custom applications**: Any application using STM32 MAVLink devices

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│            stm32_mavlink_msgs (Shared Layer)            │
│                                                         │
│  ┌──────────────┐              ┌──────────────┐       │
│  │   Messages   │              │   Services   │       │
│  │  - Servo     │              │  - Config    │       │
│  │  - Encoder   │              │  - Get/Set   │       │
│  │  - RoboMaster│              │              │       │
│  │  - DC Motor  │              │              │       │
│  └──────────────┘              └──────────────┘       │
└─────────────────────────────────────────────────────────┘
           ▲                ▲                ▲
           │                │                │
    ┌──────┴──────┐  ┌──────┴──────┐  ┌─────┴──────┐
    │ uart package│  │ udp package │  │   wizard   │
    └─────────────┘  └─────────────┘  └────────────┘
```

## Message Definitions

### Servo Motors
- **ServoCommand.msg**: Control commands for servo positioning
- **ServoState.msg**: Real-time servo position and status feedback

### Encoders
- **EncoderState.msg**: Position and velocity feedback
- **EncoderConfig.msg**: Encoder configuration parameters

### RoboMaster Motors (CAN)
- **RobomasterMotorCommand.msg**: Motor control commands
- **RobomasterMotorState.msg**: Motor feedback (position, velocity, current, temperature)
- **RobomasterMotorConfig.msg**: PID and control configuration

### DC Motors
- **DCMotorCommand.msg**: Position/velocity/current control commands
- **DCMotorState.msg**: Real-time motor state and telemetry
- **DCMotorConfig.msg**: PID parameters and operational limits

## Service Definitions

### Configuration Services
- **SetServoConfig.srv**: Configure servo parameters
- **SetEncoderConfig.srv**: Configure encoder parameters
- **SetRobomasterMotorConfig.srv**: Configure RoboMaster motor PID
- **GetRobomasterMotorConfig.srv**: Retrieve current configuration
- **SetDCMotorConfig.srv**: Configure DC motor PID and limits
- **GetDCMotorConfig.srv**: Retrieve DC motor configuration

## Usage in Other Packages

### package.xml
Add dependency in your package.xml:
```xml
<depend>stm32_mavlink_msgs</depend>
```

### CMakeLists.txt
```cmake
find_package(stm32_mavlink_msgs REQUIRED)
```

### Python Code
```python
from stm32_mavlink_msgs.msg import ServoCommand, DCMotorState
from stm32_mavlink_msgs.srv import SetServoConfig
```

### C++ Code
```cpp
#include "stm32_mavlink_msgs/msg/servo_command.hpp"
#include "stm32_mavlink_msgs/srv/set_servo_config.hpp"
```

## Building

This package is built automatically when building the workspace:

```bash
cd ~/ros2_jazzy
colcon build --packages-select stm32_mavlink_msgs
source install/setup.bash
```

## Message/Service Synchronization

**IMPORTANT**: When modifying message or service definitions in this package:

1. The changes will automatically affect **all** dependent packages
2. You must rebuild all dependent packages:
   ```bash
   colcon build --packages-up-to stm32_mavlink_uart stm32_mavlink_udp mavlink_wizard
   ```
3. Ensure MAVLink protocol compatibility if messages map to MAVLink messages
4. Update any documentation that references the modified interfaces

## Version History

- **v1.0.0**: Initial unified message/service package
  - Consolidated messages from stm32_mavlink_uart and stm32_mavlink_udp
  - Used UDP version as base (more complete status codes)
  - Standardized all device interfaces

## License

MIT License
