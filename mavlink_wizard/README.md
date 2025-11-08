# MAVLink Wizard

A comprehensive GUI application for configuring, monitoring, and calibrating MAVLink devices, similar to ROBOTIS Dynamixel Wizard. Built with ROS2 and PyQt5 for professional device management with advanced features including real-time plotting, calibration wizards, and firmware management.

**Interface-Agnostic Design**: Works seamlessly with both **UART** and **UDP** MAVLink interfaces without any configuration changes. Simply launch your preferred interface (serial or network), and the wizard will automatically connect.

## Features

### 🔧 Device Management
- **Automatic Device Discovery**: Monitors ROS2 topics for active devices
- **Multi-Device Support**: Servos, encoders, RoboMaster motors, and DC motors
- **Real-time Status Monitoring**: Connection health and communication statistics
- **Device Tree View**: Organized device hierarchy with status indicators
- **Transport Layer Agnostic**: Works with both UART (serial) and UDP (network) interfaces

### 📊 Real-time Monitoring & Plotting
- **Live Data Visualization**: Real-time plotting with PyQtGraph (optional)
- **Parameter Monitoring**: Position, velocity, current, temperature tracking
- **Historical Data**: Configurable data retention with circular buffers
- **Multi-parameter Plots**: Overlay multiple parameters on single plots
- **Fallback Mode**: Table-based display when plotting unavailable

### ⚙️ Parameter Management
- **Comprehensive Parameter Sets**: Device-specific parameter definitions
- **Batch Operations**: Update multiple devices simultaneously
- **Input Validation**: Real-time parameter range checking with visual feedback
- **Configuration Templates**: Save and load device configurations
- **Error Handling**: Service call timeouts and retry mechanisms

### 🎯 Calibration Wizards
- **Servo Calibration**: Center position, range limits, and zero offset
- **Encoder Calibration**: Zero position setting with verification
- **Step-by-step Guidance**: Interactive wizards with progress tracking
- **Safety Features**: Movement limits and emergency stop capabilities

### 💾 Firmware Management
- **Firmware Upload**: Support for .bin, .hex, and .elf files
- **Version Detection**: Automatic firmware version extraction
- **Validation**: File integrity and compatibility checks
- **Progress Tracking**: Real-time upload progress with cancellation support

### 🛠️ Advanced Features
- **Error Handling**: Comprehensive error reporting with user feedback dialogs
- **Configuration Management**: Persistent settings with JSON configuration
- **Logging**: Structured logging with configurable levels
- **Performance Optimization**: Bounded data structures and efficient threading

## How It Works: Transport Layer Independence

The wizard achieves **complete transport independence** through ROS2's publish-subscribe architecture:

```
┌─────────────────────────────────────────────────────┐
│              MAVLink Wizard (This Package)          │
│  ┌────────────────────────────────────────────┐    │
│  │ Subscribes to: /servo/states, /dcmotor/...│    │
│  │ Calls services: /set_dcmotor_config, ...  │    │
│  │ Uses: stm32_mavlink_msgs                  │    │
│  └────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
                        ▲ ▼
            ROS2 Topics & Services (stm32_mavlink_msgs)
                        ▲ ▼
    ┌────────────────────────────────────────────────┐
    │    Choose ONE transport interface:             │
    │  ┌──────────────────┐  ┌──────────────────┐  │
    │  │ stm32_mavlink_   │  │ stm32_mavlink_   │  │
    │  │      uart        │  │      udp         │  │
    │  │  (Serial/UART)   │  │  (Network/UDP)   │  │
    │  └──────────────────┘  └──────────────────┘  │
    └────────────────────────────────────────────────┘
                        ▲ ▼
                 MAVLink Protocol
                        ▲ ▼
    ┌────────────────────────────────────────────────┐
    │           STM32 Microcontroller                │
    │   F446RE (UART) or H753 (Ethernet/UDP)        │
    └────────────────────────────────────────────────┘
```

**Key Benefit**: Launch the wizard once, switch between serial and network devices without restarting or reconfiguring the GUI.

## Prerequisites

### System Requirements
- **ROS2**: Jazzy or Humble
- **Python**: 3.8 or later
- **OS**: Ubuntu 22.04 or compatible Linux distribution

### Required Dependencies
- **PyQt5**: GUI framework
- **stm32_mavlink_msgs**: Shared ROS2 message/service definitions
- **MAVLink Interface** (choose one):
  - **stm32_mavlink_uart**: For serial/UART communication
  - **stm32_mavlink_udp**: For network/UDP communication
  - (Both provide identical topics/services - wizard works with either)

### Optional Dependencies
- **PyQtGraph**: For advanced real-time plotting (recommended)
- **NumPy**: For numerical operations
- **SciPy**: For signal processing (future features)

## Installation

### Quick Install

```bash
# Install system dependencies
sudo apt update
sudo apt install python3-pyqt5 python3-numpy

# Optional: Install PyQtGraph for advanced plotting
pip3 install pyqtgraph

# Navigate to your ROS2 workspace
cd ~/ros2_jazzy

# Install ROS2 package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace (includes shared messages and wizard)
# Choose one or both interfaces based on your needs
colcon build --packages-select stm32_mavlink_msgs stm32_mavlink_uart stm32_mavlink_udp mavlink_wizard
source install/setup.bash
```

### Development Install

For development with additional features:

```bash
# Install all optional dependencies
pip3 install pyqtgraph numpy scipy matplotlib

# Build with debug symbols
colcon build --packages-select mavlink_wizard --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Enable development mode
echo "export MAVLINK_WIZARD_DEV_MODE=1" >> ~/.bashrc
```

## Usage

### Basic Usage

**IMPORTANT**: The wizard requires a MAVLink interface to be running. Launch the interface first, then launch the wizard.

#### Option 1: Using UART (Serial) Interface

```bash
# Terminal 1: Launch UART interface
ros2 launch stm32_mavlink_uart stm32_interface.launch.py

# Or with custom serial port
ros2 launch stm32_mavlink_uart stm32_interface.launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200

# Terminal 2: Launch wizard (connects to UART automatically)
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

#### Option 2: Using UDP (Network) Interface

```bash
# Terminal 1: Launch UDP interface
ros2 launch stm32_mavlink_udp stm32_udp.launch.py

# Or with custom network settings
ros2 launch stm32_mavlink_udp stm32_udp.launch.py remote_host:=192.168.11.4 remote_port:=14550

# Terminal 2: Launch wizard (connects to UDP automatically)
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

**The wizard is interface-agnostic** - it automatically connects to whichever MAVLink interface is publishing on the standard topics. You can switch between UART and UDP without changing the wizard.

### Real-World Workflow Example

**Scenario**: Developing with a bench-connected STM32F446RE (UART), then deploying to a robot with STM32H753 (UDP/Ethernet).

```bash
# Development Phase: Use UART for bench testing
# Terminal 1: UART interface for F446RE
ros2 launch stm32_mavlink_uart stm32_interface.launch.py serial_port:=/dev/ttyACM0

# Terminal 2: Wizard for configuration
ros2 launch mavlink_wizard mavlink_wizard.launch.py
# Configure servos, tune PID parameters, save configuration

# Deployment Phase: Same wizard, different transport
# Terminal 1: UDP interface for H753 on robot
ros2 launch stm32_mavlink_udp stm32_udp.launch.py remote_host:=192.168.11.4

# Terminal 2: Same wizard command!
ros2 launch mavlink_wizard mavlink_wizard.launch.py
# Load saved configuration, works immediately
```

**No code changes required** - the wizard automatically adapts to the active interface.

### GUI Interface

#### Device Scanner
1. Ensure a MAVLink interface (UART or UDP) is running
2. The wizard automatically discovers devices publishing on ROS2 topics
3. Devices appear in the device tree with their status
4. Click "Scan for Devices" to manually refresh the device list

#### Parameter Configuration
1. Select a device from the device tree
2. Switch to the "Configuration" tab
3. Modify parameters as needed
4. Click "Write Parameters" to send changes to the device

#### Real-time Monitoring
1. Switch to the "Real-time Monitor" tab
2. Click "Start Monitoring" to begin data collection
3. View live device data in the table
4. Use "Clear Data" to reset the display

### Command Line Tools

Run individual components separately:

```bash
# Device scanner only
ros2 run mavlink_wizard device_scanner.py

# Parameter manager only
ros2 run mavlink_wizard parameter_manager.py

# Message monitor only
ros2 run mavlink_wizard message_monitor.py
```

## Configuration

The application uses a YAML configuration file located at:
`~/.config/mavlink_wizard/wizard_config.yaml`

Key configuration sections:
- **serial**: Serial port settings
- **discovery**: Device discovery parameters
- **gui**: Interface appearance and behavior
- **monitoring**: Data collection settings
- **device_types**: Device-specific configurations

## Architecture

### Core Components

1. **MAVLink Wizard GUI** (`mavlink_wizard_gui.py`)
   - Main application window
   - PyQt5-based user interface
   - Integrates all other components

2. **Device Scanner** (`device_scanner.py`)
   - Serial port communication
   - MAVLink message parsing
   - Device discovery and management

3. **Parameter Manager** (`parameter_manager.py`)
   - Parameter definitions and validation
   - Configuration file handling
   - Device parameter operations

4. **Message Monitor** (`message_monitor.py`)
   - Real-time data collection
   - Statistical analysis
   - Message logging and recording

### ROS2 Integration

The wizard uses the **`stm32_mavlink_msgs`** package for all message and service definitions, making it compatible with both transport interfaces:

**Message Subscriptions** (from `stm32_mavlink_msgs`):
- `/servo/states` - Servo positions and status
- `/encoder/states` - Encoder feedback
- `/robomaster/motor_state` - RoboMaster motor telemetry
- `/dcmotor/state` - DC motor feedback
- `/diagnostics` - System diagnostics

**Service Calls** (from `stm32_mavlink_msgs`):
- `/servo/set_config` - Configure servo parameters
- `/encoder/set_config` - Configure encoder settings
- `/set_robomaster_motor_config` - Configure RoboMaster motors
- `/get_robomaster_motor_config` - Retrieve motor configuration
- `/set_dcmotor_config` - Configure DC motor PID
- `/get_dcmotor_config` - Retrieve DC motor settings

**Transport Independence**: The wizard doesn't care whether these topics/services come from:
- `stm32_mavlink_uart` (serial communication)
- `stm32_mavlink_udp` (network communication)

Both interfaces provide identical ROS2 interfaces using the same shared message definitions.

## Supported Devices

### Servo Motors
- Position, velocity, and current control
- Angle limits and speed configuration
- Torque limiting and protection

### Encoders
- Quadrature encoder support
- Configurable resolution (CPR)
- Channel inversion options
- Index channel support

### Brushless Motors
- Current and velocity control
- PID parameter tuning
- Multiple control modes
- Temperature monitoring

## File Formats

### Configuration Files
Device configurations are saved in JSON format:
```json
{
  "device_id": 1,
  "device_type": "servo",
  "parameters": {
    "SPEED": 150,
    "TORQUE_LIMIT": 75,
    ...
  }
}
```

### Monitoring Data
Monitoring data is exported in CSV format:
```csv
timestamp,message_type,device_id,position,velocity,current
1634567890.123,servo_state,1,45.2,12.5,0.8
```

## Development

### Adding New Device Types

1. Define parameters in `parameter_manager.py`:
```python
'new_device': {
    'PARAM_NAME': Parameter(
        name='PARAM_NAME',
        value=default_value,
        param_type=ParameterType.UINT16,
        description='Parameter description',
        min_value=0,
        max_value=1000
    )
}
```

2. Add message handling in `message_monitor.py`:
```python
def _handle_new_device_state(self, msg):
    # Process device-specific message
    pass
```

3. Update GUI widgets in `mavlink_wizard_gui.py` as needed.

### Testing

Run the test suite:
```bash
cd ~/ros2_jazzy
colcon test --packages-select mavlink_wizard
```

## Troubleshooting

### Common Issues

1. **No Devices Detected**
   - **Check MAVLink interface is running**:
     ```bash
     ros2 node list | grep mavlink
     ```
   - Should show either `mavlink_serial_node` (UART) or `mavlink_udp_node` (UDP)
   - If not running, launch the appropriate interface first

2. **Serial Port Access** (UART only)
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Network Connection Issues** (UDP only)
   - Verify network connectivity: `ping 192.168.11.4`
   - Check firewall: `sudo ufw allow 14550/udp`
   - Ensure STM32 board and PC are on same subnet

4. **PyQt5 Import Error**
   ```bash
   sudo apt install python3-pyqt5
   ```

5. **Wrong Message Package**
   - Ensure wizard uses `stm32_mavlink_msgs` (not `stm32_mavlink_uart` or `stm32_mavlink_udp`)
   - Check imports in scripts: `grep "from stm32_mavlink" scripts/*.py`

6. **Device Not Responding**
   - **UART**: Check serial port connections, verify baud rate settings
   - **UDP**: Verify board IP address matches remote_host parameter
   - Ensure device is powered on and firmware is running

### Debug Mode

Enable debug logging:
```bash
# Debug wizard
ros2 launch mavlink_wizard mavlink_wizard.launch.py --ros-args --log-level debug

# Debug UART interface
ros2 launch stm32_mavlink_uart stm32_interface.launch.py --ros-args --log-level debug

# Debug UDP interface
ros2 launch stm32_mavlink_udp stm32_udp.launch.py --ros-args --log-level debug
```

### Verifying Topics

Check that the MAVLink interface is publishing topics:
```bash
# List all topics
ros2 topic list

# Should see:
# /servo/states
# /encoder/states
# /robomaster/motor_state
# /dcmotor/state
# /diagnostics

# Check topic message type (should be from stm32_mavlink_msgs)
ros2 topic info /servo/states
# Type: stm32_mavlink_msgs/msg/ServoState
```

## Architecture Benefits

### Why This Design Works

The wizard's **transport-agnostic architecture** provides significant advantages:

#### 1. **Development Flexibility**
- Develop on your desk with UART (simple USB connection)
- Deploy on robots with UDP (wireless or Ethernet)
- No code changes between environments

#### 2. **Single Source of Truth**
- All packages use `stm32_mavlink_msgs` for message definitions
- Changes to messages automatically propagate to all consumers
- No message definition drift between UART and UDP packages

#### 3. **Clean Separation of Concerns**
```
Application Layer (wizard)     ← Knows about: devices, parameters, GUI
       ↕ (ROS2 messages)
Transport Layer (uart/udp)      ← Knows about: MAVLink, serial/network
       ↕ (MAVLink protocol)
Hardware Layer (STM32)          ← Knows about: motors, sensors, control
```

#### 4. **Future-Proof**
- Add new transport layers (e.g., CAN, Bluetooth) without modifying wizard
- New applications automatically work with all transports
- Interface selection is a deployment decision, not a development decision

#### 5. **Reduced Maintenance**
- Fix bugs in one place (message definitions)
- Update features once, works everywhere
- Test with one interface, confident it works with others

This architectural pattern follows ROS2 best practices for **interface abstraction** and **dependency inversion**.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Create an issue on GitHub
- Check the ROS2 MAVLink documentation
- Consult the MAVLink protocol specification

## Acknowledgments

- Inspired by Dynamixel Wizard
- Built on ROS2 and PyQt5 frameworks
- Uses MAVLink protocol for device communication