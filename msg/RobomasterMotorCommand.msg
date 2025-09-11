# RoboMaster Motor Command Message
# Motor control command for individual RoboMaster motors

std_msgs/Header header

# Motor identification
uint8 motor_id              # Motor ID (1-8)

# Control mode
uint8 CONTROL_MODE_CURRENT = 0
uint8 CONTROL_MODE_VELOCITY = 1  
uint8 CONTROL_MODE_POSITION = 2
uint8 control_mode

# Command values (only one should be used based on control_mode)
int16 target_current_ma     # Target current in milliamps (-16000 to 16000)
float32 target_velocity_rps # Target velocity in rotations per second
float32 target_position_rad # Target position in radians

# Control flags
bool enabled                # Enable/disable motor
bool reset_watchdog        # Reset watchdog timer
bool emergency_stop        # Emergency stop this motor