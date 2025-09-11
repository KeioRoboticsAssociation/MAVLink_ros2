# RoboMaster Motor State Message
# Current state and telemetry of a RoboMaster motor

std_msgs/Header header

# Motor identification  
uint8 motor_id              # Motor ID (1-8)

# Current measurements
float32 current_position_rad # Current position in radians
float32 current_velocity_rps # Current velocity in rotations per second
int16 current_milliamps     # Current draw in milliamps
uint8 temperature_celsius   # Motor temperature in Celsius

# Target values
float32 target_position_rad # Target position in radians  
float32 target_velocity_rps # Target velocity in rotations per second
int16 target_current_ma     # Target current in milliamps

# Control state
uint8 control_mode          # Current control mode (0=current, 1=velocity, 2=position)
bool enabled                # Motor enabled state

# Status and diagnostics
uint8 STATUS_OK = 0
uint8 STATUS_NOT_INITIALIZED = 1
uint8 STATUS_CAN_ERROR = 2
uint8 STATUS_OUT_OF_RANGE = 3
uint8 STATUS_TIMEOUT = 4
uint8 STATUS_CONFIG_ERROR = 5
uint8 STATUS_OVERHEAT = 6
uint8 STATUS_OVERCURRENT = 7
uint8 status

# Statistics
uint32 last_command_time_ms # Timestamp of last command
uint32 saturation_count     # Number of command saturations
uint32 timeout_count        # Number of communication timeouts
uint32 error_count          # Number of errors
uint32 overheat_count       # Number of overheating events