# DC Motor Configuration Message
uint8 motor_id                      # Motor ID
uint8 mode                         # Control mode (0: position, 1: velocity, 2: current)

# Speed control PID parameters (inner loop)
float32 speed_kp                   # Speed proportional gain
float32 speed_ki                   # Speed integral gain
float32 speed_kd                   # Speed derivative gain
float32 speed_max_integral         # Max integral value for speed control
float32 speed_max_output           # Max output for speed control

# Position control PID parameters (outer loop)
float32 position_kp                # Position proportional gain
float32 position_ki                # Position integral gain
float32 position_kd                # Position derivative gain
float32 position_max_integral      # Max integral value for position control
float32 position_max_output        # Max output for position control (max speed rad/s)

# Physical limits
float32 max_speed_rad_s            # Maximum speed in rad/s
float32 max_acceleration_rad_s2    # Maximum acceleration in rad/s²
bool use_position_limits           # Enable position limits
float32 position_limit_min_rad     # Minimum position limit in radians
float32 position_limit_max_rad     # Maximum position limit in radians

# Safety settings
uint32 watchdog_timeout_ms         # Watchdog timeout in milliseconds
uint32 control_period_ms           # Control loop period in milliseconds