# RoboMaster Motor Configuration Message
# Configuration parameters for RoboMaster motors

std_msgs/Header header

# Motor identification
uint8 motor_id              # Motor ID (1-8)

# Physical limits
float32 max_velocity_rps    # Maximum velocity in RPS
float32 max_acceleration_rps2 # Maximum acceleration in RPS^2
int16 max_current_ma        # Maximum current in milliamps
int16 min_current_ma        # Minimum current in milliamps

# Position limits
float32 min_position_rad    # Minimum position in radians
float32 max_position_rad    # Maximum position in radians
bool position_limits_enabled # Enable position limits

# Safety settings
uint32 watchdog_timeout_ms  # Watchdog timeout in milliseconds
uint8 max_temperature_celsius # Maximum temperature in Celsius
uint8 FAILSAFE_HOLD_POSITION = 0
uint8 FAILSAFE_BRAKE = 1
uint8 FAILSAFE_DISABLE_OUTPUT = 2
uint8 failsafe_behavior

# PID control parameters
float32 position_kp         # Position P gain
float32 position_ki         # Position I gain 
float32 position_kd         # Position D gain
float32 velocity_kp         # Velocity P gain
float32 velocity_ki         # Velocity I gain
float32 velocity_kd         # Velocity D gain

# Startup settings
float32 startup_position_rad # Initial position on startup
uint8 startup_mode          # Startup control mode (0=current, 1=velocity, 2=position)
bool start_disabled         # Start in disabled state

# Direction and offset
bool direction_inverted     # Invert rotation direction
float32 position_offset_rad # Position offset in radians