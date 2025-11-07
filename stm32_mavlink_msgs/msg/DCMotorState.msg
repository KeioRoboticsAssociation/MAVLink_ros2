# DC Motor State Message
# Real-time feedback and status information from DC motor

# Motor identification
uint8 motor_id          # Motor ID (matches DCMotorCommand.motor_id)

# Position and motion feedback
float32 position_rad    # Current absolute position in radians
                       # Measured from motor's calibrated zero point
float32 velocity_rad_s  # Current rotational velocity in radians per second
                       # Positive = forward rotation, negative = reverse

# Electrical measurements
float32 current_a       # Current draw in amperes
                       # Actual current consumption (always positive)
float32 temperature_c   # Motor temperature in Celsius
                       # Used for thermal protection and monitoring

# Status and diagnostics
uint8 STATUS_OK = 0
uint8 STATUS_NOT_INITIALIZED = 1    # Motor not yet calibrated or configured
uint8 STATUS_ERROR = 2              # General error condition
uint8 STATUS_OVERHEAT = 3           # Temperature above safe threshold
uint8 STATUS_OVERCURRENT = 4        # Current above safe threshold
uint8 STATUS_TIMEOUT = 5            # Communication timeout with motor controller
uint8 status                        # Current motor status (see STATUS_* constants)

# Control state
bool enabled           # Motor enabled state (matches command)
                      # true = motor is actively controlled
                      # false = motor is freewheeling

# Control mode and duty cycle feedback (from SERVO_OUTPUT_RAW telemetry)
uint8 control_mode     # Current control mode (0=position, 1=velocity, 2=current, 3=duty-to-position)
float32 current_duty_cycle    # Current duty cycle (-1.0 to 1.0)
float32 target_duty_cycle     # Target duty cycle (-1.0 to 1.0)
float32 target_velocity_rad_s # Target velocity in rad/s (from LOCAL_POSITION_NED)

# Timing
uint64 timestamp       # Timestamp of the measurement (nanoseconds since epoch)
                      # Used for data synchronization and timeout detection