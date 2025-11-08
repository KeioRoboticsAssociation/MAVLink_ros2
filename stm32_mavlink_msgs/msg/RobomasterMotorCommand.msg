# RoboMaster Motor Command Message
# Motor control command for individual RoboMaster motors

std_msgs/Header header

# Motor identification
uint8 motor_id              # Motor ID (20-29 for RoboMaster motors)

# Control mode - Determines motor control behavior (matches STM32 MAVLink MOTOR_CONTROL_MODE enum)
# CONTROL_MODE_OPEN_LOOP (0): Open loop PWM/current control using target_current_ma
#   - Direct motor drive without feedback control
#   - Use for torque control or when precise current limiting is needed
#   - Range: -16000 to 16000 mA (approximately -16A to +16A)
#   - Motor applies specified current regardless of speed or position
# CONTROL_MODE_SPEED (1): Closed-loop velocity control using target_velocity_rps
#   - Use for constant speed applications (wheels, conveyors, etc.)
#   - PID controller maintains target rotational speed
#   - Unit: rotations per second (positive = forward, negative = reverse)
# CONTROL_MODE_POSITION (2): Closed-loop position control using target_position_rad
#   - Use for precise positioning (arms, turrets, actuators)
#   - PID controller moves motor to target absolute position
#   - Unit: radians (relative to motor's zero position)
# CONTROL_MODE_DISABLED (3): Motor disabled state
#   - Motor is completely disabled and freewheels
#   - All target values are ignored
#   - Use for emergency stops or power saving
# CONTROL_MODE_DUTY_TO_POSITION (4): Apply duty cycle until target position reached
#   - Hybrid control: applies constant duty cycle (open loop) until position target is met
#   - Uses target_current_ma as duty cycle percentage (-16000 to +16000 = -100% to +100%)
#   - Uses target_position_rad as position target to stop at
#   - Useful for fast positioning without overshoot in systems with known dynamics
uint8 CONTROL_MODE_OPEN_LOOP = 0
uint8 CONTROL_MODE_SPEED = 1
uint8 CONTROL_MODE_POSITION = 2
uint8 CONTROL_MODE_DISABLED = 3
uint8 CONTROL_MODE_DUTY_TO_POSITION = 4
# Legacy aliases for backward compatibility
uint8 CONTROL_MODE_CURRENT = 0     # Same as OPEN_LOOP
uint8 CONTROL_MODE_VELOCITY = 1    # Same as SPEED
uint8 control_mode

# Command values - Usage depends on control_mode
int16 target_current_ma     # Target current in milliamps (-16000 to 16000)
                           # CONTROL_MODE_OPEN_LOOP: Direct current/torque control
                           # CONTROL_MODE_DUTY_TO_POSITION: Duty cycle (-16000 to +16000 = -100% to +100%)
float32 target_velocity_rps # Target velocity in rotations per second
                           # CONTROL_MODE_SPEED: Target rotational speed
                           # Positive values = clockwise rotation (when viewed from motor shaft end)
float32 target_position_rad # Target position in radians
                           # CONTROL_MODE_POSITION: Target absolute position
                           # CONTROL_MODE_DUTY_TO_POSITION: Position to stop at when using duty cycle
                           # Absolute position relative to motor's calibrated zero point

# Control flags
bool enabled                # Enable/disable motor
                           # false = motor freewheels (no active control)
                           # true = motor applies control based on control_mode
bool reset_watchdog        # Reset communication watchdog timer
                           # Set to true to reset timeout counter on STM32 side
bool emergency_stop        # Emergency stop this motor immediately
                           # Overrides all other commands, motor stops and disables