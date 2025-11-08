# DC Motor Control Command Message
# Simple brushed/brushless DC motor control interface

# Motor identification
uint8 motor_id          # Motor ID (10-15 for direct PWM DC motors)

# Control mode - Determines interpretation of target_value
# CONTROL_MODE_POSITION (0): Position control using target_value as radians
#   - Use for precise angular positioning (servo-like behavior)
#   - PID controller moves motor to target absolute position
#   - Unit: radians (relative to motor's zero position)
# CONTROL_MODE_VELOCITY (1): Velocity control using target_value as rad/s
#   - Use for constant speed applications (fans, pumps, wheels)
#   - PID controller maintains target rotational speed
#   - Unit: radians per second (positive = forward, negative = reverse)
# CONTROL_MODE_CURRENT (2): Current control using target_value as amperes
#   - Use for torque control or current limiting
#   - Direct current control for precise force application
#   - Unit: amperes (positive = forward torque, negative = reverse torque)
# CONTROL_MODE_DUTY_TO_POSITION (3): Duty cycle control until position reached
#   - Apply fixed duty cycle until target position is reached
#   - Combines open-loop duty control with position feedback
#   - target_value = duty cycle (-1.0 to 1.0), target_position_rad = position target
uint8 CONTROL_MODE_POSITION = 0
uint8 CONTROL_MODE_VELOCITY = 1
uint8 CONTROL_MODE_CURRENT = 2
uint8 CONTROL_MODE_DUTY_TO_POSITION = 3
uint8 control_mode

# Command value - Interpretation depends on control_mode
float32 target_value    # Target position (rad), velocity (rad/s), current (A), or duty cycle (-1.0 to 1.0)
                       # See control_mode constants for unit interpretation

# Additional parameter for duty-to-position control mode
float32 target_position_rad  # Target position for CONTROL_MODE_DUTY_TO_POSITION (ignored for other modes)

# Control flag
bool enabled            # Enable/disable motor
                       # false = motor freewheels (no active control)
                       # true = motor applies control based on control_mode and target_value