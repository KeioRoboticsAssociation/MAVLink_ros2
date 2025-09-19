# DC Motor Control Command Message
uint8 motor_id          # Motor ID (10 for the DC motor)
uint8 control_mode      # 0: position control, 1: velocity control, 2: current control
float32 target_value    # Target position (rad), velocity (rad/s), or current (A)
bool enabled            # Motor enable/disable