# RS485 Motor Command Message (IDs 30-49)
# Command message for RS485-connected motors

uint8 motor_id              # Motor ID (30-49 for RS485 motors)
uint8 control_mode          # Control mode (0=position, 1=velocity, 2=duty)
float32 target_position     # Target position in rotations (for position mode)
float32 target_velocity     # Target velocity in rev/s (for velocity mode)
float32 target_duty         # Target duty cycle -1.0 to 1.0 (for duty mode)
float32 acceleration        # Acceleration in rev/s^2
bool enabled                # Enable/disable motor
