# RS485 Motor Configuration Message

uint8 motor_id              # Motor ID (30-49 for RS485 motors)
uint8 device_id             # RS485 board device ID (DIP switch setting)
uint8 motor_index           # Motor index on board (0-2 for 3-motor boards)
float32 max_velocity_rps    # Maximum velocity limit in rev/s
float32 max_acceleration    # Maximum acceleration in rev/s^2
bool enabled                # Motor enable state
