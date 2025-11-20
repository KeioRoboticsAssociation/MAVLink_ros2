# RS485 Motor State Message (IDs 30-49)
# Telemetry data from RS485-connected motors (e.g., Ikeya MD)

uint8 motor_id                        # Motor ID (30-49 for RS485 motors)
uint8 device_id                       # RS485 board device ID (DIP switch setting)
uint8 motor_index                     # Motor index on board (0-2 for 3-motor boards)
uint8 control_mode                    # Control mode (0=position, 1=velocity)
uint8 status                          # Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
uint8 error_code                      # RS485 protocol error code (0=no error)
float32 current_position_rotations    # Current position in rotations
float32 current_velocity_rps          # Current velocity in revolutions per second
float32 target_velocity_rps           # Target velocity in revolutions per second
float32 acceleration_rps2             # Acceleration setting in rev/s^2
uint32 timestamp_ms                   # Timestamp in milliseconds
