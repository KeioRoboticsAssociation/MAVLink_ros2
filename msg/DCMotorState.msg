# DC Motor State Message
uint8 motor_id          # Motor ID
float32 position_rad    # Current position in radians
float32 velocity_rad_s  # Current velocity in rad/s
float32 current_a       # Current draw in amperes
float32 temperature_c   # Motor temperature in Celsius
uint8 status           # Motor status (0: OK, 1: NOT_INITIALIZED, 2: ERROR, 3: OVERHEAT, 4: OVERCURRENT, 5: TIMEOUT)
bool enabled           # Motor enabled state
uint64 timestamp       # Timestamp of the measurement