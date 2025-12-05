# RS485 Read Response Message (MAVLink ID 12008)
# Response containing data read from RS485 motor control table

uint8 motor_id       # Motor ID (30-49 for RS485 motors)
uint16 address       # Control table address that was read
uint8 length         # Number of bytes read
uint8[64] data       # Read data (up to 64 bytes, little-endian)
uint8 status         # Status code (0=success, see error_code_t)
uint8 rs485_error    # RS485 protocol error code (0=no error)
