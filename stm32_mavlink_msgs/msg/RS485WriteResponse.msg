# RS485 Write Response Message (MAVLink ID 12010)
# Response confirming write success or failure

uint8 motor_id       # Motor ID (30-49 for RS485 motors)
uint16 address       # Control table address that was written
uint8 length         # Number of bytes written
uint8 status         # Status code (0=success, see error_code_t)
uint8 rs485_error    # RS485 protocol error code (0=no error)
