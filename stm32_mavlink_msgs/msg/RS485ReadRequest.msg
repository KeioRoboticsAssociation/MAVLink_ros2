# RS485 Read Request Message (MAVLink ID 12007)
# Request to read parameters from RS485 motor control table

uint8 motor_id       # Motor ID (30-49 for RS485 motors)
uint16 address       # Control table address to read
uint8 length         # Number of bytes to read (1-64)
