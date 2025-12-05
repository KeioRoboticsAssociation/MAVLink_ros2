# RS485 Write Request Message (MAVLink ID 12009)
# Request to write parameters to RS485 motor control table

uint8 motor_id       # Motor ID (30-49 for RS485 motors)
uint16 address       # Control table address to write
uint8 length         # Number of bytes to write (1-64)
uint8[64] data       # Data to write (up to 64 bytes, little-endian)
