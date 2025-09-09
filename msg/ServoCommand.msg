# Servo command message
std_msgs/Header header
uint8 servo_id          # Servo ID (1-16)
float32 angle_deg       # Target angle in degrees
uint16 pulse_us         # Direct PWM pulse width in microseconds (0 to use angle)
bool enable             # Enable/disable servo