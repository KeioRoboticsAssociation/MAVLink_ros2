# Servo state feedback message
std_msgs/Header header
uint8 servo_id          # Servo ID
float32 current_angle_deg   # Current angle in degrees
float32 target_angle_deg    # Target angle in degrees
uint16 pulse_us         # Current PWM pulse width
bool enabled            # Servo enabled state
uint8 status            # Status code (0=OK, 1=NOT_INITIALIZED, 2=TIMER_ERROR, 3=OUT_OF_RANGE, 4=TIMEOUT, 5=CONFIG_ERROR)
uint32 error_count      # Total error count