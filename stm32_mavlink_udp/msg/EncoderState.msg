# Encoder state message
std_msgs/Header header  # Header with timestamp
uint8 encoder_id        # Encoder ID
int32 position          # Current encoder position in counts
float32 angle_rad       # Current angle in radians
float32 angle_deg       # Current angle in degrees
uint32 revolutions      # Number of complete revolutions
bool z_detected         # Z-index pulse detected
uint8 status            # Encoder status
uint32 error_count      # Error count