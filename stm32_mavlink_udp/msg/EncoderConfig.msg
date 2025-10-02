# Encoder configuration message
uint8 encoder_id        # Encoder ID
uint16 cpr              # Counts per revolution
bool invert_a           # Invert A signal
bool invert_b           # Invert B signal
bool use_z              # Use Z index signal
uint32 watchdog_timeout_ms  # Watchdog timeout in milliseconds
int32 offset_counts     # Position offset
bool wrap_around        # Enable position wrap-around