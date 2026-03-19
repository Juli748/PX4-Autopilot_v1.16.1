uint64 timestamp         # [us] Time of publication (since system start)
uint64 timestamp_sample  # [us] Time of raw data capture

uint32 device_id         # [-] Unique device ID for the sensor that does not change between power cycles
uint32 error_count       # [-] Number of errors detected by driver

uint16 raw_angle         # [-] Raw 12-bit angle count from the AS5600 hall sensor (0..4095)
float32 angle_rad        # [rad] Angle converted from raw count
float32 angle_deg        # [deg] Angle converted from raw count
