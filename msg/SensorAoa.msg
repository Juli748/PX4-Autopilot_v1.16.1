uint64 timestamp         # [us] Time of publication (since system start)
uint64 timestamp_sample  # [us] Time of raw data capture

uint32 device_id         # [-] Unique device ID for the sensor that does not change between power cycles
uint32 error_count       # [-] Number of errors detected by driver

uint16 raw_angle         # [-] Raw 12-bit angle count from the AS5600 hall sensor (0..4095)
float32 angle_rad        # [rad] AoA angle after optional parameter-based calibration/interpolation, wrapped to [-pi, pi]
float32 angle_deg        # [deg] AoA angle after optional parameter-based calibration/interpolation, wrapped to [-180, 180]
uint8 status             # [-] Raw AS5600 STATUS register
uint8 agc                # [-] AS5600 automatic gain control value
uint16 magnitude         # [-] Raw AS5600 magnetic magnitude (0..4095)
bool magnet_detected     # True if the AS5600 reports a valid magnet detected
bool magnet_too_weak     # True if the AS5600 reports the magnet field as too weak
bool magnet_too_strong   # True if the AS5600 reports the magnet field as too strong
