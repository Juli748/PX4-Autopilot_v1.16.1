uint64 timestamp         # [us] Time of publication (since system start)
uint64 timestamp_sample  # [us] Latest raw data capture time from either vane

bool aoa_valid           # True when an AOA vane sample has been received recently
bool ssa_valid           # True when an SSA vane sample has been received recently

uint32 aoa_device_id     # [-] AOA vane device ID
uint32 ssa_device_id     # [-] SSA vane device ID
uint32 aoa_error_count   # [-] AOA vane communication error counter
uint32 ssa_error_count   # [-] SSA vane communication error counter

uint16 aoa_raw_angle     # [-] AOA vane raw 12-bit angle count
uint16 ssa_raw_angle     # [-] SSA vane raw 12-bit angle count

float32 aoa_angle_rad    # [rad] AOA vane angle, wrapped to [-pi, pi]
float32 ssa_angle_rad    # [rad] SSA vane angle, wrapped to [-pi, pi]
float32 aoa_angle_deg    # [deg] AOA vane angle, wrapped to [-180, 180]
float32 ssa_angle_deg    # [deg] SSA vane angle, wrapped to [-180, 180]
