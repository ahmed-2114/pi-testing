std_msgs/Header header
string mode
uint32 seq

bool imu_ok
float32 yaw_deg
float32 raw_yaw_deg
float32 gyro_dps
float32 gyro_filt_dps

float32 forward_cm
float32 strafe_cm
float32 progress_cm
float32 remaining_cm
float32 local_forward_cm
float32 local_strafe_cm

uint8 move_phase
float32 move_angle_deg
float32 move_distance_cm
float32 heading_target_deg
float32 heading_error_deg
bool move_done

float32[4] rpm
float32[4] target_rpm
int32[4] pwm
int32[4] raw_encoder_counts
int32[4] signed_encoder_counts

string raw_json
