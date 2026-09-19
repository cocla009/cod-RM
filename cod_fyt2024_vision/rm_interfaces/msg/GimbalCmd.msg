std_msgs/Header header
float64 pitch
float64 yaw
float64 yaw_diff
float64 pitch_diff
float64 distance
float64 yaw_velocity
float64 yaw_acceleration
float64 pitch_velocity
float64 pitch_acceleration
float64 yaw_jerk
float64 pitch_jerk
bool mpc_valid
bool fire_advice
# Command contract: 0=position-compatible, 1=jerk-controlled.
uint8 control_mode
uint64 command_sequence
float64 command_dt
