# One detector decision, stamped at acquisition/window end in the ROS clock domain.
# source_id identifies a producer session. sequence and stamp strictly increase.
# Invalid, stale, missing, repeated, or discontinuous decisions cannot establish recovery.
std_msgs/Header header
uint32 schema_version
string source_id
uint64 sequence
bool window_valid
bool gps_outage
bool odometry_scale
bool imu_drift
bool uav_loss
