std_msgs/Header header

uint32 FAULT_LIDAR_STALE=1
uint32 FAULT_IMU_STALE=2
uint32 FAULT_CHASSIS_STALE=4
uint32 FAULT_SAFETY_STALE=8
uint32 FAULT_LOCALIZATION_STALE=16
uint32 FAULT_VISION_STALE=32
uint32 FAULT_CHASSIS_UNSAFE=64
uint32 FAULT_SAFETY_STOP=128

bool lidar_fresh
bool imu_fresh
bool chassis_fresh
bool safety_fresh
bool localization_fresh
bool vision_fresh
bool autonomy_ready
uint32 fault_mask
string reason
