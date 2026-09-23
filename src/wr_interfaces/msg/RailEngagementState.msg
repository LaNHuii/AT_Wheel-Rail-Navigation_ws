uint8 UNKNOWN=0
uint8 DISENGAGED=1
uint8 VERIFYING=2
uint8 FULLY_ENGAGED=3
uint8 FRONT_PARTIAL=4
uint8 REAR_PARTIAL=5
uint8 LEFT_PARTIAL=6
uint8 RIGHT_PARTIAL=7
uint8 OBSTRUCTED=8
uint8 SENSOR_FAULT=9

std_msgs/Header header
uint8 state
float32 confidence
float32[4] steering_torque
float32[4] steering_angle_error
string reason

