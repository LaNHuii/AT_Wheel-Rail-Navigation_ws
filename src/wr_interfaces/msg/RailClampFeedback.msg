uint8 IDLE=0
uint8 PROBING=1
uint8 PARTIAL_CONTACT=2
uint8 ENGAGED=3
uint8 TIMEOUT=4
uint8 OVERTORQUE=5
uint8 SENSOR_FAULT=6
uint8 CANCELLED=7
uint8 RELEASING=8

std_msgs/Header header
uint32 sequence_id
uint8 state
bool[4] contact_confirmed
float32[4] steering_angle
float32[4] steering_velocity
float32[4] steering_torque
string reason
