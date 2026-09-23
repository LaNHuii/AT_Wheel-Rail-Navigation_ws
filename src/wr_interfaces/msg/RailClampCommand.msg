uint8 IDLE=0
uint8 START_PROBE=1
uint8 CANCEL=2
uint8 RELEASE=3

std_msgs/Header header
uint32 sequence_id
uint8 action
float32 max_probe_angle
float32 probe_angular_speed
float32 timeout
