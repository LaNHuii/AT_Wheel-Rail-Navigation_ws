uint8 IDLE=0
uint8 START_PROBE=1
uint8 CANCEL=2
uint8 RELEASE=3

uint8 MODULE_FL=1
uint8 MODULE_FR=2
uint8 MODULE_RR=4
uint8 MODULE_RL=8
uint8 FRONT_PAIR=3
uint8 ALL_MODULES=15

std_msgs/Header header
uint32 sequence_id
uint8 action
uint8 module_mask
float32 max_probe_angle
float32 probe_angular_speed
float32 timeout
