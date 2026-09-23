uint8 UNKNOWN=0
uint8 PASSABLE=1
uint8 BLOCKED=2

std_msgs/Header header
geometry_msgs/PoseWithCovariance entry_pose
geometry_msgs/Vector3 rail_direction
float32 rail_spacing
float32 visible_length
float32 lateral_error
float32 heading_error
float32 confidence
uint8 traversability
string reason

