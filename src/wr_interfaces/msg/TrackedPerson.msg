uint8 NONE=0
uint8 TRACKED=1
uint8 TEMPORARILY_LOST=2
uint8 IDENTITY_UNCERTAIN=3
uint8 SENSOR_FAULT=4

uint8 ACTIVITY_UNKNOWN=0
uint8 MOVING=1
uint8 STATIONARY=2
uint8 DEPOSITING=3

# The visual pipeline owns identity association. binding_id changes whenever a
# replacement person is accepted; tracker_id may change without rebinding.
std_msgs/Header header
uint32 session_id
uint32 binding_id
int64 tracker_id
uint8 tracking_state
uint8 activity
geometry_msgs/Point position
float32[9] position_covariance
float32 nearest_body_distance
float32 longitudinal_distance
float32 lateral_offset
float32 detection_confidence
float32 identity_confidence
float32 depth_valid_fraction
bool original_target
string reason
