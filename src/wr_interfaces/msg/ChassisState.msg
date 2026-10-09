uint8 SOURCE_NONE=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTONOMOUS=2
uint8 SOURCE_ESTOP=3

uint8 MODE_FREE_SURFACE=0
uint8 MODE_RAIL_ENTRY=1
uint8 MODE_ON_RAIL=2
uint8 MODE_RAIL_EXIT=3

std_msgs/Header header
uint32 boot_id
uint32 last_command_sequence_id
uint8 active_control_source
uint8 active_mode
bool enabled
bool physical_estop
bool remote_online
bool command_timed_out
bool motor_power_ready
geometry_msgs/Twist measured_twist
uint32 chassis_fault_bits
float32 battery_voltage
float32 battery_current
ModuleState[4] modules
