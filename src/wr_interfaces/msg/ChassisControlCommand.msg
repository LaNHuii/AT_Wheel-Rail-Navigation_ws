uint8 SOURCE_NONE=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTONOMOUS=2

uint8 MODE_FREE_SURFACE=0
uint8 MODE_RAIL_ENTRY=1
uint8 MODE_ON_RAIL=2
uint8 MODE_RAIL_EXIT=3

std_msgs/Header header
uint32 sequence_id
bool enable
bool quick_stop
bool clear_fault
uint8 requested_control_source
uint8 requested_mode
uint16 valid_for_ms
