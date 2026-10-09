uint8 IDLE=0
uint8 EXECUTING=1
uint8 HOLDING=2
uint8 COMPLETED=3
uint8 CANCELLED=4
uint8 TIMEOUT=5
uint8 SAFETY_STOPPED=6
uint8 FAULT=7

std_msgs/Header header
uint32 task_sequence_id
uint8 action
uint8 state
uint8 active_module_mask
uint8 holding_module_mask
uint8 completed_module_mask
uint8 safety_stopped_module_mask
uint16 result_code
