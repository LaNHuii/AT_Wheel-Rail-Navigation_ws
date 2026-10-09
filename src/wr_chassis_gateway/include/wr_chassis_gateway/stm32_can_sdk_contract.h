#ifndef WR_CHASSIS_GATEWAY__STM32_CAN_SDK_CONTRACT_H_
#define WR_CHASSIS_GATEWAY__STM32_CAN_SDK_CONTRACT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dedicated 11-bit CAN-FD identifiers. Do not place this protocol on motor buses. */
#define WR_CAN_ID_CHASSIS_COMMAND       0x100U
#define WR_CAN_ID_STEERING_TASK         0x101U
#define WR_CAN_ID_CONTROL_COMMAND       0x102U
#define WR_CAN_ID_TIME_SYNC_REQUEST     0x103U
#define WR_CAN_ID_CHASSIS_STATE         0x180U
#define WR_CAN_ID_MODULE_FL             0x181U
#define WR_CAN_ID_MODULE_FR             0x182U
#define WR_CAN_ID_MODULE_RR             0x183U
#define WR_CAN_ID_MODULE_RL             0x184U
#define WR_CAN_ID_IMU_RAW               0x185U
#define WR_CAN_ID_IMU_ORIENTATION       0x186U
#define WR_CAN_ID_STEERING_TASK_STATE   0x187U
#define WR_CAN_ID_COMMAND_ACK           0x188U
#define WR_CAN_ID_FAULT_EVENT           0x189U
#define WR_CAN_ID_HEARTBEAT             0x18AU
#define WR_CAN_ID_TIME_SYNC_RESPONSE    0x18BU

#define WR_CAN_PROTOCOL_VERSION 1U
#define WR_CAN_MAX_DATA_LENGTH 64U

#define WR_MODULE_FL 0x01U
#define WR_MODULE_FR 0x02U
#define WR_MODULE_RR 0x04U
#define WR_MODULE_RL 0x08U
#define WR_MODULE_FRONT_PAIR 0x03U
#define WR_MODULE_ALL 0x0FU

typedef enum WrControlSource {
  WR_SOURCE_NONE = 0,
  WR_SOURCE_MANUAL = 1,
  WR_SOURCE_AUTONOMOUS = 2,
  WR_SOURCE_ESTOP = 3
} WrControlSource;

typedef enum WrChassisMode {
  WR_MODE_FREE_SURFACE = 0,
  WR_MODE_RAIL_ENTRY = 1,
  WR_MODE_ON_RAIL = 2,
  WR_MODE_RAIL_EXIT = 3
} WrChassisMode;

typedef enum WrSteeringTaskAction {
  WR_TASK_IDLE = 0,
  WR_TASK_PROBE = 1,
  WR_TASK_CANCEL = 2,
  WR_TASK_RELEASE = 3,
  WR_TASK_HOLD = 4
} WrSteeringTaskAction;

typedef struct WrCanFrame {
  uint32_t can_id;
  uint8_t length;
  uint8_t data[WR_CAN_MAX_DATA_LENGTH];
} WrCanFrame;

/* Semantic structures. They must be explicitly encoded; never memcpy them to CAN. */
typedef struct WrChassisCommand {
  uint32_t frame_sequence;
  uint32_t session_id;
  float vx_mps;
  float vy_mps;
  float wz_radps;
  uint16_t valid_for_ms;
} WrChassisCommand;

typedef struct WrControlCommand {
  uint32_t frame_sequence;
  uint32_t session_id;
  uint8_t requested_control_source;
  uint8_t requested_mode;
  bool enable;
  bool quick_stop;
  bool clear_fault;
} WrControlCommand;

typedef struct WrSteeringTaskCommand {
  uint32_t frame_sequence;
  uint32_t task_sequence;
  uint32_t session_id;
  uint8_t action;
  uint8_t module_mask;
  float max_angle_rad;
  float angular_speed_radps;
  float timeout_s;
} WrSteeringTaskCommand;

typedef struct WrMainImuSample {
  uint64_t sample_time_us;
  float angular_velocity_radps[3];
  float linear_acceleration_mps2[3];
  float orientation_xyzw[4];
  float temperature_c;
  bool gyro_valid;
  bool accelerometer_valid;
  bool orientation_valid;
  bool calibrated;
  uint32_t fault_bits;
} WrMainImuSample;

typedef struct WrCanSdkConfig {
  uint32_t boot_id;
  uint32_t command_timeout_ms;
  uint32_t chassis_state_period_us;
  uint32_t module_state_period_us;
  uint32_t imu_period_us;
} WrCanSdkConfig;

/* Functions implemented by the STM32 protocol module. */
bool wr_can_sdk_init(const WrCanSdkConfig * config);
void wr_can_sdk_on_rx_frame(const WrCanFrame * frame, uint64_t receive_time_us);
void wr_can_sdk_tick(uint64_t now_us);
bool wr_can_sdk_pop_tx_frame(WrCanFrame * frame);

/* Hooks implemented by the existing STM32 chassis application. */
void wr_chassis_apply_velocity(const WrChassisCommand * command);
void wr_chassis_apply_control(const WrControlCommand * command);
void wr_chassis_apply_steering_task(const WrSteeringTaskCommand * command);
void wr_chassis_force_safe_stop(void);
bool wr_chassis_read_main_imu(WrMainImuSample * sample);

#ifdef __cplusplus
}
#endif

#endif  /* WR_CHASSIS_GATEWAY__STM32_CAN_SDK_CONTRACT_H_ */
