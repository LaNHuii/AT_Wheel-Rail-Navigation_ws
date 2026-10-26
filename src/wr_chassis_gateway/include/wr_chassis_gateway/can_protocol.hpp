// 文件用途：定义上位机与 STM32 之间的 CAN-FD 帧编解码数据结构。
// 缩放系数、字段单位和故障位必须与 docs/can_bt_fsm_stm32_interface.md 保持一致。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wr_chassis_gateway
{
namespace can_protocol
{

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaxDataLength = 64;

constexpr std::uint32_t kIdChassisCommand = 0x100;
constexpr std::uint32_t kIdSteeringTaskCommand = 0x101;
constexpr std::uint32_t kIdControlCommand = 0x102;
constexpr std::uint32_t kIdTimeSyncRequest = 0x103;
constexpr std::uint32_t kIdChassisState = 0x180;
constexpr std::uint32_t kIdModuleStateBase = 0x181;  // FL, FR, RR, RL
constexpr std::uint32_t kIdImuRaw = 0x185;
constexpr std::uint32_t kIdImuOrientation = 0x186;
constexpr std::uint32_t kIdSteeringTaskState = 0x187;
constexpr std::uint32_t kIdCommandAck = 0x188;
constexpr std::uint32_t kIdFaultEvent = 0x189;
constexpr std::uint32_t kIdHeartbeat = 0x18A;
constexpr std::uint32_t kIdTimeSyncResponse = 0x18B;

enum class MessageType : std::uint8_t
{
  kChassisCommand = 0x10,
  kSteeringTaskCommand = 0x11,
  kControlCommand = 0x12,
  kTimeSyncRequest = 0x13,
  kChassisState = 0x20,
  kModuleState = 0x21,
  kImuRaw = 0x22,
  kImuOrientation = 0x23,
  kSteeringTaskState = 0x24,
  kCommandAck = 0x25,
  kFaultEvent = 0x26,
  kHeartbeat = 0x27,
  kTimeSyncResponse = 0x28,
};

struct CanFdFrame
{
  std::uint32_t id{0};
  std::uint8_t length{0};
  std::array<std::uint8_t, kMaxDataLength> data{};
};

struct ChassisCommand
{
  std::uint32_t frame_sequence{0};
  std::uint32_t session_id{0};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  std::uint16_t valid_for_ms{200};
};

struct SteeringTaskCommand
{
  std::uint32_t frame_sequence{0};
  std::uint32_t task_sequence{0};
  std::uint32_t session_id{0};
  std::uint8_t action{0};
  std::uint8_t module_mask{0};
  float max_angle{0.0F};
  float angular_speed{0.0F};
  float timeout{0.0F};
};

struct ControlCommand
{
  std::uint32_t frame_sequence{0};
  std::uint32_t session_id{0};
  std::uint8_t requested_control_source{0};
  std::uint8_t requested_mode{0};
  bool enable{false};
  bool quick_stop{false};
  bool clear_fault{false};
};

struct ChassisState
{
  std::uint32_t frame_sequence{0};
  std::uint64_t sample_time_us{0};
  std::uint32_t boot_id{0};
  std::uint32_t last_command_sequence{0};
  std::uint8_t active_control_source{0};
  std::uint8_t active_mode{0};
  bool enabled{false};
  bool physical_estop{false};
  bool remote_online{false};
  bool command_timed_out{false};
  bool motor_power_ready{false};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  std::uint32_t fault_bits{0};
  float battery_voltage{0.0F};
  float battery_current{0.0F};
};

struct ModuleState
{
  std::uint32_t frame_sequence{0};
  std::uint64_t sample_time_us{0};
  std::uint8_t module_id{0};
  bool steering_online{false};
  bool drive_online{false};
  bool steering_zero_valid{false};
  bool steering_torque_valid{false};
  bool drive_position_valid{false};
  std::uint16_t steering_fault_bits{0};
  std::uint16_t drive_fault_bits{0};
  float steering_angle{0.0F};
  float steering_velocity{0.0F};
  float steering_current{0.0F};
  float steering_torque{0.0F};
  float drive_position{0.0F};
  float drive_velocity{0.0F};
  float drive_current{0.0F};
  float drive_torque{0.0F};
  float steering_temperature{0.0F};
  float drive_temperature{0.0F};
};

struct ImuRaw
{
  std::uint32_t frame_sequence{0};
  std::uint64_t sample_time_us{0};
  std::array<float, 3> angular_velocity{};
  std::array<float, 3> linear_acceleration{};
  float temperature{0.0F};
  std::uint32_t status_bits{0};
};

struct ImuOrientation
{
  std::uint32_t frame_sequence{0};
  std::uint64_t sample_time_us{0};
  std::array<float, 4> quaternion{{0.0F, 0.0F, 0.0F, 1.0F}};  // x,y,z,w
  std::uint32_t status_bits{0};
};

struct SteeringTaskState
{
  std::uint32_t frame_sequence{0};
  std::uint32_t task_sequence{0};
  std::uint8_t action{0};
  std::uint8_t state{0};
  std::uint8_t active_mask{0};
  std::uint8_t holding_mask{0};
  std::uint8_t completed_mask{0};
  std::uint8_t safety_stopped_mask{0};
  std::uint16_t result_code{0};
};

struct CommandAck
{
  std::uint32_t frame_sequence{0};
  std::uint32_t acknowledged_sequence{0};
  std::uint8_t message_type{0};
  bool accepted{false};
  std::uint16_t result_code{0};
  std::uint8_t active_module_mask{0};
};

struct TimeSyncRequest
{
  std::uint32_t frame_sequence{0};
  std::uint64_t host_send_time_ns{0};
};

struct TimeSyncResponse
{
  std::uint32_t frame_sequence{0};
  std::uint64_t host_send_time_ns{0};
  std::uint64_t mcu_receive_time_us{0};
  std::uint64_t mcu_send_time_us{0};
};

CanFdFrame encode(const ChassisCommand & value);
CanFdFrame encode(const SteeringTaskCommand & value);
CanFdFrame encode(const ControlCommand & value);
CanFdFrame encode(const TimeSyncRequest & value);

bool decode(const CanFdFrame & frame, ChassisState & value);
bool decode(const CanFdFrame & frame, ModuleState & value);
bool decode(const CanFdFrame & frame, ImuRaw & value);
bool decode(const CanFdFrame & frame, ImuOrientation & value);
bool decode(const CanFdFrame & frame, SteeringTaskState & value);
bool decode(const CanFdFrame & frame, CommandAck & value);
bool decode(const CanFdFrame & frame, TimeSyncResponse & value);

}  // namespace can_protocol
}  // namespace wr_chassis_gateway
