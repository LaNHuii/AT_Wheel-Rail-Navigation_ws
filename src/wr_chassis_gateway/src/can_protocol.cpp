// 文件用途：实现底盘 CAN-FD 命令和状态帧的严格编解码。
// 本层只处理协议字节，不在此处实现任务状态机或电机保护。
#include "wr_chassis_gateway/can_protocol.hpp"

#include <cstring>
#include <type_traits>

namespace wr_chassis_gateway
{
namespace can_protocol
{
namespace
{

constexpr std::size_t kHeaderSize = 8;

void put_u16(CanFdFrame & frame, std::size_t offset, std::uint16_t value)
{
  frame.data[offset] = static_cast<std::uint8_t>(value);
  frame.data[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void put_u32(CanFdFrame & frame, std::size_t offset, std::uint32_t value)
{
  for (std::size_t index = 0; index < 4; ++index) {
    frame.data[offset + index] = static_cast<std::uint8_t>(value >> (8U * index));
  }
}

void put_u64(CanFdFrame & frame, std::size_t offset, std::uint64_t value)
{
  for (std::size_t index = 0; index < 8; ++index) {
    frame.data[offset + index] = static_cast<std::uint8_t>(value >> (8U * index));
  }
}

void put_float(CanFdFrame & frame, std::size_t offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "CAN protocol requires IEEE-754 float32");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_u32(frame, offset, bits);
}

std::uint16_t get_u16(const CanFdFrame & frame, std::size_t offset)
{
  return static_cast<std::uint16_t>(frame.data[offset]) |
         static_cast<std::uint16_t>(frame.data[offset + 1]) << 8U;
}

std::uint32_t get_u32(const CanFdFrame & frame, std::size_t offset)
{
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(frame.data[offset + index]) << (8U * index);
  }
  return value;
}

std::uint64_t get_u64(const CanFdFrame & frame, std::size_t offset)
{
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(frame.data[offset + index]) << (8U * index);
  }
  return value;
}

float get_float(const CanFdFrame & frame, std::size_t offset)
{
  const std::uint32_t bits = get_u32(frame, offset);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

CanFdFrame make_frame(
  std::uint32_t id, MessageType type, std::uint32_t sequence, std::uint8_t length)
{
  CanFdFrame frame;
  frame.id = id;
  frame.length = length;
  frame.data[0] = kProtocolVersion;
  frame.data[1] = static_cast<std::uint8_t>(type);
  put_u16(frame, 2, 0);
  put_u32(frame, 4, sequence);
  return frame;
}

bool valid(
  const CanFdFrame & frame, std::uint32_t id, MessageType type, std::uint8_t minimum_length)
{
  return frame.id == id && frame.length >= minimum_length &&
         frame.length <= kMaxDataLength && frame.data[0] == kProtocolVersion &&
         frame.data[1] == static_cast<std::uint8_t>(type);
}

}  // namespace

CanFdFrame encode(const ChassisCommand & value)
{
  auto frame = make_frame(
    kIdChassisCommand, MessageType::kChassisCommand, value.frame_sequence, 28);
  put_float(frame, 8, value.vx);
  put_float(frame, 12, value.vy);
  put_float(frame, 16, value.wz);
  put_u16(frame, 20, value.valid_for_ms);
  put_u32(frame, 24, value.session_id);
  return frame;
}

CanFdFrame encode(const SteeringTaskCommand & value)
{
  auto frame = make_frame(
    kIdSteeringTaskCommand, MessageType::kSteeringTaskCommand,
    value.frame_sequence, 32);
  put_u32(frame, 8, value.task_sequence);
  frame.data[12] = value.action;
  frame.data[13] = value.module_mask;
  put_float(frame, 16, value.max_angle);
  put_float(frame, 20, value.angular_speed);
  put_float(frame, 24, value.timeout);
  put_u32(frame, 28, value.session_id);
  return frame;
}

CanFdFrame encode(const ControlCommand & value)
{
  auto frame = make_frame(
    kIdControlCommand, MessageType::kControlCommand, value.frame_sequence, 16);
  frame.data[8] = value.requested_control_source;
  frame.data[9] = value.requested_mode;
  frame.data[10] = static_cast<std::uint8_t>(value.enable ? 0x01U : 0U) |
    static_cast<std::uint8_t>(value.quick_stop ? 0x02U : 0U) |
    static_cast<std::uint8_t>(value.clear_fault ? 0x04U : 0U);
  put_u32(frame, 12, value.session_id);
  return frame;
}

CanFdFrame encode(const TimeSyncRequest & value)
{
  auto frame = make_frame(
    kIdTimeSyncRequest, MessageType::kTimeSyncRequest, value.frame_sequence, 16);
  put_u64(frame, 8, value.host_send_time_ns);
  return frame;
}

bool decode(const CanFdFrame & frame, ChassisState & value)
{
  if (!valid(frame, kIdChassisState, MessageType::kChassisState, 52)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.sample_time_us = get_u64(frame, 8);
  value.boot_id = get_u32(frame, 16);
  value.last_command_sequence = get_u32(frame, 20);
  value.active_control_source = frame.data[24];
  value.active_mode = frame.data[25];
  const auto flags = frame.data[26];
  value.enabled = flags & 0x01U;
  value.physical_estop = flags & 0x02U;
  value.remote_online = flags & 0x04U;
  value.command_timed_out = flags & 0x08U;
  value.motor_power_ready = flags & 0x10U;
  value.vx = get_float(frame, 28);
  value.vy = get_float(frame, 32);
  value.wz = get_float(frame, 36);
  value.fault_bits = get_u32(frame, 40);
  value.battery_voltage = get_float(frame, 44);
  value.battery_current = get_float(frame, 48);
  return true;
}

bool decode(const CanFdFrame & frame, ModuleState & value)
{
  if (frame.id < kIdModuleStateBase || frame.id > kIdModuleStateBase + 3U ||
    !valid(frame, frame.id, MessageType::kModuleState, 64))
  {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.sample_time_us = get_u64(frame, 8);
  value.module_id = frame.data[16];
  if (value.module_id > 3U || frame.id != kIdModuleStateBase + value.module_id) {
    return false;
  }
  const auto flags = frame.data[17];
  value.steering_online = flags & 0x01U;
  value.drive_online = flags & 0x02U;
  value.steering_zero_valid = flags & 0x04U;
  value.steering_torque_valid = flags & 0x08U;
  value.drive_position_valid = flags & 0x10U;
  value.steering_fault_bits = get_u16(frame, 18);
  value.drive_fault_bits = get_u16(frame, 20);
  value.steering_angle = get_float(frame, 24);
  value.steering_velocity = get_float(frame, 28);
  value.steering_current = get_float(frame, 32);
  value.steering_torque = get_float(frame, 36);
  value.drive_position = get_float(frame, 40);
  value.drive_velocity = get_float(frame, 44);
  value.drive_current = get_float(frame, 48);
  value.drive_torque = get_float(frame, 52);
  value.steering_temperature = get_float(frame, 56);
  value.drive_temperature = get_float(frame, 60);
  return true;
}

bool decode(const CanFdFrame & frame, ImuRaw & value)
{
  if (!valid(frame, kIdImuRaw, MessageType::kImuRaw, 48)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.sample_time_us = get_u64(frame, 8);
  for (std::size_t i = 0; i < 3; ++i) {
    value.angular_velocity[i] = get_float(frame, 16 + i * 4);
    value.linear_acceleration[i] = get_float(frame, 28 + i * 4);
  }
  value.temperature = get_float(frame, 40);
  value.status_bits = get_u32(frame, 44);
  return true;
}

bool decode(const CanFdFrame & frame, ImuOrientation & value)
{
  if (!valid(frame, kIdImuOrientation, MessageType::kImuOrientation, 36)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.sample_time_us = get_u64(frame, 8);
  for (std::size_t i = 0; i < 4; ++i) {
    value.quaternion[i] = get_float(frame, 16 + i * 4);
  }
  value.status_bits = get_u32(frame, 32);
  return true;
}

bool decode(const CanFdFrame & frame, SteeringTaskState & value)
{
  if (!valid(frame, kIdSteeringTaskState, MessageType::kSteeringTaskState, 20)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.task_sequence = get_u32(frame, 8);
  value.action = frame.data[12];
  value.state = frame.data[13];
  value.active_mask = frame.data[14];
  value.holding_mask = frame.data[15];
  value.completed_mask = frame.data[16];
  value.safety_stopped_mask = frame.data[17];
  value.result_code = get_u16(frame, 18);
  return true;
}

bool decode(const CanFdFrame & frame, CommandAck & value)
{
  if (!valid(frame, kIdCommandAck, MessageType::kCommandAck, 17)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.acknowledged_sequence = get_u32(frame, 8);
  value.message_type = frame.data[12];
  value.accepted = frame.data[13] != 0;
  value.result_code = get_u16(frame, 14);
  value.active_module_mask = frame.data[16];
  return true;
}

bool decode(const CanFdFrame & frame, TimeSyncResponse & value)
{
  if (!valid(frame, kIdTimeSyncResponse, MessageType::kTimeSyncResponse, 32)) {
    return false;
  }
  value.frame_sequence = get_u32(frame, 4);
  value.host_send_time_ns = get_u64(frame, 8);
  value.mcu_receive_time_us = get_u64(frame, 16);
  value.mcu_send_time_us = get_u64(frame, 24);
  return true;
}

}  // namespace can_protocol
}  // namespace wr_chassis_gateway
