#include <cmath>
#include <cstdint>
#include <cstring>

#include "gtest/gtest.h"
#include "wr_chassis_gateway/can_protocol.hpp"

namespace cp = wr_chassis_gateway::can_protocol;

TEST(CanProtocol, EncodesChassisCommandWithStableLayout)
{
  cp::ChassisCommand command;
  command.frame_sequence = 0x11223344U;
  command.session_id = 0x55667788U;
  command.vx = 0.25F;
  command.vy = -0.125F;
  command.wz = 0.5F;
  const auto frame = cp::encode(command);
  EXPECT_EQ(frame.id, cp::kIdChassisCommand);
  EXPECT_EQ(frame.length, 28);
  EXPECT_EQ(frame.data[0], cp::kProtocolVersion);
  EXPECT_EQ(frame.data[1], static_cast<std::uint8_t>(cp::MessageType::kChassisCommand));
  EXPECT_EQ(frame.data[4], 0x44);
  EXPECT_EQ(frame.data[7], 0x11);
  EXPECT_EQ(frame.data[24], 0x88);
  EXPECT_EQ(frame.data[27], 0x55);
}

TEST(CanProtocol, DecodesMainImuRawFrame)
{
  cp::ImuRaw expected;
  expected.frame_sequence = 9;
  expected.sample_time_us = 1234567;
  expected.angular_velocity = {0.1F, -0.2F, 0.3F};
  expected.linear_acceleration = {1.0F, 2.0F, 9.81F};
  expected.temperature = 32.5F;
  expected.status_bits = 0x05;

  cp::CanFdFrame frame;
  frame.id = cp::kIdImuRaw;
  frame.length = 48;
  frame.data[0] = cp::kProtocolVersion;
  frame.data[1] = static_cast<std::uint8_t>(cp::MessageType::kImuRaw);
  auto put_u32 = [&frame](std::size_t offset, std::uint32_t value) {
      for (std::size_t i = 0; i < 4; ++i) {
        frame.data[offset + i] = static_cast<std::uint8_t>(value >> (8U * i));
      }
    };
  auto put_u64 = [&frame](std::size_t offset, std::uint64_t value) {
      for (std::size_t i = 0; i < 8; ++i) {
        frame.data[offset + i] = static_cast<std::uint8_t>(value >> (8U * i));
      }
    };
  auto put_float = [&put_u32](std::size_t offset, float value) {
      std::uint32_t bits = 0;
      std::memcpy(&bits, &value, sizeof(bits));
      put_u32(offset, bits);
    };
  put_u32(4, expected.frame_sequence);
  put_u64(8, expected.sample_time_us);
  for (std::size_t i = 0; i < 3; ++i) {
    put_float(16 + i * 4, expected.angular_velocity[i]);
    put_float(28 + i * 4, expected.linear_acceleration[i]);
  }
  put_float(40, expected.temperature);
  put_u32(44, expected.status_bits);

  cp::ImuRaw actual;
  ASSERT_TRUE(cp::decode(frame, actual));
  EXPECT_EQ(actual.frame_sequence, expected.frame_sequence);
  EXPECT_EQ(actual.sample_time_us, expected.sample_time_us);
  EXPECT_NEAR(actual.angular_velocity[1], -0.2, 1e-6);
  EXPECT_NEAR(actual.linear_acceleration[2], 9.81, 1e-6);
  EXPECT_NEAR(actual.temperature, 32.5, 1e-6);
  EXPECT_EQ(actual.status_bits, 0x05U);
}

TEST(CanProtocol, RejectsWrongVersionAndModuleIdMismatch)
{
  cp::CanFdFrame frame;
  frame.id = cp::kIdModuleStateBase;
  frame.length = 64;
  frame.data[0] = cp::kProtocolVersion + 1;
  frame.data[1] = static_cast<std::uint8_t>(cp::MessageType::kModuleState);
  cp::ModuleState state;
  EXPECT_FALSE(cp::decode(frame, state));
  frame.data[0] = cp::kProtocolVersion;
  frame.data[16] = 1;
  EXPECT_FALSE(cp::decode(frame, state));
}

TEST(CanProtocol, SteeringTaskCarriesIndependentFrameAndTaskSequences)
{
  cp::SteeringTaskCommand task;
  task.frame_sequence = 101;
  task.task_sequence = 7;
  task.action = 4;
  task.module_mask = 0x01;
  task.max_angle = 0.174532925F;
  task.angular_speed = 0.087266463F;
  task.timeout = 5.0F;
  const auto frame = cp::encode(task);
  EXPECT_EQ(frame.id, cp::kIdSteeringTaskCommand);
  EXPECT_EQ(frame.data[4], 101);
  EXPECT_EQ(frame.data[8], 7);
  EXPECT_EQ(frame.data[12], 4);
  EXPECT_EQ(frame.data[13], 1);
}
