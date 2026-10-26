// 测试用途：验证速度限幅、加速度约束以及轨内禁止横移。
#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "wr_chassis_gateway/velocity_limiter.hpp"

using wr_chassis_gateway::VelocityCommand;
using wr_chassis_gateway::VelocityLimiter;

TEST(VelocityLimiter, PassesCommandInsideLimits)
{
  const auto result = VelocityLimiter(0.5, 0.5, 1.0).apply({0.2, -0.1, 0.3});
  EXPECT_TRUE(result.input_valid);
  EXPECT_FALSE(result.limited);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.2);
  EXPECT_DOUBLE_EQ(result.command.vy, -0.1);
  EXPECT_DOUBLE_EQ(result.command.wz, 0.3);
}

TEST(VelocityLimiter, UniformlyScalesCombinedCommand)
{
  const auto result = VelocityLimiter(0.5, 0.5, 1.0).apply({1.0, 0.4, 0.5});
  EXPECT_TRUE(result.input_valid);
  EXPECT_TRUE(result.limited);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.5);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.2);
  EXPECT_DOUBLE_EQ(result.command.wz, 0.25);
}

TEST(VelocityLimiter, RejectsNonFiniteCommand)
{
  const auto result = VelocityLimiter(0.5, 0.5, 1.0).apply(
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_FALSE(result.input_valid);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.command.wz, 0.0);
}

TEST(VelocityLimiter, RejectsInvalidLimits)
{
  EXPECT_THROW(VelocityLimiter(0.0, 0.5, 1.0), std::invalid_argument);
}
