#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "wr_swerve_controller/swerve_kinematics.hpp"

namespace
{
constexpr double kWheelRadius = 0.1;
using Kinematics = wr_swerve_controller::SwerveKinematics;

Kinematics make_kinematics(double max_velocity = 20.0)
{
  return Kinematics(
    {{{0.355, 0.335}, {0.355, -0.335}, {-0.355, -0.335}, {-0.355, 0.335}}},
    kWheelRadius, 2.88, max_velocity);
}

TEST(SwerveKinematics, ForwardUsesZeroSteering)
{
  const auto result = make_kinematics().calculate(1.0, 0.0, 0.0, {{0.0, 0.0, 0.0, 0.0}});
  for (const auto & module : result) {
    EXPECT_NEAR(module.steering_angle, 0.0, 1.0e-9);
    EXPECT_NEAR(module.wheel_angular_velocity, 10.0, 1.0e-9);
  }
}

TEST(SwerveKinematics, BackwardReversesWheelInsteadOfTurningPi)
{
  const auto result = make_kinematics().calculate(-0.5, 0.0, 0.0, {{0.0, 0.0, 0.0, 0.0}});
  for (const auto & module : result) {
    EXPECT_NEAR(module.steering_angle, 0.0, 1.0e-9);
    EXPECT_NEAR(module.wheel_angular_velocity, -5.0, 1.0e-9);
  }
}

TEST(SwerveKinematics, LateralMotionProducesEquivalentVelocityWithinLimit)
{
  const auto result = make_kinematics().calculate(0.0, 0.4, 0.0, {{0.0, 0.0, 0.0, 0.0}});
  for (const auto & module : result) {
    const double linear_speed = kWheelRadius * module.wheel_angular_velocity;
    EXPECT_NEAR(linear_speed * std::cos(module.steering_angle), 0.0, 1.0e-9);
    EXPECT_NEAR(linear_speed * std::sin(module.steering_angle), 0.4, 1.0e-9);
    EXPECT_LE(std::abs(module.steering_angle), 2.88);
  }
}

TEST(SwerveKinematics, PureYawProducesSymmetricModuleSpeeds)
{
  const auto result = make_kinematics().calculate(0.0, 0.0, 0.5, {{0.0, 0.0, 0.0, 0.0}});
  const double magnitude = std::hypot(0.335, 0.355) * 0.5 / kWheelRadius;
  for (const auto & module : result) {
    EXPECT_NEAR(std::abs(module.wheel_angular_velocity), magnitude, 1.0e-9);
    EXPECT_LE(std::abs(module.steering_angle), 2.88);
  }
}

TEST(SwerveKinematics, ScalesAllWheelsTogetherAtVelocityLimit)
{
  const auto result = make_kinematics(3.0).calculate(1.0, 0.0, 0.0, {{0.0, 0.0, 0.0, 0.0}});
  for (const auto & module : result) {
    EXPECT_NEAR(module.wheel_angular_velocity, 3.0, 1.0e-9);
  }
}

TEST(SwerveKinematics, RecentersAtBoundedRateWithWheelsStopped)
{
  const auto result = make_kinematics().recenter({{0.5, -0.2, 0.01, 0.0}}, 0.1);
  EXPECT_NEAR(result[0].steering_angle, 0.4, 1.0e-9);
  EXPECT_NEAR(result[1].steering_angle, -0.1, 1.0e-9);
  EXPECT_NEAR(result[2].steering_angle, 0.0, 1.0e-9);
  EXPECT_NEAR(result[3].steering_angle, 0.0, 1.0e-9);
  for (const auto & module : result) {
    EXPECT_DOUBLE_EQ(module.wheel_angular_velocity, 0.0);
  }
}

TEST(SwerveKinematics, RejectsInvalidGeometry)
{
  EXPECT_THROW(
    Kinematics(
      {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}},
      0.0, 1.0, 1.0),
    std::invalid_argument);
}
}  // namespace
