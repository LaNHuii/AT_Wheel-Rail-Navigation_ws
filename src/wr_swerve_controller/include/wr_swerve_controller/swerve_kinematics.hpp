#pragma once

#include <array>
#include <cstddef>

namespace wr_swerve_controller
{

struct ModuleLocation
{
  double x;
  double y;
};

struct ModuleCommand
{
  double steering_angle;
  double wheel_angular_velocity;
};

class SwerveKinematics
{
public:
  static constexpr std::size_t kModuleCount = 4;
  using Locations = std::array<ModuleLocation, kModuleCount>;
  using SteeringAngles = std::array<double, kModuleCount>;
  using Commands = std::array<ModuleCommand, kModuleCount>;

  SwerveKinematics(
    Locations locations,
    double wheel_radius,
    double steering_limit,
    double max_wheel_angular_velocity);

  Commands calculate(
    double linear_x,
    double linear_y,
    double angular_z,
    const SteeringAngles & current_angles) const;

  Commands recenter(
    const SteeringAngles & current_angles,
    double maximum_step) const;

private:
  ModuleCommand choose_feasible_command(
    double velocity_x,
    double velocity_y,
    double current_angle) const;

  Locations locations_;
  double wheel_radius_;
  double steering_limit_;
  double max_wheel_angular_velocity_;
};

}  // namespace wr_swerve_controller
