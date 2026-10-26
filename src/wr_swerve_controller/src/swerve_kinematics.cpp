// 文件用途：实现四舍轮逆运动学、最短转向等价化和转向限位。
#include "wr_swerve_controller/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wr_swerve_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStoppedEpsilon = 1.0e-8;
}

SwerveKinematics::SwerveKinematics(
  Locations locations,
  double wheel_radius,
  double steering_limit,
  double max_wheel_angular_velocity)
: locations_(locations),
  wheel_radius_(wheel_radius),
  steering_limit_(steering_limit),
  max_wheel_angular_velocity_(max_wheel_angular_velocity)
{
  if (wheel_radius_ <= 0.0 || steering_limit_ <= 0.0 ||
    max_wheel_angular_velocity_ <= 0.0)
  {
    throw std::invalid_argument("Swerve geometry and velocity limits must be positive");
  }
}

ModuleCommand SwerveKinematics::choose_feasible_command(
  double velocity_x,
  double velocity_y,
  double current_angle) const
{
  const double speed = std::hypot(velocity_x, velocity_y);
  if (speed < kStoppedEpsilon) {
    return {std::clamp(current_angle, -steering_limit_, steering_limit_), 0.0};
  }

  const double base_angle = std::atan2(velocity_y, velocity_x);
  double best_cost = std::numeric_limits<double>::infinity();
  ModuleCommand best{std::clamp(base_angle, -steering_limit_, steering_limit_), 0.0};

  // Reversing wheel rotation makes steering angles separated by pi equivalent.
  for (int half_turns = -2; half_turns <= 2; ++half_turns) {
    const double candidate = base_angle + static_cast<double>(half_turns) * kPi;
    if (candidate < -steering_limit_ || candidate > steering_limit_) {
      continue;
    }
    const double cost = std::abs(candidate - current_angle);
    if (cost < best_cost) {
      const bool reverse = std::abs(half_turns) % 2 == 1;
      best = {candidate, reverse ? -speed / wheel_radius_ : speed / wheel_radius_};
      best_cost = cost;
    }
  }

  if (!std::isfinite(best_cost)) {
    throw std::runtime_error("No steering solution exists within the configured limit");
  }
  return best;
}

SwerveKinematics::Commands SwerveKinematics::calculate(
  double linear_x,
  double linear_y,
  double angular_z,
  const SteeringAngles & current_angles) const
{
  Commands commands{};
  double peak_velocity = 0.0;
  for (std::size_t index = 0; index < kModuleCount; ++index) {
    const double module_x = linear_x - angular_z * locations_[index].y;
    const double module_y = linear_y + angular_z * locations_[index].x;
    commands[index] = choose_feasible_command(module_x, module_y, current_angles[index]);
    peak_velocity = std::max(peak_velocity, std::abs(commands[index].wheel_angular_velocity));
  }

  if (peak_velocity > max_wheel_angular_velocity_) {
    const double scale = max_wheel_angular_velocity_ / peak_velocity;
    for (auto & command : commands) {
      command.wheel_angular_velocity *= scale;
    }
  }
  return commands;
}

SwerveKinematics::Commands SwerveKinematics::recenter(
  const SteeringAngles & current_angles,
  double maximum_step) const
{
  if (!std::isfinite(maximum_step) || maximum_step <= 0.0) {
    throw std::invalid_argument("maximum_step must be finite and positive");
  }
  Commands commands{};
  for (std::size_t index = 0; index < kModuleCount; ++index) {
    if (!std::isfinite(current_angles[index])) {
      throw std::invalid_argument("current steering angles must be finite");
    }
    const double current = std::clamp(current_angles[index], -steering_limit_, steering_limit_);
    if (std::abs(current) <= maximum_step) {
      commands[index] = {0.0, 0.0};
    } else {
      commands[index] = {current - std::copysign(maximum_step, current), 0.0};
    }
  }
  return commands;
}

}  // namespace wr_swerve_controller
