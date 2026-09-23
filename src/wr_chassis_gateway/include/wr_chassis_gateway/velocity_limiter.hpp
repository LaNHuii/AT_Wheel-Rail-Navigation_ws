#pragma once

#include <array>

namespace wr_chassis_gateway
{

struct VelocityCommand
{
  double vx;
  double vy;
  double wz;
};

struct LimitResult
{
  VelocityCommand command;
  bool input_valid;
  bool limited;
};

class VelocityLimiter
{
public:
  VelocityLimiter(double max_vx, double max_vy, double max_wz);
  LimitResult apply(const VelocityCommand & input) const;

private:
  std::array<double, 3> limits_;
};

}  // namespace wr_chassis_gateway
