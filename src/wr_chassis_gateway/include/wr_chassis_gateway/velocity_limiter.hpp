// 文件用途：对上位机底盘速度执行限幅、加速度限制与模式约束。
// 轨内横移和角速度必须被置零；各限制值通过 chassis_gateway.yaml 调整。
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
