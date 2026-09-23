#include "wr_chassis_gateway/velocity_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wr_chassis_gateway
{

VelocityLimiter::VelocityLimiter(double max_vx, double max_vy, double max_wz)
: limits_{max_vx, max_vy, max_wz}
{
  for (const double limit : limits_) {
    if (!std::isfinite(limit) || limit <= 0.0) {
      throw std::invalid_argument("velocity limits must be finite and positive");
    }
  }
}

LimitResult VelocityLimiter::apply(const VelocityCommand & input) const
{
  const std::array<double, 3> values{input.vx, input.vy, input.wz};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return {{0.0, 0.0, 0.0}, false, false};
    }
  }

  double scale = 1.0;
  for (std::size_t index = 0; index < values.size(); ++index) {
    const double magnitude = std::abs(values[index]);
    if (magnitude > limits_[index]) {
      scale = std::min(scale, limits_[index] / magnitude);
    }
  }
  return {{input.vx * scale, input.vy * scale, input.wz * scale}, true, scale < 1.0};
}

}  // namespace wr_chassis_gateway
