// 文件用途：对仲裁后的底盘速度再执行限幅、超时和模式约束。
// 限速与超时参数在 config/chassis_gateway.yaml 中调整，STM32 仍保留最终硬保护权。
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wr_chassis_gateway/velocity_limiter.hpp"

namespace wr_chassis_gateway
{

class CmdVelGuardNode : public rclcpp::Node
{
public:
  CmdVelGuardNode()
  : Node("cmd_vel_guard"),
    limiter_(
      declare_parameter("max_vx", 0.45),
      declare_parameter("max_vy", 0.45),
      declare_parameter("max_wz", 0.80))
  {
    timeout_ = declare_parameter("command_timeout", 0.20);
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    if (timeout_ <= 0.0 || publish_rate <= 0.0) {
      throw std::invalid_argument("command_timeout and publish_rate must be positive");
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_chassis", rclcpp::QoS(10).reliable());
    timeout_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "cmd_vel_guard/timed_out", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10).reliable(),
      std::bind(&CmdVelGuardNode::on_command, this, std::placeholders::_1));

    last_input_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelGuardNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "Chassis velocity guard ready; timeout %.3f s", timeout_);
  }

private:
  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const auto result = limiter_.apply({message->linear.x, message->linear.y, message->angular.z});
    std::lock_guard<std::mutex> lock(mutex_);
    if (!result.input_valid) {
      latest_command_ = geometry_msgs::msg::Twist{};
      input_valid_ = false;
      RCLCPP_ERROR(get_logger(), "Rejected non-finite chassis velocity command");
      return;
    }
    latest_command_.linear.x = result.command.vx;
    latest_command_.linear.y = result.command.vy;
    latest_command_.angular.z = result.command.wz;
    last_input_time_ = now();
    input_valid_ = true;
    received_command_ = true;
    if (result.limited) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Chassis velocity command uniformly scaled to configured limits");
    }
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool timed_out = !received_command_ || !input_valid_ ||
      (now() - last_input_time_).seconds() > timeout_;
    command_publisher_->publish(timed_out ? geometry_msgs::msg::Twist{} : latest_command_);
    std_msgs::msg::Bool status;
    status.data = timed_out;
    timeout_publisher_->publish(status);
  }

  VelocityLimiter limiter_;
  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time last_input_time_;
  double timeout_{0.20};
  bool received_command_{false};
  bool input_valid_{false};
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr timeout_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wr_chassis_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wr_chassis_gateway::CmdVelGuardNode>());
  rclcpp::shutdown();
  return 0;
}
