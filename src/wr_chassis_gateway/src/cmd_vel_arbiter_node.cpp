// 文件用途：仲裁人物跟随、自由路面导航、轨道过渡和遥控速度源。
// 仲裁器不代替 STM32 安全保护；底盘未授权、状态超时、急停或故障时只输出零速度。
// 可调参数 source_timeout、chassis_timeout 和 publish_rate 位于底盘网关配置或 launch 文件。

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "wr_interfaces/msg/chassis_state.hpp"
#include "wr_interfaces/srv/set_motion_source.hpp"

namespace wr_chassis_gateway
{

class CmdVelArbiterNode : public rclcpp::Node
{
public:
  CmdVelArbiterNode()
  : Node("cmd_vel_arbiter")
  {
    source_timeout_ = declare_parameter("source_timeout", 0.20);
    chassis_timeout_ = declare_parameter("chassis_timeout", 0.25);
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    if (source_timeout_ <= 0.0 || chassis_timeout_ <= 0.0 || publish_rate <= 0.0) {
      throw std::invalid_argument("arbiter rates and timeouts must be positive");
    }
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_selected", 10);
    source_publisher_ = create_publisher<std_msgs::msg::UInt8>("motion_source/active", 10);
    create_source_subscription("cmd_vel_follow", kPersonFollow);
    create_source_subscription("cmd_vel_navigation", kFreeNavigation);
    create_source_subscription("cmd_vel_transition", kRailTransition);
    create_source_subscription("cmd_vel_manual", kManual);
    chassis_subscription_ = create_subscription<wr_interfaces::msg::ChassisState>(
      "chassis_state", 10,
      std::bind(&CmdVelArbiterNode::on_chassis, this, std::placeholders::_1));
    service_ = create_service<wr_interfaces::srv::SetMotionSource>(
      "set_motion_source",
      std::bind(
        &CmdVelArbiterNode::on_set_source, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate)),
      std::bind(&CmdVelArbiterNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "Velocity arbiter ready with no initial motion owner");
  }

private:
  using SetSource = wr_interfaces::srv::SetMotionSource;
  static constexpr std::uint8_t kNone = 0;
  static constexpr std::uint8_t kPersonFollow = 1;
  static constexpr std::uint8_t kFreeNavigation = 2;
  static constexpr std::uint8_t kRailTransition = 3;
  static constexpr std::uint8_t kManual = 4;

  struct SourceState
  {
    geometry_msgs::msg::Twist command;
    std::chrono::steady_clock::time_point received_at{};
    bool received{false};
  };

  void create_source_subscription(const std::string & topic, std::uint8_t source)
  {
    source_subscriptions_.at(source) = create_subscription<geometry_msgs::msg::Twist>(
      topic, 10, [this, source](geometry_msgs::msg::Twist::SharedPtr message) {
        const bool valid = std::isfinite(message->linear.x) &&
          std::isfinite(message->linear.y) && std::isfinite(message->angular.z);
        std::lock_guard<std::mutex> lock(mutex_);
        auto & state = sources_.at(source);
        state.command = valid ? *message : geometry_msgs::msg::Twist{};
        state.received_at = std::chrono::steady_clock::now();
        state.received = valid;
        if (!valid) {
          RCLCPP_ERROR(get_logger(), "Rejected non-finite velocity from source %u", source);
        }
      });
  }

  void on_chassis(const wr_interfaces::msg::ChassisState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool autonomous_was_active =
      active_source_ == kPersonFollow || active_source_ == kFreeNavigation ||
      active_source_ == kRailTransition;
    const bool autonomy_interrupted =
      message->active_control_source != wr_interfaces::msg::ChassisState::SOURCE_AUTONOMOUS ||
      message->physical_estop || message->chassis_fault_bits;
    // 遥控接管的优先级高于上位机。一旦 STM32 不再报告 AUTONOMOUS，
    // 立即清空当前自动速度源，遥控释放后也不会恢复旧指令。
    if (autonomous_was_active && autonomy_interrupted) {
      active_source_ = kNone;
      publisher_->publish(geometry_msgs::msg::Twist{});
      RCLCPP_WARN(
        get_logger(),
        "Autonomous motion source cleared after manual takeover, estop, or chassis fault");
    }
    chassis_ = *message;
    chassis_received_at_ = std::chrono::steady_clock::now();
    chassis_received_ = true;
  }

  bool source_allowed_for_mode(std::uint8_t source, std::uint8_t mode) const
  {
    if (source == kNone || source == kManual) {
      return true;
    }
    if (source == kPersonFollow) {
      return mode == wr_interfaces::msg::ChassisState::MODE_FREE_SURFACE ||
             mode == wr_interfaces::msg::ChassisState::MODE_ON_RAIL;
    }
    if (source == kFreeNavigation) {
      return mode == wr_interfaces::msg::ChassisState::MODE_FREE_SURFACE;
    }
    if (source == kRailTransition) {
      return mode == wr_interfaces::msg::ChassisState::MODE_RAIL_ENTRY ||
             mode == wr_interfaces::msg::ChassisState::MODE_RAIL_EXIT;
    }
    return false;
  }

  bool source_request_allowed_for_mode(std::uint8_t source, std::uint8_t mode) const
  {
    if (source == kRailTransition) {
      return mode == wr_interfaces::msg::ChassisState::MODE_FREE_SURFACE ||
             mode == wr_interfaces::msg::ChassisState::MODE_RAIL_ENTRY ||
             mode == wr_interfaces::msg::ChassisState::MODE_ON_RAIL ||
             mode == wr_interfaces::msg::ChassisState::MODE_RAIL_EXIT;
    }
    return source_allowed_for_mode(source, mode);
  }

  void on_set_source(
    const std::shared_ptr<SetSource::Request> request,
    std::shared_ptr<SetSource::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->requested_source > kManual) {
      response->accepted = false;
      response->active_source = active_source_;
      response->message = "unknown motion source";
      return;
    }
    if (request->requested_source != kNone &&
      (!chassis_received_ ||
      !source_request_allowed_for_mode(request->requested_source, chassis_.active_mode)))
    {
      response->accepted = false;
      response->active_source = active_source_;
      response->message = "source is not allowed in the current chassis mode";
      return;
    }
    if (request->requested_source == kManual &&
      chassis_.active_control_source != wr_interfaces::msg::ChassisState::SOURCE_MANUAL)
    {
      response->accepted = false;
      response->active_source = active_source_;
      response->message = "STM32 has not granted manual control";
      return;
    }
    if (request->requested_source != kNone && request->requested_source != kManual &&
      chassis_.active_control_source != wr_interfaces::msg::ChassisState::SOURCE_AUTONOMOUS)
    {
      response->accepted = false;
      response->active_source = active_source_;
      response->message = "STM32 has not granted autonomous control";
      return;
    }
    // 任何速度源交接都先插入一帧零速度，避免上一个控制器的指令泄漏。
    active_source_ = request->requested_source;
    publisher_->publish(geometry_msgs::msg::Twist{});
    response->accepted = true;
    response->active_source = active_source_;
    response->message = "motion source changed; zero command inserted at handover";
  }

  void on_timer()
  {
    // source_timeout 约束速度指令新鲜度，chassis_timeout 约束 STM32 状态新鲜度。
    // 任一超时均不保持上次速度，而是输出默认零速度。
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool chassis_safe = chassis_received_ &&
      std::chrono::duration<double>(now - chassis_received_at_).count() <= chassis_timeout_ &&
      chassis_.enabled && chassis_.motor_power_ready && !chassis_.physical_estop &&
      !chassis_.chassis_fault_bits;
    if (active_source_ == kManual) {
      chassis_safe = chassis_safe && chassis_.remote_online &&
        chassis_.active_control_source == wr_interfaces::msg::ChassisState::SOURCE_MANUAL;
    } else if (active_source_ != kNone) {
      chassis_safe = chassis_safe &&
        chassis_.active_control_source == wr_interfaces::msg::ChassisState::SOURCE_AUTONOMOUS;
    }
    geometry_msgs::msg::Twist output;
    if (active_source_ != kNone && chassis_safe &&
      source_allowed_for_mode(active_source_, chassis_.active_mode))
    {
      const auto & source = sources_.at(active_source_);
      if (source.received &&
        std::chrono::duration<double>(now - source.received_at).count() <= source_timeout_)
      {
        output = source.command;
      }
    }
    publisher_->publish(output);
    std_msgs::msg::UInt8 status;
    status.data = active_source_;
    source_publisher_->publish(status);
  }

  double source_timeout_{0.20};
  double chassis_timeout_{0.25};
  std::uint8_t active_source_{kNone};
  std::array<SourceState, 5> sources_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr, 5>
    source_subscriptions_{};
  wr_interfaces::msg::ChassisState chassis_;
  std::chrono::steady_clock::time_point chassis_received_at_{};
  bool chassis_received_{false};
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr source_publisher_;
  rclcpp::Subscription<wr_interfaces::msg::ChassisState>::SharedPtr chassis_subscription_;
  rclcpp::Service<SetSource>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wr_chassis_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wr_chassis_gateway::CmdVelArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
