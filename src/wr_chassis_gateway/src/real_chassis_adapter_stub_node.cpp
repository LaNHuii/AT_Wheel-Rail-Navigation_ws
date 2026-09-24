#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wr_interfaces/msg/rail_clamp_command.hpp"

namespace wr_chassis_gateway
{

class RealChassisAdapterStubNode : public rclcpp::Node
{
public:
  RealChassisAdapterStubNode()
  : Node("real_chassis_adapter")
  {
    const auto transport = declare_parameter<std::string>("transport", "dry_run");
    if (transport != "dry_run") {
      throw std::invalid_argument(
              "Only transport=dry_run exists; serial/CAN/Ethernet must not be selected before implementation");
    }
    preview_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "real_chassis_adapter/command_preview", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_chassis", rclcpp::QoS(10).reliable(),
      std::bind(&RealChassisAdapterStubNode::on_command, this, std::placeholders::_1));
    clamp_preview_publisher_ = create_publisher<wr_interfaces::msg::RailClampCommand>(
      "real_chassis_adapter/clamp_command_preview", rclcpp::QoS(10).reliable());
    clamp_subscription_ = create_subscription<wr_interfaces::msg::RailClampCommand>(
      "rail_clamp_command", rclcpp::QoS(10).reliable(),
      std::bind(&RealChassisAdapterStubNode::on_clamp_command, this, std::placeholders::_1));
    RCLCPP_WARN(
      get_logger(),
      "Real chassis adapter is in dry-run mode: no bytes or CAN frames will be transmitted");
  }

private:
  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    geometry_msgs::msg::TwistStamped preview;
    preview.header.stamp = now();
    preview.header.frame_id = "base_link";
    preview.twist = *message;
    preview_publisher_->publish(preview);
  }

  void on_clamp_command(const wr_interfaces::msg::RailClampCommand::SharedPtr message)
  {
    clamp_preview_publisher_->publish(*message);
    RCLCPP_WARN(
      get_logger(),
      "Dry-run clamp command: sequence=%u action=%u mask=0x%02x; no motor command transmitted",
      message->sequence_id, message->action, message->module_mask);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr preview_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<wr_interfaces::msg::RailClampCommand>::SharedPtr clamp_preview_publisher_;
  rclcpp::Subscription<wr_interfaces::msg::RailClampCommand>::SharedPtr clamp_subscription_;
};

}  // namespace wr_chassis_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wr_chassis_gateway::RealChassisAdapterStubNode>());
  rclcpp::shutdown();
  return 0;
}
