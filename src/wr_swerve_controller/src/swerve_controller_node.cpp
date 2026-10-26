// 文件用途：将底盘 Twist 解算为四个舍轮的转向角和驱动速度。
// 轮距、轴距、轮半径、方向符号和转向限位位于 config/mock_controllers.yaml。
// 实车中 STM32 可保留高频电机闭环，上位机与下位机之间通过通用底盘协议适配。
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "wr_interfaces/msg/swerve_state.hpp"
#include "wr_swerve_controller/swerve_kinematics.hpp"

namespace wr_swerve_controller
{

class SwerveControllerNode : public rclcpp::Node
{
public:
  SwerveControllerNode()
  : Node("swerve_controller"), current_angles_{0.0, 0.0, 0.0, 0.0}
  {
    module_names_ = declare_parameter<std::vector<std::string>>(
      "module_names", {"front_left", "front_right", "rear_right", "rear_left"});
    const auto module_x = declare_parameter<std::vector<double>>(
      "module_x", {0.355, 0.355, -0.355, -0.355});
    const auto module_y = declare_parameter<std::vector<double>>(
      "module_y", {0.335, -0.335, -0.335, 0.335});
    const double wheel_radius = declare_parameter("wheel_radius", 0.0935);
    const double steering_limit = declare_parameter("steering_limit", 2.88);
    const double max_wheel_velocity = declare_parameter("max_wheel_angular_velocity", 20.0);
    command_timeout_ = declare_parameter("command_timeout", 0.25);
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    recenter_on_stop_ = declare_parameter("recenter_on_stop", true);
    recenter_delay_ = declare_parameter("recenter_delay", 0.50);
    recenter_angular_speed_ = declare_parameter("recenter_angular_speed", 0.50);

    if (module_names_.size() != SwerveKinematics::kModuleCount ||
      module_x.size() != SwerveKinematics::kModuleCount ||
      module_y.size() != SwerveKinematics::kModuleCount)
    {
      throw std::invalid_argument("module_names, module_x and module_y must each contain four entries");
    }
    if (command_timeout_ <= 0.0 || publish_rate <= 0.0 || recenter_delay_ < 0.0 ||
      recenter_angular_speed_ <= 0.0)
    {
      throw std::invalid_argument("controller timeouts, rates, and recenter speed are invalid");
    }

    SwerveKinematics::Locations locations{};
    for (std::size_t index = 0; index < locations.size(); ++index) {
      locations[index] = {module_x[index], module_y[index]};
    }
    kinematics_ = std::make_unique<SwerveKinematics>(
      locations, wheel_radius, steering_limit, max_wheel_velocity);

    steering_command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "steering_position_controller/commands", rclcpp::QoS(10).reliable());
    wheel_command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "wheel_velocity_controller/commands", rclcpp::QoS(10).reliable());
    state_publisher_ = create_publisher<wr_interfaces::msg::SwerveState>(
      "swerve_state", rclcpp::QoS(10).reliable());

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10).reliable(),
      std::bind(&SwerveControllerNode::on_command, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SwerveControllerNode::on_joint_state, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SwerveControllerNode::on_timer, this));

    last_command_time_ = now();
    last_motion_time_ = now();
    recenter_step_ = recenter_angular_speed_ / publish_rate;
    publish_initial_stop_command();
    RCLCPP_INFO(
      get_logger(), "Swerve controller ready: FL, FR, RR, RL; timeout %.3f s",
      command_timeout_);
  }

private:
  struct JointSample
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
  };

  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool finite = std::isfinite(message->linear.x) && std::isfinite(message->linear.y) &&
      std::isfinite(message->angular.z);
    if (!finite) {
      RCLCPP_ERROR(get_logger(), "Rejected non-finite cmd_vel");
      last_command_ = geometry_msgs::msg::Twist{};
      command_received_ = false;
      return;
    }
    last_command_ = *message;
    last_command_time_ = now();
    command_received_ = true;
    timed_out_ = false;
    if (!command_is_zero(last_command_)) {
      last_motion_time_ = last_command_time_;
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      auto & sample = joint_samples_[message->name[index]];
      if (index < message->position.size()) {
        sample.position = message->position[index];
        sample.has_position = true;
      }
      if (index < message->velocity.size()) {
        sample.velocity = message->velocity[index];
        sample.has_velocity = true;
      }
      if (index < message->effort.size()) {
        sample.effort = message->effort[index];
        sample.has_effort = true;
      }
    }
    for (std::size_t index = 0; index < module_names_.size(); ++index) {
      const auto found = joint_samples_.find(module_names_[index] + "_steering_joint");
      if (found != joint_samples_.end() && found->second.has_position) {
        current_angles_[index] = found->second.position;
      }
    }
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool expired = !command_received_ || (now() - last_command_time_).seconds() > command_timeout_;
    if (expired && !timed_out_ && command_received_) {
      RCLCPP_WARN(get_logger(), "cmd_vel timed out; wheel velocities forced to zero");
    }
    timed_out_ = expired;

    if (!expired && !command_is_zero(last_command_)) {
      try {
        publish_commands(kinematics_->calculate(
            last_command_.linear.x, last_command_.linear.y,
            last_command_.angular.z, current_angles_));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Rejected cmd_vel: %s", error.what());
        publish_stop_command(false);
      }
    } else {
      const bool recenter = recenter_on_stop_ &&
        (now() - last_motion_time_).seconds() >= recenter_delay_;
      publish_stop_command(recenter);
    }
    publish_state();
  }

  static bool command_is_zero(const geometry_msgs::msg::Twist & command)
  {
    constexpr double epsilon = 1.0e-6;
    return std::abs(command.linear.x) < epsilon && std::abs(command.linear.y) < epsilon &&
           std::abs(command.angular.z) < epsilon;
  }

  void publish_stop_command(bool recenter)
  {
    if (recenter) {
      publish_commands(kinematics_->recenter(current_angles_, recenter_step_));
      return;
    }
    SwerveKinematics::Commands commands{};
    for (std::size_t index = 0; index < commands.size(); ++index) {
      commands[index] = {current_angles_[index], 0.0};
    }
    publish_commands(commands);
  }

  void publish_initial_stop_command()
  {
    SwerveKinematics::Commands commands{};
    for (std::size_t index = 0; index < commands.size(); ++index) {
      commands[index] = {current_angles_[index], 0.0};
    }
    publish_commands(commands);
  }

  void publish_commands(const SwerveKinematics::Commands & commands)
  {
    std_msgs::msg::Float64MultiArray steering;
    std_msgs::msg::Float64MultiArray wheel;
    steering.data.reserve(commands.size());
    wheel.data.reserve(commands.size());
    for (const auto & command : commands) {
      steering.data.push_back(command.steering_angle);
      wheel.data.push_back(command.wheel_angular_velocity);
    }
    steering_command_publisher_->publish(steering);
    wheel_command_publisher_->publish(wheel);
  }

  void publish_state()
  {
    wr_interfaces::msg::SwerveState state;
    state.header.stamp = now();
    state.header.frame_id = "base_link";
    state.twist = timed_out_ ? geometry_msgs::msg::Twist{} : last_command_;
    state.command_timed_out = timed_out_;
    state.state_valid = true;

    for (std::size_t index = 0; index < module_names_.size(); ++index) {
      auto & module = state.modules[index];
      module.name = module_names_[index];
      const auto steering_name = module_names_[index] + "_steering_joint";
      const auto wheel_name = module_names_[index] + "_wheel_joint";
      const auto steering = joint_samples_.find(steering_name);
      const auto wheel = joint_samples_.find(wheel_name);
      if (steering == joint_samples_.end() || wheel == joint_samples_.end() ||
        !steering->second.has_position || !wheel->second.has_velocity)
      {
        state.state_valid = false;
        continue;
      }
      module.steering_angle = steering->second.position;
      module.steering_velocity = steering->second.velocity;
      module.steering_effort = steering->second.effort;
      module.drive_velocity = wheel->second.velocity;
      module.drive_effort = wheel->second.effort;
    }
    state_publisher_->publish(state);
  }

  std::vector<std::string> module_names_;
  SwerveKinematics::SteeringAngles current_angles_;
  std::unique_ptr<SwerveKinematics> kinematics_;
  std::unordered_map<std::string, JointSample> joint_samples_;
  geometry_msgs::msg::Twist last_command_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_motion_time_;
  double command_timeout_{0.25};
  double recenter_delay_{0.50};
  double recenter_angular_speed_{0.50};
  double recenter_step_{0.01};
  bool recenter_on_stop_{true};
  bool command_received_{false};
  bool timed_out_{true};
  std::mutex mutex_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_command_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::SwerveState>::SharedPtr state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wr_swerve_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wr_swerve_controller::SwerveControllerNode>());
  rclcpp::shutdown();
  return 0;
}
