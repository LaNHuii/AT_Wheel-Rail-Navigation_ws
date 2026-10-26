// 文件用途：真车底盘适配器，连接 ROS 2 结构化接口与 CAN-FD 协议。
// dry-run 只验证上位机数据链；实车启用前必须核对 CAN 帧、缩放系数、时间戳和故障处理。
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "wr_chassis_gateway/can_protocol.hpp"
#include "wr_chassis_gateway/socketcan_transport.hpp"
#include "wr_interfaces/msg/chassis_control_command.hpp"
#include "wr_interfaces/msg/chassis_state.hpp"
#include "wr_interfaces/msg/command_ack.hpp"
#include "wr_interfaces/msg/imu_status.hpp"
#include "wr_interfaces/msg/rail_clamp_command.hpp"
#include "wr_interfaces/msg/steering_task_state.hpp"
#include "wr_interfaces/msg/swerve_state.hpp"

namespace wr_chassis_gateway
{
namespace cp = can_protocol;

class RealChassisAdapterNode : public rclcpp::Node
{
public:
  RealChassisAdapterNode()
  : Node("real_chassis_adapter")
  {
    transport_name_ = declare_parameter<std::string>("transport", "dry_run");
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    session_id_ = static_cast<std::uint32_t>(declare_parameter<int64_t>("session_id", 1));
    command_rate_ = declare_parameter("command_rate", 50.0);
    receive_rate_ = declare_parameter("receive_rate", 500.0);
    time_sync_rate_ = declare_parameter("time_sync_rate", 1.0);
    valid_for_ms_ = static_cast<std::uint16_t>(declare_parameter<int>("valid_for_ms", 200));
    upstream_command_timeout_ = declare_parameter("upstream_command_timeout", 0.25);
    module_state_timeout_ = declare_parameter("module_state_timeout", 0.20);
    angular_covariance_ = declare_parameter<std::vector<double>>(
      "imu_angular_velocity_covariance_diagonal", {0.0004, 0.0004, 0.0004});
    acceleration_covariance_ = declare_parameter<std::vector<double>>(
      "imu_linear_acceleration_covariance_diagonal", {0.04, 0.04, 0.04});
    orientation_covariance_ = declare_parameter<std::vector<double>>(
      "imu_orientation_covariance_diagonal", {0.01, 0.01, 0.02});
    if (command_rate_ <= 0.0 || receive_rate_ <= 0.0 || time_sync_rate_ <= 0.0 ||
      valid_for_ms_ == 0 || upstream_command_timeout_ <= 0.0 || module_state_timeout_ <= 0.0 ||
      angular_covariance_.size() != 3 ||
      acceleration_covariance_.size() != 3 || orientation_covariance_.size() != 3)
    {
      throw std::invalid_argument("invalid CAN gateway rate, timeout, or IMU covariance parameter");
    }
    if (transport_name_ == "socketcan") {
      transport_.open(can_interface_);
      RCLCPP_INFO(
        get_logger(), "CAN-FD chassis adapter opened SocketCAN interface %s",
        can_interface_.c_str());
    } else if (transport_name_ == "dry_run") {
      RCLCPP_WARN(get_logger(), "CAN-FD chassis adapter is in dry-run mode");
    } else {
      throw std::invalid_argument("transport must be dry_run or socketcan");
    }

    command_preview_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "real_chassis_adapter/command_preview", 10);
    clamp_preview_publisher_ = create_publisher<wr_interfaces::msg::RailClampCommand>(
      "real_chassis_adapter/clamp_command_preview", 10);
    chassis_state_publisher_ = create_publisher<wr_interfaces::msg::ChassisState>(
      "chassis_state", 10);
    task_state_publisher_ = create_publisher<wr_interfaces::msg::SteeringTaskState>(
      "steering_task_state", 10);
    ack_publisher_ = create_publisher<wr_interfaces::msg::CommandAck>(
      "chassis_command_ack", 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data_raw", rclcpp::SensorDataQoS());
    imu_status_publisher_ = create_publisher<wr_interfaces::msg::ImuStatus>(
      "imu/status", 10);
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
    swerve_state_publisher_ = create_publisher<wr_interfaces::msg::SwerveState>(
      "swerve_state", 10);

    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_chassis", 10,
      std::bind(&RealChassisAdapterNode::on_velocity, this, std::placeholders::_1));
    last_velocity_time_ = std::chrono::steady_clock::now();
    clamp_subscription_ = create_subscription<wr_interfaces::msg::RailClampCommand>(
      "rail_clamp_command", 10,
      std::bind(&RealChassisAdapterNode::on_clamp, this, std::placeholders::_1));
    control_subscription_ = create_subscription<wr_interfaces::msg::ChassisControlCommand>(
      "chassis_control_command", 10,
      std::bind(&RealChassisAdapterNode::on_control, this, std::placeholders::_1));

    for (std::size_t index = 0; index < module_states_.size(); ++index) {
      module_states_[index].module_id = static_cast<std::uint8_t>(index);
      module_received_at_[index] = std::chrono::steady_clock::now();
    }
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / command_rate_)),
      std::bind(&RealChassisAdapterNode::send_periodic_command, this));
    receive_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / receive_rate_)),
      std::bind(&RealChassisAdapterNode::receive_frames, this));
    time_sync_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / time_sync_rate_)),
      std::bind(&RealChassisAdapterNode::send_time_sync, this));
  }

private:
  static constexpr std::array<const char *, 4> kModuleNames{
    "front_left", "front_right", "rear_right", "rear_left"};

  std::uint32_t next_frame_sequence()
  {
    return ++frame_sequence_;
  }

  bool send_frame(const cp::CanFdFrame & frame)
  {
    if (transport_name_ == "dry_run") {
      return true;
    }
    if (!transport_.send(frame)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "failed to send CAN-FD frame 0x%03x", frame.id);
      return false;
    }
    return true;
  }

  void on_velocity(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
      !std::isfinite(message->angular.z))
    {
      RCLCPP_ERROR(get_logger(), "rejected non-finite chassis command");
      latest_velocity_ = geometry_msgs::msg::Twist{};
      received_velocity_ = false;
      return;
    }
    latest_velocity_ = *message;
    last_velocity_time_ = std::chrono::steady_clock::now();
    received_velocity_ = true;
    geometry_msgs::msg::TwistStamped preview;
    preview.header.stamp = now();
    preview.header.frame_id = "base_link";
    preview.twist = *message;
    command_preview_publisher_->publish(preview);
  }

  void on_clamp(const wr_interfaces::msg::RailClampCommand::SharedPtr message)
  {
    cp::SteeringTaskCommand command;
    command.frame_sequence = next_frame_sequence();
    command.task_sequence = message->sequence_id;
    command.session_id = session_id_;
    command.action = message->action;
    command.module_mask = message->module_mask;
    command.max_angle = message->max_probe_angle;
    command.angular_speed = message->probe_angular_speed;
    command.timeout = message->timeout;
    send_frame(cp::encode(command));
    clamp_preview_publisher_->publish(*message);
  }

  void on_control(const wr_interfaces::msg::ChassisControlCommand::SharedPtr message)
  {
    cp::ControlCommand command;
    command.frame_sequence = next_frame_sequence();
    command.session_id = session_id_;
    command.requested_control_source = message->requested_control_source;
    command.requested_mode = message->requested_mode;
    command.enable = message->enable;
    command.quick_stop = message->quick_stop;
    command.clear_fault = message->clear_fault;
    send_frame(cp::encode(command));
  }

  void send_periodic_command()
  {
    const auto command_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_velocity_time_).count();
    const bool input_timed_out = !received_velocity_ || command_age > upstream_command_timeout_;
    if (input_timed_out) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "upstream chassis command timed out; transmitting zero velocity");
    }
    const auto & velocity = input_timed_out ? zero_velocity_ : latest_velocity_;
    cp::ChassisCommand command;
    command.frame_sequence = next_frame_sequence();
    command.session_id = session_id_;
    command.vx = static_cast<float>(velocity.linear.x);
    command.vy = static_cast<float>(velocity.linear.y);
    command.wz = static_cast<float>(velocity.angular.z);
    command.valid_for_ms = valid_for_ms_;
    send_frame(cp::encode(command));
  }

  void send_time_sync()
  {
    cp::TimeSyncRequest request;
    request.frame_sequence = next_frame_sequence();
    request.host_send_time_ns = static_cast<std::uint64_t>(now().nanoseconds());
    send_frame(cp::encode(request));
  }

  rclcpp::Time stamp_from_mcu(std::uint64_t sample_time_us) const
  {
    if (!time_offset_ns_) {
      return now();
    }
    const auto mapped = static_cast<std::int64_t>(sample_time_us * 1000ULL) + *time_offset_ns_;
    return rclcpp::Time(mapped, get_clock()->get_clock_type());
  }

  void receive_frames()
  {
    if (!transport_.is_open()) {
      return;
    }
    cp::CanFdFrame frame;
    while (transport_.receive(frame)) {
      if (frame.id == cp::kIdChassisState) {
        cp::ChassisState value;
        if (cp::decode(frame, value)) {
          handle_chassis_state(value);
        }
      } else if (frame.id >= cp::kIdModuleStateBase && frame.id <= cp::kIdModuleStateBase + 3U) {
        cp::ModuleState value;
        if (cp::decode(frame, value)) {
          handle_module_state(value);
        }
      } else if (frame.id == cp::kIdImuRaw) {
        cp::ImuRaw value;
        if (cp::decode(frame, value)) {
          handle_imu_raw(value);
        }
      } else if (frame.id == cp::kIdImuOrientation) {
        cp::ImuOrientation value;
        if (cp::decode(frame, value)) {
          latest_orientation_ = value;
        }
      } else if (frame.id == cp::kIdSteeringTaskState) {
        cp::SteeringTaskState value;
        if (cp::decode(frame, value)) {
          handle_task_state(value);
        }
      } else if (frame.id == cp::kIdCommandAck) {
        cp::CommandAck value;
        if (cp::decode(frame, value)) {
          handle_ack(value);
        }
      } else if (frame.id == cp::kIdTimeSyncResponse) {
        cp::TimeSyncResponse value;
        if (cp::decode(frame, value)) {
          handle_time_sync(value);
        }
      }
    }
  }

  void handle_time_sync(const cp::TimeSyncResponse & value)
  {
    const auto t1 = static_cast<std::int64_t>(value.host_send_time_ns);
    const auto t2 = static_cast<std::int64_t>(value.mcu_receive_time_us * 1000ULL);
    const auto t3 = static_cast<std::int64_t>(value.mcu_send_time_us * 1000ULL);
    const auto t4 = now().nanoseconds();
    const auto round_trip = (t4 - t1) - (t3 - t2);
    if (round_trip < 0 || round_trip > 20000000LL) {
      return;
    }
    const auto estimate = ((t1 - t2) + (t4 - t3)) / 2;
    time_offset_ns_ = time_offset_ns_ ?
      static_cast<std::int64_t>(0.9 * static_cast<double>(*time_offset_ns_) +
      0.1 * static_cast<double>(estimate)) : estimate;
  }

  void handle_chassis_state(const cp::ChassisState & value)
  {
    if (boot_id_ && *boot_id_ != value.boot_id) {
      time_offset_ns_.reset();
      RCLCPP_WARN(get_logger(), "STM32 reboot detected; invalidated CAN time synchronization");
    }
    boot_id_ = value.boot_id;
    wr_interfaces::msg::ChassisState message;
    message.header.stamp = stamp_from_mcu(value.sample_time_us);
    message.header.frame_id = "base_link";
    message.boot_id = value.boot_id;
    message.last_command_sequence_id = value.last_command_sequence;
    message.active_control_source = value.active_control_source;
    message.active_mode = value.active_mode;
    message.enabled = value.enabled;
    message.physical_estop = value.physical_estop;
    message.remote_online = value.remote_online;
    message.command_timed_out = value.command_timed_out;
    message.motor_power_ready = value.motor_power_ready;
    message.measured_twist.linear.x = value.vx;
    message.measured_twist.linear.y = value.vy;
    message.measured_twist.angular.z = value.wz;
    message.chassis_fault_bits = value.fault_bits;
    message.battery_voltage = value.battery_voltage;
    message.battery_current = value.battery_current;
    message.modules = module_states_;
    const auto steady_now = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < message.modules.size(); ++index) {
      message.modules[index].sample_valid = module_received_[index] &&
        std::chrono::duration<double>(steady_now - module_received_at_[index]).count() <=
        module_state_timeout_;
    }
    chassis_state_publisher_->publish(message);
    publish_joint_and_swerve(message);
  }

  void handle_module_state(const cp::ModuleState & value)
  {
    if (value.module_id >= module_states_.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "rejected module state with invalid module id %u", value.module_id);
      return;
    }
    auto & message = module_states_.at(value.module_id);
    message.header.stamp = stamp_from_mcu(value.sample_time_us);
    message.header.frame_id = "base_link";
    message.frame_sequence_id = value.frame_sequence;
    message.module_id = value.module_id;
    message.sample_valid = true;
    message.steering_online = value.steering_online;
    message.drive_online = value.drive_online;
    message.steering_zero_valid = value.steering_zero_valid;
    message.steering_torque_valid = value.steering_torque_valid;
    message.drive_position_valid = value.drive_position_valid;
    message.steering_fault_bits = value.steering_fault_bits;
    message.drive_fault_bits = value.drive_fault_bits;
    message.steering_angle = value.steering_angle;
    message.steering_velocity = value.steering_velocity;
    message.steering_current = value.steering_current;
    message.steering_torque = value.steering_torque;
    message.drive_position = value.drive_position;
    message.drive_velocity = value.drive_velocity;
    message.drive_current = value.drive_current;
    message.drive_torque = value.drive_torque;
    message.steering_temperature = value.steering_temperature;
    message.drive_temperature = value.drive_temperature;
    module_received_at_.at(value.module_id) = std::chrono::steady_clock::now();
    module_received_.at(value.module_id) = true;
  }

  void handle_imu_raw(const cp::ImuRaw & value)
  {
    const auto stamp = stamp_from_mcu(value.sample_time_us);
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_id_;
    message.angular_velocity.x = value.angular_velocity[0];
    message.angular_velocity.y = value.angular_velocity[1];
    message.angular_velocity.z = value.angular_velocity[2];
    message.linear_acceleration.x = value.linear_acceleration[0];
    message.linear_acceleration.y = value.linear_acceleration[1];
    message.linear_acceleration.z = value.linear_acceleration[2];
    set_covariance(message.angular_velocity_covariance, angular_covariance_);
    set_covariance(message.linear_acceleration_covariance, acceleration_covariance_);
    const bool orientation_valid = latest_orientation_ &&
      latest_orientation_->status_bits & 0x04U &&
      value.sample_time_us >= latest_orientation_->sample_time_us &&
      value.sample_time_us - latest_orientation_->sample_time_us <= 20000ULL;
    if (orientation_valid) {
      message.orientation.x = latest_orientation_->quaternion[0];
      message.orientation.y = latest_orientation_->quaternion[1];
      message.orientation.z = latest_orientation_->quaternion[2];
      message.orientation.w = latest_orientation_->quaternion[3];
      set_covariance(message.orientation_covariance, orientation_covariance_);
    } else {
      message.orientation.w = 1.0;
      message.orientation_covariance[0] = -1.0;
    }
    imu_publisher_->publish(message);

    wr_interfaces::msg::ImuStatus status;
    status.header = message.header;
    status.boot_id = boot_id_.value_or(0);
    status.gyro_valid = value.status_bits & 0x01U;
    status.accelerometer_valid = value.status_bits & 0x02U;
    status.orientation_valid = orientation_valid;
    status.calibrated = value.status_bits & 0x08U;
    status.temperature = value.temperature;
    status.fault_bits = value.status_bits >> 8U;
    imu_status_publisher_->publish(status);
  }

  static void set_covariance(
    std::array<double, 9> & covariance, const std::vector<double> & diagonal)
  {
    covariance.fill(0.0);
    covariance[0] = diagonal[0];
    covariance[4] = diagonal[1];
    covariance[8] = diagonal[2];
  }

  void handle_task_state(const cp::SteeringTaskState & value)
  {
    wr_interfaces::msg::SteeringTaskState message;
    message.header.stamp = now();
    message.task_sequence_id = value.task_sequence;
    message.action = value.action;
    message.state = value.state;
    message.active_module_mask = value.active_mask;
    message.holding_module_mask = value.holding_mask;
    message.completed_module_mask = value.completed_mask;
    message.safety_stopped_module_mask = value.safety_stopped_mask;
    message.result_code = value.result_code;
    task_state_publisher_->publish(message);
  }

  void handle_ack(const cp::CommandAck & value)
  {
    wr_interfaces::msg::CommandAck message;
    message.header.stamp = now();
    message.acknowledged_sequence_id = value.acknowledged_sequence;
    message.message_type = value.message_type;
    message.accepted = value.accepted;
    message.result_code = value.result_code;
    message.active_module_mask = value.active_module_mask;
    ack_publisher_->publish(message);
  }

  void publish_joint_and_swerve(const wr_interfaces::msg::ChassisState & chassis)
  {
    sensor_msgs::msg::JointState joints;
    joints.header = chassis.header;
    wr_interfaces::msg::SwerveState swerve;
    swerve.header = chassis.header;
    swerve.twist = chassis.measured_twist;
    swerve.command_timed_out = chassis.command_timed_out;
    swerve.state_valid = true;
    for (std::size_t index = 0; index < module_states_.size(); ++index) {
      const auto & module = module_states_[index];
      const std::string prefix = kModuleNames[index];
      joints.name.push_back(prefix + "_steering_joint");
      joints.position.push_back(module.steering_angle);
      joints.velocity.push_back(module.steering_velocity);
      joints.effort.push_back(module.steering_torque_valid ? module.steering_torque : 0.0);
      joints.name.push_back(prefix + "_wheel_joint");
      joints.position.push_back(module.drive_position_valid ? module.drive_position : 0.0);
      joints.velocity.push_back(module.drive_velocity);
      joints.effort.push_back(module.drive_torque);
      auto & legacy = swerve.modules[index];
      legacy.name = prefix;
      legacy.steering_angle = module.steering_angle;
      legacy.steering_velocity = module.steering_velocity;
      legacy.steering_effort = module.steering_torque_valid ? module.steering_torque : 0.0;
      legacy.drive_velocity = module.drive_velocity;
      legacy.drive_effort = module.drive_torque;
      swerve.state_valid = swerve.state_valid && chassis.modules[index].sample_valid &&
        module.steering_online &&
        module.drive_online && module.steering_zero_valid;
    }
    joint_state_publisher_->publish(joints);
    swerve_state_publisher_->publish(swerve);
  }

  std::string transport_name_;
  std::string can_interface_;
  std::string imu_frame_id_;
  std::uint32_t session_id_{1};
  std::uint32_t frame_sequence_{0};
  std::uint16_t valid_for_ms_{200};
  double command_rate_{50.0};
  double receive_rate_{500.0};
  double time_sync_rate_{1.0};
  double upstream_command_timeout_{0.25};
  double module_state_timeout_{0.20};
  std::vector<double> angular_covariance_;
  std::vector<double> acceleration_covariance_;
  std::vector<double> orientation_covariance_;
  geometry_msgs::msg::Twist latest_velocity_;
  const geometry_msgs::msg::Twist zero_velocity_{};
  std::chrono::steady_clock::time_point last_velocity_time_;
  bool received_velocity_{false};
  std::array<wr_interfaces::msg::ModuleState, 4> module_states_;
  std::array<std::chrono::steady_clock::time_point, 4> module_received_at_;
  std::array<bool, 4> module_received_{{false, false, false, false}};
  std::optional<cp::ImuOrientation> latest_orientation_;
  std::optional<std::uint32_t> boot_id_;
  std::optional<std::int64_t> time_offset_ns_;
  SocketCanTransport transport_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_preview_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::RailClampCommand>::SharedPtr clamp_preview_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::ChassisState>::SharedPtr chassis_state_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::SteeringTaskState>::SharedPtr task_state_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::CommandAck>::SharedPtr ack_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::ImuStatus>::SharedPtr imu_status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<wr_interfaces::msg::SwerveState>::SharedPtr swerve_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<wr_interfaces::msg::RailClampCommand>::SharedPtr clamp_subscription_;
  rclcpp::Subscription<wr_interfaces::msg::ChassisControlCommand>::SharedPtr control_subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr time_sync_timer_;
};

}  // namespace wr_chassis_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wr_chassis_gateway::RealChassisAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
