// 文件用途：轮轨人物跟随任务的顶层行为树编排。
// 该节点在人物跟随与进/离轨 Action 之间切换运动权，并将遥控接管
// 与显式自动重新授权作为最高级安全条件。
// 调车参数：desired_gap、entry_confidence、person_beyond_entry、
// transition_timeout 和 tick_rate，由 launch/task_manager.launch.py 或 ROS 参数覆盖。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "wr_interfaces/action/execute_rail_transition.hpp"
#include "wr_interfaces/action/follow_person.hpp"
#include "wr_interfaces/msg/chassis_state.hpp"
#include "wr_interfaces/msg/rail_entry_estimate.hpp"
#include "wr_interfaces/msg/tracked_person.hpp"
#include "wr_interfaces/srv/set_motion_source.hpp"

using namespace std::chrono_literals;

class TaskManagerNode;

class TransitionRequested : public BT::ConditionNode
{
public:
  TransitionRequested(const std::string & name, const BT::NodeConfiguration & config,
    TaskManagerNode * manager)
  : BT::ConditionNode(name, config), manager_(manager) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  TaskManagerNode * manager_;
};

class AutonomyEnabled : public BT::ConditionNode
{
public:
  AutonomyEnabled(const std::string & name, const BT::NodeConfiguration & config,
    TaskManagerNode * manager)
  : BT::ConditionNode(name, config), manager_(manager) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  TaskManagerNode * manager_;
};

class RunPersonFollow : public BT::StatefulActionNode
{
public:
  RunPersonFollow(const std::string & name, const BT::NodeConfiguration & config,
    TaskManagerNode * manager)
  : BT::StatefulActionNode(name, config), manager_(manager) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  TaskManagerNode * manager_;
};

class RunRailTransition : public BT::StatefulActionNode
{
public:
  RunRailTransition(const std::string & name, const BT::NodeConfiguration & config,
    TaskManagerNode * manager)
  : BT::StatefulActionNode(name, config), manager_(manager) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  TaskManagerNode * manager_;
};

class TaskManagerNode : public rclcpp::Node
{
public:
  using Follow = wr_interfaces::action::FollowPerson;
  using Transition = wr_interfaces::action::ExecuteRailTransition;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
  using TransitionGoalHandle = rclcpp_action::ClientGoalHandle<Transition>;
  enum class AsyncState : std::uint8_t {IDLE, STARTING, RUNNING, SUCCESS, FAILURE};

  TaskManagerNode()
  : Node("wr_task_manager")
  {
    // 入口置信度偏高可减少误进轨，但会增加无法触发的概率；
    // 实车应用录制的 MID-360S 数据调整 entry_confidence。
    desired_gap_ = declare_parameter("desired_gap", 0.30);
    transition_timeout_ = declare_parameter("transition_timeout", 45.0);
    entry_confidence_ = declare_parameter("entry_confidence", 0.70);
    person_beyond_entry_ = declare_parameter("person_beyond_entry", 0.15);
    tick_rate_ = declare_parameter("tick_rate", 20.0);
    autonomy_armed_.store(declare_parameter("autonomy_enabled_at_startup", true));
    motion_client_ = create_client<wr_interfaces::srv::SetMotionSource>("set_motion_source");
    follow_client_ = rclcpp_action::create_client<Follow>(this, "follow_person");
    transition_client_ = rclcpp_action::create_client<Transition>(
      this, "execute_rail_transition");
    request_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "rail_transition_request", 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) {
        if (message->data == Transition::Goal::ENTER ||
          message->data == Transition::Goal::EXIT_FORWARD)
        {
          transition_request_.store(message->data);
        }
      });
    chassis_subscription_ = create_subscription<wr_interfaces::msg::ChassisState>(
      "chassis_state", 10,
      [this](wr_interfaces::msg::ChassisState::SharedPtr message) {
        chassis_mode_.store(message->active_mode);
        chassis_control_source_.store(message->active_control_source);
        chassis_safe_.store(
          message->enabled && !message->physical_estop && !message->chassis_fault_bits);
        // STM32 遥控接管或撤销自动授权后，立即解除上位机自动锁定。
        // 遥控释放后不会自动恢复旧任务，必须重新收到 /autonomy_enable=true。
        if (message->active_control_source !=
          wr_interfaces::msg::ChassisState::SOURCE_AUTONOMOUS)
        {
          autonomy_armed_.store(false);
        }
      });
    autonomy_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "autonomy_enable", 10,
      [this](std_msgs::msg::Bool::SharedPtr message) {
        const bool can_arm = message->data && chassis_safe_.load() &&
          chassis_control_source_.load() ==
          wr_interfaces::msg::ChassisState::SOURCE_AUTONOMOUS;
        autonomy_armed_.store(can_arm);
        if (can_arm && !transition_requested()) {
          const auto state = follow_state_.load();
          if (state != AsyncState::STARTING && state != AsyncState::RUNNING) {
            follow_state_.store(AsyncState::IDLE);
            start_follow();
          }
        }
      });
    estimate_subscription_ = create_subscription<wr_interfaces::msg::RailEntryEstimate>(
      "rail_entry_estimate", 10,
      [this](wr_interfaces::msg::RailEntryEstimate::SharedPtr message) {
        latest_estimate_ = *message;
        maybe_request_entry();
      });
    target_subscription_ = create_subscription<wr_interfaces::msg::TrackedPerson>(
      "tracked_person", 10,
      [this](wr_interfaces::msg::TrackedPerson::SharedPtr message) {
        latest_target_ = *message;
        maybe_request_entry();
      });
    factory_.registerBuilder<TransitionRequested>(
      "TransitionRequested",
      [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<TransitionRequested>(name, config, this);
      });
    factory_.registerBuilder<AutonomyEnabled>(
      "AutonomyEnabled",
      [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<AutonomyEnabled>(name, config, this);
      });
    factory_.registerBuilder<RunPersonFollow>(
      "RunPersonFollow",
      [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<RunPersonFollow>(name, config, this);
      });
    factory_.registerBuilder<RunRailTransition>(
      "RunRailTransition",
      [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<RunRailTransition>(name, config, this);
      });
    const auto xml = ament_index_cpp::get_package_share_directory("wr_task_manager") +
      "/behavior_trees/person_follow.xml";
    tree_ = factory_.createTreeFromFile(xml);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tick_rate_)),
      [this]() {
        const auto status = tree_.tickRoot();
        if (status != BT::NodeStatus::RUNNING) {
          tree_.haltTree();
        }
      });
  }

  bool transition_requested() const {return transition_request_.load() != 0;}
  bool autonomy_enabled() const {return autonomy_armed_.load();}
  AsyncState follow_state() const {return follow_state_.load();}
  AsyncState transition_state() const {return transition_state_.load();}

  void start_follow()
  {
    // generation 编号用于丢弃取消前已在途中的异步回调，
    // 防止旧 Action 回调抢回新任务的控制权。
    if (follow_state_.exchange(AsyncState::STARTING) != AsyncState::IDLE) {return;}
    const auto generation = ++follow_generation_;
    select_motion_source(wr_interfaces::srv::SetMotionSource::Request::PERSON_FOLLOW,
      [this, generation](bool accepted) {
        if (follow_generation_.load() != generation) {return;}
        if (!accepted || !follow_client_->action_server_is_ready()) {
          follow_state_.store(AsyncState::FAILURE);
          return;
        }
        Follow::Goal goal;
        goal.session_id = 0;
        goal.desired_front_gap = static_cast<float>(desired_gap_);
        goal.lost_wait_timeout = 60.0F;
        goal.search_radius = 3.0F;
        goal.allow_target_rebind = false;
        auto options = rclcpp_action::Client<Follow>::SendGoalOptions();
        options.goal_response_callback = [this, generation](FollowGoalHandle::SharedPtr handle) {
          if (follow_generation_.load() != generation) {return;}
          follow_state_.store(handle ? AsyncState::RUNNING : AsyncState::FAILURE);
        };
        options.result_callback = [this, generation](const FollowGoalHandle::WrappedResult & result) {
          if (follow_generation_.load() != generation) {return;}
          follow_state_.store(
            result.code == rclcpp_action::ResultCode::SUCCEEDED ?
            AsyncState::SUCCESS : AsyncState::FAILURE);
        };
        follow_client_->async_send_goal(goal, options);
      });
  }

  void stop_follow()
  {
    ++follow_generation_;
    follow_client_->async_cancel_all_goals();
    follow_state_.store(AsyncState::IDLE);
    if (!transition_requested()) {
      select_motion_source(wr_interfaces::srv::SetMotionSource::Request::NONE, nullptr);
    }
  }

  void start_transition()
  {
    // 过渡状态机启动前先将速度源切换为 RAIL_TRANSITION，
    // Action 结束后先释放速度源，再对行为树报告终态。
    if (transition_state_.exchange(AsyncState::STARTING) != AsyncState::IDLE) {return;}
    const auto generation = ++transition_generation_;
    const auto operation = transition_request_.load();
    select_motion_source(wr_interfaces::srv::SetMotionSource::Request::RAIL_TRANSITION,
      [this, operation, generation](bool accepted) {
        if (transition_generation_.load() != generation) {return;}
        if (!accepted || !transition_client_->action_server_is_ready()) {
          transition_state_.store(AsyncState::FAILURE);
          return;
        }
        Transition::Goal goal;
        goal.operation = operation;
        goal.timeout = static_cast<float>(transition_timeout_);
        auto options = rclcpp_action::Client<Transition>::SendGoalOptions();
        options.goal_response_callback =
          [this, generation](TransitionGoalHandle::SharedPtr handle) {
          if (transition_generation_.load() != generation) {return;}
          transition_state_.store(handle ? AsyncState::RUNNING : AsyncState::FAILURE);
        };
        options.result_callback =
          [this, generation](const TransitionGoalHandle::WrappedResult & result) {
          if (transition_generation_.load() != generation) {return;}
          transition_request_.store(0);
          const auto terminal_state =
            result.code == rclcpp_action::ResultCode::SUCCEEDED ?
            AsyncState::SUCCESS : AsyncState::FAILURE;
          select_motion_source(
            wr_interfaces::srv::SetMotionSource::Request::NONE,
            [this, terminal_state, generation](bool) {
              if (transition_generation_.load() == generation) {
                transition_state_.store(terminal_state);
              }
            });
        };
        transition_client_->async_send_goal(goal, options);
      });
  }

  void stop_transition()
  {
    ++transition_generation_;
    transition_client_->async_cancel_all_goals();
    transition_state_.store(AsyncState::IDLE);
    transition_request_.store(0);
    select_motion_source(wr_interfaces::srv::SetMotionSource::Request::NONE, nullptr);
  }

  void reset_follow() {follow_state_.store(AsyncState::IDLE);}
  void reset_transition() {transition_state_.store(AsyncState::IDLE);}

private:
  void select_motion_source(std::uint8_t source, std::function<void(bool)> callback)
  {
    if (!motion_client_->service_is_ready()) {
      if (callback) {callback(false);}
      return;
    }
    auto request = std::make_shared<wr_interfaces::srv::SetMotionSource::Request>();
    request->requested_source = source;
    motion_client_->async_send_request(
      request, [callback](rclcpp::Client<wr_interfaces::srv::SetMotionSource>::SharedFuture future) {
        if (callback) {callback(future.get()->accepted);}
      });
  }

  void maybe_request_entry()
  {
    // 只有轨道可通行、置信度达标且人已越过入口时才触发进轨。
    // entry_confidence 和 person_beyond_entry 是主要实车调参项。
    if (transition_request_.load() != 0 ||
      chassis_mode_.load() != wr_interfaces::msg::ChassisState::MODE_FREE_SURFACE ||
      latest_estimate_.traversability != wr_interfaces::msg::RailEntryEstimate::PASSABLE ||
      latest_estimate_.confidence < entry_confidence_ ||
      latest_target_.tracking_state != wr_interfaces::msg::TrackedPerson::TRACKED)
    {
      return;
    }
    const double dx = latest_target_.position.x - latest_estimate_.entry_pose.pose.position.x;
    const double dy = latest_target_.position.y - latest_estimate_.entry_pose.pose.position.y;
    const double along = dx * latest_estimate_.rail_direction.x +
      dy * latest_estimate_.rail_direction.y;
    const double across = std::abs(
      -dx * latest_estimate_.rail_direction.y + dy * latest_estimate_.rail_direction.x);
    if (along >= person_beyond_entry_ &&
      across <= 0.5 * latest_estimate_.rail_spacing + 0.15)
    {
      transition_request_.store(Transition::Goal::ENTER);
    }
  }

  double desired_gap_{0.30};
  double transition_timeout_{45.0};
  double entry_confidence_{0.70};
  double person_beyond_entry_{0.15};
  double tick_rate_{20.0};
  std::atomic<std::uint8_t> transition_request_{0};
  std::atomic<std::uint8_t> chassis_mode_{0};
  std::atomic<std::uint8_t> chassis_control_source_{0};
  std::atomic<bool> chassis_safe_{false};
  std::atomic<bool> autonomy_armed_{false};
  std::atomic<AsyncState> follow_state_{AsyncState::IDLE};
  std::atomic<AsyncState> transition_state_{AsyncState::IDLE};
  std::atomic<std::uint64_t> follow_generation_{0};
  std::atomic<std::uint64_t> transition_generation_{0};
  wr_interfaces::msg::RailEntryEstimate latest_estimate_;
  wr_interfaces::msg::TrackedPerson latest_target_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::Client<wr_interfaces::srv::SetMotionSource>::SharedPtr motion_client_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
  rclcpp_action::Client<Transition>::SharedPtr transition_client_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr request_subscription_;
  rclcpp::Subscription<wr_interfaces::msg::ChassisState>::SharedPtr chassis_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_subscription_;
  rclcpp::Subscription<wr_interfaces::msg::RailEntryEstimate>::SharedPtr estimate_subscription_;
  rclcpp::Subscription<wr_interfaces::msg::TrackedPerson>::SharedPtr target_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

BT::NodeStatus AutonomyEnabled::tick()
{
  return manager_->autonomy_enabled() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TransitionRequested::tick()
{
  return manager_->transition_requested() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RunPersonFollow::onStart()
{
  manager_->start_follow();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunPersonFollow::onRunning()
{
  const auto state = manager_->follow_state();
  if (state == TaskManagerNode::AsyncState::FAILURE) {
    manager_->reset_follow();
    return BT::NodeStatus::FAILURE;
  }
  if (state == TaskManagerNode::AsyncState::SUCCESS) {
    manager_->reset_follow();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void RunPersonFollow::onHalted() {manager_->stop_follow();}

BT::NodeStatus RunRailTransition::onStart()
{
  manager_->start_transition();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunRailTransition::onRunning()
{
  const auto state = manager_->transition_state();
  if (state == TaskManagerNode::AsyncState::FAILURE) {
    manager_->reset_transition();
    return BT::NodeStatus::FAILURE;
  }
  if (state == TaskManagerNode::AsyncState::SUCCESS) {
    manager_->reset_transition();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void RunRailTransition::onHalted() {manager_->stop_transition();}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
