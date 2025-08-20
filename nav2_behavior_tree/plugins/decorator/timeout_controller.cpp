#include "nav2_behavior_tree/plugins/decorator/timeout_controller.hpp"

namespace nav2_behavior_tree
{

TimeoutController::TimeoutController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  timeout_(0.0),
  timer_started_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("timeout", timeout_);
  if (timeout_ <= 0.0) {
    throw BT::BehaviorTreeException("TimeoutController node must have timeout > 0.0");
  }
}

BT::NodeStatus TimeoutController::tick()
{
  if (!timer_started_ || status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
    timer_started_ = true;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Timeout started counting until " << timeout_ << "s...");
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto elapsed = node_->now() - start_time_;
  if (elapsed.seconds() > timeout_) {
    timer_started_ = false;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Timeout elapsed, TimeoutController returning FAILURE!");
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus child_state = child_node_->executeTick();

  if (child_state == BT::NodeStatus::SUCCESS || child_state == BT::NodeStatus::FAILURE) {
    timer_started_ = false;
    return child_state;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeoutController>("TimeoutController");
}