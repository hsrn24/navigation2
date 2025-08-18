#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class TimeoutController : public BT::DecoratorNode
{
public:
  TimeoutController(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout", 0.0, "TimeoutController in seconds") };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double timeout_;
  bool timer_started_;
};

}  // namespace nav2_behavior_tree