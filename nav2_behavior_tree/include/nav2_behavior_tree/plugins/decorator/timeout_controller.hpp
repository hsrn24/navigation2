#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child with every execution
 * until it returns SUCCESS/FAILURE or timeout is achieved.
 * 
 * Returns BT::NodeStatus::RUNNING,
 * its child final state if finished, or
 * BT::NodeStatus::FAILURE on timeout. 
 */
class TimeoutController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::TimeoutController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TimeoutController(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout", 0.0, "Time in seconds to return FAILURE if the child keeps on RUNNING") };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double timeout_;
  bool timer_started_;
};

}  // namespace nav2_behavior_tree
