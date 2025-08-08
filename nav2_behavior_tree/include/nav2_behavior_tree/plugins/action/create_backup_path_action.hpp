#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CREATE_BACKUP_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CREATE_BACKUP_PATH_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class CreateBackupPath : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::CreateBackupPath constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CreateBackupPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "distance_backward", 1.0,
        "Distance (in meters) of the backup path"),
      BT::InputPort<double>(
        "path_resolution", 0.1,
        "Resolution (in meters) to generate path points"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<std::string>(
        "path_frame", "map",
        "Output path frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.5,
        "Transform lookup tolerance"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Straight path going backwards from current robot pose"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CREATE_BACKUP_PATH_ACTION_HPP_
