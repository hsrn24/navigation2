#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_BACKUP_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_BACKUP_POSE_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>
#include <queue>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.h"

namespace nav2_behavior_tree
{

class GetBackupPoseAction : public BT::ActionNodeBase
{
public:
  GetBackupPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Previous robot pose (backup_distance behind)"),
      BT::InputPort<double>("backup_distance", 0.0, "Minimum distance from current pose to consider"),
      BT::InputPort<double>("dist_threshold", 0.1, "How often to check for a new backup pose"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  void publishPose(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  double backup_distance_, dist_threshold_;
  std::string global_frame_, robot_frame_;
  std::vector<geometry_msgs::msg::PoseStamped> previous_poses_;
  geometry_msgs::msg::PoseStamped previous_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_BACKUP_POSE_ACTION_HPP_
