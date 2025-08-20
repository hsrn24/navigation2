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

/**
 * @brief A BT::ActionNode that remembers previous poses of the robot.
 * If the robot travels more than the backup_distance, it outputs a pose
 * closest to this distance (+/- dist_threshold).
 * 
 * The node is activated every dist_threshold meters, so this parameter
 * denotes both the granularity of consecutive backup poses as well as
 * accuracy in keeping the backup_distance.
 * 
 * If there's no remembered pose within backup_distance that could be used later,
 * it outputs current pose as initial condition. The poses outside this distance
 * get erased from the memory.
 * 
 * Returns BT::NodeStatus::SUCCESS all the time, unless internal FAILURE happens.
 */
class GetBackupPoseAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GetBackupPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
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
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "A previous robot pose in a backup_distance behind current position"),
      BT::InputPort<double>("backup_distance", 0.0, "Distance of the backup pose behind the robot"),
      BT::InputPort<double>("dist_threshold", 0.1, "How often to check for a new backup pose"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief Publishes output pose as a PoseStamped message
   * 
   * @param pose 
   */
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
