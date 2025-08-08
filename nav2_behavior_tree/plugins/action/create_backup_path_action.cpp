#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/create_backup_path_action.hpp"

namespace nav2_behavior_tree
{

CreateBackupPath::CreateBackupPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

inline BT::NodeStatus CreateBackupPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  double distance_backward, path_resolution, transform_tolerance;
  std::string robot_frame, path_frame;
  geometry_msgs::msg::PoseStamped current_pose;

  // Get necessary inputs
  getInput("distance_backward", distance_backward);
  getInput("path_resolution", path_resolution);
  getInput("transform_tolerance", transform_tolerance);
  getInput("robot_frame", robot_frame);
  getInput("path_frame", path_frame);

  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, robot_frame, robot_frame, transform_tolerance))
  {
    RCLCPP_ERROR(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Failed to lookup current robot pose for %s", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Construct a backup path by backing off the current pose
  RCLCPP_INFO(
    config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
    "CreateBackupPath: calculating backup path to a point %.1fm behind...", distance_backward);
  nav_msgs::msg::Path backup_path;
  backup_path.header = current_pose.header;
  backup_path.header.frame_id = path_frame;
  backup_path.poses.reserve(distance_backward/path_resolution);
  geometry_msgs::msg::PoseStamped backup_pose(current_pose);

  for(double i = path_resolution; i <= distance_backward; i += path_resolution) {
    backup_pose.pose.position.x += path_resolution;
    backup_path.poses.push_back(backup_pose);
    nav2_util::transformPoseInTargetFrame(
      backup_path.poses.back(), backup_path.poses.back(),
      *tf_buffer_, path_frame, transform_tolerance);
  }

  setOutput("output_path", backup_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::CreateBackupPath>(
    "CreateBackupPath");
}
