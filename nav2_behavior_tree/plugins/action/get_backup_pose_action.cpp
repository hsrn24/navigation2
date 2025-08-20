#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_backup_pose_action.hpp"

namespace nav2_behavior_tree
{

GetBackupPoseAction::GetBackupPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  transform_tolerance_(0.1),
  backup_distance_(0.0),
  dist_threshold_(0.1),
  global_frame_("map"),
  robot_frame_("base_link")
{
  getInput("backup_distance", backup_distance_);
  getInput("dist_threshold", dist_threshold_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("backup_pose", 1);
}

inline BT::NodeStatus GetBackupPoseAction::tick()
{
  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Continue ticking until the distance exceeds predefined dist_threshold
  if (nav2_util::geometry_utils::euclidean_distance(current_pose, previous_pose_) < dist_threshold_) {
    return BT::NodeStatus::SUCCESS;  // Let the BT run unaffected
  }

  // Find backup pose within distance +/- dist_threshold and forget outdated ones
  for (auto it_pose = previous_poses_.begin(); it_pose != previous_poses_.end(); ) {
    double it_distance = nav2_util::geometry_utils::euclidean_distance(current_pose, *it_pose);
    if (it_distance >= backup_distance_ - dist_threshold_ && it_distance <= backup_distance_ + dist_threshold_) {
      setOutput("output_pose", *it_pose);
      publishPose(*it_pose);
      previous_poses_.erase(it_pose);
      break;
    } else if (it_distance > backup_distance_ + dist_threshold_) {
      it_pose = previous_poses_.erase(it_pose);
    } else {
      it_pose++;
    }
  }

  // Reset condition: use current pose if there's nothing within reach
  if (previous_poses_.size() == 0) {
      setOutput("output_pose", current_pose);
      publishPose(current_pose);
  }

  // Remember the current pose for future search
  previous_pose_ = current_pose;
  previous_poses_.emplace_back(current_pose);

  return BT::NodeStatus::SUCCESS;
}

void GetBackupPoseAction::publishPose(const geometry_msgs::msg::PoseStamped & pose)
{
  auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(pose);
  if (pose_publisher_->get_subscription_count() > 0) {
    pose_publisher_->publish(std::move(msg));
  }
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetBackupPoseAction>("GetBackupPose");
}
