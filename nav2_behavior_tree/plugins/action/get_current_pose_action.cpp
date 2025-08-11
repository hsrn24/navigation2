#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_current_pose_action.hpp"

namespace nav2_behavior_tree
{

GetCurrentPoseAction::GetCurrentPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  transform_tolerance_(0.1),
  min_distance_(0.0),
  resolution_(0.1),
  global_frame_("map"),
  robot_frame_("base_link")
{
  getInput("min_distance", min_distance_);
  getInput("resolution", resolution_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("backup_pose", 1);
}

inline BT::NodeStatus GetCurrentPoseAction::tick()
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

  // Continue ticking until the distance exceeds predefined resolution
  if (nav2_util::geometry_utils::euclidean_distance(current_pose, previous_pose_) < resolution_) {
    return BT::NodeStatus::SUCCESS;  // Let the BT run unaffected
  }
  RCLCPP_WARN_STREAM(node_->get_logger(), ">> Robot moved " << nav2_util::geometry_utils::euclidean_distance(current_pose, previous_pose_) << " from previous_pose");

  // Find backup pose within distance +/- resolution and forget outdated ones
  for (auto it_pose = previous_poses_.begin(); it_pose != previous_poses_.end(); ) {
    double it_distance = nav2_util::geometry_utils::euclidean_distance(current_pose, *it_pose);
    RCLCPP_WARN_STREAM(node_->get_logger(), ">>>>>> it_distance: " << it_distance);
    if (it_distance >= min_distance_ - resolution_ && it_distance <= min_distance_ + resolution_) {
      setOutput("output_pose", *it_pose);
      publishPose(*it_pose);
      previous_poses_.erase(it_pose);
      RCLCPP_WARN_STREAM(node_->get_logger(), ">>>>>>>>>>>> backup pose found!");
      break;
    } else if (it_distance > min_distance_ + resolution_) {
      it_pose = previous_poses_.erase(it_pose);
      RCLCPP_WARN_STREAM(node_->get_logger(), ">>>>>>>>>>>> it_pose erased!");
    } else {
      it_pose++;
    }
  }

  // Reset condition: use current pose if there's nothing within reach
  if (previous_poses_.size() == 0) {
      RCLCPP_WARN_STREAM(node_->get_logger(), >> "No valid backup pose found, resetting to current one!");
      setOutput("output_pose", current_pose);
      publishPose(current_pose);
  }

  // Remember the current pose for future search
  previous_pose_ = current_pose;
  previous_poses_.emplace_back(current_pose);

  return BT::NodeStatus::SUCCESS;
}

void GetCurrentPoseAction::publishPose(const geometry_msgs::msg::PoseStamped & pose)
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
  factory.registerNodeType<nav2_behavior_tree::GetCurrentPoseAction>("GetCurrentPose");
}
