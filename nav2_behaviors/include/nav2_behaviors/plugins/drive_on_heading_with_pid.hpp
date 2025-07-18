#ifndef NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_PID_HPP_
#define NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_PID_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_util/pid_controller.hpp"
#include "nav2_util/interpolate.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define DEBUG_MESSAGES 1

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::DriveOnHeadingWithPid
 * @brief An action server Behavior for spinning in
 */
template<typename ActionT = nav2_msgs::action::BackUp>
class DriveOnHeadingWithPid : public TimedBehavior<ActionT>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::DriveOnHeadingWithPid
   */
  DriveOnHeadingWithPid()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_x_(0.0),
    command_speed_(0.0),
    simulate_ahead_time_(0.0),
    pid_angular_velocity_(std::make_shared<nav2_util::PidController>()),
    interpolate_angular_z_(std::make_shared<nav2_util::Interpolate>())
  {}

  ~DriveOnHeadingWithPid() = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void onCleanup() override;

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_x_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double simulate_ahead_time_;

private:
  /**
  * @brief Subscription callback routine
  *
  */
  void cmdVelCb(const geometry_msgs::msg::Twist::UniquePtr msg);

  /**
   * @brief Dynamic reconfigure callback
   * @param parameters Parameter list to change
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(

    std::vector<rclcpp::Parameter> parameters);

  /// @brief Subscription of the cmd vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  /// @brief Subscription of the odometry filtered
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
  /// @brief Publisher of the cmd vel post slippage prevention
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
#if DEBUG_MESSAGES
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_int_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pid_state_pub_;
#endif
  /// @brief Handle to the dynamic parameters callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Controller activated state
  bool node_active_;
  bool pid_enabled_;

  // PID Controller implementations
  std::shared_ptr<nav2_util::PidController> pid_angular_velocity_;

  // Remembered state
  std::shared_ptr<nav2_util::Interpolate> interpolate_angular_z_;
  rclcpp::Time time_prev_;
  nav_msgs::msg::Odometry int_odom_msg_;

};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_PID_HPP_
