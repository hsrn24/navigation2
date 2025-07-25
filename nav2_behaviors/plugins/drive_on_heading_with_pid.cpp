#include "nav2_behaviors/plugins/drive_on_heading_with_pid.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::DriveOnHeadingWithPid<>, nav2_core::Behavior)

namespace nav2_behaviors
{

template<typename ActionT>
Status DriveOnHeadingWithPid<ActionT>::onRun(const std::shared_ptr<const typename ActionT::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
      RCLCPP_INFO(
      this->logger_,
      "DrivingOnHeadingWithPid in Y and Z not supported, will only move in X.");
      return Status::FAILED;
  }

  // Ensure that both the speed and direction have the same sign
  if (!((command->target.x > 0.0) == (command->speed > 0.0)) ) {
      RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
      return Status::FAILED;
  }

  command_x_ = command->target.x;
  command_speed_ = command->speed;
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

template<typename ActionT>
Status DriveOnHeadingWithPid<ActionT>::onCycleUpdate() {
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Exceeded time allowance before reaching the DriveOnHeadingWithPid goal - Exiting DriveOnHeadingWithPid");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(
    current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
    this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    this->stopRobot();
    return Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;
  cmd_vel->linear.x = command_speed_;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (pid_enabled_) {
    rclcpp::Time time_now = this->clock_->now();
    double angular_z_now;

    try {
      angular_z_now = angular_velocity_extrapolator_->getValueAtTime(time_now);
    } catch(const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->logger_, "Failed to extrapolate angular velocity into the future: " 
      << e.what() << " Skipping PID control");
      this->vel_pub_->publish(std::move(cmd_vel));
      return Status::RUNNING;
    }

    if (time_prev_.seconds() != 0.0) {
      double dt = (time_now - time_prev_).seconds();
      // Reset PID state if there was a break
      if (dt > 0.2) { // HARDCODED FOR NOW
        angular_velocity_pid_->reset();
        dt = 0;
      }

      double ez = cmd_vel->angular.z - angular_z_now;
      cmd_vel->angular.z =  cmd_vel->angular.z + angular_velocity_pid_->update(ez, dt);
    }

    time_prev_ = time_now;
  }
  
  if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeadingWithPid");
    return Status::FAILED;
  }

  this->vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}

template<typename ActionT>
bool DriveOnHeadingWithPid<ActionT>::isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const double diff_dist = abs(command_x_) - distance;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
    pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
    cycle_count++;

    if (diff_dist - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

template<typename ActionT>
void DriveOnHeadingWithPid<ActionT>::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  RCLCPP_INFO(this->logger_, "PID Controller: Configuring");

  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.enable", rclcpp::ParameterValue(false));
  pid_enabled_ = node->get_parameter("pid_control.enable").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.kp", rclcpp::ParameterValue(0.0));
  angular_velocity_pid_->setKp(node->get_parameter("pid_control.kp").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.ki", rclcpp::ParameterValue(0.0));
  angular_velocity_pid_->setKi(node->get_parameter("pid_control.ki").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.kd", rclcpp::ParameterValue(0.0));
  angular_velocity_pid_->setKd(node->get_parameter("pid_control.kd").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.integral_limit", rclcpp::ParameterValue(0.0));
  angular_velocity_pid_->setIntegralLimit(node->get_parameter("pid_control.integral_limit").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "pid_control.output_limit", rclcpp::ParameterValue(0.0));
  angular_velocity_pid_->setOutputLimit(node->get_parameter("pid_control.output_limit").as_double());

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DriveOnHeadingWithPid<ActionT>::dynamicParametersCallback, this, std::placeholders::_1));

  odom_filtered_sub_ = node->template create_subscription<nav_msgs::msg::Odometry>(
    "/robo_cart/odometry/filtered", rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      angular_velocity_extrapolator_->update(msg->twist.twist.angular.z, msg->header.stamp);
      extrapolated_odom_msg_ = *msg;
    });
  
#if DEBUG_MESSAGES
  extrapolated_odom_pub_ = node->template create_publisher<nav_msgs::msg::Odometry>("/robo_cart/odometry/extrapolated", rclcpp::SystemDefaultsQoS());
  pid_state_pub_ = node->template create_publisher<geometry_msgs::msg::Twist>("/robo_cart/pid_state", rclcpp::SystemDefaultsQoS());
#endif

  RCLCPP_INFO(this->logger_, "PID Controller: Activating");
#if DEBUG_MESSAGES
  extrapolated_odom_pub_->on_activate();
  pid_state_pub_->on_activate();
#endif
  angular_velocity_pid_->reset();
  angular_velocity_extrapolator_->reset();
  
  node_active_ = true;
}

template<typename ActionT>
void DriveOnHeadingWithPid<ActionT>::onCleanup()
{
  RCLCPP_INFO(this->logger_, "PID Controller: Deactivating");
#if DEBUG_MESSAGES
  extrapolated_odom_pub_->on_deactivate();
  pid_state_pub_->on_deactivate();
#endif
  dyn_params_handler_.reset();

  RCLCPP_INFO(this->logger_, "PID Controller: Cleaning up");
  odom_filtered_sub_.reset();
#if DEBUG_MESSAGES
  extrapolated_odom_pub_.reset();
  pid_state_pub_.reset();
#endif
}

template<typename ActionT>
rcl_interfaces::msg::SetParametersResult DriveOnHeadingWithPid<ActionT>::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rclcpp::PARAMETER_DOUBLE) {
      const auto& value = parameter.as_double();
      if (name == "pid_control.kp") {
        angular_velocity_pid_->setKp(value);
      } else if (name == "pid_control.ki") {
        angular_velocity_pid_->setKi(value);
      } else if (name == "pid_control.kd") {
        angular_velocity_pid_->setKd(value);
      } else if (name == "pid_control.integral_limit") {
        angular_velocity_pid_->setIntegralLimit(value);
      } else if (name == "pid_control.output_limit") {
        angular_velocity_pid_->setOutputLimit(value);
      }
    } else if (type == rclcpp::PARAMETER_BOOL) {
      const auto& value = parameter.as_bool();
      if (name == "pid_control.enable") {
        pid_enabled_ = value;
      }
    }
  }
  angular_velocity_pid_->reset();
  angular_velocity_extrapolator_->reset();

  return result;
}

} //namespace nav2_behaviors
