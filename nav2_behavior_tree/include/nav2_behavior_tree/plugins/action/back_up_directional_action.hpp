// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_DIRECTIONAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_DIRECTIONAL_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::BackUp
 */
class BackUpDirectionalAction : public BtActionNode<nav2_msgs::action::BackUp>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BackUpDirectionalAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BackUpDirectionalAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
        BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for reversing"),
        BT::InputPort<nav_msgs::msg::Path>("truncated_path", "Truncated local path which should be shortened to the desired search distance")
      });
  }
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_DIRECTIONAL_ACTION_HPP_
