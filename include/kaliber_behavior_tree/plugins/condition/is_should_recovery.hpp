// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SHOULD_RECOVERY_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SHOULD_RECOVERY_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace kaliber_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to a flag topic and
 * returns SUCCESS when the flag is true and FAILURE otherwise
 */
class IsShouldRecoveryCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for kaliber_behavior_tree::IsShouldRecoveryCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsShouldRecoveryCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsShouldRecoveryCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "flag_topic", std::string("/should_recovery"), "Flag topic")
    };
  }

private:
  /**
   * @brief Callback function for flag topic
   * @param msg Shared pointer to std_msgs::msg::Bool message
   */
  void flagCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  std::string flag_topic_;
  bool is_should_recovery_;
};

}  // namespace kaliber_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SHOULD_RECOVERY_CONDITION_HPP_
