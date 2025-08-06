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

#include <string>

#include "kaliber_behavior_tree/plugins/condition/is_should_recovery.hpp"

namespace kaliber_behavior_tree
{

IsShouldRecoveryCondition::IsShouldRecoveryCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  flag_topic_("/should_recovery"),
  is_should_recovery_(false)
{
  getInput("flag_topic", flag_topic_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  flag_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    flag_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsShouldRecoveryCondition::flagCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsShouldRecoveryCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_should_recovery_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsShouldRecoveryCondition::flagCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  is_should_recovery_ = msg->data;
}

}  // namespace kaliber_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kaliber_behavior_tree::IsShouldRecoveryCondition>("IsShouldRecovery");
}
