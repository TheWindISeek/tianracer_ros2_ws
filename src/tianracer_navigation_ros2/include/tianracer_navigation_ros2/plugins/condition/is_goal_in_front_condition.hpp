// Copyright (c) 2024 Tianracer
// Licensed under the Apache License, Version 2.0

#ifndef TIANRACER_NAVIGATION_ROS2__PLUGINS__CONDITION__IS_GOAL_IN_FRONT_CONDITION_HPP_
#define TIANRACER_NAVIGATION_ROS2__PLUGINS__CONDITION__IS_GOAL_IN_FRONT_CONDITION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tianracer_behavior_tree
{

/**
 * @brief 条件节点：判断目标是否在机器人前方
 *
 * 如果目标相对于机器人当前朝向的角度在 ±angle_threshold 范围内，返回 SUCCESS
 * 否则返回 FAILURE
 */
class IsGoalInFrontCondition : public BT::ConditionNode
{
public:
  IsGoalInFrontCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalInFrontCondition() = delete;
  ~IsGoalInFrontCondition() override = default;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "目标位置"),
      BT::InputPort<double>("angle_threshold", 10.0, "角度阈值（度），目标在前方±此角度内返回SUCCESS")
    };
  }

private:
  void initialize();

  /**
   * @brief 计算目标相对于机器人当前朝向的角度
   * @return 角度（度），正值表示左侧，负值表示右侧，绝对值表示偏离正前方的角度
   */
  double computeAngleToGoal();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool initialized_;
  std::string robot_base_frame_;
  std::string global_frame_;
  double transform_tolerance_;
};

}  // namespace tianracer_behavior_tree

#endif  // TIANRACER_NAVIGATION_ROS2__PLUGINS__CONDITION__IS_GOAL_IN_FRONT_CONDITION_HPP_
