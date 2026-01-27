// Copyright (c) 2024 Tianracer
// Licensed under the Apache License, Version 2.0

#include "tianracer_navigation_ros2/plugins/condition/is_goal_in_front_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace tianracer_behavior_tree
{

IsGoalInFrontCondition::IsGoalInFrontCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  robot_base_frame_("base_link"),
  global_frame_("map"),
  transform_tolerance_(0.1)
{
}

void IsGoalInFrontCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 从 blackboard 获取 TF buffer，如果没有则创建
  if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_)) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // 获取参数
  node_->get_parameter_or("robot_base_frame", robot_base_frame_, std::string("base_link"));
  node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
  node_->get_parameter_or("transform_tolerance", transform_tolerance_, 0.1);

  initialized_ = true;
}

double IsGoalInFrontCondition::computeAngleToGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  // 获取机器人当前位姿
  geometry_msgs::msg::TransformStamped robot_transform;
  try {
    robot_transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "无法获取机器人位姿: %s", ex.what());
    return 180.0;  // 返回最大角度，表示无法判断
  }

  // 计算目标相对于机器人的向量
  double dx = goal.pose.position.x - robot_transform.transform.translation.x;
  double dy = goal.pose.position.y - robot_transform.transform.translation.y;

  // 计算目标方向角（全局坐标系）
  double goal_angle = std::atan2(dy, dx);

  // 获取机器人当前朝向（从四元数转换）
  tf2::Quaternion q(
    robot_transform.transform.rotation.x,
    robot_transform.transform.rotation.y,
    robot_transform.transform.rotation.z,
    robot_transform.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // 计算目标相对于机器人朝向的角度差
  double angle_diff = goal_angle - yaw;

  // 归一化到 [-π, π]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  // 转换为度
  return angle_diff * 180.0 / M_PI;
}

BT::NodeStatus IsGoalInFrontCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  double angle_threshold = 90.0;
  getInput("angle_threshold", angle_threshold);

  double angle_to_goal = computeAngleToGoal();

  RCLCPP_DEBUG(node_->get_logger(),
    "IsGoalInFront: 目标角度 = %.1f°, 阈值 = %.1f°",
    angle_to_goal, angle_threshold);

  if (std::abs(angle_to_goal) <= angle_threshold) {
    RCLCPP_DEBUG(node_->get_logger(),
      "目标在前方 (角度: %.1f°)", angle_to_goal);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(node_->get_logger(),
      "目标在后方 (角度: %.1f°), 需要调整朝向", angle_to_goal);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace tianracer_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<tianracer_behavior_tree::IsGoalInFrontCondition>(name, config);
    };
  factory.registerBuilder<tianracer_behavior_tree::IsGoalInFrontCondition>("IsGoalInFront", builder);
}
