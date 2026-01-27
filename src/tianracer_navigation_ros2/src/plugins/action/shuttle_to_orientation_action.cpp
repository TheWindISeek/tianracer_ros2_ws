// Copyright (c) 2024 Tianracer
// Licensed under the Apache License, Version 2.0

#include "tianracer_navigation_ros2/plugins/action/shuttle_to_orientation_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <limits>

namespace tianracer_behavior_tree
{

ShuttleToOrientationAction::ShuttleToOrientationAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf),
  initialized_(false),
  robot_base_frame_("base_link"),
  global_frame_("map"),
  transform_tolerance_(0.1),
  // 车辆几何参数（根据实测值和固件参数）
  wheelbase_(0.40),           // 轴距 L = 0.40m (固件 base_b * 2)
  track_width_(0.27),         // 轮距 W = 0.27m (固件 base_a * 2)
  front_overhang_(0.13),      // 前悬 = 0.13m
  rear_overhang_(0.07),       // 后悬 = 0.07m
  lidar_x_offset_(0.20),      // 激光雷达在 base_link 前方 0.20m
  state_(ShuttleState::DETECT_CLEAR_SIDE),
  turn_direction_(1),
  distance_traveled_(0.0),
  actual_forward_distance_(0.0),
  need_backup_first_(false),
  pre_forward_distance_(0.0),
  front_obstacle_distance_(0.0),
  theoretical_x_(0.0),
  theoretical_y_(0.0),
  theoretical_yaw_(0.0),
  initial_x_(0.0),
  initial_y_(0.0),
  initial_yaw_(0.0),
  shuttle_cycle_count_(0)
{
}

void ShuttleToOrientationAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 从 blackboard 获取 TF buffer
  if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_)) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // 创建发布者
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 创建专用的回调组（用于激光雷达订阅）
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // 使用与发布者匹配的 QoS：RELIABLE + VOLATILE
  rclcpp::QoS laser_qos(10);
  laser_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  laser_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", laser_qos,
    std::bind(&ShuttleToOrientationAction::laserCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), "Shuttle 订阅激光雷达话题: /scan (使用专用回调组)");

  // 获取参数
  node_->get_parameter_or("robot_base_frame", robot_base_frame_, std::string("base_link"));
  node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
  node_->get_parameter_or("transform_tolerance", transform_tolerance_, 0.1);

  initialized_ = true;
}

void ShuttleToOrientationAction::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
  // 只在第一次收到数据时打印
  static bool first_scan = true;
  if (first_scan) {
    RCLCPP_INFO(node_->get_logger(), "收到激光雷达数据，共 %zu 个点", msg->ranges.size());
    first_scan = false;
  }
}

void ShuttleToOrientationAction::spinLaserCallback()
{
  // 手动 spin 回调组以处理激光雷达消息
  if (callback_group_) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group_, node_->get_node_base_interface());
    executor.spin_some(std::chrono::milliseconds(10));
  }
}

double ShuttleToOrientationAction::computeAngleToGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  geometry_msgs::msg::TransformStamped robot_transform;
  try {
    robot_transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "无法获取机器人位姿: %s", ex.what());
    return 180.0;
  }

  // 获取机器人当前 yaw
  tf2::Quaternion robot_q(
    robot_transform.transform.rotation.x,
    robot_transform.transform.rotation.y,
    robot_transform.transform.rotation.z,
    robot_transform.transform.rotation.w);
  tf2::Matrix3x3 robot_m(robot_q);
  double robot_roll, robot_pitch, robot_yaw;
  robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);

  // 获取目标的 orientation (yaw) - 这是机器人最终需要达到的朝向
  tf2::Quaternion goal_q(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w);
  tf2::Matrix3x3 goal_m(goal_q);
  double goal_roll, goal_pitch, goal_yaw;
  goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);

  // 计算角度差：机器人需要转多少度才能与目标 orientation 一致
  double angle_diff = goal_yaw - robot_yaw;
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  // 打印调试信息
  double robot_x = robot_transform.transform.translation.x;
  double robot_y = robot_transform.transform.translation.y;

  RCLCPP_DEBUG(node_->get_logger(),
    "位置: 机器人(%.2f, %.2f), 目标(%.2f, %.2f)",
    robot_x, robot_y, goal.pose.position.x, goal.pose.position.y);

  RCLCPP_DEBUG(node_->get_logger(),
    "朝向计算: 机器人yaw=%.1f°, 目标yaw=%.1f°, angle_diff=%.1f°",
    robot_yaw * 180.0 / M_PI, goal_yaw * 180.0 / M_PI, angle_diff * 180.0 / M_PI);

  return angle_diff * 180.0 / M_PI;
}

bool ShuttleToOrientationAction::hasLaserData()
{
  // 先手动 spin 以获取最新的激光数据
  spinLaserCallback();

  std::lock_guard<std::mutex> lock(scan_mutex_);
  return latest_scan_ != nullptr;
}

int ShuttleToOrientationAction::detectClearSide()
{
  std::lock_guard<std::mutex> lock(scan_mutex_);

  if (!latest_scan_) {
    RCLCPP_WARN(node_->get_logger(), "没有激光数据，无法检测");
    return 0;  // 返回 0 表示无法检测
  }

  int num_readings = latest_scan_->ranges.size();
  int center = num_readings / 2;  // 正前方
  int quarter = num_readings / 4;  // 90° 范围

  // 激光雷达坐标系：
  // - angle_min (-135°) 在 index 0，右后方
  // - angle_max (+135°) 在 index 末尾，左后方
  // - index 中间是 0°，正前方
  // - 负角度（index < center）= 右侧
  // - 正角度（index > center）= 左侧

  // 左侧：从正前方到左边 (0° 到 +90°)
  // index: center 到 center + quarter
  double left_sum = 0.0;
  int left_count = 0;
  int left_start = center;
  int left_end = std::min(center + quarter, num_readings);
  for (int i = left_start; i < left_end; ++i) {
    if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.05) {
      left_sum += latest_scan_->ranges[i];
      left_count++;
    }
  }

  // 右侧：从正前方到右边 (0° 到 -90°)
  // index: center - quarter 到 center
  double right_sum = 0.0;
  int right_count = 0;
  int right_start = std::max(center - quarter, 0);
  int right_end = center;
  for (int i = right_start; i < right_end; ++i) {
    if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.05) {
      right_sum += latest_scan_->ranges[i];
      right_count++;
    }
  }

  double left_avg = (left_count > 0) ? left_sum / left_count : 0.0;
  double right_avg = (right_count > 0) ? right_sum / right_count : 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "激光检测: 左侧(+角度)平均=%.2fm (%d点), 右侧(-角度)平均=%.2fm (%d点)",
    left_avg, left_count, right_avg, right_count);

  // 返回更空旷的一侧：1=左转, -1=右转
  return (left_avg >= right_avg) ? 1 : -1;
}

double ShuttleToOrientationAction::computeAngularVelocity(
  double linear_vel, double steering_angle_deg)
{
  // 阿克曼运动学: ω = v * tan(δ) / L
  double steering_rad = steering_angle_deg * M_PI / 180.0;
  return linear_vel * std::tan(steering_rad) / wheelbase_;
}

double ShuttleToOrientationAction::computeTurningRadius(double steering_angle_deg)
{
  // 转弯半径 R = L / tan(δ)
  double steering_rad = std::abs(steering_angle_deg) * M_PI / 180.0;
  if (steering_rad < 0.01) {
    return std::numeric_limits<double>::max();  // 直行
  }
  return wheelbase_ / std::tan(steering_rad);
}

double ShuttleToOrientationAction::computeAngleChange(double distance, double steering_angle_deg)
{
  // 行驶距离 d 后转过的角度 θ = d / R = d * tan(δ) / L
  double R = computeTurningRadius(steering_angle_deg);
  if (R > 1000.0) return 0.0;  // 直行时角度不变
  return distance / R;  // 返回弧度
}

void ShuttleToOrientationAction::computeSweptRadii(
  double steering_angle_deg, int turn_dir,
  double& r_inner, double& r_outer)
{
  // 计算车辆转弯时的内半径和外半径
  // turn_dir: 1=左转, -1=右转
  
  double R = computeTurningRadius(steering_angle_deg);
  double half_width = track_width_ / 2.0;
  
  // 车身四个角相对于后轮轴中心的坐标
  double front_x = wheelbase_ + front_overhang_;  // 车头 x = 0.53m
  double rear_x = -rear_overhang_;                 // 车尾 x = -0.07m
  
  if (turn_dir > 0) {
    // 左转：圆心在左侧 (0, R)
    // 内侧 = 左后角，外侧 = 右前角
    double inner_y = R - half_width;  // 左后角到圆心的 y 距离
    double outer_y = R + half_width;  // 右前角到圆心的 y 距离
    
    r_inner = std::sqrt(rear_x * rear_x + inner_y * inner_y);
    r_outer = std::sqrt(front_x * front_x + outer_y * outer_y);
  } else {
    // 右转：圆心在右侧 (0, -R)
    // 内侧 = 右后角，外侧 = 左前角
    double inner_y = R - half_width;
    double outer_y = R + half_width;
    
    r_inner = std::sqrt(rear_x * rear_x + inner_y * inner_y);
    r_outer = std::sqrt(front_x * front_x + outer_y * outer_y);
  }
}

double ShuttleToOrientationAction::computeMaxForwardFromLaser(
  double steering_angle_deg, int turn_dir)
{
  // 根据激光雷达数据计算最大可前进距离
  // 检查转弯扫掠区域内是否有障碍物
  
  std::lock_guard<std::mutex> lock(scan_mutex_);
  
  if (!latest_scan_) {
    return forward_distance_;  // 无激光数据，返回默认值
  }
  
  double R = computeTurningRadius(steering_angle_deg);
  double r_inner, r_outer;
  computeSweptRadii(steering_angle_deg, turn_dir, r_inner, r_outer);
  
  // 圆心位置（相对于后轮轴中心）
  double center_x = 0.0;
  double center_y = (turn_dir > 0) ? R : -R;
  
  // 激光雷达位置（相对于后轮轴中心）
  double lidar_x = lidar_x_offset_;  // 0.20m
  double lidar_y = 0.0;
  
  double min_arc_distance = forward_distance_;  // 初始化为最大值
  
  int num_readings = latest_scan_->ranges.size();
  double angle_min = latest_scan_->angle_min;
  double angle_increment = latest_scan_->angle_increment;
  
  for (int i = 0; i < num_readings; ++i) {
    double range = latest_scan_->ranges[i];
    if (!std::isfinite(range) || range < 0.1) {
      continue;  // 忽略无效数据
    }
    
    // 激光点在激光雷达坐标系中的位置
    double angle = angle_min + i * angle_increment;
    double point_x_lidar = range * std::cos(angle);
    double point_y_lidar = range * std::sin(angle);
    
    // 转换到后轮轴中心坐标系
    double point_x = point_x_lidar + lidar_x;
    double point_y = point_y_lidar + lidar_y;
    
    // 计算激光点到圆心的距离
    double dx = point_x - center_x;
    double dy = point_y - center_y;
    double dist_to_center = std::sqrt(dx * dx + dy * dy);
    
    // 检查是否在扫掠区域内（环形区域）
    // 添加一些余量
    double margin = 0.05;
    if (dist_to_center < r_inner - margin || dist_to_center > r_outer + margin) {
      continue;  // 不在扫掠区域内
    }
    
    // 检查是否在转向方向的前方
    // 左转时，检查左前方和正前方区域
    // 右转时，检查右前方和正前方区域
    double half_w = track_width_ / 2.0;
    if (turn_dir > 0) {
      // 左转：检查左侧和前方
      if (point_y < -half_w * 0.5 || point_x < -rear_overhang_) {
        continue;  // 不在有效区域
      }
    } else {
      // 右转：检查右侧和前方
      if (point_y > half_w * 0.5 || point_x < -rear_overhang_) {
        continue;  // 不在有效区域
      }
    }
    
    // 计算碰到这个点时车辆需要转过的角度
    // 激光点相对于圆心的角度
    double point_angle = std::atan2(dx, -dy * turn_dir);  // 注意坐标变换
    
    // 车辆初始时，后轮轴中心相对于圆心的角度
    double initial_angle = std::atan2(center_x, -center_y * turn_dir);
    
    // 需要转过的角度
    double turn_angle = point_angle - initial_angle;
    if (turn_angle < 0) turn_angle += 2 * M_PI;
    if (turn_angle > M_PI) continue;  // 太大了，不考虑
    
    // 转换为弧长（前进距离）
    double arc_distance = turn_angle * R;
    
    // 减去安全余量
    arc_distance -= 0.10;  // 10cm 安全余量
    
    if (arc_distance > 0.1 && arc_distance < min_arc_distance) {
      min_arc_distance = arc_distance;
      
      RCLCPP_DEBUG(node_->get_logger(),
        "障碍点: (%.2f, %.2f), 到圆心距离=%.2fm, 扫掠范围=[%.2f, %.2f], 可行距离=%.2fm",
        point_x, point_y, dist_to_center, r_inner, r_outer, arc_distance);
    }
  }
  
  return std::max(0.1, min_arc_distance);
}

double ShuttleToOrientationAction::computeMaxForwardDistance(
  double obstacle_distance, double steering_angle_deg)
{
  // 这个函数现在主要用于简单估算，实际计算在 computeMaxForwardFromLaser 中
  double R = computeTurningRadius(steering_angle_deg);
  double front_length = wheelbase_ + front_overhang_;
  double safety_margin = 0.15;
  double available_space = obstacle_distance - front_length - safety_margin;
  return std::max(0.1, std::min(available_space, forward_distance_));
}

double ShuttleToOrientationAction::getMinDistanceInSector(
  double angle_start_deg, double angle_end_deg)
{
  // 获取指定角度范围内的最小障碍物距离
  // 角度以激光雷达坐标系为准：0°=正前方，正值=左侧，负值=右侧
  
  std::lock_guard<std::mutex> lock(scan_mutex_);
  
  if (!latest_scan_) {
    return 0.0;
  }
  
  double angle_min = latest_scan_->angle_min;
  double angle_max = latest_scan_->angle_max;
  double angle_increment = latest_scan_->angle_increment;
  int num_readings = latest_scan_->ranges.size();
  
  // 转换为弧度
  double start_rad = angle_start_deg * M_PI / 180.0;
  double end_rad = angle_end_deg * M_PI / 180.0;
  
  // 确保 start < end
  if (start_rad > end_rad) {
    std::swap(start_rad, end_rad);
  }
  
  // 计算索引范围
  int start_idx = std::max(0, (int)((start_rad - angle_min) / angle_increment));
  int end_idx = std::min(num_readings - 1, (int)((end_rad - angle_min) / angle_increment));
  
  double min_dist = std::numeric_limits<double>::max();
  
  for (int i = start_idx; i <= end_idx; ++i) {
    double range = latest_scan_->ranges[i];
    if (std::isfinite(range) && range > 0.05 && range < min_dist) {
      min_dist = range;
    }
  }
  
  return (min_dist == std::numeric_limits<double>::max()) ? 10.0 : min_dist;
}

bool ShuttleToOrientationAction::updateForwardDistance()
{
  // 先 spin 以获取最新的激光数据
  spinLaserCallback();
  
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!latest_scan_) {
      RCLCPP_WARN(node_->get_logger(), "没有激光数据，无法更新前进距离");
      return false;
    }
  }

  // 1. 检测正前方的障碍物距离（±15°范围）
  double front_dist = getMinDistanceInSector(-15.0, 15.0);
  
  // 2. 检测转向方向的障碍物距离
  double turn_dist;
  if (turn_direction_ > 0) {
    // 左转：检测左前方（15° ~ 60°）
    turn_dist = getMinDistanceInSector(15.0, 60.0);
  } else {
    // 右转：检测右前方（-60° ~ -15°）
    turn_dist = getMinDistanceInSector(-60.0, -15.0);
  }
  
  // 减去激光雷达到车头的距离（激光雷达在车身前方，但不是最前端）
  // 激光雷达位置 0.20m，车头在 0.53m，所以车头比激光雷达前 0.33m
  double lidar_to_front = 0.33;
  double effective_front = front_dist - lidar_to_front;
  double effective_turn = turn_dist - lidar_to_front;
  
  // 记录前方障碍物有效距离（用于判断是否可以预前进）
  front_obstacle_distance_ = effective_front;
  
  // 取两个方向的最小值作为可用空间
  double available_space = std::min(effective_front, effective_turn);
  
  RCLCPP_INFO(node_->get_logger(), 
    "【距离检测】正前方=%.2fm, %s=%.2fm, 有效空间=%.2fm",
    front_dist, (turn_direction_ > 0) ? "左前方" : "右前方", turn_dist, available_space);
  
  // 3. 如果空间太小，需要标记需要先后退
  need_backup_first_ = (available_space < 0.15);
  
  // 4. 阶梯式判断可前进距离
  if (available_space >= 0.35) {
    max_forward_distance_ = 0.30;
  } else if (available_space >= 0.30) {
    max_forward_distance_ = 0.25;
  } else if (available_space >= 0.25) {
    max_forward_distance_ = 0.20;
  } else if (available_space >= 0.20) {
    max_forward_distance_ = 0.15;
  } else if (available_space >= 0.15) {
    max_forward_distance_ = 0.10;
  } else {
    // 空间不足，需要先后退
    max_forward_distance_ = 0.10;
    need_backup_first_ = true;
  }
  
  // 不超过参数设定的最大值
  max_forward_distance_ = std::min(max_forward_distance_, forward_distance_);

  // 计算这次前进能累积的角度变化
  double angle_change_deg = computeAngleChange(max_forward_distance_, steering_angle_) * 180.0 / M_PI;
  
  RCLCPP_INFO(node_->get_logger(), 
    "【前进距离】可行驶=%.2fm, 预计角度变化=%.1f°, 需要先后退=%s",
    max_forward_distance_, angle_change_deg * 2, need_backup_first_ ? "是" : "否");

  return true;
}

void ShuttleToOrientationAction::publishVelocity(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

void ShuttleToOrientationAction::stopRobot()
{
  publishVelocity(0.0, 0.0);
}

void ShuttleToOrientationAction::computeTheoreticalMotion(
  double distance, double steering_angle_deg, bool is_forward,
  double& dx, double& dy, double& dtheta)
{
  // 阿克曼运动学：
  // 转弯半径 R = L / tan(δ)
  // 角度变化 θ = d / R = d * tan(δ) / L
  // 
  // 前进时（在车体坐标系下）：
  //   如果转向角 > 0（左转），车辆绕左侧圆心转动
  //   dx ≈ R * sin(θ) ≈ d（小角度近似）
  //   dy ≈ R * (1 - cos(θ))（向转弯内侧偏移）
  //
  // 后退+同向转向时（阿克曼特性）：
  //   dx 为负（向后）
  //   dtheta 与前进时同向（继续累积转向）

  double steering_rad = steering_angle_deg * M_PI / 180.0;
  double R = wheelbase_ / std::tan(std::abs(steering_rad));  // 转弯半径
  double theta = distance / R;  // 转过的角度
  
  if (is_forward) {
    // 前进：dx正向，角度累积
    dx = R * std::sin(theta);
    dy = R * (1.0 - std::cos(theta));  // 侧向偏移（向转弯方向）
    dtheta = theta;
  } else {
    // 后退+同向转向：dx负向，但角度继续同向累积
    dx = -R * std::sin(theta);  // 向后
    dy = -R * (1.0 - std::cos(theta));  // 侧向偏移（与前进时相反方向）
    dtheta = theta;  // 角度仍然累积（阿克曼倒车特性）
  }
  
  RCLCPP_DEBUG(node_->get_logger(),
    "【理论运动】距离=%.3f, %s, R=%.3f, θ=%.1f°, dx=%.3f, dy=%.3f",
    distance, is_forward ? "前进" : "后退", R, theta * 180.0 / M_PI, dx, dy);
}

void ShuttleToOrientationAction::correctTFDrift()
{
  // 获取当前 TF 实际值
  geometry_msgs::msg::TransformStamped current_tf;
  try {
    current_tf = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF校正失败，无法获取当前位置: %s", ex.what());
    return;
  }
  
  double actual_x = current_tf.transform.translation.x;
  double actual_y = current_tf.transform.translation.y;
  
  tf2::Quaternion q(
    current_tf.transform.rotation.x,
    current_tf.transform.rotation.y,
    current_tf.transform.rotation.z,
    current_tf.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, actual_yaw;
  m.getRPY(roll, pitch, actual_yaw);
  
  // 计算差异
  double dx = actual_x - theoretical_x_;
  double dy = actual_y - theoretical_y_;
  double position_error = std::sqrt(dx * dx + dy * dy);
  
  // 计算角度差异（处理 -π 到 π 的边界）
  double yaw_error = actual_yaw - theoretical_yaw_;
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
  
  RCLCPP_INFO(node_->get_logger(),
    "【TF校正】周期%d: 理论(%.3f,%.3f,%.1f°) vs 实际(%.3f,%.3f,%.1f°), "
    "位置误差=%.3fm, 角度误差=%.1f°",
    shuttle_cycle_count_,
    theoretical_x_, theoretical_y_, theoretical_yaw_ * 180.0 / M_PI,
    actual_x, actual_y, actual_yaw * 180.0 / M_PI,
    position_error, yaw_error * 180.0 / M_PI);
  
  // 如果误差超过阈值，进行修正
  if (position_error > TF_POSITION_TOLERANCE || std::abs(yaw_error) > TF_YAW_TOLERANCE) {
    // 策略：取理论值和实际值的中间值
    // 位置：使用加权平均（偏向实际值，因为TF通常更可信）
    double weight_actual = 0.7;  // 实际值权重
    double corrected_x = theoretical_x_ * (1 - weight_actual) + actual_x * weight_actual;
    double corrected_y = theoretical_y_ * (1 - weight_actual) + actual_y * weight_actual;
    
    // 朝向：取中间值
    double corrected_yaw = theoretical_yaw_ + yaw_error * 0.5;
    
    RCLCPP_WARN(node_->get_logger(),
      "【TF修正】误差过大! 修正后位置(%.3f,%.3f,%.1f°)",
      corrected_x, corrected_y, corrected_yaw * 180.0 / M_PI);
    
    // 更新理论值为修正后的值
    theoretical_x_ = corrected_x;
    theoretical_y_ = corrected_y;
    theoretical_yaw_ = corrected_yaw;
    
    // TODO: 如果有重定位服务（如 AMCL 的 global_localization），可以在这里调用
    // 但朝向可能无法精确重定位，所以这里采用折中方案
  }
}

BT::NodeStatus ShuttleToOrientationAction::onStart()
{
  if (!initialized_) {
    initialize();
  }

  // 读取参数
  getInput("angle_threshold", angle_threshold_);
  getInput("clearance_threshold", clearance_threshold_);
  getInput("forward_distance", forward_distance_);
  getInput("backward_distance", backward_distance_);
  getInput("speed", speed_);
  getInput("steering_angle", steering_angle_);
  getInput("timeout", timeout_);

  // 初始化状态
  state_ = ShuttleState::DETECT_CLEAR_SIDE;
  start_time_ = std::chrono::steady_clock::now();
  state_start_time_ = std::chrono::steady_clock::now();  // 必须初始化！
  distance_traveled_ = 0.0;
  turn_direction_ = 0;  // 重置转向方向
  max_forward_distance_ = forward_distance_;  // 初始化前进距离
  need_backup_first_ = false;  // 重置后退标志
  pre_forward_distance_ = 0.0;  // 重置预前进距离
  front_obstacle_distance_ = 0.0;  // 重置前方障碍物距离
  shuttle_cycle_count_ = 0;  // 重置周期计数
  
  // 初始化理论位置（稍后在获取TF后设置）
  theoretical_x_ = 0.0;
  theoretical_y_ = 0.0;
  theoretical_yaw_ = 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "开始 Shuttle 调整朝向 (阈值: %.1f°, 超时: %.1fs)",
    angle_threshold_, timeout_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ShuttleToOrientationAction::onRunning()
{
  // 检查超时
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  double elapsed_sec = std::chrono::duration<double>(elapsed).count();
  if (elapsed_sec > timeout_) {
    RCLCPP_WARN(node_->get_logger(), "Shuttle 调整超时 (%.1fs)", elapsed_sec);
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }

  // 首先检查目标是否已在前方
  double angle_to_goal = computeAngleToGoal();
  if (std::abs(angle_to_goal) <= angle_threshold_) {
    RCLCPP_INFO(node_->get_logger(),
      "目标已在前方 (角度: %.1f°), Shuttle 完成", angle_to_goal);
    stopRobot();
    return BT::NodeStatus::SUCCESS;
  }

  // 状态机
  auto state_elapsed = std::chrono::steady_clock::now() - state_start_time_;
  double state_elapsed_sec = std::chrono::duration<double>(state_elapsed).count();

  switch (state_) {
    case ShuttleState::DETECT_CLEAR_SIDE:
      {
        // 等待激光数据，最多等 3 秒
        if (!hasLaserData()) {
          if (state_elapsed_sec > 3.0) {
            RCLCPP_ERROR(node_->get_logger(), "等待激光数据超时 (3s)，无法执行 Shuttle");
            return BT::NodeStatus::FAILURE;
          }
          // 继续等待
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "等待激光数据... (%.1fs)", state_elapsed_sec);
          break;
        }

        // 有激光数据了，检测空旷方向
        turn_direction_ = detectClearSide();
        if (turn_direction_ == 0) {
          RCLCPP_WARN(node_->get_logger(), "无法检测空旷方向，重试...");
          break;
        }

        RCLCPP_INFO(node_->get_logger(),
          "检测到 %s 更空旷，将向 %s 转向",
          (turn_direction_ > 0) ? "左侧" : "右侧",
          (turn_direction_ > 0) ? "左" : "右");

        // 获取初始前进距离（同时检测是否需要先后退）
        if (!updateForwardDistance()) {
          RCLCPP_WARN(node_->get_logger(), "无法获取前进距离，重试...");
          break;
        }

        // 记录初始位置
        try {
          last_transform_ = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));
          
          // 初始化理论位置跟踪
          initial_x_ = last_transform_.transform.translation.x;
          initial_y_ = last_transform_.transform.translation.y;
          tf2::Quaternion q(
            last_transform_.transform.rotation.x,
            last_transform_.transform.rotation.y,
            last_transform_.transform.rotation.z,
            last_transform_.transform.rotation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          initial_yaw_ = yaw;
          
          // 理论值初始化为与初始位置相同
          theoretical_x_ = initial_x_;
          theoretical_y_ = initial_y_;
          theoretical_yaw_ = initial_yaw_;
          
          RCLCPP_INFO(node_->get_logger(), 
            "【理论位置初始化】x=%.3f, y=%.3f, yaw=%.1f°",
            initial_x_, initial_y_, initial_yaw_ * 180.0 / M_PI);
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN(node_->get_logger(), "无法获取初始位置: %s", ex.what());
        }

        // 根据空间情况决定下一个状态
        if (need_backup_first_) {
          // 检查前方是否还有空间可以先前进（> 0.3m时先前进利用空间）
          if (front_obstacle_distance_ > 0.30) {
            pre_forward_distance_ = front_obstacle_distance_ - 0.05;  // 留5cm安全距离
            pre_forward_distance_ = std::min(pre_forward_distance_, 0.50);  // 最大0.5m
            RCLCPP_INFO(node_->get_logger(), 
              "前方有%.2fm空间，先直线前进%.2fm再后退", 
              front_obstacle_distance_, pre_forward_distance_);
            state_ = ShuttleState::PRE_FORWARD_STRAIGHT;
          } else {
            RCLCPP_INFO(node_->get_logger(), "前方空间不足(%.2fm)，需要先后退 0.1m", 
              front_obstacle_distance_);
            state_ = ShuttleState::INITIAL_BACKUP;
          }
        } else {
          state_ = ShuttleState::FORWARD_TURN;
        }
        state_start_time_ = std::chrono::steady_clock::now();
        distance_traveled_ = 0.0;
      }
      break;

    case ShuttleState::PRE_FORWARD_STRAIGHT:
      {
        // 后退前先直线前进，利用前方空间
        try {
          auto current_transform = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));

          double dx = current_transform.transform.translation.x -
                      last_transform_.transform.translation.x;
          double dy = current_transform.transform.translation.y -
                      last_transform_.transform.translation.y;
          distance_traveled_ = std::sqrt(dx * dx + dy * dy);
        } catch (tf2::TransformException & ex) {
          distance_traveled_ = speed_ * state_elapsed_sec;
        }

        if (distance_traveled_ >= pre_forward_distance_) {
          RCLCPP_INFO(node_->get_logger(), 
            "【预前进完成】距离: %.2fm，现在可以后退更多空间", distance_traveled_);
          stopRobot();
          
          // 更新理论位置（直线前进，只有x变化）
          theoretical_x_ += distance_traveled_ * std::cos(theoretical_yaw_);
          theoretical_y_ += distance_traveled_ * std::sin(theoretical_yaw_);
          
          state_ = ShuttleState::PAUSE_AFTER_PRE_FORWARD;
          state_start_time_ = std::chrono::steady_clock::now();
        } else {
          // 直线前进（不转向）
          publishVelocity(speed_, 0.0);
        }
      }
      break;

    case ShuttleState::PAUSE_AFTER_PRE_FORWARD:
      {
        if (state_elapsed_sec >= 0.3) {
          // 记录新的位置（用于后续后退距离计算）
          try {
            last_transform_ = tf_buffer_->lookupTransform(
              global_frame_, robot_base_frame_,
              tf2::TimePointZero,
              tf2::durationFromSec(transform_tolerance_));
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "无法获取位置: %s", ex.what());
          }
          
          // 后退距离 = 预前进距离 + 原本要后退的0.1m
          double extended_backup = pre_forward_distance_ + 0.10;
          RCLCPP_INFO(node_->get_logger(), 
            "开始后退 %.2fm（预前进%.2fm + 原后退0.1m）", 
            extended_backup, pre_forward_distance_);
          
          // 暂存扩展后退距离，用于 INITIAL_BACKUP 状态
          actual_forward_distance_ = extended_backup;  // 复用这个变量
          
          state_ = ShuttleState::INITIAL_BACKUP;
          state_start_time_ = std::chrono::steady_clock::now();
          distance_traveled_ = 0.0;
        }
      }
      break;

    case ShuttleState::INITIAL_BACKUP:
      {
        // 空间不足时先后退
        try {
          auto current_transform = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));

          double dx = current_transform.transform.translation.x -
                      last_transform_.transform.translation.x;
          double dy = current_transform.transform.translation.y -
                      last_transform_.transform.translation.y;
          distance_traveled_ = std::sqrt(dx * dx + dy * dy);
        } catch (tf2::TransformException & ex) {
          distance_traveled_ = speed_ * state_elapsed_sec;
        }

        // 后退目标：如果经过了预前进，使用扩展距离；否则使用默认0.1m
        double backup_target = (pre_forward_distance_ > 0.0) ? actual_forward_distance_ : 0.10;

        if (distance_traveled_ >= backup_target) {
          RCLCPP_INFO(node_->get_logger(), "初始后退完成 (距离: %.2fm)", distance_traveled_);
          stopRobot();
          
          // 更新理论位置（直线后退）
          theoretical_x_ -= distance_traveled_ * std::cos(theoretical_yaw_);
          theoretical_y_ -= distance_traveled_ * std::sin(theoretical_yaw_);
          
          state_ = ShuttleState::PAUSE_AFTER_INITIAL_BACKUP;
          state_start_time_ = std::chrono::steady_clock::now();
        } else {
          // 直线后退（不转向）
          publishVelocity(-speed_, 0.0);
        }
      }
      break;

    case ShuttleState::PAUSE_AFTER_INITIAL_BACKUP:
      {
        if (state_elapsed_sec >= 0.3) {
          // 重新检测前进距离
          if (!updateForwardDistance()) {
            RCLCPP_WARN(node_->get_logger(), "无法获取前进距离，使用默认值");
          }
          
          // 记录新的位置
          try {
            last_transform_ = tf_buffer_->lookupTransform(
              global_frame_, robot_base_frame_,
              tf2::TimePointZero,
              tf2::durationFromSec(transform_tolerance_));
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "无法获取位置: %s", ex.what());
          }
          
          state_ = ShuttleState::FORWARD_TURN;
          state_start_time_ = std::chrono::steady_clock::now();
          distance_traveled_ = 0.0;
        }
      }
      break;

    case ShuttleState::FORWARD_TURN:
      {
        // 计算已行驶距离
        try {
          auto current_transform = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));

          double dx = current_transform.transform.translation.x -
                      last_transform_.transform.translation.x;
          double dy = current_transform.transform.translation.y -
                      last_transform_.transform.translation.y;
          distance_traveled_ = std::sqrt(dx * dx + dy * dy);
        } catch (tf2::TransformException & ex) {
          // 忽略，使用时间估算
          distance_traveled_ = speed_ * state_elapsed_sec;
        }

        // 使用激光雷达限制的前进距离
        if (distance_traveled_ >= max_forward_distance_) {
          RCLCPP_INFO(node_->get_logger(),
            "【前进完成】已走: %.2fm, 目标: %.2fm", distance_traveled_, max_forward_distance_);
          // 保存实际前进距离，用于后退
          actual_forward_distance_ = distance_traveled_;
          
          // 更新理论位置（前进+转向）
          double dx_theo, dy_theo, dtheta_theo;
          computeTheoreticalMotion(distance_traveled_, steering_angle_, true, dx_theo, dy_theo, dtheta_theo);
          theoretical_x_ += dx_theo * std::cos(theoretical_yaw_) - dy_theo * std::sin(theoretical_yaw_);
          theoretical_y_ += dx_theo * std::sin(theoretical_yaw_) + dy_theo * std::cos(theoretical_yaw_);
          theoretical_yaw_ += dtheta_theo * turn_direction_;
          
          stopRobot();
          state_ = ShuttleState::PAUSE_AFTER_FORWARD;
          state_start_time_ = std::chrono::steady_clock::now();
        } else {
          // 前进 + 转向
          double angular = computeAngularVelocity(speed_, steering_angle_) * turn_direction_;
          publishVelocity(speed_, angular);
        }
      }
      break;

    case ShuttleState::PAUSE_AFTER_FORWARD:
      {
        if (state_elapsed_sec >= 0.3) {
          state_ = ShuttleState::BACKWARD_TURN;
          state_start_time_ = std::chrono::steady_clock::now();
          distance_traveled_ = 0.0;

          try {
            last_transform_ = tf_buffer_->lookupTransform(
              global_frame_, robot_base_frame_,
              tf2::TimePointZero,
              tf2::durationFromSec(transform_tolerance_));
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "无法获取位置: %s", ex.what());
          }
        }
      }
      break;

    case ShuttleState::BACKWARD_TURN:
      {
        try {
          auto current_transform = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));

          double dx = current_transform.transform.translation.x -
                      last_transform_.transform.translation.x;
          double dy = current_transform.transform.translation.y -
                      last_transform_.transform.translation.y;
          distance_traveled_ = std::sqrt(dx * dx + dy * dy);
        } catch (tf2::TransformException & ex) {
          distance_traveled_ = speed_ * state_elapsed_sec;
        }

        // 后退距离 = 前进距离（回到原来的 x 位置，因为原来位置是空的）
        // 这样每次 shuttle 完整周期后：
        //   - x 位置大致回到原点
        //   - y 位置有一定偏移（这是正常的）
        //   - 朝向累积 2θ 的变化（θ = d * tan(δ) / L）
        double backward_target = actual_forward_distance_;
        // 最小后退 0.1m，最大不超过 backward_distance_ 参数
        backward_target = std::max(0.1, std::min(backward_target, backward_distance_));

        // 计算这次 shuttle 周期预计的角度变化
        double angle_per_cycle = 2.0 * computeAngleChange(actual_forward_distance_, steering_angle_) * 180.0 / M_PI;

        if (distance_traveled_ >= backward_target) {
          RCLCPP_INFO(node_->get_logger(),
            "【后退完成】距离: %.2fm, 前进距离: %.2fm, 本周期角度变化: %.1f°",
            distance_traveled_, actual_forward_distance_, angle_per_cycle);
          
          // 更新理论位置（后退+转向，倒车时同向角速度累积转向）
          double dx_theo, dy_theo, dtheta_theo;
          computeTheoreticalMotion(distance_traveled_, steering_angle_, false, dx_theo, dy_theo, dtheta_theo);
          // 倒车时位移是负的（向后），但转向累积是正的（同向）
          theoretical_x_ += dx_theo * std::cos(theoretical_yaw_) - dy_theo * std::sin(theoretical_yaw_);
          theoretical_y_ += dx_theo * std::sin(theoretical_yaw_) + dy_theo * std::cos(theoretical_yaw_);
          theoretical_yaw_ += dtheta_theo * turn_direction_;  // 倒车时继续同向累积
          
          shuttle_cycle_count_++;
          
          stopRobot();
          state_ = ShuttleState::PAUSE_AFTER_BACKWARD;
          state_start_time_ = std::chrono::steady_clock::now();
        } else {
          // 后退 + 同向转向（阿克曼底盘倒车时，相同转向角会继续同方向旋转）
          // 倒车时 linear.x 为负，但角速度方向与前进时相同才能累积转向
          double angular = computeAngularVelocity(speed_, steering_angle_) * turn_direction_;
          publishVelocity(-speed_, angular);
        }
      }
      break;

    case ShuttleState::PAUSE_AFTER_BACKWARD:
      {
        if (state_elapsed_sec >= 0.3) {
          state_ = ShuttleState::CHECK_ORIENTATION;
          state_start_time_ = std::chrono::steady_clock::now();
        }
      }
      break;

    case ShuttleState::CHECK_ORIENTATION:
      {
        // 检查朝向，如果还没对准，继续下一个周期
        double current_angle = computeAngleToGoal();
        RCLCPP_INFO(node_->get_logger(),
          "【周期%d完成】当前角度差: %.1f°, 阈值: %.1f°", 
          shuttle_cycle_count_, current_angle, angle_threshold_);
        
        // 每个周期完成后进行 TF 校正
        correctTFDrift();

        if (std::abs(current_angle) <= angle_threshold_) {
          RCLCPP_INFO(node_->get_logger(), "朝向调整完成!");
          return BT::NodeStatus::SUCCESS;
        }

        // 继续下一个周期，重新检测前方距离
        if (!updateForwardDistance()) {
          // 如果没有激光数据，等待一下再重试
          if (state_elapsed_sec > 2.0) {
            RCLCPP_WARN(node_->get_logger(), "无法获取前进距离，使用上次值: %.2fm", max_forward_distance_);
          } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
              "等待激光数据更新前进距离...");
            break;  // 继续等待
          }
        }

        // 记录位置
        try {
          last_transform_ = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN(node_->get_logger(), "无法获取位置: %s", ex.what());
        }

        // 重置预前进距离（每个周期重新判断）
        pre_forward_distance_ = 0.0;

        // 根据空间情况决定下一个状态
        if (need_backup_first_) {
          // 检查前方是否还有空间可以先前进（> 0.3m时先前进利用空间）
          if (front_obstacle_distance_ > 0.30) {
            pre_forward_distance_ = front_obstacle_distance_ - 0.05;  // 留5cm安全距离
            pre_forward_distance_ = std::min(pre_forward_distance_, 0.50);  // 最大0.5m
            RCLCPP_INFO(node_->get_logger(), 
              "前方有%.2fm空间，先直线前进%.2fm再后退", 
              front_obstacle_distance_, pre_forward_distance_);
            state_ = ShuttleState::PRE_FORWARD_STRAIGHT;
          } else {
            RCLCPP_INFO(node_->get_logger(), "前方空间不足(%.2fm)，需要先后退 0.1m", 
              front_obstacle_distance_);
            state_ = ShuttleState::INITIAL_BACKUP;
          }
        } else {
          state_ = ShuttleState::FORWARD_TURN;
        }
        state_start_time_ = std::chrono::steady_clock::now();
        distance_traveled_ = 0.0;
      }
      break;

    case ShuttleState::COMPLETED:
      return BT::NodeStatus::SUCCESS;

    case ShuttleState::FAILED:
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ShuttleToOrientationAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Shuttle 动作被中断");
  stopRobot();
}

}  // namespace tianracer_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<tianracer_behavior_tree::ShuttleToOrientationAction>(name, config);
    };
  factory.registerBuilder<tianracer_behavior_tree::ShuttleToOrientationAction>("ShuttleToOrientation", builder);
}
