// Copyright (c) 2024 Tianracer
// Licensed under the Apache License, Version 2.0

#ifndef TIANRACER_NAVIGATION_ROS2__PLUGINS__ACTION__SHUTTLE_TO_ORIENTATION_ACTION_HPP_
#define TIANRACER_NAVIGATION_ROS2__PLUGINS__ACTION__SHUTTLE_TO_ORIENTATION_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tianracer_behavior_tree
{

/**
 * @brief 动作节点：通过 Shuttle 机动调整机器人朝向
 *
 * Shuttle 机动：通过交替前进+转向、后退+反向转向来逐渐改变机器人朝向
 * 适用于阿克曼底盘在狭窄空间中无法原地旋转的情况
 *
 * 执行流程：
 * 1. 检测左右哪边更空旷，决定转向方向
 * 2. 循环执行 Shuttle 周期，直到目标在前方或超时
 * 3. 每个周期：前进+转向 → 后退+反向转向
 */
class ShuttleToOrientationAction : public BT::StatefulActionNode
{
public:
  ShuttleToOrientationAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  ShuttleToOrientationAction() = delete;
  ~ShuttleToOrientationAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "目标位置"),
      BT::InputPort<double>("angle_threshold", 90.0, "角度阈值(°)"),
      BT::InputPort<double>("clearance_threshold", 0.8, "判断空旷的距离阈值(m)"),
      BT::InputPort<double>("forward_distance", 0.3, "每次最大前进距离(m)"),  // 限制0.3m更安全
      BT::InputPort<double>("backward_distance", 0.3, "每次最大后退距离(m)"),  // 限制0.3m更安全
      BT::InputPort<double>("speed", 0.1, "运动速度(m/s)"),
      BT::InputPort<double>("steering_angle", 50.0, "转向角度(°)"),
      BT::InputPort<double>("timeout", 120.0, "超时时间(s)")
    };
  }

  // StatefulActionNode 接口
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void initialize();

  /**
   * @brief 计算目标相对于机器人当前朝向的角度
   */
  double computeAngleToGoal();

  /**
   * @brief 检测是否有激光数据
   * @return true 有数据，false 无数据
   */
  bool hasLaserData();

  /**
   * @brief 检测左右哪边更空旷
   * @return 1 表示左边更空旷，-1 表示右边更空旷，0 表示无法检测（无激光数据）
   */
  int detectClearSide();

  /**
   * @brief 发布速度命令
   */
  void publishVelocity(double linear, double angular);

  /**
   * @brief 停止机器人
   */
  void stopRobot();

  /**
   * @brief 激光雷达回调
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief 手动 spin 激光雷达回调组
   */
  void spinLaserCallback();

  /**
   * @brief 根据线速度和转向角计算角速度（阿克曼运动学）
   * @param linear_vel 线速度 (m/s)
   * @param steering_angle_deg 转向角 (度)
   * @return 角速度 (rad/s)
   */
  double computeAngularVelocity(double linear_vel, double steering_angle_deg);

  /**
   * @brief 计算转弯半径 R = L / tan(δ)
   * @param steering_angle_deg 转向角 (度)
   * @return 转弯半径 (m)
   */
  double computeTurningRadius(double steering_angle_deg);

  /**
   * @brief 计算行驶距离后转过的角度 θ = d / R
   * @param distance 行驶距离 (m)
   * @param steering_angle_deg 转向角 (度)
   * @return 转过的角度 (弧度)
   */
  double computeAngleChange(double distance, double steering_angle_deg);

  /**
   * @brief 计算车辆转弯时的内半径和外半径（扫掠区域）
   * @param steering_angle_deg 转向角 (度)
   * @param turn_dir 转向方向 (1=左转, -1=右转)
   * @param r_inner 输出：内半径
   * @param r_outer 输出：外半径
   */
  void computeSweptRadii(double steering_angle_deg, int turn_dir,
                         double& r_inner, double& r_outer);

  /**
   * @brief 根据激光雷达数据和扫掠几何计算最大前进距离
   * @param steering_angle_deg 转向角 (度)
   * @param turn_dir 转向方向 (1=左转, -1=右转)
   * @return 最大前进距离 (m)
   */
  double computeMaxForwardFromLaser(double steering_angle_deg, int turn_dir);

  /**
   * @brief 根据前方障碍物距离和转向角计算最大前进距离（简化版）
   * @param obstacle_distance 前方障碍物距离 (m)
   * @param steering_angle_deg 转向角 (度)
   * @return 最大前进距离 (m)
   */
  double computeMaxForwardDistance(double obstacle_distance, double steering_angle_deg);

  /**
   * @brief 获取指定角度范围内的最小障碍物距离
   * @param angle_start_deg 起始角度（度，0=正前方，正=左，负=右）
   * @param angle_end_deg 结束角度（度）
   * @return 最小距离 (m)
   */
  double getMinDistanceInSector(double angle_start_deg, double angle_end_deg);

  /**
   * @brief 根据激光雷达更新前方最大可行驶距离
   * @return true 成功更新，false 无激光数据
   */
  bool updateForwardDistance();

  /**
   * @brief 根据运动学模型计算理论位置和朝向变化
   * @param distance 行驶距离 (m)
   * @param steering_angle_deg 转向角 (度)
   * @param is_forward 是否前进
   * @param dx 输出：x方向位移
   * @param dy 输出：y方向位移
   * @param dtheta 输出：朝向变化 (弧度)
   */
  void computeTheoreticalMotion(double distance, double steering_angle_deg, bool is_forward,
                                 double& dx, double& dy, double& dtheta);

  /**
   * @brief 校正TF误差，对比理论值与实际值
   * 当差异过大时尝试重定位或取中间值
   */
  void correctTFDrift();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;  // 专用回调组

  bool initialized_;
  std::string robot_base_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  
  // 车辆几何参数（根据实测值）
  double wheelbase_;           // 轴距 L = 0.40m
  double track_width_;         // 轮距 W = 0.27m (2 * base_a)
  double front_overhang_;      // 前悬（前轮到车头）= 0.13m
  double rear_overhang_;       // 后悬（后轮到车尾）= 0.07m
  double lidar_x_offset_;      // 激光雷达相对后轮轴的 x 偏移 = 0.20m

  // 激光数据
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;

  // 状态机
  enum class ShuttleState {
    DETECT_CLEAR_SIDE,    // 检测空旷方向
    PRE_FORWARD_STRAIGHT, // 后退前先直线前进（利用前方空间）
    PAUSE_AFTER_PRE_FORWARD, // 预前进后暂停
    INITIAL_BACKUP,       // 空间不足时先后退
    PAUSE_AFTER_INITIAL_BACKUP, // 初始后退后暂停
    FORWARD_TURN,         // 前进+转向
    PAUSE_AFTER_FORWARD,  // 前进后暂停
    BACKWARD_TURN,        // 后退+反向转向
    PAUSE_AFTER_BACKWARD, // 后退后暂停
    CHECK_ORIENTATION,    // 检查朝向
    COMPLETED,            // 完成
    FAILED                // 失败
  };

  ShuttleState state_;
  int turn_direction_;  // 1=左转, -1=右转
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point state_start_time_;
  double distance_traveled_;
  geometry_msgs::msg::TransformStamped last_transform_;

  // 参数缓存
  double angle_threshold_;
  double clearance_threshold_;
  double forward_distance_;
  double backward_distance_;
  double speed_;
  double steering_angle_;
  double timeout_;
  double max_forward_distance_;  // 激光雷达限制的最大前进距离
  double actual_forward_distance_;  // 实际前进距离，用于后退时保持轨迹重合
  bool need_backup_first_;  // 是否需要先后退（空间不足时）
  
  // 预前进相关（后退前先向前利用空间）
  double pre_forward_distance_;  // 预前进的目标距离
  double front_obstacle_distance_;  // 前方障碍物距离（用于判断是否可以预前进）
  
  // 理论位置跟踪（用于校正TF漂移）
  double theoretical_x_;      // 理论x位置（相对于起始点）
  double theoretical_y_;      // 理论y位置
  double theoretical_yaw_;    // 理论yaw朝向
  double initial_x_;          // 初始x位置
  double initial_y_;          // 初始y位置
  double initial_yaw_;        // 初始yaw朝向
  int shuttle_cycle_count_;   // shuttle周期计数
  
  // TF校正阈值
  static constexpr double TF_POSITION_TOLERANCE = 0.15;  // 位置差异阈值 15cm
  static constexpr double TF_YAW_TOLERANCE = 0.15;       // 朝向差异阈值 ~8.6度
};

}  // namespace tianracer_behavior_tree

#endif  // TIANRACER_NAVIGATION_ROS2__PLUGINS__ACTION__SHUTTLE_TO_ORIENTATION_ACTION_HPP_
