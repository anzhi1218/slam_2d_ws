/*
 * slam_2d - 里程计（先验位姿）预处理模块
 *
 * 数据包中的 /odom 为轮式里程计绝对位姿（odom 系下的 x、y + 四元数），
 * 与老 data.txt 里的 "time / yaw / X / Y" 先验位姿是同一类数据。
 * 本模块负责把它处理成新的 x、y、yaw 先验位姿，并做预积分处理：
 *
 *   1) 消息 -> (stamp, x, y, yaw)；姿态四元数只取 yaw（平面运动）
 *   2) 相邻消息求增量（预积分基础），增量在上一帧坐标系下表示
 *   3) 异常剔除：增量对应的速度超过阈值（里程计跳变）时，改用上一次速度按 dt 外推
 *   4) 低通平滑（可选）：对增量速度做一阶滤波，抑制轮式里程计抖动
 *   5) 把清洗后的增量积分成一条“干净”的位姿链
 *   6) 按激光时间戳在位姿链上插值，输出与激光严格时间对齐的先验位姿
 */
#ifndef SLAM_2D_ODOM_PROCESS_H
#define SLAM_2D_ODOM_PROCESS_H

#include <cstddef>
#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "frame.h"

namespace slam_2d {

class OdomProcess {
 public:
  explicit OdomProcess(ros::NodeHandle& pnh);

  /// 接收一条 /odom 消息（建议按消息原始频率 40Hz 调用，不要抽稀）
  void AddMessage(const nav_msgs::Odometry& odom);

  /**
   * @brief 查询 stamp 时刻的先验位姿（在预处理后的位姿链上线性插值）
   * @param stamp 激光帧时间戳
   * @param pose 输出先验位姿 x, y, yaw(rad)
   * @return 是否查询成功（还没有任何里程计数据时返回 false）
   */
  bool GetPoseAt(const ros::Time& stamp, Pose2D& pose) const;

 private:
  struct Sample {
    ros::Time stamp;  ///< 时间戳
    Pose2D pose;      ///< 处理后的位姿（增量积分链）
  };

  /// Odometry -> Pose2D：位置取 x,y；姿态四元数只取 yaw
  static Pose2D ToPose2D(const nav_msgs::Odometry& odom);

  /// 归一化 yaw 到 [-pi, pi]
  static double NormalizeYaw(double yaw);

  // ----- 参数 -----
  size_t buffer_size_;        ///< 位姿链缓存长度（条）
  double max_linear_speed_;   ///< 最大线速度(m/s)，超过视为跳变
  double max_angular_speed_;  ///< 最大角速度(rad/s)，超过视为跳变
  double smooth_alpha_;       ///< 增量速度低通系数 (0,1]，1=不平滑

  // ----- 状态 -----
  std::deque<Sample> buffer_;  ///< 处理后的位姿链（时间递增）
  bool has_last_ = false;      ///< 是否已收到第一条消息
  Sample last_raw_;            ///< 上一条原始消息
  Sample last_out_;            ///< 上一条处理后消息（位姿链末端）
  bool has_prev_delta_ = false;
  double prev_vx_ = 0.0;  ///< 上一次增量线速度(局部 x, m/s)
  double prev_vy_ = 0.0;  ///< 上一次增量线速度(局部 y, m/s)
  double prev_wz_ = 0.0;  ///< 上一次增量角速度(rad/s)
};

}  // namespace slam_2d

#endif  // SLAM_2D_ODOM_PROCESS_H
