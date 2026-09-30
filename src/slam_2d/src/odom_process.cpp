/*
 * slam_2d - 里程计（先验位姿）预处理实现
 */
#include "odom_process.h"

#include <algorithm>
#include <cmath>

#include <tf/transform_datatypes.h>

namespace slam_2d {

OdomProcess::OdomProcess(ros::NodeHandle& pnh) {
  int buffer_size = 200;
  pnh.param<int>("odom_buffer_size", buffer_size, 200);
  buffer_size_ = static_cast<size_t>(std::max(10, buffer_size));

  // 跳变判定阈值（默认值略高于本数据集实际最大速度，避免误删正常运动）
  pnh.param<double>("odom_max_linear_speed", max_linear_speed_, 8.0);      // m/s
  double max_angular_speed_deg = 180.0;
  pnh.param<double>("odom_max_angular_speed", max_angular_speed_deg, 180.0);  // deg/s
  max_angular_speed_ = max_angular_speed_deg * M_PI / 180.0;

  // 增量速度低通系数(0,1]，1 表示不平滑
  pnh.param<double>("odom_smooth_alpha", smooth_alpha_, 1.0);
  smooth_alpha_ = std::min(1.0, std::max(0.0, smooth_alpha_));
}

// Odometry -> Pose2D：与老 data.txt 的 yaw/X/Y 先验位姿对应
Pose2D OdomProcess::ToPose2D(const nav_msgs::Odometry& odom) {
  Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  pose.yaw = tf::getYaw(odom.pose.pose.orientation);  // 平面运动：四元数只含 yaw
  return pose;
}

double OdomProcess::NormalizeYaw(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

void OdomProcess::AddMessage(const nav_msgs::Odometry& odom) {
  Sample raw;
  raw.stamp = odom.header.stamp;
  raw.pose = ToPose2D(odom);

  // 第一条消息：直接作为位姿链起点
  if (!has_last_) {
    has_last_ = true;
    last_raw_ = raw;
    last_out_ = raw;
    buffer_.push_back(raw);
    return;
  }

  const double dt = (raw.stamp - last_raw_.stamp).toSec();

  // 时间戳大幅回退：数据包被重新播放，重置位姿链，按新的时间轴重新开始
  if (dt < -1.0) {
    ROS_WARN("odom_process: odom time jumped backward %.1f s (bag replayed?), reset timeline", -dt);
    buffer_.clear();
    has_prev_delta_ = false;
    last_raw_ = raw;
    last_out_ = raw;
    buffer_.push_back(raw);
    return;
  }

  if (dt <= 1e-6) {
    return;  // 时间非递增或重复消息，丢弃
  }

  // step1 预积分基础：求相邻两帧增量，并转到上一帧局部坐标系下
  const double dx_world = raw.pose.x - last_raw_.pose.x;
  const double dy_world = raw.pose.y - last_raw_.pose.y;
  const double cos_prev = std::cos(last_raw_.pose.yaw);
  const double sin_prev = std::sin(last_raw_.pose.yaw);
  double dx = cos_prev * dx_world + sin_prev * dy_world;
  double dy = -sin_prev * dx_world + cos_prev * dy_world;
  double dyaw = NormalizeYaw(raw.pose.yaw - last_raw_.pose.yaw);

  double vx = dx / dt;
  double vy = dy / dt;
  double wz = dyaw / dt;

  // step2 异常剔除：速度超限（里程计跳变）时，用上一次速度按时间外推
  if (std::hypot(vx, vy) > max_linear_speed_ || std::fabs(wz) > max_angular_speed_) {
    ROS_WARN_THROTTLE(1.0, "odom_process: reject odom jump (v=%.2f m/s, w=%.1f deg/s)",
                      std::hypot(vx, vy), wz * 180.0 / M_PI);
    if (has_prev_delta_) {
      vx = prev_vx_;
      vy = prev_vy_;
      wz = prev_wz_;
    } else {
      vx = 0.0;
      vy = 0.0;
      wz = 0.0;
    }
  }

  // step3 平滑：对增量速度做一阶低通（smooth_alpha_=1 时不平滑）
  if (has_prev_delta_ && smooth_alpha_ < 1.0) {
    vx = smooth_alpha_ * vx + (1.0 - smooth_alpha_) * prev_vx_;
    vy = smooth_alpha_ * vy + (1.0 - smooth_alpha_) * prev_vy_;
    wz = smooth_alpha_ * wz + (1.0 - smooth_alpha_) * prev_wz_;
  }
  prev_vx_ = vx;
  prev_vy_ = vy;
  prev_wz_ = wz;
  has_prev_delta_ = true;

  // step4 积分：把清洗后的增量叠加到位姿链末端
  Sample out;
  out.stamp = raw.stamp;
  const double cos_out = std::cos(last_out_.pose.yaw);
  const double sin_out = std::sin(last_out_.pose.yaw);
  out.pose.x = last_out_.pose.x + cos_out * vx * dt - sin_out * vy * dt;
  out.pose.y = last_out_.pose.y + sin_out * vx * dt + cos_out * vy * dt;
  out.pose.yaw = NormalizeYaw(last_out_.pose.yaw + wz * dt);

  // step5 入缓存并限制长度
  buffer_.push_back(out);
  while (buffer_.size() > buffer_size_) {
    buffer_.pop_front();
  }

  last_raw_ = raw;
  last_out_ = out;
}

bool OdomProcess::GetPoseAt(const ros::Time& stamp, Pose2D& pose) const {
  if (buffer_.empty()) {
    return false;
  }

  // 只接受与位姿链时间范围相差不大的查询：
  // 超出窗口（里程计中断、时间轴异常）时返回失败，由调用方跳过该帧，
  // 避免用错误的（冻结）位姿继续建图
  const double kValidWindow = 0.3;  // 允许的最大时间偏差(s)
  if (stamp < buffer_.front().stamp - ros::Duration(kValidWindow) ||
      stamp > buffer_.back().stamp + ros::Duration(kValidWindow)) {
    return false;
  }

  // 窗口内超出范围的查询：取最接近的样本
  if (stamp <= buffer_.front().stamp) {
    pose = buffer_.front().pose;
    return true;
  }
  if (stamp >= buffer_.back().stamp) {
    pose = buffer_.back().pose;
    return true;
  }

  // 找到第一个时间戳 >= stamp 的样本
  auto it = std::lower_bound(buffer_.begin(), buffer_.end(), stamp,
                             [](const Sample& s, const ros::Time& t) { return s.stamp < t; });
  const Sample& next = *it;
  const Sample& prev = *(it - 1);

  const double span = (next.stamp - prev.stamp).toSec();
  const double r = span > 1e-9 ? (stamp - prev.stamp).toSec() / span : 0.0;

  // 位置线性插值；yaw 走最短角路径插值
  pose.x = prev.pose.x + r * (next.pose.x - prev.pose.x);
  pose.y = prev.pose.y + r * (next.pose.y - prev.pose.y);
  pose.yaw = NormalizeYaw(prev.pose.yaw + r * NormalizeYaw(next.pose.yaw - prev.pose.yaw));
  return true;
}

}  // namespace slam_2d
