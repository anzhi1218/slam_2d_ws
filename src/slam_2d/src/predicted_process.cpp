/*
 * slam_2d - 2D激光SLAM系统的先验位姿处理模块
 * 该模块处理数据，生成先验位姿估计，并提供路径发布和TF变换功能
 */
#include "predicted_process.h"

#include <cmath>

namespace slam_2d {

namespace {

geometry_msgs::PoseStamped MakePoseStamped(const std::string& frame,
                                           const ros::Time& stamp,
                                           const Pose2D& pose) {
  geometry_msgs::PoseStamped message;
  message.header.frame_id = frame;
  message.header.stamp = stamp;
  message.pose.position.x = pose.x;
  message.pose.position.y = pose.y;
  message.pose.position.z = 0.0;
  message.pose.orientation.x = 0.0;
  message.pose.orientation.y = 0.0;
  message.pose.orientation.z = std::sin(pose.yaw / 2.0);
  message.pose.orientation.w = std::cos(pose.yaw / 2.0);
  return message;
}

void SetPathHeader(nav_msgs::Path& path, const std::string& frame,
                   const ros::Time& stamp) {
  path.header.frame_id = frame;
  path.header.stamp = stamp;
}

}  // namespace

PredictedProcess::PredictedProcess(ros::NodeHandle& pnh) {
  predict_path_pub_ = pnh.advertise<nav_msgs::Path>("predict_path", 1, true);
  predicted_path_pub_ = pnh.advertise<nav_msgs::Path>("predicted_path", 1, true);
  location_path_pub_ = pnh.advertise<nav_msgs::Path>("location_path", 1, true);
}

Eigen::Matrix4f PredictedProcess::PoseToMatrix4f(const Pose2D& pose) {
  const float c = static_cast<float>(std::cos(pose.yaw));
  const float s = static_cast<float>(std::sin(pose.yaw));
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 0) = c;
  transform(0, 1) = -s;
  transform(0, 3) = static_cast<float>(pose.x);
  transform(1, 0) = s;
  transform(1, 1) = c;
  transform(1, 3) = static_cast<float>(pose.y);
  return transform;
}

Pose2D PredictedProcess::MatrixToPose(const Eigen::Matrix4f& matrix) {
  return {matrix(0, 3), matrix(1, 3),
          std::atan2(matrix(1, 0), matrix(0, 0))};
}

double PredictedProcess::normalizeYaw(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

Pose2D PredictedProcess::PoseSubtract(const Pose2D& pose,
                                     const Pose2D& previous_pose) {
  const double dx = pose.x - previous_pose.x;
  const double dy = pose.y - previous_pose.y;
  const double c = std::cos(previous_pose.yaw);
  const double s = std::sin(previous_pose.yaw);
  return {c * dx + s * dy, -s * dx + c * dy,
          normalizeYaw(pose.yaw - previous_pose.yaw)};
}

Pose2D PredictedProcess::PoseAdd(const Pose2D& pose,
                                 const Pose2D& local_delta) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * local_delta.x - s * local_delta.y,
          pose.y + s * local_delta.x + c * local_delta.y,
          normalizeYaw(pose.yaw + local_delta.yaw)};
}


/// 发布路径，包括先验路径、预测路径和ICP估计路径
void PredictedProcess::publishPaths(const std::string& map_frame,
                                    const ros::Time& timestamp,
                                    const Pose2D& prior_pose,
                                    const Pose2D& predicted_pose,
                                    const Pose2D& location_pose) {
  SetPathHeader(mvPredictPath_, map_frame, timestamp);
  SetPathHeader(mvPredictedPath_, map_frame, timestamp);
  SetPathHeader(mvLocationPath_, map_frame, timestamp);

  mvPredictPath_.poses.push_back(MakePoseStamped(map_frame, timestamp, prior_pose));
  mvPredictedPath_.poses.push_back(MakePoseStamped(map_frame, timestamp, predicted_pose));
  mvLocationPath_.poses.push_back(MakePoseStamped(map_frame, timestamp, location_pose));

  predict_path_pub_.publish(mvPredictPath_);
  predicted_path_pub_.publish(mvPredictedPath_);
  location_path_pub_.publish(mvLocationPath_);
}

/// 发布TF变换，将地图坐标系转换到里程计坐标系和机器人基坐标系
void PredictedProcess::publishTf(const std::string& map_frame, const std::string& odom_frame,
                            const std::string& base_frame, const ros::Time& timestamp,
                            const Pose2D& location_pose) {

  //map系到odom系的变换
  tf::StampedTransform map_to_odom;
  map_to_odom.frame_id_ = map_frame;
  map_to_odom.child_frame_id_ = odom_frame;
  map_to_odom.stamp_ = timestamp;
  map_to_odom.setOrigin(tf::Vector3(location_pose.x, location_pose.y, 0));
  map_to_odom.setRotation(tf::createQuaternionFromYaw(location_pose.yaw));
  tf_broadcaster_.sendTransform(map_to_odom);

  //odom系到base_link系的变换
  tf::StampedTransform odom_to_base;
  odom_to_base.frame_id_ = odom_frame;
  odom_to_base.child_frame_id_ = base_frame;
  odom_to_base.stamp_ = timestamp;
  odom_to_base.setOrigin(tf::Vector3(0, 0, 0));
  odom_to_base.setRotation(tf::createQuaternionFromYaw(0));
  tf_broadcaster_.sendTransform(odom_to_base);
 }

}  // predicted_process.cpp
