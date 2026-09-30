#ifndef SLAM_2D_PREDICTED_PROCESS_H
#define SLAM_2D_PREDICTED_PROCESS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <string>

#include <Eigen/Dense>

#include "frame.h"

namespace slam_2d {

class PredictedProcess {
 public:
  explicit PredictedProcess(ros::NodeHandle& pnh);

  /// 位姿与齐次矩阵转换
  static Eigen::Matrix4f PoseToMatrix4f(const Pose2D& pose);
  static Pose2D MatrixToPose(const Eigen::Matrix4f& pose_matrix);

  /// 局部坐标系下位姿相减
  static Pose2D PoseSubtract(const Pose2D& pose, const Pose2D& previous_pose);

  /// 局部坐标系下位姿相加
  static Pose2D PoseAdd(const Pose2D& pose, const Pose2D& local_delta);

  /// 发布路径
  void publishPaths(const std::string& map_frame, const ros::Time& timestamp,
                    const Pose2D& prior_pose, const Pose2D& predicted_pose,
                    const Pose2D& location_pose);

  //坐标系
  void publishTf(const std::string& map_frame, const std::string& odom_frame,
                const std::string& base_frame, const ros::Time& timestamp,
                const Pose2D& location_pose);

 private:
  static double normalizeYaw(double yaw);

  //发布器
  ros::Publisher predict_path_pub_;
  ros::Publisher predicted_path_pub_;
  ros::Publisher location_path_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  //路径
  nav_msgs::Path mvPredictPath_;
  nav_msgs::Path mvPredictedPath_;
  nav_msgs::Path mvLocationPath_;


};

}  // namespace slam_2d

#endif
//predicted_process.h
