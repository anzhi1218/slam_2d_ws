#ifndef LASTER_SLAM_2D_PREDICTED_PROCESS_H
#define LASTER_SLAM_2D_PREDICTED_PROCESS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#include <Eigen/Dense>
#include <cmath>

#include "frame.h"

namespace laster_slam_2d {

class PredictedProcess {
 public:
  PredictedProcess(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  /// PoseToMatrix: 将位姿转换为 2D 齐次变换矩阵 (3x3)
  Eigen::Matrix3f PoseToMatrix(const laster_slam_2d::Pose2D& pose) const;
  
  
  void MatrixToPose(const Eigen::Matrix4f& pose_matrix, laster_slam_2d::Pose2D& pose) const;
   /// 将角度转换为弧度
  double degToRad(double yaw_deg) const {
    return yaw_deg * M_PI / 180.0;
  };
  
  //归一化偏航角到 [-pi, pi]
  void normalizeYaw(double& yaw) {
    while (yaw > M_PI)  yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
  }


  /// 局部坐标系下位姿相减
  void PoseSubtract(const Pose2D& predict_pose, const Pose2D& last_predict_pose,
                   double& dx_l, double& dy_l, double& dyaw);

  /// 局部坐标系下位姿相加
  void PoseAdd(const Pose2D& last_location_pose, double dx_l, double dy_l,
               double dyaw, Pose2D& predicted_pose);

  /// 发布路径
  void publishPaths(const std::string& map_frame, const ros::Time& timestamp,
                   double predict_x, double predict_y, double predict_yaw,
                   double predicted_x, double predicted_y, double predicted_yaw,
                   double location_x, double location_y, double location_yaw);

  //坐标系
  void publishTf(const std::string& map_frame, const std::string& odom_frame,
                const std::string& base_frame, const ros::Time& timestamp,
                double location_x, double location_y, double location_yaw);

 private:
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

}  // namespace laster_slam_2d

#endif
//predicted_process.h