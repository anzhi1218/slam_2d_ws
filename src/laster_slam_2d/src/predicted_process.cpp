/*
 * laster_slam_2d - 2D激光SLAM系统的先验位姿处理模块
 * 该模块处理数据，生成先验位姿估计，并提供路径发布和TF变换功能
 */
 #include "predicted_process.h"
 #include <cmath>
 #include <Eigen/Dense>
 #include "frame.h"
 
 namespace laster_slam_2d {
 
  PredictedProcess::PredictedProcess(ros::NodeHandle& nh, ros::NodeHandle& pnh){
    predict_path_pub_ = pnh.advertise<nav_msgs::Path>("predict_path", 1, true);
    predicted_path_pub_ = pnh.advertise<nav_msgs::Path>("predicted_path", 1, true);
    location_path_pub_ = pnh.advertise<nav_msgs::Path>("location_path", 1, true);
  }

  /*
  【cos(yaw) -sin(yaw)  x】
  【sin(yaw)  cos(yaw)  y】
  【0         0         1】
  */
  Eigen::Matrix3f PredictedProcess::PoseToMatrix(const laster_slam_2d::Pose2D& pose) const {
   const double x = pose.x, y = pose.y, yaw = pose.yaw;
   Eigen::Matrix3f pose_matrix = Eigen::Matrix3f::Identity();
   pose_matrix(0, 0) = static_cast<float>(cos(yaw));
   pose_matrix(0, 1) = static_cast<float>(-sin(yaw));
   pose_matrix(0, 2) = static_cast<float>(x);
   pose_matrix(1, 0) = static_cast<float>(sin(yaw));
   pose_matrix(1, 1) = static_cast<float>(cos(yaw));
   pose_matrix(1, 2) = static_cast<float>(y);
   return pose_matrix;
  }

  void PredictedProcess::MatrixToPose(const Eigen::Matrix4f& pose_matrix, laster_slam_2d::Pose2D& pose) const {
    pose.x = pose_matrix(0, 3);
    pose.y = pose_matrix(1, 3);
    pose.yaw = std::atan2(pose_matrix(1, 0), pose_matrix(0, 0));
  }

 void PredictedProcess::PoseSubtract(const laster_slam_2d::Pose2D& predict_pose, const laster_slam_2d::Pose2D& last_predict_pose,
                                     double& dx_l, double& dy_l, double& dyaw) {
 
   Eigen::Matrix3f last_predict_pose_matrix = PoseToMatrix(last_predict_pose);
   Eigen::Matrix3f predict_pose_matrix = PoseToMatrix(predict_pose);
   Eigen::Matrix3f predict_pose_delta = last_predict_pose_matrix.inverse() * predict_pose_matrix;
   dx_l = predict_pose_delta(0, 2);
   dy_l = predict_pose_delta(1, 2);
   dyaw = std::atan2(predict_pose_delta(1, 0), predict_pose_delta(0, 0));
   normalizeYaw(dyaw);

   ROS_INFO(" predicte_pose: x=%.4f, y=%.4f, yaw=%.2f degree", predict_pose.x, predict_pose.y, predict_pose.yaw * 180.0 / M_PI);
   ROS_INFO(" last_predict_pose: x=%.4f, y=%.4f, yaw=%.2f degree", last_predict_pose.x, last_predict_pose.y, last_predict_pose.yaw * 180.0 / M_PI);
   ROS_INFO(" predict_pose_delta: x=%.4f, y=%.4f, yaw=%.2f degree", predict_pose_delta(0, 2), predict_pose_delta(1, 2), dyaw * 180.0 / M_PI);
   
 }
 
 void PredictedProcess::PoseAdd(const laster_slam_2d::Pose2D& last_location_pose, double dx_l, double dy_l,
                                 double dyaw, laster_slam_2d::Pose2D& predicted_pose) {
 
   // 2D 齐次变换矩阵：用 PoseToMatrix(dx, dy, yaw) 得到增量矩阵
   Eigen::Matrix3f last_location_pose_ = PoseToMatrix(last_location_pose);
   Eigen::Matrix3f predict_pose_delta = PoseToMatrix(Pose2D{dx_l, dy_l, dyaw});
   Eigen::Matrix3f predicted_pose_matrix = last_location_pose_ * predict_pose_delta;
   predicted_pose.x = predicted_pose_matrix(0, 2);
   predicted_pose.y = predicted_pose_matrix(1, 2);
   predicted_pose.yaw = std::atan2(predicted_pose_matrix(1, 0), predicted_pose_matrix(0, 0));
   normalizeYaw(predicted_pose.yaw);
   
   ROS_INFO("poseAdd: predicted_pose: x=%.4f, y=%.4f, yaw=%.2f degree", 
             predicted_pose.x, predicted_pose.y, predicted_pose.yaw * 180.0 / M_PI);
 }
 

/// 发布路径，包括先验路径、预测路径和ICP估计路径
void PredictedProcess::publishPaths(const std::string& map_frame, const ros::Time& timestamp,
                                    double predict_x, double predict_y, double predict_yaw,
                                    double predicted_x, double predicted_y, double predicted_yaw,
                                    double location_x, double location_y, double location_yaw) {
  // 设置路径坐标系和时间戳
  mvPredictPath_.header.frame_id = map_frame;
  mvPredictPath_.header.stamp = timestamp;
  mvPredictedPath_.header.frame_id = map_frame;
  mvPredictedPath_.header.stamp = timestamp;
  mvLocationPath_.header.frame_id = map_frame;
  mvLocationPath_.header.stamp = timestamp;

  // ----- 1. 先验路径 -----
  geometry_msgs::PoseStamped predict_ps;
  predict_ps.header.frame_id = map_frame;
  predict_ps.header.stamp = timestamp;
  predict_ps.pose.position.x = predict_x;
  predict_ps.pose.position.y = predict_y;
  predict_ps.pose.position.z = 0.0;
  predict_ps.pose.orientation.x = 0.0;
  predict_ps.pose.orientation.y = 0.0;
  predict_ps.pose.orientation.z = std::sin(predict_yaw / 2.0);
  predict_ps.pose.orientation.w = std::cos(predict_yaw / 2.0);
  mvPredictPath_.poses.push_back(predict_ps);

  // ----- 2. 预测路径 -----
  geometry_msgs::PoseStamped predicted_ps;
  predicted_ps.header.frame_id = map_frame;
  predicted_ps.header.stamp = timestamp;
  predicted_ps.pose.position.x = predicted_x;
  predicted_ps.pose.position.y = predicted_y;
  predicted_ps.pose.position.z = 0.0;
  predicted_ps.pose.orientation.x = 0.0;
  predicted_ps.pose.orientation.y = 0.0;
  predicted_ps.pose.orientation.z = std::sin(predicted_yaw / 2.0);
  predicted_ps.pose.orientation.w = std::cos(predicted_yaw / 2.0);
  mvPredictedPath_.poses.push_back(predicted_ps);

  // ----- 3. ICP估计路径 -----
  geometry_msgs::PoseStamped location_ps;
  location_ps.header.frame_id = map_frame;
  location_ps.header.stamp = timestamp;
  location_ps.pose.position.x = location_x;
  location_ps.pose.position.y = location_y;
  location_ps.pose.position.z = 0.0;
  location_ps.pose.orientation.x = 0.0;
  location_ps.pose.orientation.y = 0.0;
  location_ps.pose.orientation.z = std::sin(location_yaw / 2.0);
  location_ps.pose.orientation.w = std::cos(location_yaw / 2.0);
  mvLocationPath_.poses.push_back(location_ps);

  // ----- 4. 发布路径消息 -----
  predict_path_pub_.publish(mvPredictPath_);
  predicted_path_pub_.publish(mvPredictedPath_);
  location_path_pub_.publish(mvLocationPath_);
}

/// 发布TF变换，将地图坐标系转换到里程计坐标系和机器人基坐标系
void PredictedProcess::publishTf(const std::string& map_frame, const std::string& odom_frame,
                            const std::string& base_frame, const ros::Time& timestamp,
                            double location_x, double location_y, double location_yaw) {
  
  //map系到odom系的变换
  tf::StampedTransform map_to_odom;
  map_to_odom.frame_id_ = map_frame;
  map_to_odom.child_frame_id_ = odom_frame;
  map_to_odom.stamp_ = timestamp;
  map_to_odom.setOrigin(tf::Vector3(location_x, location_y, 0));
  map_to_odom.setRotation(tf::createQuaternionFromYaw(location_yaw));
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