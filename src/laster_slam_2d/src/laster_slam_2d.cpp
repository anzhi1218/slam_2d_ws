#include "laster_slam_2d.h"
 #include "frame.h"
 #include <cstddef>
 #include <ros/ros.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/common/transforms.h>
 #include <Eigen/Dense>
 #include <cmath>
 #include <sstream>
 #include <iomanip>
 #include <termios.h>
 #include <unistd.h>
 


namespace laster_slam_2d {
  
 Slam2dLaster::Slam2dLaster()
  : private_nh_("~") {
   private_nh_.param<std::string>("map_frame", map_frame_, "map");
   private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
   private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
   predicted_process_ = std::make_unique<PredictedProcess>(node_, private_nh_);
   laser_process_ = std::make_unique<LaserProcess>(private_nh_);
   mapping_ = std::make_unique<Mapping>(node_, private_nh_);
   mpCurrentGlobalCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
 }
 
 // 析构函数
 Slam2dLaster::~Slam2dLaster() {}
 

// 从 data.txt 回放的 SLAM 主流程
void Slam2dLaster::Process_2d_Laster_Slam(const std::string& data_file_path) {
  
  //加载数据。读入frames
  ros::WallTime t_start_load_data = ros::WallTime::now();
  std::vector<Frame> mvFrames;
  if(!LoadDataFile(data_file_path, mvFrames)){
    ROS_ERROR("laster_slam_2d: load data file failed");
    return;
  }

  ros::WallTime t_end_load_data = ros::WallTime::now();
  double Load_data_time = (t_end_load_data - t_start_load_data).toSec() * 1000.0;


  for (int i = 0; i < static_cast<int>(mvFrames.size()); ++i) {

    ros::WallTime total_time = ros::WallTime::now();

    ros::WallTime t_start_preprocess = ros::WallTime::now();
    //step 1: 获取时间戳和预测位姿以及点云数据
    //step 1.1: 获取时间戳
    ros::Time timestamp(mvFrames[i].GetTimeStamp());
    //step 1.2: 获取预测位姿
    double predict_x, predict_y, predict_yaw_deg;
    mvFrames[i].Get_Predicte_Pose(predict_x, predict_y, predict_yaw_deg);
    double predict_yaw_rad = predicted_process_->degToRad(predict_yaw_deg);
    Pose2D predict_pose{predict_x, predict_y, predict_yaw_rad};

    //step 1.3: 获取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpCurrentCloud = mvFrames[i].GetPclCloud(base_frame_);
    //laser_process_->voxelFilter(mpCurrentCloud);
    ros::WallTime t_end_preprocess = ros::WallTime::now();
    double Preprocess_time = (t_end_preprocess - t_start_preprocess).toSec() * 1000.0;

    //step 2: 初始化
    if (i < 1) {
      
      location_pose = Pose2D{0, 0, 0};
      predicted_pose = Pose2D{0, 0, 0};
      icp_location_pose = Pose2D{0, 0, 0};
    }
    //step 3 执行SLAM主流程
    else {
      ros::WallTime t_start_predicted_pose_process = ros::WallTime::now();
      //step 3.1: 局部坐标系下计算predicted_pose
      double dx_l, dy_l, dyaw;
      predicted_process_->PoseSubtract(predict_pose, last_predict_pose_, dx_l, dy_l, dyaw);
      predicted_process_->PoseAdd(last_location_pose_, dx_l, dy_l, dyaw, predicted_pose);
      ros::WallTime t_end_predicted_pose_process = ros::WallTime::now();
      double Predicted_pose_time = (t_end_predicted_pose_process - t_start_predicted_pose_process).toSec() * 1000.0;

      //step 3.2: 将局部坐标系下的点云转换到全局坐标系
      ros::WallTime t_start_transform_point_cloud = ros::WallTime::now();

      Eigen::Matrix3f Predicted_Pose_ = predicted_process_->PoseToMatrix(predicted_pose);
      Eigen::Matrix4f Predicted_pose_ = Eigen::Matrix4f::Identity();
      Predicted_pose_.block<2, 2>(0, 0) = Predicted_Pose_.block<2, 2>(0, 0);
      Predicted_pose_.block<2, 1>(0, 3) = Predicted_Pose_.block<2, 1>(0, 2);
      pcl::transformPointCloud(*mpCurrentCloud, *mpCurrentGlobalCloud, Predicted_pose_);
      mpGridMap = mapping_->getWholeMapCloud();

      ros::WallTime t_end_transform_point_cloud = ros::WallTime::now();
      double Transform_point_cloud_time = (t_end_transform_point_cloud - t_start_transform_point_cloud).toSec() * 1000.0;
      
      ros::WallTime t_start_icp_process = ros::WallTime::now();
      //step 3.3: ICP配准
      double icp_fitness_score = 0.0;
      laser_process_->IcpAlignMapFrame(mpCurrentGlobalCloud, mpGridMap, predicted_pose,
                                       icp_location_pose, *predicted_process_, &icp_fitness_score);

      ros::WallTime t_end_icp_process = ros::WallTime::now();
      double Icp_process_time = (t_end_icp_process - t_start_icp_process).toSec() * 1000.0;
      double Csm_process_time = 0.0;

      if (icp_fitness_score > 0.001) {

        ros::WallTime t_start_csm_process = ros::WallTime::now();
        // CSM 粗匹配，输出更好的先验位姿
        Pose2D csm_location_pose_ = predicted_pose;
        laser_process_->CSMCoarseMatch(mpCurrentGlobalCloud, mpGridMap,
                                       predicted_pose, csm_location_pose_,
                                       *predicted_process_);
        location_pose = csm_location_pose_;

        ros::WallTime t_end_csm_process = ros::WallTime::now();
        Csm_process_time = (t_end_csm_process - t_start_csm_process).toSec() * 1000.0;

        ROS_INFO(" CSM coarse match: location_pose: x=%.4f, y=%.4f, yaw=%.2f degree",
                 location_pose.x, location_pose.y, location_pose.yaw * 180.0 / M_PI);
      } 

      //step 3.4: 如果ICP配准失败，则使用CSM粗匹配
      else {
        location_pose = icp_location_pose;
      }

      //step 3.5: icp_cloud转换到全局坐标系
      pcl::PointCloud<pcl::PointXYZ>::Ptr mpIcpCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Eigen::Matrix3f Location_Pose_ = predicted_process_->PoseToMatrix(location_pose);
      Eigen::Matrix4f Location_Pose_matrix = Eigen::Matrix4f::Identity();
      Location_Pose_matrix.block<2, 2>(0, 0) = Location_Pose_.block<2, 2>(0, 0);
      Location_Pose_matrix.block<2, 1>(0, 3) = Location_Pose_.block<2, 1>(0, 2);
      pcl::transformPointCloud(*mpCurrentCloud, *mpIcpCloud, Location_Pose_matrix);

      ros::WallTime t_start_update_grid_map = ros::WallTime::now();
      // 5) 更新地图
      if (!mpIcpCloud->empty())
        mapping_->UpdateGridMap(mpIcpCloud, location_pose);

      ros::WallTime t_end_update_grid_map = ros::WallTime::now();
      double Update_grid_map_time = (t_end_update_grid_map - t_start_update_grid_map).toSec() * 1000.0;

      ros::WallTime t_start_publish_paths = ros::WallTime::now();
      // 6) 发布路径
      predicted_process_->publishPaths(map_frame_, timestamp,
                                      predict_pose.x, predict_pose.y, predict_pose.yaw,
                                      predicted_pose.x, predicted_pose.y, predicted_pose.yaw,
                                      location_pose.x, location_pose.y, location_pose.yaw);

      ros::WallTime t_end_publish_paths = ros::WallTime::now();
      double Publish_paths_time = (t_end_publish_paths - t_start_publish_paths).toSec() * 1000.0;

      ros::WallTime t_start_publish_map_visualization = ros::WallTime::now();
      // 7) 发布current_global_cloud、icp_cloud等
      mapping_->publishMapVisualization(mpCurrentGlobalCloud, mpIcpCloud, timestamp, i);

      ros::WallTime t_end_publish_map_visualization = ros::WallTime::now();
      double Publish_map_visualization_time = (t_end_publish_map_visualization - t_start_publish_map_visualization).toSec() * 1000.0;
      
      ros::WallTime t_start_publish_tf = ros::WallTime::now();
      // 8) 发布TF
      predicted_process_->publishTf(map_frame_, odom_frame_, base_frame_, timestamp,
                                      location_pose.x, location_pose.y, location_pose.yaw);

      ros::WallTime t_end_publish_tf = ros::WallTime::now();
      double Publish_tf_time = (t_end_publish_tf - t_start_publish_tf).toSec() * 1000.0;

      ros::WallTime total_time_end = ros::WallTime::now();
      double Total_time = (total_time_end - total_time).toSec() * 1000.0;

      ROS_INFO(" || Load_data_time: %.2f ms || Frame %d:|| total_time: %.2f ms || Preprocess_time: %.2f ms || Predicted_pose_time: %.2f ms || Transform_point_cloud_time: %.2f ms || Icp_process_time: %.2f ms || Csm_process_time: %.2f ms || Update_grid_map_time: %.2f ms || Publish_paths_time: %.2f ms || Publish_map_visualization_time: %.2f ms || Publish_tf_time: %.2f ms",
        Load_data_time, i, Total_time, Preprocess_time, Predicted_pose_time, Transform_point_cloud_time, 
        Icp_process_time, Csm_process_time, Update_grid_map_time, Publish_paths_time, Publish_map_visualization_time, Publish_tf_time);
      

      ROS_INFO(" || Frame %d:|| predict_pose: x=%.4f y=%.4f yaw=%.2f deg || ICP_location_pose: x=%.4f y=%.4f yaw=%.2f deg fit=%.4f || predicted_pose: x=%.4f y=%.4f yaw=%.2f deg || location_pose: x=%.4f y=%.4f yaw=%.2f deg || cloud: %zu grid_map: %zu current_global_cloud: %zu",
      i,predict_pose.x, predict_pose.y, predict_pose.yaw * 180.0 / M_PI,
      icp_location_pose.x, icp_location_pose.y, icp_location_pose.yaw * 180.0 / M_PI, icp_fitness_score,
      predicted_pose.x, predicted_pose.y, predicted_pose.yaw * 180.0 / M_PI,
      location_pose.x, location_pose.y, location_pose.yaw * 180.0 / M_PI,
      mpCurrentCloud->size(), mpGridMap->size(), mpCurrentGlobalCloud->size());

      ros::spinOnce();
      ros::Rate(10).sleep();
    }
    last_predict_pose_ = predict_pose;
    last_location_pose_ = location_pose;
  }
}

} // laster_slam_2d.cpp