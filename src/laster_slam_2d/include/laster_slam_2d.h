#ifndef LASTER_SLAM_2D_SLAM_LASTER_SLAM_2D_H
#define LASTER_SLAM_2D_SLAM_LASTER_SLAM_2D_H
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <memory>
#include <string>
#include <atomic>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "frame.h"
#include "predicted_process.h"
#include "data_preprocess.h"
#include "laser_process.h"
#include "mapping.h"

namespace laster_slam_2d {
 
 class Slam2dLaster {
 public:
    Slam2dLaster();
    ~Slam2dLaster();

    void Process_2d_Laster_Slam(const std::string& data_file_path);
 
 private:
    //// ----- ROS节点句柄 -----
    ros::NodeHandle node_; //公共节点句柄
    ros::NodeHandle private_nh_; //私有节点句柄

    // ----- 坐标系名称 -----
    std::string map_frame_; //地图坐标系
    std::string odom_frame_; //里程计坐标系
    std::string base_frame_; //机器人基坐标系
    
    
    std::unique_ptr<PredictedProcess> predicted_process_;
    std::unique_ptr<LaserProcess> laser_process_;
    std::unique_ptr<Mapping> mapping_;

    Pose2D last_location_pose_;   ///< 上一帧最终位姿
    Pose2D last_predict_pose_;    ///< 上一帧 predict_pose

    Pose2D location_pose;
    Pose2D predicted_pose;
    Pose2D icp_location_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpGridMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpCurrentGlobalCloud;
};
 
 } 
 
 #endif //laster_slam_2d.h