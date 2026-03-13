#ifndef LASTER_SLAM_2D_FRAME_H
#define LASTER_SLAM_2D_FRAME_H

#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace laster_slam_2d {

// 2D位姿：x、y、yaw(弧度)
struct Pose2D {
   double x = 0;
   double y = 0;
   double yaw = 0;
 };
 
 
 //单帧数据：timestamp, pose: yaw x,y; cloud: x,y
 class Frame {
 public:
   
   //构造函数
   Frame(int frame_id, double time_stamp,
         double predict_x, double predict_y, double predict_yaw_deg,
         const std::vector<double>& cloud_x, const std::vector<double>& cloud_y);
 
   // ----------获取时间戳，单位：秒----------
   double GetTimeStamp() const { return TimeStamp_ / 1000.0; }
 
   // ---------- 先验位姿 ---------
   /// 一次性获取先验位姿 (x, y, yaw_deg)
   void Get_Predicte_Pose(double& predict_x, double& predict_y, double& predict_yaw_deg) const;
 
   /// 获取点云
   pcl::PointCloud<pcl::PointXYZ>::Ptr GetPclCloud(const std::string& frame_id = "base_link") const;
 
 private:
 
   //----frame_id、时间戳、先验位姿、点云----
   int frame_id_ = 0;
   double TimeStamp_ = 0.0;
   double predict_x_ = 0.0;
   double predict_y_ = 0.0;
   double predict_yaw_deg_ = 0.0;
   std::vector<double> mvCloudX_;
   std::vector<double> mvCloudY_;
 
 };
 
 }  // namespace laster_slam_2d
 
 #endif  // LASTER_SLAM_2D_FRAME_H
 //frame.h