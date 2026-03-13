/*
 * laster_slam_2d - 2D激光SLAM。
 */
 #include "laster_slam_2d.h"
 #include <ros/ros.h>
 #include <string>
 
 
 int main(int argc, char** argv) {
 
   // 初始化 ROS 节点
   ros::init(argc, argv, "laster_slam_2d");
 
   
   ros::NodeHandle pnh("~");
 
   std::string data_file;
   pnh.param<std::string>("data_file", data_file, "");
   if (data_file.empty()) {
     ROS_ERROR("laster_slam_2d: set private param data_file is empty");
     return -1;
   }
 
   
   laster_slam_2d::Slam2dLaster gn;
   gn.Process_2d_Laster_Slam(data_file);
   ROS_INFO("laster_slam_2d: data file replay stopped.");
 
   ros::spin();
   return 0;
 }//main.cpp