/*
 * slam_2d - 2D激光SLAM。
 */
#include "slam_2d.h"
#include <ros/ros.h>


int main(int argc, char** argv) {

  // 初始化 ROS 节点
  ros::init(argc, argv, "slam_2d");

  // 订阅激光与里程计话题（rosbag play 回放或实际传感器）在线建图
  slam_2d::Slam2d gn;
  gn.Process_2d_Slam();
  ROS_INFO("slam_2d: node started.");

  ros::spin();
  return 0;
}//main.cpp
