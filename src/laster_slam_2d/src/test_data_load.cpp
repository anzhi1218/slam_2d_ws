/*
 * 测试读入的数据是否正确
 */
 #include "data_preprocess.h"
 #include "frame.h"
 #include <ros/ros.h>
 #include <string>
 #include <vector>
 
 int main(int argc, char** argv) {
   ros::init(argc, argv, "test_data_load");
   ros::NodeHandle nh;
   std::string data_file = argv[1];
   std::vector<laster_slam_2d::Frame> mvFrames;
   laster_slam_2d::LoadDataFile(data_file, mvFrames);
   for (int i = 0; i < static_cast<int>(mvFrames.size()); ++i) {
     //先验位姿打印
     double x, y, yaw_deg;
     mvFrames[i].Get_Predicte_Pose(x, y, yaw_deg);
     ROS_INFO("frame %d: time = %f, yaw = %f, x = %f, y = %f", i, mvFrames[i].GetTimeStamp(), yaw_deg, x, y);
     // 激光点云打印
     pcl::PointCloud<pcl::PointXYZ>::Ptr mpCloud = mvFrames[i].GetPclCloud(std::to_string(i));
     ROS_INFO("cloud size: %zu", mpCloud->points.size());

     ros::spinOnce();
     ros::Rate(50).sleep();
   }
   return 0;
 }//test_data_load.cpp