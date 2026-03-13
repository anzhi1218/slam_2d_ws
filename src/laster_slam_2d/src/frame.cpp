#include "frame.h"

namespace laster_slam_2d {
 
 
 //构造函数
 Frame::Frame(int frame_id, double time_stamp,
              double predict_x, double predict_y, double predict_yaw_deg,
              const std::vector<double>& cloud_x, const std::vector<double>& cloud_y)
     : frame_id_(frame_id),
       TimeStamp_(time_stamp),
       predict_x_(predict_x),
       predict_y_(predict_y),
       predict_yaw_deg_(predict_yaw_deg),
       mvCloudX_(cloud_x),
       mvCloudY_(cloud_y) {}
 
 /**
 * @brief 获取先验位姿
 * @param x 先验位姿x
 * @param y 先验位姿y
 * @param yaw_deg 先验位姿yaw_deg
 * @return predicted_pose
 */
 void Frame::Get_Predicte_Pose(double& predict_x, double& predict_y, double& predict_yaw_deg) const {
  predict_x = predict_x_;
  predict_y = predict_y_;
  predict_yaw_deg = predict_yaw_deg_;
 }


 /**
  * @brief 获取点云
  * @param frame_id 帧id
  * @return 点云
  */
pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::GetPclCloud(const std::string& frame_id) const {
  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr mpCloud(new pcl::PointCloud<pcl::PointXYZ>);
  mpCloud->header.frame_id = frame_id;
  mpCloud->points.reserve(std::min(static_cast<int>(mvCloudX_.size()), static_cast<int>(mvCloudY_.size())));
  for (int i = 0; i < static_cast<int>(mvCloudX_.size()) && i < static_cast<int>(mvCloudY_.size()); ++i) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(mvCloudX_[i]);
    pt.y = static_cast<float>(mvCloudY_[i]);
    pt.z = 0;
    mpCloud->points.push_back(pt);
  }
  return mpCloud;
}

} 