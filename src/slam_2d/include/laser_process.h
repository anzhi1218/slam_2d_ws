#ifndef SLAM_2D_LASER_PROCESS_H
#define SLAM_2D_LASER_PROCESS_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "frame.h"

namespace slam_2d {

class LaserProcess {
 public:
  explicit LaserProcess(ros::NodeHandle& pnh);

  /// ICP 配准
  void IcpAlignMapFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                        const Pose2D& predicted_pose, Pose2D& icp_location_pose,
                        double& fitness_score);

  /// CSM 粗匹配
  bool CSMCoarseMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                      const Pose2D& predicted_pose, Pose2D& csm_location_pose);

 private:

  int icp_max_iterations_;
  double icp_max_correspondence_dist_;
  double icp_transform_epsilon_;
  double icp_euclidean_fitness_epsilon_;

  double map_resolution_;
  int translation_search_range_cells_;
  double translation_search_step_m_;
  double yaw_search_range_deg_;
  double angular_step_deg_;
};

}  // namespace slam_2d

#endif  // SLAM_2D_LASER_PROCESS_H
