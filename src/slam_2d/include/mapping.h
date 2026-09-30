#ifndef SLAM_2D_MAPPING_H
#define SLAM_2D_MAPPING_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "frame.h"

namespace slam_2d {

class Mapping {
 public:
  Mapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getWholeMapCloud();
  void UpdateGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                     const Pose2D& location_pose);
  void publishMapVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                               const ros::Time& timestamp);
  bool SaveMap(const std::string& file_prefix);

 private:
  void GetLinePathCells(int origin_x_cell, int origin_y_cell,
                        int point_x_cell, int point_y_cell,
                        int grid_map_boundary_x, int grid_map_boundary_y,
                        std::vector<std::pair<int, int>>& cells) const;

  static int grid2DToOccupancy(int8_t value) {
    const int score = static_cast<int>(value);
    return std::max(0, std::min(100, (5 - score) * 10));
  }

  ros::Publisher map_pub_;
  ros::Publisher grid_map_cloud_pub_;
  ros::Publisher current_global_cloud_pub_;
  ros::Publisher icp_cloud_pub_;

  nav_msgs::OccupancyGrid occupancy_map_;
  std::string map_frame_;
  double map_resolution_;
  double map_x_min_, map_y_min_, map_x_max_, map_y_max_;
  double map_save_trim_margin_;
  double map_save_occupied_thresh_;
  double map_save_free_thresh_;
  std::mutex map_mutex_;
  std::unordered_set<std::size_t> occupied_cells_;
  std::vector<int8_t> mvGrid2D_;
};

}  // namespace slam_2d

#endif  // SLAM_2D_MAPPING_H
