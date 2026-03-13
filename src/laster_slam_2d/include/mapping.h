#ifndef LASTER_SLAM_2D_MAPPING_H
 #define LASTER_SLAM_2D_MAPPING_H
 
 #include <ros/ros.h>
 #include <nav_msgs/OccupancyGrid.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/common/transforms.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <Eigen/Dense>
 #include <algorithm>
 #include <map>
 #include <set>
 #include <vector>
 #include <mutex>
 
 #include "frame.h"
 #include "predicted_process.h"
 
 namespace laster_slam_2d {
 
 class Mapping {
  public:
    //
    Mapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /// 获取整个地图点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getWholeMapCloud();


    /// 使用当前帧 ICP 结果更新栅格地图/累积点云
    void UpdateGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                        const Pose2D& location_pose);

    /// 发布地图及相关可视化点云
    void publishMapVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                                const ros::Time& timestamp,
                                int frame_id);

  private:



    /// 计算从 (x0,y0) 到 (x1,y1) 的射线经过的所有栅格 (gx,gy)
    std::vector<std::pair<int, int>> GetLinePathCells(int origin_x_cell, int origin_y_cell, int point_x_cell, int point_y_cell,
                                                     int Grid_map_Boundary_x, int Gridmap_Boundary_y) const;
    
    /// 发布地图及相关可视化点云的发布器
    ros::Publisher map_pub_;
    ros::Publisher grid_map_cloud_pub_;
    ros::Publisher current_global_cloud_pub_;
    ros::Publisher icp_cloud_pub_;

    //< 占据栅格地图
    nav_msgs::OccupancyGrid occupancy_map_;

    //地图坐标系
    std::string map_frame_;
    //地图分辨率
    double map_resolution_;
    //地图边界
    double map_x_min_, map_y_min_, map_x_max_, map_y_max_;
    //体素滤波栅格大小
    double voxel_size_;  
    //地图互斥锁
    std::mutex map_mutex_;
    // 累积点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpGridMap_;
    //Grid2D：每格 [-5, 5]，穿过+1、终点-1，到-5 时该栅格中心加入 mpGridMap_
    std::vector<int8_t> mvGrid2D_;

    //Grid2D 值 v 映射到 OccupancyGrid格值：-5->100(黑)，5->0(白)
    static int grid2DToOccupancy(int8_t v) {
      int x = static_cast<int>(v);
      return std::max(0, std::min(100, (5 - x) * 100 / 10));
    }

 };
 
 }  // namespace laster_slam_2d
 
 #endif
 //mapping.h