/*
 * laster_slam_2d 建图模块实现文件
 * 负责维护和管理SLAM中的地图数据
 */
 #include "mapping.h"
 #include <ros/ros.h>
 #include <algorithm>
 #include <cmath>
 #include <pcl/filters/voxel_grid.h>
 
 namespace laster_slam_2d {
 
 
Mapping::Mapping(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

  // 加载参数
  pnh.param<std::string>("map_frame", map_frame_, "map");
  pnh.param<double>("map_resolution", map_resolution_, 0.02);
  pnh.param<double>("map_x_min", map_x_min_, -50.0);
  pnh.param<double>("map_y_min", map_y_min_, -50.0);
  pnh.param<double>("map_x_max", map_x_max_, 50.0);
  pnh.param<double>("map_y_max", map_y_max_, 50.0);

  // 地图和点云发布
  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  grid_map_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("grid_map_cloud", 1, true);
  current_global_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("current_cloud", 1, true);
  icp_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("icp_cloud", 1, true);

}

/**
 * @brief 获取整个地图点云
 * @return 全局地图点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Mapping::getWholeMapCloud() {

  if (!mpGridMap_) {
    mpGridMap_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mpGridMap_->header.frame_id = map_frame_;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mpGridMapCloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (mpGridMap_) {
    *mpGridMapCloud = *mpGridMap_;
  }
  mpGridMapCloud->header.frame_id = map_frame_;
  return mpGridMapCloud;
 
}

/**
 * @brief 将 icp_cloud 映射到栅格地图：有投影点的栅格只保留该栅格几何中心点，并用于更新 grid_map
 * @param icp_cloud 经过ICP配准后的输入点云（世界坐标系）
 * @param pose 当前扫描对应的机器人位姿（二维）
 */
void Mapping::UpdateGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud, const Pose2D& pose) {

  
  if (!icp_cloud || icp_cloud->empty()) return;

  //step1 计算地图栅格数量
  int Grid_map_Boundary_x = (int)((map_x_max_ - map_x_min_) / map_resolution_);
  int Grid_map_Boundary_y = (int)((map_y_max_ - map_y_min_) / map_resolution_);
  if (Grid_map_Boundary_x <= 0 || Grid_map_Boundary_y <= 0) return;
  
  //step1.1 计算地图栅格数量
  long long Grid_map_size = (long long)Grid_map_Boundary_x * Grid_map_Boundary_y;
  std::lock_guard<std::mutex> lock(map_mutex_);
  if ((long long)mvGrid2D_.size() != Grid_map_size) {
    mvGrid2D_.assign(Grid_map_size, 0);

    // 初始化占据栅格地图
    occupancy_map_.header.frame_id = map_frame_;
    occupancy_map_.info.resolution = map_resolution_;
    occupancy_map_.info.width = static_cast<int>(Grid_map_Boundary_x);
    occupancy_map_.info.height = static_cast<int>(Grid_map_Boundary_y);
    occupancy_map_.info.origin.position.x = map_x_min_;
    occupancy_map_.info.origin.position.y = map_y_min_;
    occupancy_map_.info.origin.position.z = 0;
    occupancy_map_.info.origin.orientation.w = 1;
    occupancy_map_.info.origin.orientation.x = 0;
    occupancy_map_.info.origin.orientation.y = 0;
    occupancy_map_.info.origin.orientation.z = 0;

    occupancy_map_.data.assign(
        Grid_map_size,
        static_cast<int8_t>(grid2DToOccupancy(0))); 
  }
   
  //step2 计算机器人起始栅格
  int origin_gx = (int)std::floor((pose.x - map_x_min_) / map_resolution_);
  int origin_gy = (int)std::floor((pose.y - map_y_min_) / map_resolution_);

  
  //step3 遍历点云中每个点
  for (int i_pt = 0; i_pt < static_cast<int>(icp_cloud->points.size()); i_pt++) {
    
    //step3.1 计算点云中每个点对应的栅格
    double point_x = icp_cloud->points[i_pt].x;
    double point_y = icp_cloud->points[i_pt].y;
    //step3.1.1 计算点云中每个点对应的栅格
    int point_gx = (int)std::floor((point_x - map_x_min_) / map_resolution_);
    int point_gy = (int)std::floor((point_y - map_y_min_) / map_resolution_);
    
    //step3.2 计算从机器人起始栅格到点云中每个点对应的栅格的射线经过的所有栅格
    std::vector<std::pair<int, int>> mvRay = GetLinePathCells(origin_gx, origin_gy, point_gx, point_gy, 
                                                              Grid_map_Boundary_x, Grid_map_Boundary_y);
    
    //step3.3 遍历射线经过的所有栅格
    for (int i = 0; i < static_cast<int>(mvRay.size()); i++) {

      //step3.3.1 计算射线经过的栅格
      int grid_x = mvRay[i].first;
      int grid_y = mvRay[i].second;
      if (grid_x < 0 || grid_x >= Grid_map_Boundary_x || grid_y < 0 || grid_y >= Grid_map_Boundary_y) continue;

      //step3.3.2 计算栅格索引
      long long idx = static_cast<long long>(grid_y) * Grid_map_Boundary_x + grid_x;
      
      int old_v = static_cast<int>(mvGrid2D_[idx]);
      // 如果已经是确定占据（-20），则不再更新该栅格
      if (old_v <= -20) {
        continue;
      }

      int v = old_v;
      if (i == static_cast<int>(mvRay.size()) - 1)
        v -= 1;
      else
        v += 1;

      // 限制栅格值范围到 [-20, 5]
      v = std::max(-20, std::min(5, v));
      // 用 v 更新 mvGrid2D_
      mvGrid2D_[idx] = static_cast<int8_t>(v);

      // 同步更新占据栅格地图对应栅格（局部增量更新）
      if (idx >= 0 && idx < static_cast<long long>(occupancy_map_.data.size())) {
        occupancy_map_.data[idx] =
            static_cast<int8_t>(grid2DToOccupancy(mvGrid2D_[idx]));
      }

      //step3.3.3 如果射线经过的栅格是终点，且该栅格第一次进入 [-20,-5] 区间，则将栅格几何中心点加入累积点云
      if (i == static_cast<int>(mvRay.size()) - 1 && old_v > -5 && v <= -5) {
        double grid_x_center = map_x_min_ + (grid_x + 0.5) * map_resolution_;
        double grid_y_center = map_y_min_ + (grid_y + 0.5) * map_resolution_;
        pcl::PointXYZ center_point;
        center_point.x = static_cast<float>(grid_x_center);
        center_point.y = static_cast<float>(grid_y_center);
        center_point.z = 0.0f;
        mpGridMap_->points.push_back(center_point);
      }
    }
  }
  
  if (mpGridMap_) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(mpGridMap_);
    const float leaf_size = static_cast<float>(map_resolution_);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*tmp);
    ROS_INFO("voxelFilter: grid_map size: %zu, filtered size: %zu",
            mpGridMap_->size(), tmp->size());
    mpGridMap_.swap(tmp);
  }

}

void Mapping::publishMapVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                                      const ros::Time& timestamp, int frame_id) {
  sensor_msgs::PointCloud2 msg;

  // 当前帧点云
  if (current_global_cloud && !current_global_cloud->empty()) {

    pcl::toROSMsg(*current_global_cloud, msg);
    msg.header.stamp = timestamp;           // 设置时间戳
    msg.header.frame_id = map_frame_;        // 设置坐标系为map
    current_global_cloud_pub_.publish(msg);  // 发布当前帧点云
  }

  // ICP 对齐后的点云
  if (icp_cloud && !icp_cloud->empty()) {
    pcl::toROSMsg(*icp_cloud, msg);
    msg.header.stamp = timestamp;
    msg.header.frame_id = map_frame_;
    icp_cloud_pub_.publish(msg);
  }

  // 累积地图点云
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (mpGridMap_ && !mpGridMap_->empty()) {
      pcl::toROSMsg(*mpGridMap_, msg);
      msg.header.stamp = timestamp;
      msg.header.frame_id = map_frame_;
      grid_map_cloud_pub_.publish(msg);
    }

    // 占据栅格：只发布已在 UpdateGridMap 中增量更新好的 occupancy_map_
    if (!occupancy_map_.data.empty()) {
      occupancy_map_.header.frame_id = map_frame_;
      occupancy_map_.header.stamp = timestamp;
      map_pub_.publish(occupancy_map_);
    }
  }
}

// 计算从 (x0,y0) 到 (x1,y1) 的射线经过的所有栅格 (gx,gy)
std::vector<std::pair<int, int>> Mapping::GetLinePathCells(int origin_x_cell, int origin_y_cell, int point_x_cell, int point_y_cell,
                                                             int Grid_map_Boundary_x, int Grid_map_Boundary_y) const {


  std::vector<std::pair<int, int>> mvLinePathCells;//射线经过的所有栅格
  int dx = std::abs(point_x_cell - origin_x_cell);
  int dy = std::abs(point_y_cell - origin_y_cell);
  
  int sx, sy;
  sx = (origin_x_cell < point_x_cell) ? 1 : -1;
  sy = (origin_y_cell < point_y_cell) ? 1 : -1;
  //误差项
  int err = dx - dy;
                                                              
  //主循环
  while (1) {
    if (origin_x_cell >= 0 && origin_x_cell < Grid_map_Boundary_x && origin_y_cell >= 0 && origin_y_cell < Grid_map_Boundary_y) {
      std::pair<int, int> grid_cell(origin_x_cell, origin_y_cell);
      mvLinePathCells.push_back(grid_cell);
    }
    if (origin_x_cell == point_x_cell && origin_y_cell == point_y_cell)
      break;
    
    //更新经过的栅格
    int err2 = 2 * err;
    if (err2 > -dy) {
      err = err - dy;
      origin_x_cell = origin_x_cell + sx;
    }
    if (err2 < dx) {
      err = err + dx;
      origin_y_cell = origin_y_cell + sy;
    }
  }
  return mvLinePathCells;
}

}  // mapping.cpp