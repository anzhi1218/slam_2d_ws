/*
 * slam_2d 建图模块实现文件
 * 负责维护和管理SLAM中的地图数据
 */
#include "mapping.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace slam_2d {

// 递归创建目录（用于地图保存路径）：不存在则逐级创建，已存在则直接返回
namespace {
bool EnsureDirectoryExists(const std::string& path) {
  if (path.empty()) return true;
  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    return S_ISDIR(info.st_mode);
  }
  const std::size_t slash = path.find_last_of('/');
  const std::string parent =
      (slash == std::string::npos) ? std::string()
      : (slash == 0 ? std::string("/") : path.substr(0, slash));
  if (!parent.empty() && !EnsureDirectoryExists(parent)) return false;
  return ::mkdir(path.c_str(), 0755) == 0;
}
}  // namespace

Mapping::Mapping(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  // 加载参数
  pnh.param<std::string>("map_frame", map_frame_, "map");
  pnh.param<double>("map_resolution", map_resolution_, 0.05);
  pnh.param<double>("map_x_min", map_x_min_, -50.0);
  pnh.param<double>("map_y_min", map_y_min_, -50.0);
  pnh.param<double>("map_x_max", map_x_max_, 50.0);
  pnh.param<double>("map_y_max", map_y_max_, 50.0);
  pnh.param<double>("map_save_trim_margin", map_save_trim_margin_, 1.0);
  pnh.param<double>("map_save_occupied_thresh", map_save_occupied_thresh_, 0.65);
  pnh.param<double>("map_save_free_thresh", map_save_free_thresh_, 0.196);

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
  std::lock_guard<std::mutex> lock(map_mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  map_cloud->header.frame_id = map_frame_;
  map_cloud->points.reserve(occupied_cells_.size());
  const std::size_t width = occupancy_map_.info.width;
  for (std::size_t index : occupied_cells_) {
    const std::size_t gx = index % width;
    const std::size_t gy = index / width;
    pcl::PointXYZ point;
    point.x = static_cast<float>(map_x_min_ + (gx + 0.5) * map_resolution_);
    point.y = static_cast<float>(map_y_min_ + (gy + 0.5) * map_resolution_);
    point.z = 0.0f;
    map_cloud->points.push_back(point);
  }
  map_cloud->width = static_cast<std::uint32_t>(map_cloud->points.size());
  map_cloud->height = 1;
  return map_cloud;
}

/**
 * @brief 将 icp_cloud 映射到栅格地图：有投影点的栅格只保留该栅格几何中心点，并用于更新 grid_map
 * @param icp_cloud 经过ICP配准后的输入点云（世界坐标系）
 * @param pose 当前扫描对应的机器人位姿（二维）
 */
void Mapping::UpdateGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud, const Pose2D& pose) {
  if (!icp_cloud || icp_cloud->empty()) return;

  const int grid_width = static_cast<int>((map_x_max_ - map_x_min_) / map_resolution_);
  const int grid_height = static_cast<int>((map_y_max_ - map_y_min_) / map_resolution_);
  if (grid_width <= 0 || grid_height <= 0) return;
  const std::size_t grid_size = static_cast<std::size_t>(grid_width) * grid_height;
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (mvGrid2D_.size() != grid_size) {
    mvGrid2D_.assign(grid_size, 0);
    occupied_cells_.clear();

    occupancy_map_.header.frame_id = map_frame_;
    occupancy_map_.info.resolution = map_resolution_;
    occupancy_map_.info.width = grid_width;
    occupancy_map_.info.height = grid_height;
    occupancy_map_.info.origin.position.x = map_x_min_;
    occupancy_map_.info.origin.position.y = map_y_min_;
    occupancy_map_.info.origin.orientation.w = 1;

    occupancy_map_.data.assign(grid_size, static_cast<int8_t>(grid2DToOccupancy(0)));
  }

  const int origin_gx = static_cast<int>(std::floor((pose.x - map_x_min_) / map_resolution_));
  const int origin_gy = static_cast<int>(std::floor((pose.y - map_y_min_) / map_resolution_));

  std::vector<std::pair<int, int>> ray_cells;
  std::unordered_set<std::size_t> free_cells;
  std::unordered_set<std::size_t> hit_cells;
  hit_cells.reserve(icp_cloud->size());
  free_cells.reserve(icp_cloud->size() * 8);
  for (const pcl::PointXYZ& point : icp_cloud->points) {
    const int point_gx = static_cast<int>(std::floor((point.x - map_x_min_) / map_resolution_));
    const int point_gy = static_cast<int>(std::floor((point.y - map_y_min_) / map_resolution_));
    GetLinePathCells(origin_gx, origin_gy, point_gx, point_gy,
                     grid_width, grid_height, ray_cells);

    for (const auto& cell : ray_cells) {
      const std::size_t idx = static_cast<std::size_t>(cell.second) * grid_width + cell.first;
      const bool is_endpoint = cell.first == point_gx && cell.second == point_gy;
      if (is_endpoint) hit_cells.insert(idx);
      else free_cells.insert(idx);
    }
  }

  // One scan contributes at most one vote per cell. A measured endpoint
  // takes precedence over rays from other beams passing through that cell.
  auto update_cell = [this](std::size_t idx, int delta) {
    const int old_score = mvGrid2D_[idx];
    const int score = std::max(-20, std::min(5, old_score + delta));
    if (score == old_score) return;
    mvGrid2D_[idx] = static_cast<int8_t>(score);
    occupancy_map_.data[idx] = static_cast<int8_t>(grid2DToOccupancy(mvGrid2D_[idx]));
    if (old_score > -5 && score <= -5) occupied_cells_.insert(idx);
    else if (old_score <= -5 && score > -5) occupied_cells_.erase(idx);
  };
  for (std::size_t idx : hit_cells) update_cell(idx, -1);
  for (std::size_t idx : free_cells) {
    if (hit_cells.count(idx) == 0) update_cell(idx, 1);
  }
}

void Mapping::publishMapVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_cloud,
                                      const ros::Time& timestamp) {
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
  const auto map_cloud = getWholeMapCloud();
  if (!map_cloud->empty()) {
    pcl::toROSMsg(*map_cloud, msg);
    msg.header.stamp = timestamp;
    msg.header.frame_id = map_frame_;
    grid_map_cloud_pub_.publish(msg);
  }
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    // 占据栅格：只发布已在 UpdateGridMap 中增量更新好的 occupancy_map_
    if (!occupancy_map_.data.empty()) {
      occupancy_map_.header.frame_id = map_frame_;
      occupancy_map_.header.stamp = timestamp;
      map_pub_.publish(occupancy_map_);
    }
  }
}

// 计算从 (x0,y0) 到 (x1,y1) 的射线经过的所有栅格 (gx,gy)
void Mapping::GetLinePathCells(int origin_x_cell, int origin_y_cell, int point_x_cell, int point_y_cell,
                               int Grid_map_Boundary_x, int Grid_map_Boundary_y,
                               std::vector<std::pair<int, int>>& cells) const {
  cells.clear();
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
      cells.emplace_back(origin_x_cell, origin_y_cell);
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
}

bool Mapping::SaveMap(const std::string& file_prefix) {
  if (file_prefix.empty()) {
    ROS_ERROR("SaveMap: empty file prefix");
    return false;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  const int width = static_cast<int>(occupancy_map_.info.width);
  const int height = static_cast<int>(occupancy_map_.info.height);
  if (width <= 0 || height <= 0 ||
      static_cast<int>(occupancy_map_.data.size()) != width * height) {
    ROS_WARN("SaveMap: no map to save (run the SLAM node to build a map first)");
    return false;
  }

  // 1) 求已知区域包围盒（mvGrid2D_ != 0 视为已知，0 为未观测）
  int gx0 = width, gy0 = height, gx1 = -1, gy1 = -1;
  for (int gy = 0; gy < height; ++gy) {
    for (int gx = 0; gx < width; ++gx) {
      if (mvGrid2D_[static_cast<std::size_t>(gy) * width + gx] != 0) {
        gx0 = std::min(gx0, gx);
        gx1 = std::max(gx1, gx);
        gy0 = std::min(gy0, gy);
        gy1 = std::max(gy1, gy);
      }
    }
  }
  if (gx1 < gx0 || gy1 < gy0) {
    ROS_WARN("SaveMap: map is empty (no observed cell), nothing to save");
    return false;
  }

  // 2) 裁剪边界外扩边距，并夹到地图范围内
  const int margin_cells = std::max(
      1, static_cast<int>(std::ceil(map_save_trim_margin_ / map_resolution_)));
  gx0 = std::max(0, gx0 - margin_cells);
  gy0 = std::max(0, gy0 - margin_cells);
  gx1 = std::min(width - 1, gx1 + margin_cells);
  gy1 = std::min(height - 1, gy1 + margin_cells);

  // 3) 确保输出目录存在
  const std::size_t slash = file_prefix.find_last_of('/');
  const std::string dir = (slash == std::string::npos) ? std::string()
      : (slash == 0 ? std::string("/") : file_prefix.substr(0, slash));
  if (!dir.empty() && !EnsureDirectoryExists(dir)) {
    ROS_ERROR("SaveMap: cannot create directory: %s", dir.c_str());
    return false;
  }

  // 4) 写 PGM（P5 二进制；行序自最大 y 到最小 y，与 map_server/nav2 读取约定一致）
  const int out_width = gx1 - gx0 + 1;
  const int out_height = gy1 - gy0 + 1;
  std::string pixels;
  pixels.resize(static_cast<std::size_t>(out_width) * out_height);
  std::size_t write_index = 0;
  for (int gy = gy1; gy >= gy0; --gy) {
    for (int gx = gx0; gx <= gx1; ++gx) {
      const int8_t value = occupancy_map_.data[static_cast<std::size_t>(gy) * width + gx];
      unsigned char pixel = 205;  // 未知（灰）
      if (value >= 0) {
        const double occ = static_cast<double>(value) / 100.0;
        if (occ > map_save_occupied_thresh_) {
          pixel = 0;    // 占据（黑）
        } else if (occ < map_save_free_thresh_) {
          pixel = 254;  // 空闲（白）
        }
      }
      pixels[write_index++] = static_cast<char>(pixel);
    }
  }

  const std::string pgm_path = file_prefix + ".pgm";
  std::ofstream pgm(pgm_path.c_str(), std::ios::out | std::ios::binary);
  if (!pgm.is_open()) {
    ROS_ERROR("SaveMap: cannot open %s for writing", pgm_path.c_str());
    return false;
  }
  pgm << "P5\n" << out_width << ' ' << out_height << "\n255\n";
  pgm.write(pixels.data(), static_cast<std::streamsize>(pixels.size()));
  const bool pgm_ok = pgm.good();
  pgm.close();
  if (!pgm_ok) {
    ROS_ERROR("SaveMap: failed to write %s", pgm_path.c_str());
    return false;
  }

  // 5) 写 YAML（字段与 map_server/nav2_map_server 一致；mode 为 Nav2 可选字段）
  const std::string base_name =
      (slash == std::string::npos) ? file_prefix : file_prefix.substr(slash + 1);
  const double origin_x = map_x_min_ + static_cast<double>(gx0) * map_resolution_;
  const double origin_y = map_y_min_ + static_cast<double>(gy0) * map_resolution_;
  const std::string yaml_path = file_prefix + ".yaml";
  std::ofstream yaml(yaml_path.c_str());
  if (!yaml.is_open()) {
    ROS_ERROR("SaveMap: cannot open %s for writing", yaml_path.c_str());
    return false;
  }
  yaml << "image: " << base_name << ".pgm\n"
       << "resolution: " << map_resolution_ << "\n"
       << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
       << "negate: 0\n"
       << "occupied_thresh: " << map_save_occupied_thresh_ << "\n"
       << "free_thresh: " << map_save_free_thresh_ << "\n"
       << "mode: trinary\n";
  const bool yaml_ok = yaml.good();
  yaml.close();
  if (!yaml_ok) {
    ROS_ERROR("SaveMap: failed to write %s", yaml_path.c_str());
    return false;
  }

  ROS_INFO("SaveMap: saved %s / %s (%d x %d px, resolution %.3f, origin [%.3f, %.3f])",
           pgm_path.c_str(), yaml_path.c_str(), out_width, out_height,
           map_resolution_, origin_x, origin_y);
  return true;
}

}  // mapping.cpp
