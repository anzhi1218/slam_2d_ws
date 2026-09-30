/*
 * slam_2d - 点云处理实现
 */
 #include "laser_process.h"
#include "predicted_process.h"
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <cstdint>
#include <cmath>
#include <functional>
#include <utility>
#include <unordered_set>

namespace slam_2d {

namespace {

using GridCell = std::pair<std::int64_t, std::int64_t>;

struct GridCellHash {
  std::size_t operator()(const GridCell& cell) const {
    const std::size_t x_hash = std::hash<std::int64_t>{}(cell.first);
    const std::size_t y_hash = std::hash<std::int64_t>{}(cell.second);
    return x_hash ^ (y_hash + 0x9e3779b9 + (x_hash << 6) + (x_hash >> 2));
  }
};

}  // namespace

LaserProcess::LaserProcess(ros::NodeHandle& pnh) {

  // ICP 配准参数设置
  pnh.param<int>("icp_max_iterations", icp_max_iterations_, 20);
  pnh.param<double>("icp_max_correspondence_dist", icp_max_correspondence_dist_, 0.3); //最大对应距离
  pnh.param<double>("icp_transform_epsilon", icp_transform_epsilon_, 1e-8);
  pnh.param<double>("icp_euclidean_fitness_epsilon", icp_euclidean_fitness_epsilon_, 1e-6);

  // CSM 参数
  pnh.param<double>("map_resolution",  map_resolution_, 0.05);
  pnh.param<int>("translation_search_range_cells", translation_search_range_cells_, 2);
  pnh.param<double>("translation_search_step_cells", translation_search_step_m_, 0.02);
  pnh.param<double>("angular_step_deg", angular_step_deg_, 0.5);
  pnh.param<double>("yaw_search_range_deg", yaw_search_range_deg_, 2);
}


/**
 * @brief ICP配准流程
 * @param current_global_cloud 当前全局点云
 * @param grid_map 地图点云
 * @param predicted_pose 预测位姿
 * @param icp_location_pose ICP位置
 * @param icp_fitness_score ICP拟合得分
 */
void LaserProcess::IcpAlignMapFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                                    const Pose2D& predicted_pose,
                                    Pose2D& icp_location_pose,
                                    double& icp_fitness_score) {
  icp_fitness_score = 0.0;

  //step1 检查输入
  if (!current_global_cloud || !grid_map ||
      current_global_cloud->empty() || grid_map->empty()) {
    icp_location_pose = predicted_pose;
    return;
  }

  // step2. ICP配准流程
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_align;
  //设置输入源和目标
  icp_align.setInputSource(current_global_cloud);
  icp_align.setInputTarget(grid_map);
  //设置最大迭代次数、最大对应距离、变换阈值、欧氏拟合阈值
  icp_align.setMaximumIterations(icp_max_iterations_);
  icp_align.setMaxCorrespondenceDistance(icp_max_correspondence_dist_);
  icp_align.setTransformationEpsilon(icp_transform_epsilon_);
  icp_align.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);

  //执行配准
  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp_align.align(aligned, Eigen::Matrix4f::Identity());

  Eigen::Matrix4f Icp_correction_pose = icp_align.getFinalTransformation();
  const Eigen::Matrix4f predicted_transform =
      PredictedProcess::PoseToMatrix4f(predicted_pose);
  const Eigen::Matrix4f icp_transform = Icp_correction_pose * predicted_transform;

  icp_fitness_score = icp_align.getFitnessScore();
  icp_location_pose = PredictedProcess::MatrixToPose(icp_transform);
}



/**
 * @brief CSM 粗匹配
 * @param current_global_cloud 当前全局点云
 * @param grid_map 地图点云
 * @param predicted_pose 预测位姿
 * @param csm_location_pose CSM估计的位姿
 */
bool LaserProcess::CSMCoarseMatch(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
    const Pose2D& predicted_pose, Pose2D& csm_location_pose) {
  if (!current_global_cloud || !grid_map || current_global_cloud->empty() || grid_map->empty()) {
    csm_location_pose = predicted_pose;
    return false;
  }

  // Rasterize the map once; each candidate only needs point lookups.
  std::unordered_set<GridCell, GridCellHash> grid_map_points;
  grid_map_points.reserve(grid_map->size());
  for (const pcl::PointXYZ& point : grid_map->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
    const auto gx = static_cast<std::int64_t>(std::floor(point.x / map_resolution_));
    const auto gy = static_cast<std::int64_t>(std::floor(point.y / map_resolution_));
    grid_map_points.emplace(gx, gy);
  }
  if (grid_map_points.empty()) {
    csm_location_pose = predicted_pose;
    return false;
  }

  std::size_t valid_point_count = 0;
  for (const pcl::PointXYZ& point : current_global_cloud->points) {
    valid_point_count += std::isfinite(point.x) && std::isfinite(point.y);
  }
  if (valid_point_count == 0) {
    csm_location_pose = predicted_pose;
    return false;
  }

  double best_score = 0;
  Pose2D best_csm_pose = predicted_pose;
  const double predicted_cos = std::cos(predicted_pose.yaw);
  const double predicted_sin = std::sin(predicted_pose.yaw);

  for (double yaw_deg = -yaw_search_range_deg_;
       yaw_deg <= yaw_search_range_deg_ + 1e-6; yaw_deg += angular_step_deg_) {
    const double delta_yaw = yaw_deg * M_PI / 180.0;
    const double cos_delta = std::cos(delta_yaw);
    const double sin_delta = std::sin(delta_yaw);
    const double correction_x = predicted_pose.x -
        (cos_delta * predicted_pose.x - sin_delta * predicted_pose.y);
    const double correction_y = predicted_pose.y -
        (sin_delta * predicted_pose.x + cos_delta * predicted_pose.y);

    for (int dx_cells = -translation_search_range_cells_; dx_cells <= translation_search_range_cells_; ++dx_cells) {
      for (int dy_cells = -translation_search_range_cells_; dy_cells <= translation_search_range_cells_; ++dy_cells) {
        const double dx_local = dx_cells * translation_search_step_m_;
        const double dy_local = dy_cells * translation_search_step_m_;
        const double candidate_x = predicted_pose.x + predicted_cos * dx_local - predicted_sin * dy_local;
        const double candidate_y = predicted_pose.y + predicted_sin * dx_local + predicted_cos * dy_local;
        const double candidate_yaw = std::atan2(
            std::sin(predicted_pose.yaw + delta_yaw),
            std::cos(predicted_pose.yaw + delta_yaw));
        const double transform_x = correction_x + predicted_cos * dx_local - predicted_sin * dy_local;
        const double transform_y = correction_y + predicted_sin * dx_local + predicted_cos * dy_local;

        std::size_t hit_count = 0;
        for (const pcl::PointXYZ& point : current_global_cloud->points) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
          const double x = cos_delta * point.x - sin_delta * point.y + transform_x;
          const double y = sin_delta * point.x + cos_delta * point.y + transform_y;
          const auto gx = static_cast<std::int64_t>(std::floor(x / map_resolution_));
          const auto gy = static_cast<std::int64_t>(std::floor(y / map_resolution_));
          hit_count += grid_map_points.count({gx, gy}) > 0;
        }
        const double score = static_cast<double>(hit_count) / valid_point_count;
        if (score > best_score) {
          best_score = score;
          best_csm_pose = {candidate_x, candidate_y, candidate_yaw};
        }
      }
    }
  }

  if (best_score <= 0.0) {
    csm_location_pose = predicted_pose;
    return false;
  }
  csm_location_pose = best_csm_pose;
  return true;
}

}
