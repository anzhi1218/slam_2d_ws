/*
 * laster_slam_2d - 点云处理实现
 */
 #include "laser_process.h"
 #include "predicted_process.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>
#include <limits>
#include <set>

namespace laster_slam_2d {

LaserProcess::LaserProcess(ros::NodeHandle& pnh) {
  
  // 体素滤波的叶子大小（降采样分辨率）
  pnh.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.05f);

  // ICP 配准参数设置
  pnh.param<int>("icp_max_iterations", icp_max_iterations_, 20);
  pnh.param<double>("icp_max_correspondence_dist", icp_max_correspondence_dist_, 0.3); //最大对应距离
  pnh.param<double>("icp_transform_epsilon", icp_transform_epsilon_, 1e-8); 
  pnh.param<double>("icp_euclidean_fitness_epsilon", icp_euclidean_fitness_epsilon_, 1e-6); 

  // CSM 参数
  pnh.param<double>("map_resolution",  map_resolution_, 0.02);
  pnh.param<int>("translation_search_range_cells", translation_search_range_cells_, 2);
  pnh.param<double>("translation_search_step_cells", translation_search_step_cells_, 0.02);
  pnh.param<double>("angular_step_deg", angular_step_deg_, 0.5);
  pnh.param<double>("yaw_search_range_deg", yaw_search_range_deg_, 2);
}
 

 /**
  * @brief 体素滤波降采样
  * @param cloud 点云数据
  */
void LaserProcess::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  
  if (!cloud || cloud->empty() || voxel_leaf_size_ <= 0.0f) return;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_cloud;
  voxel_cloud.setInputCloud(cloud);
  voxel_cloud.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_cloud.filter(*filtered);
  ROS_INFO("voxelFilter: cloud size: %zu, filtered size: %zu", cloud->size(), filtered->size());
  cloud->swap(*filtered);
}

/**
 * @brief ICP配准流程
 * @param current_global_cloud 当前全局点云
 * @param grid_matranslation_search_cellsp 地图点云
 * @param predicted_pose 预测位姿
 * @param icp_location_pose ICP位置
 * @param predicted_process 预测过程
 * @param icp_fitness_score ICP拟合得分
 */
bool LaserProcess::IcpAlignMapFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                                    const Pose2D& predicted_pose,
                                    Pose2D& icp_location_pose,
                                    PredictedProcess& predicted_process,
                                    double* icp_fitness_score) {


  //step1 检查输入
  if (!current_global_cloud || !grid_map ||
      current_global_cloud->empty() || grid_map->empty()) {
    icp_location_pose = predicted_pose;
    return false;
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
  Eigen::Matrix3f Predicted_Pose_ = predicted_process.PoseToMatrix(predicted_pose);
  Eigen::Matrix4f Predicted_pose_  = Eigen::Matrix4f::Identity();
  Predicted_pose_.block<2, 2>(0, 0) = Predicted_Pose_.block<2, 2>(0, 0);
  Predicted_pose_.block<2, 1>(0, 3) = Predicted_Pose_.block<2, 1>(0, 2);
  Eigen::Matrix4f Icp_Predicted_pose = Icp_correction_pose * Predicted_pose_;

  double fitness = icp_align.getFitnessScore();
  if (icp_fitness_score) {
    *icp_fitness_score = fitness;
  }

  predicted_process.MatrixToPose(Icp_Predicted_pose, icp_location_pose);
  return true;
}



/**
 * @brief CSM 粗匹配
 * @param current_global_cloud 当前全局点云
 * @param grid_map 地图点云
 * @param predicted_pose 预测位姿
 * @param csm_location_pose CSM估计的位姿
 * @param predicted_process 预测过程
 */
bool LaserProcess::CSMCoarseMatch(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
    const Pose2D& predicted_pose, Pose2D& csm_location_pose,
    PredictedProcess& predicted_process) {
  
  //step1 检查输入
  if (!current_global_cloud || !grid_map || current_global_cloud->empty() || grid_map->empty()) {
    csm_location_pose = predicted_pose;
    return false;
  }

  //step2 计算预测位姿的矩阵
  Eigen::Matrix3d predicted_pose_ = predicted_process.PoseToMatrix(predicted_pose).cast<double>();
  Eigen::Matrix3d predicted_pose_inv = predicted_pose_.inverse();

  //step3 计算地图点云的栅格
  std::set<std::pair<std::int64_t, std::int64_t>> grid_map_points;

  //step4 计算地图点云的栅格
  for (int i = 0; i < static_cast<int>(grid_map->points.size()); ++i) {
    //step4.1 检查点云是否有效
    const pcl::PointXYZ& pt = grid_map->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
    //step4.2 计算地图点云的栅格
    int gx = static_cast<int>(std::floor(pt.x / map_resolution_));
    int gy = static_cast<int>(std::floor(pt.y / map_resolution_));
    //step4.3 计算地图点云的栅格索引
    std::pair<std::int64_t, std::int64_t> grid_map_grid_index{gx, gy};
    grid_map_points.insert(grid_map_grid_index);
  }
  //step4.4 检查地图点云的栅格是否为空
  if (grid_map_points.empty()) {
    csm_location_pose = predicted_pose;
    return false;
  }

  //step5 统计当前帧点云中的有效点数量
  int valid_current_cloud_points = current_global_cloud->size();

  //step5.4 检查源点云的有效点是否为0
  if (valid_current_cloud_points == 0) {
    csm_location_pose = predicted_pose;
    return false;
  }

  //step6 计算最佳得分
  double best_score = 0;
  Pose2D best_csm_pose = predicted_pose;

  //step7 三层for循环，遍历所有可能的位姿
  //step7.1 遍历所有可能的偏航角(yaw)
  for (double yaw_deg = -yaw_search_range_deg_;yaw_deg <= yaw_search_range_deg_+ 1e-6;yaw_deg += angular_step_deg_) {
    //step7.1.1 计算偏航角(yaw)
    double dtheta = yaw_deg * M_PI / 180.0;
    //step7.1.2 在当前 yaw 下预计算旋转矩阵，避免在平移循环中重复计算
    double cos_theta = std::cos(dtheta);
    double sin_theta = std::sin(dtheta);
    Eigen::Matrix3d rotation_delta;
    rotation_delta << cos_theta, -sin_theta, 0.0,
                      sin_theta,  cos_theta, 0.0,
                      0.0,        0.0,       1.0;

    //step7.2 遍历所有可能的平移
    for (int dx_cells = -translation_search_range_cells_; dx_cells <= translation_search_range_cells_; ++dx_cells) {
      //step7.3 遍历所有可能的平移
      for (int dy_cells = -translation_search_range_cells_; dy_cells <= translation_search_range_cells_; ++dy_cells) {

        //step7.4 计算位姿的增量
        double dx_l = static_cast<double>(dx_cells) * translation_search_step_cells_;
        double dy_l = static_cast<double>(dy_cells) * translation_search_step_cells_;
        Eigen::Matrix3d predict_pose_delta_l = rotation_delta;
        predict_pose_delta_l(0, 2) = dx_l;
        predict_pose_delta_l(1, 2) = dy_l;
        Eigen::Matrix3d Candidate_Pose_ = predicted_pose_ * predict_pose_delta_l;
        Pose2D Candidate_pose;
        Candidate_pose.x = Candidate_Pose_(0, 2);
        Candidate_pose.y = Candidate_Pose_(1, 2);
        Candidate_pose.yaw = std::atan2(Candidate_Pose_(1, 0), Candidate_Pose_(0, 0));
        Eigen::Matrix3d Candidate_pose_delta = Candidate_Pose_ * predicted_pose_inv;
        
        //step7.5 计算得分
        double Hit_score = 0.0;
        //step7.5.1 遍历所有可能的当前帧点云
        for (int i = 0; i < static_cast<int>(current_global_cloud->points.size()); ++i) {
          //step7.5.1.1 检查点云是否有效
          double px = static_cast<double>(current_global_cloud->points[i].x);
          double py = static_cast<double>(current_global_cloud->points[i].y);
          //step7.5.1.2 检查点云是否有效
          if (!std::isfinite(px) || !std::isfinite(py)) continue;
          //step7.5.1.3 计算当前帧点云在候选位姿下的栅格
          Eigen::Vector3d Point_in_current_cloud(px, py, 1.0);
          //step7.5.1.4 计算当前帧点云在候选位姿下的栅格
          Eigen::Vector3d Current_cloud_in_grid_map = Candidate_pose_delta* Point_in_current_cloud;
          int gx = static_cast<int>(std::floor(Current_cloud_in_grid_map(0) / map_resolution_));
          int gy = static_cast<int>(std::floor(Current_cloud_in_grid_map(1) / map_resolution_));
          //step7.5.1.5 计算候选位姿的栅格索引
          std::pair<std::int64_t, std::int64_t> Candidate_Pose_grid_index{gx, gy};
          if (grid_map_points.count(Candidate_Pose_grid_index) > 0) Hit_score += 1.0;
        }
        //step7.5.2 计算得分
        double Score = Hit_score / static_cast<double>(valid_current_cloud_points);
        if (Score > best_score) {
          best_score = Score;
          best_csm_pose = Candidate_pose;
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