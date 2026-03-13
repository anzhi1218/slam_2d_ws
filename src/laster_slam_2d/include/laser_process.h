/*
 * laster_slam_2d - 激光点云处理
 */
 #ifndef LASTER_SLAM_2D_LASER_PROCESS_H
 #define LASTER_SLAM_2D_LASER_PROCESS_H
 
 #include <ros/ros.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <vector>
 
 #include "frame.h"
 
 namespace laster_slam_2d {
 
 class PredictedProcess;
 
 class LaserProcess {
 public:
   explicit LaserProcess(ros::NodeHandle& pnh);

   
   void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
   
   /// ICP 配准
   bool IcpAlignMapFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                        const Pose2D& prior, Pose2D& icp_location_pose,
                        PredictedProcess& predicted_process,
                        double* fitness_score = nullptr);
 
  /// CSM 粗匹配
  bool CSMCoarseMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_global_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_map,
                      const Pose2D& predicted_pose, Pose2D& csm_location_pose,
                      PredictedProcess& predicted_process);
 
 private:
 
   // 体素滤波
   float voxel_leaf_size_;

   // ICP
   int icp_max_iterations_;
   double icp_max_correspondence_dist_;
   double icp_transform_epsilon_;
   double icp_euclidean_fitness_epsilon_;
 
   // CSM
   double map_resolution_; //地图分辨率
   int translation_search_range_cells_; //平移搜索范围栅格数
   double translation_search_step_cells_; //平移搜索步长栅格数
   double yaw_search_range_deg_; //偏航搜索范围
   double angular_step_deg_; //偏航搜索步长
 };
 
 }  // namespace laster_slam_2d
 
 #endif
 //laser_process.h