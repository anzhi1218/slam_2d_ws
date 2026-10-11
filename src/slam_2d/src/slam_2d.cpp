/*
 * slam_2d - 在线 2D 激光 SLAM 主流程
 *
 * 数据来源：
 *   - 激光话题  /scan (sensor_msgs/LaserScan)   -> 转成 (x,y) 点云
 *   - 里程计话题 /odom (nav_msgs/Odometry)      -> 预积分成先验位姿(x,y,yaw)
 * 可直接配合 rosbag play 回放数据集（如 GP-SLAM-2D_dataset 下的 bag）。
 */
#include "slam_2d.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

namespace slam_2d {

Slam2d::Slam2d()
 : private_nh_("~") {
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
  private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
  private_nh_.param<int>("scan_queue_size", scan_queue_size_, 30);
  private_nh_.param<int>("odom_queue_size", odom_queue_size_, 100);
  private_nh_.param<double>("icp_fitness_score_threshold", icp_fitness_score_threshold_, 0.001);
  private_nh_.param<int>("line_fitting_min_points", line_fitting_min_points_, 5);
  private_nh_.param<int>("line_fitting_line_length_threshold", line_fitting_line_length_threshold_, 10);

  odom_process_ = std::make_unique<OdomProcess>(private_nh_);
  predicted_process_ = std::make_unique<PredictedProcess>(private_nh_);
  laser_process_ = std::make_unique<LaserProcess>(private_nh_);
  mapping_ = std::make_unique<Mapping>(node_, private_nh_);
  line_fitting_ = std::make_unique<LineFitting>(line_fitting_min_points_, line_fitting_line_length_threshold_);
  mpCurrentGlobalCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // 地图保存：默认保存到 <本包目录>/maps/map（参数 map_save_file 可改，含文件名、不带扩展名）
  default_map_save_file_ = ros::package::getPath("slam_2d") + "/maps/map";
  private_nh_.param<std::string>("map_save_file", map_save_file_, default_map_save_file_);
  if (map_save_file_.empty()) {
    map_save_file_ = default_map_save_file_;
  }
  private_nh_.param<bool>("save_map_on_shutdown", save_map_on_shutdown_, true);
  save_map_srv_ = private_nh_.advertiseService("save_map", &Slam2d::SaveMapCallback, this);
}

// 析构函数：按参数在退出时自动保存地图（Ctrl-C 正常退出时生效）
Slam2d::~Slam2d() {
  if (save_map_on_shutdown_ && mapping_) {
    if (SaveMapToFile()) {
      ROS_INFO("slam_2d: map saved on shutdown: %s.pgm / %s.yaml",
               map_save_file_.c_str(), map_save_file_.c_str());
    }
  }
}

// 订阅激光、里程计话题（rosbag play 回放 / 实际传感器），驱动 SLAM 主流程
void Slam2d::Process_2d_Slam() {
  odom_sub_ = node_.subscribe(odom_topic_, odom_queue_size_, &Slam2d::OdomCallback, this);
  scan_sub_ = node_.subscribe(scan_topic_, scan_queue_size_, &Slam2d::ScanCallback, this);

  ROS_INFO("slam_2d: subscribed scan[%s] and odom[%s], waiting for messages (rosbag play) ...",
           scan_topic_.c_str(), odom_topic_.c_str());
}

// 里程计回调：逐条送入预积分预处理（保持原始频率，不要抽稀）
void Slam2d::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  odom_process_->AddMessage(*odom);
}

// 激光回调：取与激光时间戳对齐的先验位姿，然后进入单帧处理
void Slam2d::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  // step1: 查询激光时刻的先验位姿（由里程计预积分链插值得到）
  Pose2D predict_pose;
  if (!odom_process_->GetPoseAt(scan->header.stamp, predict_pose)) {
    ROS_WARN_THROTTLE(1.0, "slam_2d: waiting for odom data, skip this scan");
    return;
  }

  // step2: 激光点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr mpCurrentCloud = LaserScanToPointCloud(*scan);
  if (mpCurrentCloud->empty()) {
    ROS_WARN_THROTTLE(1.0, "slam_2d: empty cloud after filtering, skip this scan");
    return;
  }

  // step3: 单帧 SLAM 主流程
  ProcessFrame(scan->header.stamp, predict_pose, mpCurrentCloud);
}

// LaserScan -> PCL 点云（雷达坐标系，z=0）

pcl::PointCloud<pcl::PointXYZ>::Ptr Slam2d::LaserScanToPointCloud(const sensor_msgs::LaserScan& scan) const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr mpCloud(new pcl::PointCloud<pcl::PointXYZ>);
  mpCloud->header.frame_id = scan.header.frame_id;
  mpCloud->points.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) continue;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(range * std::cos(angle));
    pt.y = static_cast<float>(range * std::sin(angle));
    pt.z = 0.0f;
    mpCloud->points.push_back(pt);
  }
  return mpCloud;
}

// 把 odom 系下的先验位姿变换到 map 系：
// map 系原点 = 首帧位置，x 轴 = 首帧朝向，这样先验轨迹能与估计轨迹直接对比
Pose2D Slam2d::PriorToMapFrame(const Pose2D& prior_pose) const {
  if (!has_first_predict_) return prior_pose;
  return PredictedProcess::PoseSubtract(prior_pose, first_predict_pose_);
}

// 退化检测：把雷达系单帧点云送入直线拟合模块，可用直线不足或方向过于一致则视为退化
bool Slam2d::IsDegradedFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector2f> scan_points;
  scan_points.reserve(cloud.size());
  for (const pcl::PointXYZ& pt : cloud.points) {
    scan_points.emplace_back(pt.x, pt.y);
  }
  line_fitting_->runLineExtraction(scan_points);
  return line_fitting_->hasDegradation();
}

// 把当前地图保存为 PGM + YAML（导航可直接使用）
bool Slam2d::SaveMapToFile() {
  if (map_save_file_.empty()) {
    ROS_ERROR("slam_2d: map_save_file is empty, cannot save map");
    return false;
  }
  if (!mapping_->SaveMap(map_save_file_)) {
    return false;
  }
  return true;
}

// 保存地图服务：rosservice call /slam_2d/save_map
bool Slam2d::SaveMapCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
  // 每次调用时重读参数，便于 rosparam set /slam_2d/map_save_file 后立即生效；空值回退默认路径
  private_nh_.param<std::string>("map_save_file", map_save_file_, map_save_file_);
  if (map_save_file_.empty()) {
    map_save_file_ = default_map_save_file_;
  }
  res.success = SaveMapToFile();
  res.message = res.success
      ? ("map saved: " + map_save_file_ + ".pgm / " + map_save_file_ + ".yaml")
      : "save map failed (empty map or write error, see node log)";
  return true;
}

// 单帧 SLAM 主流程
void Slam2d::ProcessFrame(const ros::Time& timestamp, const Pose2D& predict_pose,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& mpCurrentCloud) {
  Pose2D location_pose;
  Pose2D predicted_pose;
  Pose2D icp_location_pose;

  // step 1: 初始化。以第一帧位置为地图原点，第一帧不建图
  if (first_frame_) {
    first_frame_ = false;
    last_predict_pose_ = predict_pose;
    last_location_pose_ = Pose2D{};
    first_predict_pose_ = predict_pose;
    has_first_predict_ = true;
    ROS_INFO("slam_2d: init frame %d as map origin (cloud: %zu points, predict_pose: x=%.2f y=%.2f yaw=%.1f deg)",
             frame_id_, mpCurrentCloud->size(), predict_pose.x, predict_pose.y, predict_pose.yaw * 180.0 / M_PI);
    ++frame_id_;
    return;
  }

  // step 2: 局部坐标系下计算 predicted_pose
  const Pose2D local_delta =
      PredictedProcess::PoseSubtract(predict_pose, last_predict_pose_);
  predicted_pose = PredictedProcess::PoseAdd(last_location_pose_, local_delta);

  // step 3: 按 predicted_pose 将当前帧点云变换到全局坐标系，并取上一帧为止的地图
  const Eigen::Matrix4f predicted_transform =
      PredictedProcess::PoseToMatrix4f(predicted_pose);
  pcl::transformPointCloud(*mpCurrentCloud, *mpCurrentGlobalCloud, predicted_transform);
  const auto grid_map = mapping_->getWholeMapCloud();

  // step 3.5: 退化检测（基于雷达系单帧点云）：退化帧直接用里程计预测位姿，不配准、不更新地图
  const bool is_degraded = IsDegradedFrame(*mpCurrentCloud);

  double icp_fitness_score = 0.0;

  if (is_degraded) {
    location_pose = predicted_pose;      // 采用里程计（预积分）更新位姿
    icp_location_pose = predicted_pose;  // 保持日志/流程一致
    ROS_WARN_THROTTLE(1.0,
                      "slam_2d: frame %d degraded (fitted lines: %d, valid lines: %d), "
                      "use odometry prediction, skip map update",
                      frame_id_, line_fitting_->getLinesSize(), line_fitting_->getValidLineCount());
  }
  else {
    // step 4: ICP 配准
    laser_process_->IcpAlignMapFrame(mpCurrentGlobalCloud, grid_map, predicted_pose,
                                     icp_location_pose, icp_fitness_score);
  }

  // step 5: ICP 拟合较差时，用 CSM 粗匹配修正位姿；CSM 无结果时保留 ICP 结果；退化帧跳过
  if (!is_degraded && icp_fitness_score > icp_fitness_score_threshold_) {
    Pose2D csm_location_pose_ = predicted_pose;
    const bool csm_ok = laser_process_->CSMCoarseMatch(mpCurrentGlobalCloud, grid_map,
                                                       predicted_pose, csm_location_pose_);
    location_pose = csm_ok ? csm_location_pose_ : icp_location_pose;

    if (csm_ok) {
      ROS_INFO(" CSM coarse match: location_pose: x=%.4f, y=%.4f, yaw=%.2f degree",
               location_pose.x, location_pose.y, location_pose.yaw * 180.0 / M_PI);
    } else {
      ROS_WARN_THROTTLE(1.0, " CSM got no result, keep ICP result");
    }
  } else {
    location_pose = icp_location_pose;
  }

  // step 6: 按 location_pose 将当前帧点云变换到全局坐标系
  pcl::PointCloud<pcl::PointXYZ>::Ptr mpIcpCloud(new pcl::PointCloud<pcl::PointXYZ>);
  const Eigen::Matrix4f location_transform =
      PredictedProcess::PoseToMatrix4f(location_pose);
  pcl::transformPointCloud(*mpCurrentCloud, *mpIcpCloud, location_transform);

  // step 7: 更新地图
  if (!is_degraded && !mpIcpCloud->empty())
    mapping_->UpdateGridMap(mpIcpCloud, location_pose);

  // step 8: 发布路径（先验轨迹先对齐到 map 系，便于与预测/估计轨迹对比）
  const Pose2D predict_pose_in_map = PriorToMapFrame(predict_pose);
  predicted_process_->publishPaths(map_frame_, timestamp,
                                  predict_pose_in_map, predicted_pose, location_pose);

  // step 9: 发布地图可视化（栅格地图、当前帧点云、ICP点云等）
  mapping_->publishMapVisualization(mpCurrentGlobalCloud, mpIcpCloud, timestamp);

  // step 10: 发布 TF
  predicted_process_->publishTf(map_frame_, odom_frame_, base_frame_, timestamp,
                                location_pose);

  ROS_INFO(" || Frame %d:|| predict_pose: x=%.4f y=%.4f yaw=%.2f deg || ICP_location_pose: x=%.4f y=%.4f yaw=%.2f deg fit=%.4f || predicted_pose: x=%.4f y=%.4f yaw=%.2f deg || location_pose: x=%.4f y=%.4f yaw=%.2f deg || cloud: %zu grid_map: %zu current_global_cloud: %zu || degraded: %d fitted_lines: %d valid_lines: %d",
    frame_id_, predict_pose.x, predict_pose.y, predict_pose.yaw * 180.0 / M_PI,
    icp_location_pose.x, icp_location_pose.y, icp_location_pose.yaw * 180.0 / M_PI, icp_fitness_score,
    predicted_pose.x, predicted_pose.y, predicted_pose.yaw * 180.0 / M_PI,
    location_pose.x, location_pose.y, location_pose.yaw * 180.0 / M_PI,
    mpCurrentCloud->size(), grid_map->size(), mpCurrentGlobalCloud->size(),
    is_degraded ? 1 : 0, line_fitting_->getLinesSize(), line_fitting_->getValidLineCount());

  // step 11: 更新上一帧状态
  last_predict_pose_ = predict_pose;
  last_location_pose_ = location_pose;
  ++frame_id_;
}

} // slam_2d.cpp
