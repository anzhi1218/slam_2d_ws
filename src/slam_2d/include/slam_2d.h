#ifndef SLAM_2D_H
#define SLAM_2D_H

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "frame.h"
#include "odom_process.h"
#include "predicted_process.h"
#include "laser_process.h"
#include "mapping.h"
#include "linear_fitting.h"

namespace slam_2d {

class Slam2d {
 public:
  Slam2d();
  ~Slam2d();

  /**
   * @brief 在线 SLAM 主流程：订阅 scan_topic/odom_topic（默认 /scan、/odom）
   *        可直接由 rosbag play 回放数据集驱动，也可接实际传感器
   */
  void Process_2d_Slam();

 private:
  /// 激光回调：取与激光时间对齐的先验位姿后进入单帧处理
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  /// 里程计回调：消息送入预积分预处理
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

  /**
   * @brief 单帧 SLAM 处理流程
   * @param timestamp 该帧时间戳（取激光消息时间）
   * @param predict_pose 先验位姿（里程计预积分结果，已与激光时间对齐）
   * @param current_cloud 当前帧激光点云（z=0）
   */
  void ProcessFrame(const ros::Time& timestamp, const Pose2D& predict_pose,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_cloud);

  /// LaserScan 转 PCL 点云（雷达坐标系，z=0），过滤无效值与无回波束
  pcl::PointCloud<pcl::PointXYZ>::Ptr LaserScanToPointCloud(const sensor_msgs::LaserScan& scan) const;

  /// 把 odom 系下的先验位姿变换到 map 系（以首帧位置为原点、首帧朝向为 x 轴）
  Pose2D PriorToMapFrame(const Pose2D& prior_pose) const;

  /// 退化检测：对雷达系单帧点云做直线拟合（可用直线不足或方向过于一致视为退化）
  bool IsDegradedFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  /// 保存地图服务回调：rosservice call /slam_2d/save_map
  bool SaveMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /// 把当前地图保存为 <map_save_file_>.pgm / .yaml
  bool SaveMapToFile();

  //// ----- ROS节点句柄 -----
  ros::NodeHandle node_;       // 公共节点句柄
  ros::NodeHandle private_nh_; // 私有节点句柄

  // ----- 订阅 -----
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;

  // ----- 坐标系名称 -----
  std::string map_frame_;  // 地图坐标系
  std::string odom_frame_; // 里程计坐标系
  std::string base_frame_; // 机器人基坐标系

  // ----- 输入话题与订阅队列 -----
  std::string scan_topic_;
  std::string odom_topic_;
  int scan_queue_size_;
  int odom_queue_size_;
  double icp_fitness_score_threshold_;  ///< ICP 拟合分数大于该值时用 CSM 粗匹配
  int line_fitting_min_points_ = 5;              ///< 退化检测：聚类/拟合最少点数
  int line_fitting_line_length_threshold_ = 10;  ///< 退化检测：计为有效直线的最少点数

  /// 里程计（先验位姿）预处理：预积分 + 时间对齐
  std::unique_ptr<OdomProcess> odom_process_;

  std::unique_ptr<PredictedProcess> predicted_process_;
  std::unique_ptr<LaserProcess> laser_process_;
  std::unique_ptr<Mapping> mapping_;
  std::unique_ptr<LineFitting> line_fitting_;

  /// 地图保存：服务 + 退出自动保存（PGM + YAML）
  ros::ServiceServer save_map_srv_;
  std::string map_save_file_;          ///< 保存路径前缀（不含扩展名）
  std::string default_map_save_file_;  ///< 默认保存路径前缀（参数为空时使用）
  bool save_map_on_shutdown_ = true;   ///< Ctrl-C 退出时是否自动保存地图

  Pose2D last_location_pose_;   ///< 上一帧最终位姿
  Pose2D last_predict_pose_;    ///< 上一帧 predict_pose

  pcl::PointCloud<pcl::PointXYZ>::Ptr mpCurrentGlobalCloud;

  bool first_frame_ = true;    ///< 首帧仅初始化（以首帧位置为地图原点），不建图
  int frame_id_ = 0;           ///< 已处理帧序号（日志/可视化用）
  Pose2D first_predict_pose_;  ///< 首帧先验位姿，用于把先验轨迹对齐到 map 系
  bool has_first_predict_ = false;
};

}

#endif //slam_2d.h
