# slam_2d_ws

基于激光雷达与轮式里程计先验位姿的 **2D 激光 SLAM** 工作空间（ROS 1 / catkin）。

在线订阅 `/scan` 与 `/odom`，用 **ICP + CSM（相关性扫描匹配）** 做帧间配准，
增量式构建 2D 占据栅格地图，并输出轨迹与 TF。可直接由 `rosbag play`
回放数据集（`GP-SLAM-2D_dataset`）驱动，也支持接入真实传感器。

## 功能特性

- **在线建图**：激光 `/scan` + 里程计 `/odom` 增量式构图，无需离线数据文件。
- **里程计预积分**：对 `/odom` 做增量提取、跳变剔除、低通平滑与激光时间对齐，得到与激光
  严格同步的先验位姿。
- **ICP 精配准**：当前帧点云与已建地图做 ICP，得到估计位姿。
- **CSM 粗匹配回退**：ICP 拟合分数较差时用分支搜索做平移/角度粗匹配修正位姿。
- **占据栅格建图**：每帧按命中/空闲证据更新栅格，短时空闲观测不会立即抹掉稳定墙面。
- **地图保存**：内置 PGM + YAML 导出，兼容 ROS1 `map_server` 与 Nav2 `nav2_map_server`。
- **RViz 可视化**：栅格地图、当前帧点云、ICP 点云、先验/预测/估计轨迹。

## 目录结构

```
slam_2d_ws/
├── src/
│   └── slam_2d/                 # slam_2d 功能包
│       ├── include/             # 头文件（frame.h / odom_process.h / ...）
│       ├── src/                 # 源码（main.cpp / slam_2d.cpp / mapping.cpp / ...）
│       ├── launch/              # slam_2d.launch
│       ├── config/              # params.yaml、slam_2d.rviz
│       ├── maps/                # 保存的栅格地图（map.pgm / map.yaml）
│       ├── data/                # 数据集（子模块 GP-SLAM-2D_dataset）
│       ├── CMakeLists.txt
│       └── package.xml
├── build/  devel/  install/     # catkin 编译产物（已忽略）
└── README.md
```

功能包的详细设计、话题与参数说明见 [src/slam_2d/README.md](src/slam_2d/README.md)。

## 依赖

- Ubuntu 20.04 + **ROS Noetic**（`roscpp`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`、
  `std_srvs`、`tf`、`pcl_conversions`、`roslib`）
- **PCL**（`common`、`registration`）
- **Eigen3**
- 运行可视化需要 `rviz`；回放数据需要 `rosbag`

## 编译

```bash
cd ~/slam_2d_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行

### 方式一：launch 启动节点 + 另开终端播放 bag（推荐）

```bash
# 终端1：启动 SLAM 节点与 RViz
roslaunch slam_2d slam_2d.launch

# 终端2：播放数据包（-r 0.2 为 0.2 倍速，建议慢速回放以跟上处理速度）
rosbag play -r 0.2 ~/slam_2d_ws/src/slam_2d/data/GP-SLAM-2D_dataset/Yongqian_data_2019-05-29-13-12-39.bag
```

不启动 RViz：`roslaunch slam_2d slam_2d.launch use_rviz:=false`

### 方式二：launch 直接把 bag 一起启动

```bash
roslaunch slam_2d slam_2d.launch use_bag:=true
roslaunch slam_2d slam_2d.launch use_bag:=true rate:=0.2 bag_file:=/path/to/your.bag
```

### 保存地图

- 运行中随时保存：`rosservice call /slam_2d/save_map`
- 退出（Ctrl-C）自动保存：由参数 `save_map_on_shutdown` 控制（默认 `true`）

默认保存到 `src/slam_2d/maps/map.pgm` 与 `map.yaml`。

## 主要话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| `/slam_2d/grid_map_cloud` | `sensor_msgs/PointCloud2` | 地图点云 |
| `/slam_2d/current_cloud` | `sensor_msgs/PointCloud2` | 当前帧先验位姿下的点云 |
| `/slam_2d/icp_cloud` | `sensor_msgs/PointCloud2` | 匹配后位姿下的点云 |
| `/slam_2d/predict_path` | `nav_msgs/Path` | 先验位姿（里程计）轨迹 |
| `/slam_2d/predicted_path` | `nav_msgs/Path` | 预测位姿轨迹 |
| `/slam_2d/location_path` | `nav_msgs/Path` | ICP/CSM 估计位姿轨迹 |
| `/tf` | `tf2_msgs/TFMessage` | `map -> odom -> base_link` |

服务：`/slam_2d/save_map`（`std_srvs/Trigger`）。

## 处理流程（每帧）

1. 预积分先验位姿：取与激光时间对齐的 `predict_pose`；
2. 预测位姿：里程计相邻帧增量叠加到上一帧最终位姿，得到 `predicted_pose`；
3. 当前帧点云按 `predicted_pose` 变换到全局坐标系；
4. 与已建地图做 ICP，拟合分数较差时用 CSM 粗匹配修正位姿；
5. 用最终位姿把点云加入栅格地图，发布地图/点云/路径/TF；
6. 首帧仅初始化：以第一帧位置为地图原点，不建图。

## 数据集

`src/slam_2d/data/GP-SLAM-2D_dataset/`（来自 RuanJY/GP-SLAM-2D_dataset）：

| 文件 | 说明 |
| --- | --- |
| `Yongqian_data_2019-05-29-13-12-39.bag` | 永谦学生活动中心，214s，`/scan` 6414 帧、`/odom` 8466 条 |
| `ChouYeiChing_data_2019-05-29-13-17-41.bag` | 周亦卿科技大楼，约 303s，`/scan` 9098 帧、`/odom` 12563 条 |

子模块初始化：

```bash
git submodule update --init --recursive
```

## 许可

见 `src/slam_2d/package.xml`（BSD / Apache 2.0）。
