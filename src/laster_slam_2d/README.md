# laster_slam_2d

基于激光雷达与 IMU 先验位姿的 2D SLAM，适配自定义 data.txt 数据格式，使用 laster_slam_2d_openslam 核心库建图。

## 数据格式 (data.txt)

- **前两行**：无用，解析时跳过。
- **后续块**：重复的 `Pose` + `Cloud`：
  - **Pose**：时间戳与 IMU 处理得到的先验位姿  
    `time: 263119, yaw: 125.3, X: 2.749, Y: -0.501`  
    - `time`: 时间戳  
    - `yaw`: 航向角（度）  
    - `X`, `Y`: 平面位置 (m)
  - **Cloud**：该帧激光点云，空格分隔的 `x y` 对（雷达坐标系，单位 m）。

## 依赖

本工作空间已包含 **laster_slam_2d_openslam** 核心库（`src/laster_slam_2d_openslam`），无需单独安装。  
若已删除，可：`cd src && git clone https://github.com/ros-perception/openslam_gmapping.git` 克隆后，将包名与目录改为 `laster_slam_2d_openslam`，并修改其 `package.xml` 与 `CMakeLists.txt` 中的 `project` 名。

## 编译

```bash
cd <工作空间根目录>   # 即 laster_slam_2d_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行

### 使用 data.txt 建图（推荐）

```bash
# 推荐：使用 launch，数据文件默认用相对路径 src/data/data.txt，可覆盖
roslaunch laster_slam_2d laser_slam_2d.launch
roslaunch laster_slam_2d laser_slam_2d.launch data_file:=src/data/data.txt
```

或直接运行节点（需传私有参数 data_file，可用相对路径）：

```bash
rosrun laster_slam_2d laster_slam_2d _data_file:=src/data/data.txt
```

若不需要 RViz：`roslaunch laster_slam_2d laser_slam_2d.launch rviz:=false`

建图完成后可保存地图：

```bash
rosrun map_server map_saver -f mymap map:=dynamic_map
```

### 使用 rosbag 回放

```bash
rosrun laster_slam_2d laster_slam_2d_replay --bag_filename path/to/your.bag --scan_topic /scan
```

### 实时 /scan + TF 建图

```bash
rosrun laster_slam_2d laster_slam_2d
```

## 说明

- 使用 data.txt 时，程序会解析 Pose 与 Cloud，用 **IMU 先验位姿** 作为 odom，配合 **laster_slam_2d_openslam** 核心库进行建图与位姿估计。
- **轨迹可视化**：data.txt 回放时发布两条 Path 轨迹：
  - `~prior_path`（红色）：读入的 x, y, yaw 先验轨迹（IMU，对应 1.txt wheelPoseGyro）
  - `~estimated_path`（绿色）：激光匹配后 SLAM 估计位姿（对应 1.txt locationPose）
- **方向校正**：若估计位姿与 IMU 方向相反，在 launch 中设 `estimated_yaw_offset:=3.14159`
- **INFO 日志**：每帧输出 `IMU_prior x/y/yaw_deg`、`laser_est x/y/yaw_deg`、`prior_in_map x/y`
- 节点名、包名、类名、launch、测试等均已使用 **laster_slam_2d** 命名。
