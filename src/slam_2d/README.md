# slam_2d

基于激光雷达与轮式里程计先验位姿的 2D 激光 SLAM：在线订阅 `/scan` 与 `/odom`，
可直接由 `rosbag play` 回放数据集（GP-SLAM-2D_dataset）驱动，输出 2D 占据栅格地图与轨迹。

## 数据来源与适配方式

| 话题 | 类型 | 处理方式 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 极坐标转笛卡尔点云：`x = r·cos(θ)`、`y = r·sin(θ)`、`z = 0`；过滤非有限值、小于 `range_min` 的盲区点、以及顶到 `range_max` 的**无回波**束（本数据集无回波写成 60.0，否则会在地图外围生成一圈假点） |
| `/odom` | `nav_msgs/Odometry` | 转成先验位姿 `x, y, yaw`（与老 data.txt 的 `time / yaw / X / Y` 语义一致），再做**预积分预处理**后与激光时间对齐（见下） |

### 里程计预积分处理（`OdomProcess`）

1. **转换**：`/odom` 消息 → `(stamp, x, y, yaw)`，姿态四元数只取 yaw（平面运动）；
2. **增量提取**：相邻消息求相对运动，转到上一帧局部坐标系；
3. **异常剔除**：增量对应的速度超过阈值（`odom_max_linear_speed` / `odom_max_angular_speed`）时
   视为跳变，改用上一次速度按 `dt` 外推，避免预测位姿瞬间飞掉；
4. **低通平滑（可选）**：对增量速度做一阶滤波（`odom_smooth_alpha`，1=不平滑）；
5. **积分**：把清洗后的增量积成一条“干净”的位姿链；
6. **时间对齐**：按激光时间戳在位姿链上线性插值（yaw 走最短角路径），
   得到与激光严格同步的先验位姿（原方案取“最近一条 odom”，最多有约 26ms 偏差）。

> 说明：本数据集的 `/odom` 直接提供绝对位姿（twist 全 0，无法按速度积分），
> 因此这里的“预积分”是指按时间对齐的增量提取、清洗与积分。

### 先验轨迹坐标

`predict_path`（先验轨迹）发布前会统一变换到 `map` 系（以首帧位置为原点、首帧朝向为 x 轴），
因此 RViz 中先验轨迹、预测轨迹、估计轨迹可直接叠加对比。

## 数据集

位于 `data/GP-SLAM-2D_dataset/`：

| 文件 | 说明 |
| --- | --- |
| `Yongqian_data_2019-05-29-13-12-39.bag` | 永谦学生活动中心，214s，`/scan` 6414 帧、`/odom` 8466 条 |
| `ChouYeiChing_data_2019-05-29-13-17-41.bag` | 周亦卿科技大楼，约 303s，`/scan` 9098 帧、`/odom` 12563 条 |

## 编译

```bash
cd <工作空间根目录>   # 即 slam_2d_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行

### 方式一：launch 启动节点 + 另开终端播放 bag（推荐）

```bash
# 终端1：启动 SLAM 节点与 RViz
roslaunch slam_2d slam_2d.launch

# 终端2：播放数据包（-r 0.2 表示 0.2 倍速，单帧处理约 270ms，建议慢速回放）
rosbag play -r 0.2 ~/slam_2d_ws/src/slam_2d/data/GP-SLAM-2D_dataset/Yongqian_data_2019-05-29-13-12-39.bag
```

不想启动 RViz：`roslaunch slam_2d slam_2d.launch use_rviz:=false`

### 方式二：launch 直接把 bag 一起启动

```bash
roslaunch slam_2d slam_2d.launch use_bag:=true
roslaunch slam_2d slam_2d.launch use_bag:=true rate:=0.2 bag_file:=/path/to/your.bag
```

### 建图结束后保存地图

节点内置保存功能（输出标准 PGM + YAML，兼容 ROS1 `map_server` 与 Nav2 `nav2_map_server`）：

- 运行中随时保存：`rosservice call /slam_2d/save_map`
- 退出（Ctrl-C）时自动保存：由参数 `save_map_on_shutdown` 控制（默认 `true`）

默认保存到 `<包目录>/maps/map.pgm` 与 `map.yaml`，保存时自动裁剪到已知区域（外扩 1m）。
地图更新按每帧激光扫描汇总：同一栅格每帧最多获得一次命中或空闲证据；若某束激光在该格结束，命中优先于其他光束穿过该格的空闲证据。障碍证据有上限，后续持续穿过的空闲光束仍能清除旧障碍；短时空闲观测不会立即抹掉稳定墙面。
也可用参数 `map_save_file` 指定其它路径（不含扩展名），`rosparam set` 后下一次服务调用即生效：

```bash
rosparam set /slam_2d/map_save_file /home/slam/maps/lab1
rosservice call /slam_2d/save_map
```

> 注：本机未安装 `map_server`，旧版文档中的 `map_saver` 命令不可用；上面的内置保存不依赖它。

## 主要话题

节点发布（除地图外均在私有命名空间 `slam_2d` 下）：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| `/slam_2d/grid_map_cloud` | `sensor_msgs/PointCloud2` | 地图点云 |
| `/slam_2d/current_cloud` | `sensor_msgs/PointCloud2` | 当前帧先验位姿下的点云 |
| `/slam_2d/icp_cloud` | `sensor_msgs/PointCloud2` | 匹配后位姿下的点云 |
| `/slam_2d/predict_path` | `nav_msgs/Path` | 先验位姿（里程计）轨迹，已对齐到 map 系 |
| `/slam_2d/predicted_path` | `nav_msgs/Path` | 预测位姿轨迹 |
| `/slam_2d/location_path` | `nav_msgs/Path` | ICP/CSM 估计位姿轨迹 |
| `/tf` | `tf2_msgs/TFMessage` | `map -> odom -> base_link` |

服务：`/slam_2d/save_map`（`std_srvs/Trigger`）—— 保存地图为 PGM + YAML（详见上文「建图结束后保存地图」）。

## 参数（`config/params.yaml`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `scan_topic` / `odom_topic` | `/scan` / `/odom` | 输入话题 |
| `scan_queue_size` / `odom_queue_size` | 30 / 100 | 订阅队列长度 |
| `odom_buffer_size` | 200 | 里程计位姿链缓存条数 |
| `odom_max_linear_speed` | 8.0 | 跳变剔除：最大线速度 (m/s)，本数据集实测最大约 6.2 |
| `odom_max_angular_speed` | 180.0 | 跳变剔除：最大角速度 (deg/s)，本数据集实测最大约 68 |
| `odom_smooth_alpha` | 1.0 | 增量速度低通系数 (0,1]，1=不平滑 |
| `map_frame`/`odom_frame`/`base_frame` | `map`/`odom`/`base_link` | TF 坐标系 |
| `map_resolution` | 0.05 | 地图分辨率 (m) |
| `icp_max_iterations` | 20 | ICP 最大迭代次数 |
| `icp_max_correspondence_dist` | 0.3 | ICP 最大对应点距离 (m) |
| `icp_fitness_score_threshold` | 0.001 | 拟合分数大于该值时触发 CSM 粗匹配；调大（如 0.05~0.3）可减少 CSM、加快处理 |
| `translation_search_range_cells` / `translation_search_step_cells` | 2 / 0.02 | CSM 平移搜索半径（格数）/步长（m） |
| `map_save_file` | 空（= `<包目录>/maps/map`） | 地图保存路径前缀（不含扩展名） |
| `save_map_on_shutdown` | true | 退出时自动保存地图 |
| `map_save_trim_margin` | 1.0 | 地图保存裁剪外扩边距 (m) |
| `map_save_occupied_thresh` / `map_save_free_thresh` | 0.65 / 0.196 | PGM 黑/白判定阈值 |

## 处理流程（每帧）

1. 预积分先验位姿：取与激光时间对齐的 `predict_pose`（OdomProcess 插值结果）；
2. 预测位姿：里程计相邻帧增量叠加到上一帧最终位姿，得到 `predicted_pose`；
3. 当前帧点云按 `predicted_pose` 变换到全局坐标系；
4. 与上一帧为止的地图做 ICP，拟合分数较差时用 CSM 粗匹配（分支搜索）修正位姿；
5. 用最终位姿把点云加入栅格地图，并发布地图/点云/路径/TF；
6. 首帧仅初始化：以第一帧位置为地图原点，不建图。
