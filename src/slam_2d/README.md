# slam_2d

2D LiDAR SLAM based on LiDAR scans and wheel-odometry prior poses: it subscribes online to
`/scan` and `/odom`, can be driven directly by replaying a dataset bag (GP-SLAM-2D_dataset)
with `rosbag play`, and outputs a 2D occupancy grid map and trajectories.

## Data sources and adaption

| Topic | Type | Handling |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | Polar → Cartesian point cloud: `x = r·cos(θ)`, `y = r·sin(θ)`, `z = 0`; filters non-finite values, blind-zone points below `range_min`, and **no-return** beams that hit `range_max` (this dataset encodes no-return as 60.0, which would otherwise create a ring of false points around the map) |
| `/odom` | `nav_msgs/Odometry` | Converted to a prior pose `x, y, yaw` (same semantics as the legacy `time / yaw / X / Y` data.txt), then **pre-integrated** and time-aligned with the scan (see below) |

### Odometry pre-integration (`OdomProcess`)

1. **Convert**: `/odom` message → `(stamp, x, y, yaw)`, taking only yaw from the orientation quaternion (planar motion);
2. **Increment extraction**: compute the relative motion between consecutive messages and express it in the previous frame's local frame;
3. **Outlier rejection**: if an increment implies a speed above a threshold (`odom_max_linear_speed` / `odom_max_angular_speed`),
   treat it as a jump and extrapolate with the previous speed over `dt`, so the predicted pose never flies off;
4. **Low-pass smoothing (optional)**: first-order filtering of the increment velocity (`odom_smooth_alpha`, 1 = no smoothing);
5. **Integration**: integrate the cleaned increments into a "clean" pose chain;
6. **Time alignment**: linearly interpolate the pose chain at the scan timestamp (yaw follows the shortest angular path)
   to obtain a prior pose strictly synchronized with the scan (the original scheme used the "nearest odom" message,
   which could be off by up to ~26 ms).

> Note: this dataset's `/odom` already provides absolute poses (twist is all zero, so it cannot be integrated by
> velocity). The "pre-integration" here therefore means time-aligned increment extraction, cleaning and integration.

### Prior-trajectory coordinates

Before publishing, `predict_path` (the prior trajectory) is transformed into the `map` frame (origin at the first
pose, x axis along the first heading), so in RViz the prior, predicted and estimated trajectories can be overlaid
and compared directly.

## Dataset

Located in `data/GP-SLAM-2D_dataset/`:

| File | Description |
| --- | --- |
| `Yongqian_data_2019-05-29-13-12-39.bag` | Yongqian Student Activity Center, 214 s, 6414 `/scan` frames, 8466 `/odom` messages |
| `ChouYeiChing_data_2019-05-29-13-17-41.bag` | Chou Yei Ching Building, ~303 s, 9098 `/scan` frames, 12563 `/odom` messages |

## Build

```bash
cd <workspace root>   # i.e. slam_2d_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

### Option 1: launch the node, play the bag in another terminal (recommended)

```bash
# Terminal 1: start the SLAM node and RViz
roslaunch slam_2d slam_2d.launch

# Terminal 2: play the bag (-r 0.2 = 0.2x speed; one frame takes ~270 ms, so slow replay is recommended)
rosbag play -r 0.2 ~/slam_2d_ws/src/slam_2d/data/GP-SLAM-2D_dataset/Yongqian_data_2019-05-29-13-12-39.bag
```

Without RViz: `roslaunch slam_2d slam_2d.launch use_rviz:=false`

### Option 2: launch the node together with the bag

```bash
roslaunch slam_2d slam_2d.launch use_bag:=true
roslaunch slam_2d slam_2d.launch use_bag:=true rate:=0.2 bag_file:=/path/to/your.bag
```

### Saving the map when mapping finishes

The node has a built-in save feature (standard PGM + YAML, compatible with ROS 1 `map_server` and
Nav2 `nav2_map_server`):

- Save at any time while running: `rosservice call /slam_2d/save_map`
- Auto-save on exit (Ctrl-C): controlled by `save_map_on_shutdown` (default `true`)

By default it saves to `<package>/maps/map.pgm` and `map.yaml`, trimming to the known region (1 m margin) on save.
Map updates are aggregated per scan: each cell gets at most one hit or free evidence per frame; if a beam ends in
a cell, its hit takes precedence over the free evidence of other beams crossing that cell. Obstacle evidence is
capped, so continued free beams can still clear stale obstacles, while brief free observations do not immediately
erase stable walls.
You can also set another path (without extension) via `map_save_file`; it takes effect on the next service call
after `rosparam set`:

```bash
rosparam set /slam_2d/map_save_file /home/slam/maps/lab1
rosservice call /slam_2d/save_map
```

> Note: `map_server` is not installed on this machine, so the legacy `map_saver` command is unavailable;
> the built-in save above does not depend on it.

## Topics

Published by the node (all except the map under the private namespace `slam_2d`):

| Topic | Type | Description |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | occupancy grid map |
| `/slam_2d/grid_map_cloud` | `sensor_msgs/PointCloud2` | map point cloud |
| `/slam_2d/current_cloud` | `sensor_msgs/PointCloud2` | current scan in the prior pose |
| `/slam_2d/icp_cloud` | `sensor_msgs/PointCloud2` | current scan in the matched pose |
| `/slam_2d/predict_path` | `nav_msgs/Path` | prior (odometry) trajectory, aligned to the map frame |
| `/slam_2d/predicted_path` | `nav_msgs/Path` | predicted trajectory |
| `/slam_2d/location_path` | `nav_msgs/Path` | ICP/CSM estimated trajectory |
| `/tf` | `tf2_msgs/TFMessage` | `map -> odom -> base_link` |

Service: `/slam_2d/save_map` (`std_srvs/Trigger`) — saves the map as PGM + YAML (see "Saving the map when
mapping finishes" above).

## Parameters (`config/params.yaml`)

| Parameter | Default | Description |
| --- | --- | --- |
| `scan_topic` / `odom_topic` | `/scan` / `/odom` | input topics |
| `scan_queue_size` / `odom_queue_size` | 30 / 100 | subscriber queue sizes |
| `odom_buffer_size` | 200 | number of cached odometry poses |
| `odom_max_linear_speed` | 8.0 | jump rejection: max linear speed (m/s); measured max on this dataset ≈ 6.2 |
| `odom_max_angular_speed` | 180.0 | jump rejection: max angular speed (deg/s); measured max on this dataset ≈ 68 |
| `odom_smooth_alpha` | 1.0 | increment-velocity low-pass coefficient (0,1]; 1 = no smoothing |
| `map_frame`/`odom_frame`/`base_frame` | `map`/`odom`/`base_link` | TF frames |
| `map_resolution` | 0.05 | map resolution (m) |
| `icp_max_iterations` | 20 | max ICP iterations |
| `icp_max_correspondence_dist` | 0.3 | max ICP correspondence distance (m) |
| `icp_fitness_score_threshold` | 0.001 | CSM coarse matching is triggered when the fitness score exceeds this; increase (e.g. 0.05~0.3) to reduce CSM and speed up |
| `translation_search_range_cells` / `translation_search_step_cells` | 2 / 0.02 | CSM translation search half-window (cells) / step (m) |
| `map_save_file` | empty (= `<package>/maps/map`) | map save path prefix (without extension) |
| `save_map_on_shutdown` | true | auto-save the map on exit |
| `map_save_trim_margin` | 1.0 | margin (m) added when trimming the map on save |
| `map_save_occupied_thresh` / `map_save_free_thresh` | 0.65 / 0.196 | PGM black/white thresholds |

## Processing pipeline (per frame)

1. Prior pose: take the `predict_pose` time-aligned with the scan (OdomProcess interpolation result);
2. Predicted pose: accumulate the odometry increment onto the previous final pose to get `predicted_pose`;
3. Transform the current scan into the global frame using `predicted_pose`;
4. Run ICP against the map built so far; fall back to CSM coarse matching (branch-and-bound) when the fitness score is poor;
5. Insert the cloud into the grid map at the final pose, and publish map/clouds/paths/TF;
6. First frame: initialization only — the first pose becomes the map origin, no mapping.
