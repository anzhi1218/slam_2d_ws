# slam_2d_ws

A **2D LiDAR SLAM** workspace (ROS 1 / catkin) that fuses LiDAR scans with wheel-odometry prior poses.

It subscribes to `/scan` and `/odom` online, aligns scans with **ICP + CSM (correlative scan matching)**,
incrementally builds a 2D occupancy grid map, and publishes trajectories and TF. It can be driven
directly by replaying a dataset bag (`GP-SLAM-2D_dataset`) with `rosbag play`, or connected to real sensors.

## Features

- **Online mapping**: incrementally builds the map from `/scan` + `/odom`, with no offline data file.
- **Odometry pre-integration**: extracts, de-glitches, low-pass filters and time-aligns `/odom` increments
  to produce a prior pose strictly synchronized with each scan.
- **ICP fine registration**: aligns the current scan against the map built so far.
- **CSM coarse-match fallback**: branch-and-bound coarse matching over translation/rotation when the ICP
  fitness score is poor.
- **Occupancy grid mapping**: per-frame hit/miss evidence; brief free-space observations do not
  immediately erase stable walls.
- **Map saving**: built-in PGM + YAML export, compatible with ROS 1 `map_server` and Nav2 `nav2_map_server`.
- **RViz visualization**: grid map, current scan cloud, ICP cloud, and prior/predicted/estimated trajectories.

## Directory layout

```
slam_2d_ws/
├── src/
│   └── slam_2d/                 # the slam_2d package
│       ├── include/             # headers (frame.h / odom_process.h / ...)
│       ├── src/                 # sources (main.cpp / slam_2d.cpp / mapping.cpp / ...)
│       ├── launch/              # slam_2d.launch
│       ├── config/              # params.yaml, slam_2d.rviz
│       ├── maps/                # saved grid maps (map.pgm / map.yaml)
│       ├── data/                # dataset (submodule GP-SLAM-2D_dataset)
│       ├── CMakeLists.txt
│       └── package.xml
├── build/  devel/  install/     # catkin build artifacts (git-ignored)
└── README.md
```

For the package-level design, topics and parameters, see [src/slam_2d/README.md](src/slam_2d/README.md).

## Dependencies

- Ubuntu 20.04 + **ROS Noetic** (`roscpp`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`,
  `std_srvs`, `tf`, `pcl_conversions`, `roslib`)
- **PCL** (`common`, `registration`)
- **Eigen3**
- `rviz` for visualization; `rosbag` for replaying bags

## Build

```bash
cd ~/slam_2d_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

### Option 1: launch the node, play the bag in another terminal (recommended)

```bash
# Terminal 1: start the SLAM node and RViz
roslaunch slam_2d slam_2d.launch

# Terminal 2: play the bag (-r 0.2 = 0.2x speed; slow replay is recommended)
rosbag play -r 0.2 ~/slam_2d_ws/src/slam_2d/data/GP-SLAM-2D_dataset/Yongqian_data_2019-05-29-13-12-39.bag
```

Without RViz: `roslaunch slam_2d slam_2d.launch use_rviz:=false`

### Option 2: launch the node together with the bag

```bash
roslaunch slam_2d slam_2d.launch use_bag:=true
roslaunch slam_2d slam_2d.launch use_bag:=true rate:=0.2 bag_file:=/path/to/your.bag
```

### Save the map

- Save at any time while running: `rosservice call /slam_2d/save_map`
- Auto-save on exit (Ctrl-C): controlled by `save_map_on_shutdown` (default `true`)

By default the map is written to `src/slam_2d/maps/map.pgm` and `map.yaml`.

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | occupancy grid map |
| `/slam_2d/grid_map_cloud` | `sensor_msgs/PointCloud2` | map point cloud |
| `/slam_2d/current_cloud` | `sensor_msgs/PointCloud2` | current scan in the prior pose |
| `/slam_2d/icp_cloud` | `sensor_msgs/PointCloud2` | current scan in the matched pose |
| `/slam_2d/predict_path` | `nav_msgs/Path` | prior (odometry) trajectory |
| `/slam_2d/predicted_path` | `nav_msgs/Path` | predicted trajectory |
| `/slam_2d/location_path` | `nav_msgs/Path` | ICP/CSM estimated trajectory |
| `/tf` | `tf2_msgs/TFMessage` | `map -> odom -> base_link` |

Service: `/slam_2d/save_map` (`std_srvs/Trigger`).

## Processing pipeline (per frame)

1. Prior pose: take the `predict_pose` time-aligned with the scan;
2. Predicted pose: accumulate the odometry increment onto the previous final pose to get `predicted_pose`;
3. Transform the current scan into the global frame using `predicted_pose`;
4. Run ICP against the map built so far; fall back to CSM coarse matching when the fitness score is poor;
5. Insert the cloud into the grid map at the final pose, and publish map/clouds/paths/TF;
6. First frame: initialization only — the first pose becomes the map origin, no mapping.

## Dataset

`src/slam_2d/data/GP-SLAM-2D_dataset/` (from RuanJY/GP-SLAM-2D_dataset):

| File | Description |
| --- | --- |
| `Yongqian_data_2019-05-29-13-12-39.bag` | Yongqian Student Activity Center, 214 s, 6414 `/scan` frames, 8466 `/odom` messages |
| `ChouYeiChing_data_2019-05-29-13-17-41.bag` | Chou Yei Ching Building, ~303 s, 9098 `/scan` frames, 12563 `/odom` messages |

Initialize the submodule:

```bash
git submodule update --init --recursive
```

## License

See `src/slam_2d/package.xml` (BSD / Apache 2.0).
