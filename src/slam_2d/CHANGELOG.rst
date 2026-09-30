^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laster_slam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (未标注日期)
------------------
* 由 slam_gmapping 改名为 laster_slam_2d，适配自定义 data.txt（Pose + Cloud）与 IMU 先验位姿。
* 使用 laster_slam_2d_openslam 核心库（源自 openslam_gmapping）进行栅格粒子滤波建图。
* 节点、launch、测试等命名统一为 laster_slam_2d。
