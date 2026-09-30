#ifndef SLAM_2D_FRAME_H
#define SLAM_2D_FRAME_H

// 通用数据类型定义

namespace slam_2d {

// 2D位姿：x、y、yaw(弧度)
struct Pose2D {
  double x = 0;
  double y = 0;
  double yaw = 0;
};

}  // namespace slam_2d

#endif  // SLAM_2D_FRAME_H
