#ifndef LASTER_SLAM_2D_DATA_PREPROCESS_H
#define LASTER_SLAM_2D_DATA_PREPROCESS_H

#include <vector>
#include <string>
#include "frame.h"

namespace laster_slam_2d {
    /// 加载数据文件
    bool LoadDataFile(const std::string& path, std::vector<Frame>& mvFrames);
}

#endif //data_preprocess.h