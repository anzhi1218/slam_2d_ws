#include "data_preprocess.h"
#include "frame.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <ros/ros.h>

namespace laster_slam_2d {

bool LoadDataFile(const std::string& path, std::vector<Frame>& mvFrames) {
    
    
    std::ifstream f(path);
    if (!f.is_open()) {
        ROS_ERROR("laster_slam_2d: cannot open data file: %s", path.c_str());
        return false;
    }
    
    
    std::string line;
    //跳过前两行数据
    std::getline(f, line);
    std::getline(f, line);
    
    int frame_id = 0;
    while (std::getline(f, line)) {

        if (line.find("Pose:") != std::string::npos) {
            // step2 解析先验位姿
            double time_stamp = 0.0;
            double predict_x = 0.0, predict_y = 0.0;
            double predict_yaw_deg = 0.0;
            std::vector<double> mvCloudX, mvCloudY;
            
            if (!std::getline(f, line)) break;
            
            size_t pos = 0;
            
            // 解析 time（单位：ms）
            pos = line.find("time:") + 5;  
            while (pos < line.length() && std::isspace(line[pos])) pos++;  
            size_t time_end = line.find(",", pos);
            time_stamp = std::stod(line.substr(pos, time_end - pos));
            
            // 解析 yaw（单位：度）
            pos = line.find("yaw:") + 4; 
            while (pos < line.length() && std::isspace(line[pos])) pos++;
            size_t yaw_end = line.find(",", pos);
            predict_yaw_deg = std::stod(line.substr(pos, yaw_end - pos));
            
            // 解析 X
            pos = line.find("X:") + 2;  
            while (pos < line.length() && std::isspace(line[pos])) pos++;
            size_t x_end = line.find(",", pos);
            predict_x = std::stod(line.substr(pos, x_end - pos));
            
            // 解析 Y
            pos = line.find("Y:") + 2;  
            while (pos < line.length() && std::isspace(line[pos])) pos++;
            
            std::string y_str = line.substr(pos);
            predict_y = std::stod(y_str);
            
            // step3 解析点云
            if (!std::getline(f, line) || line.find("Cloud:") == std::string::npos) {
                ROS_WARN("Missing Cloud line");
                continue;
            }
            
            if (!std::getline(f, line)) break;
            
            std::istringstream iss(line);
            double cx, cy;
            while (iss >> cx >> cy) {
                mvCloudX.push_back(cx);
                mvCloudY.push_back(cy);
            }
            
            std::getline(f, line);
            

            mvFrames.emplace_back(frame_id, time_stamp, predict_x, predict_y, predict_yaw_deg, mvCloudX, mvCloudY);
            frame_id++;
            
        }
    }
    
    ROS_INFO("Successfully loaded %zu frames from %s", mvFrames.size(), path.c_str());
    return true;
}


}