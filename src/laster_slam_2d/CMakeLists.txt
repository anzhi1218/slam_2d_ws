cmake_minimum_required(VERSION 2.8)
project(laster_slam_2d)

find_package(catkin REQUIRED nav_msgs sensor_msgs std_msgs visualization_msgs roscpp tf rosbag_storage pcl_ros pcl_conversions)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs visualization_msgs roscpp tf pcl_ros pcl_conversions
)

# 主节点：可执行文件名 laster_slam_2d
add_executable(laster_slam_2d
  src/main.cpp
  src/laster_slam_2d.cpp
  src/frame.cpp
  src/data_preprocess.cpp
  src/predicted_process.cpp
  src/laser_process.cpp
  src/mapping.cpp
)
target_link_libraries(laster_slam_2d ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(laster_slam_2d ${catkin_EXPORTED_TARGETS})
endif()

# 测试节点：仅读入 data 文件并做 INFO 打印
add_executable(test_data_load
  src/test_data_load.cpp
  src/data_preprocess.cpp
  src/frame.cpp
)
target_link_libraries(test_data_load ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(test_data_load ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS laster_slam_2d test_data_load
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/step_playback_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)