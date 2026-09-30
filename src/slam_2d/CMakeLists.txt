cmake_minimum_required(VERSION 2.8)
project(slam_2d)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs sensor_msgs std_srvs roscpp roslib tf pcl_conversions)
find_package(PCL REQUIRED COMPONENTS common registration)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs std_srvs roscpp roslib tf pcl_conversions
)

add_executable(slam_2d
  src/main.cpp
  src/slam_2d.cpp
  src/odom_process.cpp
  src/predicted_process.cpp
  src/laser_process.cpp
  src/mapping.cpp
  src/linear_fitting.cpp
)
target_link_libraries(slam_2d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_2d ${catkin_EXPORTED_TARGETS})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(mapping_update_test
    test/mapping_update.test
    test/mapping_update_test.cpp
    src/mapping.cpp
  )
  target_link_libraries(mapping_update_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()

install(TARGETS slam_2d
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

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
