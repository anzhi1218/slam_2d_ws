#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <string>

#include <unistd.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "mapping.h"

namespace slam_2d {
namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr Scan(std::initializer_list<float> x_positions) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (float x : x_positions) {
    pcl::PointXYZ point;
    point.x = x;
    point.y = 0.0f;
    point.z = 0.0f;
    cloud->push_back(point);
  }
  return cloud;
}

bool HasNearObstacle(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  return std::any_of(cloud.points.begin(), cloud.points.end(),
                     [](const pcl::PointXYZ& point) {
                       return std::fabs(point.x - 0.525f) < 0.03f &&
                              std::fabs(point.y - 0.025f) < 0.03f;
                     });
}

int SavedPixel(const std::string& pgm_path, int x, int y) {
  std::ifstream pgm(pgm_path, std::ios::binary);
  std::string magic;
  int width = 0, height = 0, max_value = 0;
  if (!(pgm >> magic >> width >> height >> max_value) ||
      magic != "P5" || width != 40 || height != 40 || max_value != 255) {
    return -1;
  }
  pgm.get();  // Consume the newline before the binary pixels.
  pgm.seekg(static_cast<std::streamoff>(y) * width + x, std::ios::cur);
  return pgm.get();
}

class MappingUpdateTest : public ::testing::Test {
 protected:
  MappingUpdateTest() : private_nh_("~") {}

  void SetUp() override {
    private_nh_.setParam("map_resolution", 0.05);
    private_nh_.setParam("map_x_min", -1.0);
    private_nh_.setParam("map_y_min", -1.0);
    private_nh_.setParam("map_x_max", 1.0);
    private_nh_.setParam("map_y_max", 1.0);
    mapping_.reset(new Mapping(node_, private_nh_));
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<Mapping> mapping_;
  Pose2D origin_;
};

TEST_F(MappingUpdateTest, RepeatedEndpointsCountOncePerScan) {
  auto cloud = Scan({0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
  mapping_->UpdateGridMap(cloud, origin_);
  EXPECT_FALSE(HasNearObstacle(*mapping_->getWholeMapCloud()));
}

TEST_F(MappingUpdateTest, EndpointWinsOverAnotherBeamPassingThroughItsCell) {
  auto cloud = Scan({0.5f, 0.8f});
  for (int frame = 0; frame < 5; ++frame) {
    mapping_->UpdateGridMap(cloud, origin_);
  }
  EXPECT_TRUE(HasNearObstacle(*mapping_->getWholeMapCloud()));
}

TEST_F(MappingUpdateTest, SustainedFreeObservationsClearAnOldObstacle) {
  auto near_hit = Scan({0.5f});
  auto farther_hit = Scan({0.8f});
  for (int frame = 0; frame < 20; ++frame) {
    mapping_->UpdateGridMap(near_hit, origin_);
  }
  ASSERT_TRUE(HasNearObstacle(*mapping_->getWholeMapCloud()));
  for (int frame = 0; frame < 20; ++frame) {
    mapping_->UpdateGridMap(farther_hit, origin_);
  }
  EXPECT_FALSE(HasNearObstacle(*mapping_->getWholeMapCloud()));
}

TEST_F(MappingUpdateTest, BriefFreeRunDoesNotEraseAStableWall) {
  auto near_hit = Scan({0.5f});
  auto farther_hit = Scan({0.8f});
  for (int frame = 0; frame < 20; ++frame) {
    mapping_->UpdateGridMap(near_hit, origin_);
  }
  for (int frame = 0; frame < 8; ++frame) {
    mapping_->UpdateGridMap(farther_hit, origin_);
  }
  EXPECT_TRUE(HasNearObstacle(*mapping_->getWholeMapCloud()));
}

TEST_F(MappingUpdateTest, SavedMapClearsAnOldEndpoint) {
  char directory[] = "/tmp/slam_mapping_test_XXXXXX";
  ASSERT_NE(nullptr, mkdtemp(directory));
  const std::string prefix = std::string(directory) + "/map";
  auto near_hit = Scan({0.5f});
  auto farther_hit = Scan({0.8f});
  for (int frame = 0; frame < 20; ++frame) {
    mapping_->UpdateGridMap(near_hit, origin_);
  }
  ASSERT_TRUE(mapping_->SaveMap(prefix));
  EXPECT_EQ(0, SavedPixel(prefix + ".pgm", 30, 19));

  for (int frame = 0; frame < 25; ++frame) {
    mapping_->UpdateGridMap(farther_hit, origin_);
  }
  ASSERT_TRUE(mapping_->SaveMap(prefix));
  EXPECT_EQ(254, SavedPixel(prefix + ".pgm", 30, 19));
  std::remove((prefix + ".pgm").c_str());
  std::remove((prefix + ".yaml").c_str());
  rmdir(directory);
}

}  // namespace
}  // namespace slam_2d

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_update_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
