#include "elevation_mapping/ElevationMap.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {
namespace {

struct TestPoint {
  float x;
  float y;
  float z;
};

}  // namespace

class ElevationMapMixedReturnTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  float runMixedReturnSequence(bool highPointFirst) {
    static int nodeId = 0;
    const auto node = std::make_shared<rclcpp::Node>(
        "elevation_map_mixed_return_test_" + std::to_string(nodeId++));
    node->declare_parameter("postprocessor_num_threads", 1);

    ElevationMap map(node);
    map.setGeometry(grid_map::Length(0.6, 0.6), 0.04,
                    grid_map::Position(0.0, 0.0));
    map.enableSkipLowerPoints_ = true;
    map.skipLowerPointsDuration_ = 0.8;
    map.lowerPointRecoveryCount_ = 8;
    map.enableAdaptiveLowerSurface_ = true;
    map.lowerSurfaceNeighborRadius_ = 0.12;
    map.lowerSurfaceMinSupport_ = 4;
    map.lowerSurfaceMinCandidateSupport_ = 2;
    map.lowerSurfaceRecoveryCount_ = 2;
    map.lowerSurfaceHeightThreshold_ = 0.05;
    map.lowerSurfaceMaxTimeGap_ = 0.25;

    const std::array<std::array<float, 2>, 4> positions{{
        {0.0f, 0.0f},
        {0.04f, 0.0f},
        {0.0f, 0.04f},
        {0.04f, 0.04f},
    }};
    const auto baseTime = node->get_clock()->now();

    std::vector<TestPoint> upperPoints;
    std::vector<TestPoint> lowerPoints;
    for (const auto& position : positions) {
      upperPoints.push_back({position[0], position[1], 0.20f});
      lowerPoints.push_back({position[0], position[1], -0.30f});
    }

    addScan(map, upperPoints, baseTime + rclcpp::Duration::from_seconds(0.1));
    addScan(map, lowerPoints, baseTime + rclcpp::Duration::from_seconds(0.2));

    std::vector<TestPoint> mixedPoints = lowerPoints;
    const TestPoint highPoint{positions[0][0], positions[0][1], 0.20f};
    if (highPointFirst) {
      mixedPoints.insert(mixedPoints.begin(), highPoint);
    } else {
      mixedPoints.push_back(highPoint);
    }
    addScan(map, mixedPoints, baseTime + rclcpp::Duration::from_seconds(0.3));

    grid_map::Index targetIndex;
    EXPECT_TRUE(map.getRawGridMap().getIndex(grid_map::Position(0.0, 0.0),
                                             targetIndex));
    return map.getRawGridMap().at("elevation", targetIndex);
  }

 private:
  static void addScan(ElevationMap& map, const std::vector<TestPoint>& points,
                      const rclcpp::Time& timestamp) {
    PointCloudType::Ptr cloud(new PointCloudType);
    cloud->reserve(points.size());
    for (const auto& source : points) {
      pcl::PointXYZRGBConfidenceRatio point;
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
      point.r = 0;
      point.g = 0;
      point.b = 0;
      cloud->push_back(point);
    }

    Eigen::VectorXf variances =
        Eigen::VectorXf::Constant(static_cast<Eigen::Index>(points.size()), 0.00001f);
    ASSERT_TRUE(map.add(cloud, variances, timestamp, Eigen::Affine3d::Identity()));
  }
};

TEST_F(ElevationMapMixedReturnTest, ConfirmsLowerSurfaceWithHighReturnFirst) {
  EXPECT_NEAR(runMixedReturnSequence(true), -0.30f, 0.01f);
}

TEST_F(ElevationMapMixedReturnTest, ConfirmsLowerSurfaceWithLowReturnFirst) {
  EXPECT_NEAR(runMixedReturnSequence(false), -0.30f, 0.01f);
}

}  // namespace elevation_mapping
