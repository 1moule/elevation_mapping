#include "elevation_mapping/ElevationMap.hpp"

#include <cmath>
#include <limits>
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

std::shared_ptr<rclcpp::Node> makeNode() {
  static int nodeId = 0;
  auto node = std::make_shared<rclcpp::Node>(
      "elevation_map_multimodal_test_" + std::to_string(nodeId++));
  node->declare_parameter("postprocessor_num_threads", 1);
  return node;
}

std::vector<TestPoint> makeAmbiguousPoints(
    double x, double y, bool upperCenterSupported) {
  const auto appendSurface = [x, y](
                                 std::vector<TestPoint>& points, float height,
                                 bool centerSupported) {
    if (centerSupported) {
      points.push_back(
          {static_cast<float>(x), static_cast<float>(y), height});
      points.push_back(
          {static_cast<float>(x + 0.018), static_cast<float>(y), height});
      points.push_back(
          {static_cast<float>(x), static_cast<float>(y + 0.018), height});
      return;
    }
    points.push_back(
        {static_cast<float>(x - 0.018), static_cast<float>(y - 0.018),
         height});
    points.push_back(
        {static_cast<float>(x + 0.018), static_cast<float>(y - 0.018),
         height});
    points.push_back(
        {static_cast<float>(x - 0.018), static_cast<float>(y + 0.018),
         height});
  };

  std::vector<TestPoint> points;
  points.reserve(6);
  appendSurface(points, -0.30f, !upperCenterSupported);
  appendSurface(points, 0.20f, upperCenterSupported);
  return points;
}

}  // namespace

class ElevationMapMultimodalTest : public testing::Test {
 protected:
  ElevationMapMultimodalTest()
      : node_(makeNode()), map_(node_), baseTime_(node_->get_clock()->now()) {
    map_.setGeometry(grid_map::Length(0.6, 0.6), 0.04,
                     grid_map::Position(0.0, 0.0));
    configureMultimodal(true);
  }

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

  void configureMultimodal(bool enabled) {
    map_.enableMultimodalCells_ = enabled;
  }

  void addScan(const std::vector<TestPoint>& points, double seconds) {
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

    Eigen::VectorXf variances = Eigen::VectorXf::Constant(
        static_cast<Eigen::Index>(points.size()), 1e-4f);
    ASSERT_TRUE(map_.add(
        cloud, variances, baseTime_ + rclcpp::Duration::from_seconds(seconds),
        Eigen::Affine3d::Identity()));
  }

  void initializeUpperAndLowerSurfaces() {
    initializeTwoModesAt(0.0, 0.0);
  }

  void initializeTwoModesAt(double x, double y) {
    addScan(makeAmbiguousPoints(x, y, true), 0.0);
  }

  void addAmbiguousScan(bool upperCenterSupported, double seconds) {
    addScan(makeAmbiguousPoints(0.0, 0.0, upperCenterSupported), seconds);
  }

  void addAmbiguousScanAt(double x, double y, double seconds) {
    addScan(makeAmbiguousPoints(x, y, false), seconds);
  }

  void addSingleSurfaceScan(float height, double seconds) {
    addScan(
        {{-0.018f, -0.018f, height},
         {-0.006f, -0.018f, height},
         {-0.018f, -0.006f, height}},
        seconds);
  }

  float rawValueAt(
      const std::string& layer, double x, double y) const {
    grid_map::Index index;
    if (!map_.rawMap_.getIndex(grid_map::Position(x, y), index)) {
      ADD_FAILURE() << "Position is outside raw map.";
      return std::numeric_limits<float>::quiet_NaN();
    }
    return map_.rawMap_.at(layer, index);
  }

  float elevationAtTarget() const {
    return rawValueAt("elevation", 0.0, 0.0);
  }

  float secondaryElevationAtTarget() const {
    return rawValueAt("secondary_elevation", 0.0, 0.0);
  }

  float elevationAt(double x, double y) const {
    return rawValueAt("elevation", x, y);
  }

  std::shared_ptr<rclcpp::Node> node_;
  ElevationMap map_;
  rclcpp::Time baseTime_;
};

TEST_F(
    ElevationMapMultimodalTest,
    KeepsCenterSupportedHeightAcrossViewChanges) {
  initializeUpperAndLowerSurfaces();
  addAmbiguousScan(/* upperCenterSupported = */ true, 0.1);
  addAmbiguousScan(/* upperCenterSupported = */ false, 0.2);
  addAmbiguousScan(/* upperCenterSupported = */ true, 0.3);

  EXPECT_NEAR(elevationAtTarget(), 0.20f, 0.02f);
  EXPECT_NEAR(secondaryElevationAtTarget(), -0.30f, 0.02f);
  EXPECT_FLOAT_EQ(rawValueAt("height_mode_count", 0.0, 0.0), 2.0f);
  EXPECT_NEAR(
      rawValueAt("height_mode_separation", 0.0, 0.0), 0.50f, 0.02f);
  EXPECT_TRUE(
      std::isfinite(rawValueAt("primary_mode_confidence", 0.0, 0.0)));
}

TEST_F(ElevationMapMultimodalTest, UsesLegacyPathWhenFeatureDisabled) {
  configureMultimodal(false);
  addSingleSurfaceScan(-0.10f, 0.1);

  EXPECT_NEAR(elevationAtTarget(), -0.10f, 0.02f);
  EXPECT_TRUE(std::isnan(secondaryElevationAtTarget()));
}

TEST_F(ElevationMapMultimodalTest, MovesModeStateWithCircularBuffer) {
  initializeTwoModesAt(0.0, 0.0);
  map_.move(Eigen::Vector2d(0.08, 0.08));
  addAmbiguousScanAt(0.0, 0.0, 0.2);

  EXPECT_NEAR(elevationAt(0.0, 0.0), 0.20f, 0.02f);
}

TEST_F(ElevationMapMultimodalTest, LegacyPointOrderDoesNotOverwritePrimary) {
  addScan(makeAmbiguousPoints(0.0, 0.0, false), 0.0);
  addAmbiguousScan(/* upperCenterSupported = */ true, 0.1);

  EXPECT_NEAR(elevationAtTarget(), -0.30f, 0.02f);
  EXPECT_NEAR(secondaryElevationAtTarget(), 0.20f, 0.02f);
}

TEST_F(ElevationMapMultimodalTest, ExpiresSecondaryWithoutPointInCell) {
  initializeTwoModesAt(0.0, 0.0);
  addScan({}, 0.6);

  EXPECT_FLOAT_EQ(rawValueAt("height_mode_count", 0.0, 0.0), 1.0f);
  EXPECT_TRUE(std::isnan(secondaryElevationAtTarget()));
  EXPECT_TRUE(
      std::isnan(rawValueAt("height_mode_separation", 0.0, 0.0)));
  EXPECT_TRUE(
      std::isfinite(rawValueAt("primary_mode_confidence", 0.0, 0.0)));
}

TEST_F(ElevationMapMultimodalTest, ClearsModeState) {
  initializeTwoModesAt(0.0, 0.0);
  ASSERT_TRUE(map_.clear());
  addSingleSurfaceScan(-0.10f, 0.1);

  EXPECT_NEAR(elevationAtTarget(), -0.10f, 0.02f);
  EXPECT_TRUE(std::isnan(secondaryElevationAtTarget()));
  EXPECT_FLOAT_EQ(rawValueAt("height_mode_count", 0.0, 0.0), 0.0f);
}

TEST_F(ElevationMapMultimodalTest, ResetsModeStateWithGeometry) {
  initializeTwoModesAt(0.0, 0.0);
  map_.setGeometry(grid_map::Length(0.6, 0.6), 0.04,
                   grid_map::Position(0.0, 0.0));
  addSingleSurfaceScan(-0.10f, 0.1);

  EXPECT_NEAR(elevationAtTarget(), -0.10f, 0.02f);
  EXPECT_TRUE(std::isnan(secondaryElevationAtTarget()));
  EXPECT_FLOAT_EQ(rawValueAt("height_mode_count", 0.0, 0.0), 0.0f);
}

}  // namespace elevation_mapping
