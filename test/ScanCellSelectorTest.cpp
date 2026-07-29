#include "elevation_mapping/ScanCellSelector.hpp"

#include <cstddef>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {
namespace {

grid_map::GridMap makeMap(double length, double resolution) {
  grid_map::GridMap map;
  map.setGeometry(
      grid_map::Length(length, length), resolution,
      grid_map::Position(0.0, 0.0));
  return map;
}

TEST(ScanCellSelectorTest, KeepsOnePointPerCell) {
  const auto map = makeMap(2.0, 1.0);
  PointCloudType cloud;
  cloud.push_back({-0.5f, -0.5f, 0.0f});
  cloud.push_back({0.5f, 0.5f, 1.0f});

  const auto selected = selectScanCellRepresentatives(map, cloud);

  EXPECT_EQ(selected, (std::vector<std::size_t>{0, 1}));
}

TEST(ScanCellSelectorTest, ChoosesPointNearestCellCenter) {
  const auto map = makeMap(1.0, 1.0);
  PointCloudType cloud;
  cloud.push_back({0.40f, 0.40f, 2.0f});
  cloud.push_back({0.05f, 0.05f, 0.0f});
  cloud.push_back({-0.30f, -0.30f, 1.0f});

  const auto selected = selectScanCellRepresentatives(map, cloud);

  ASSERT_EQ(selected.size(), 1u);
  EXPECT_EQ(selected.front(), 1u);
}

TEST(ScanCellSelectorTest, SelectsSameTiedPointDataAcrossPermutations) {
  const auto map = makeMap(1.0, 1.0);
  const pcl::PointXYZRGBConfidenceRatio preferredPoint(
      0.10f, 0.0f, 1.0f, 1u, 2u, 3u, 0.5f);
  const pcl::PointXYZRGBConfidenceRatio otherPoint(
      -0.10f, 0.0f, 2.0f, 4u, 5u, 6u, 0.25f);

  PointCloudType firstOrdering;
  firstOrdering.push_back(otherPoint);
  firstOrdering.push_back(preferredPoint);
  PointCloudType secondOrdering;
  secondOrdering.push_back(preferredPoint);
  secondOrdering.push_back(otherPoint);

  const auto firstSelected =
      selectScanCellRepresentatives(map, firstOrdering);
  const auto secondSelected =
      selectScanCellRepresentatives(map, secondOrdering);

  ASSERT_EQ(firstSelected.size(), 1u);
  ASSERT_EQ(secondSelected.size(), 1u);
  EXPECT_NE(firstSelected.front(), secondSelected.front());

  const auto& firstPoint = firstOrdering[firstSelected.front()];
  const auto& secondPoint = secondOrdering[secondSelected.front()];
  EXPECT_FLOAT_EQ(firstPoint.x, preferredPoint.x);
  EXPECT_FLOAT_EQ(firstPoint.y, preferredPoint.y);
  EXPECT_FLOAT_EQ(firstPoint.z, preferredPoint.z);
  EXPECT_EQ(firstPoint.rgba, preferredPoint.rgba);
  EXPECT_FLOAT_EQ(firstPoint.confidence_ratio, preferredPoint.confidence_ratio);
  EXPECT_FLOAT_EQ(secondPoint.x, preferredPoint.x);
  EXPECT_FLOAT_EQ(secondPoint.y, preferredPoint.y);
  EXPECT_FLOAT_EQ(secondPoint.z, preferredPoint.z);
  EXPECT_EQ(secondPoint.rgba, preferredPoint.rgba);
  EXPECT_FLOAT_EQ(secondPoint.confidence_ratio, preferredPoint.confidence_ratio);
}

TEST(ScanCellSelectorTest, IgnoresPointsOutsideMap) {
  const auto map = makeMap(1.0, 1.0);
  PointCloudType cloud;
  cloud.push_back({2.0f, 2.0f, 0.0f});
  cloud.push_back({0.0f, 0.0f, 0.0f});

  const auto selected = selectScanCellRepresentatives(map, cloud);

  EXPECT_EQ(selected, (std::vector<std::size_t>{1}));
}

TEST(ScanCellSelectorTest, PreservesVarianceInSiUnits) {
  EXPECT_FLOAT_EQ(sanitizePointVariance(0.0025f, 0.0001), 0.0025f);
}

TEST(ScanCellSelectorTest, AppliesMinimumVariance) {
  EXPECT_FLOAT_EQ(sanitizePointVariance(1.0e-9f, 0.0001), 0.0001f);
  EXPECT_FLOAT_EQ(sanitizePointVariance(-1.0f, 0.0001), 0.0001f);
  EXPECT_FLOAT_EQ(
      sanitizePointVariance(
          std::numeric_limits<float>::quiet_NaN(), 0.0001),
      0.0001f);
}

}  // namespace
}  // namespace elevation_mapping
