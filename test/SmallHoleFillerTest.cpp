#include "elevation_mapping/SmallHoleFiller.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {
namespace {

grid_map::GridMap makeMap() {
  grid_map::GridMap map({"elevation", "upper_bound", "lower_bound"});
  map.setGeometry(grid_map::Length(5.0, 5.0), 1.0,
                  grid_map::Position(0.0, 0.0));
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  map["elevation"].setConstant(invalid);
  map["upper_bound"].setConstant(invalid);
  map["lower_bound"].setConstant(invalid);
  return map;
}

grid_map::Index indexAt(const grid_map::GridMap& map, double x, double y) {
  grid_map::Index index;
  EXPECT_TRUE(map.getIndex(grid_map::Position(x, y), index));
  return index;
}

void setSample(grid_map::GridMap& map, double x, double y, float elevation,
               float upperBound, float lowerBound) {
  const auto index = indexAt(map, x, y);
  map["elevation"](index(0), index(1)) = elevation;
  map["upper_bound"](index(0), index(1)) = upperBound;
  map["lower_bound"](index(0), index(1)) = lowerBound;
}

SmallHoleFillingParameters defaultParameters() {
  return {4u, 4u, 0.05f};
}

void addCardinalSupport(grid_map::GridMap& map) {
  setSample(map, 0.0, 1.0, 0.01f, 10.0f, -10.0f);
  setSample(map, 1.0, 0.0, 0.04f, 50.0f, -50.0f);
  setSample(map, 0.0, -1.0, 0.02f, 30.0f, -30.0f);
  setSample(map, -1.0, 0.0, 0.03f, 70.0f, -70.0f);
}

void addTwoCellHoleSupport(grid_map::GridMap& map) {
  setSample(map, -1.0, 0.0, 0.02f, 10.0f, -10.0f);
  setSample(map, 0.0, 1.0, 0.02f, 20.0f, -20.0f);
  setSample(map, 0.0, -1.0, 0.02f, 30.0f, -30.0f);
  setSample(map, 1.0, 1.0, 0.02f, 40.0f, -40.0f);
  setSample(map, 1.0, -1.0, 0.02f, 50.0f, -50.0f);
  setSample(map, 2.0, 0.0, 0.02f, 60.0f, -60.0f);
}

TEST(SmallHoleFillerTest, FillsSingleCellHoleWithMedianSupport) {
  auto map = makeMap();
  addCardinalSupport(map);
  setSample(map, 1.0, 1.0, 0.02f,
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN());

  const auto filled = fillSmallElevationHoles(map, defaultParameters());

  const auto hole = indexAt(map, 0.0, 0.0);
  EXPECT_EQ(filled, 1u);
  EXPECT_FLOAT_EQ(map["elevation"](hole(0), hole(1)), 0.025f);
  EXPECT_FLOAT_EQ(map["upper_bound"](hole(0), hole(1)), 40.0f);
  EXPECT_FLOAT_EQ(map["lower_bound"](hole(0), hole(1)), -40.0f);
}

TEST(SmallHoleFillerTest, RejectsComponentLargerThanMaximum) {
  auto map = makeMap();
  addTwoCellHoleSupport(map);

  const auto firstHole = indexAt(map, 0.0, 0.0);
  const auto secondHole = indexAt(map, 1.0, 0.0);
  auto parameters = defaultParameters();
  parameters.maxHoleSize = 1u;

  EXPECT_EQ(fillSmallElevationHoles(map, parameters), 0u);
  EXPECT_FALSE(std::isfinite(map["elevation"](firstHole(0), firstHole(1))));
  EXPECT_FALSE(std::isfinite(map["upper_bound"](firstHole(0), firstHole(1))));
  EXPECT_FALSE(std::isfinite(map["lower_bound"](firstHole(0), firstHole(1))));
  EXPECT_FALSE(std::isfinite(map["elevation"](secondHole(0), secondHole(1))));
  EXPECT_FALSE(std::isfinite(map["upper_bound"](secondHole(0), secondHole(1))));
  EXPECT_FALSE(std::isfinite(map["lower_bound"](secondHole(0), secondHole(1))));
}

TEST(SmallHoleFillerTest, RejectsHoleAcrossHeightDiscontinuity) {
  auto map = makeMap();
  setSample(map, 0.0, 1.0, 0.0f, 10.0f, -10.0f);
  setSample(map, 1.0, 0.0, 0.1f, 20.0f, -20.0f);
  setSample(map, 0.0, -1.0, 0.0f, 30.0f, -30.0f);
  setSample(map, -1.0, 0.0, 0.1f, 40.0f, -40.0f);

  const auto hole = indexAt(map, 0.0, 0.0);
  EXPECT_EQ(fillSmallElevationHoles(map, defaultParameters()), 0u);
  EXPECT_FALSE(std::isfinite(map["elevation"](hole(0), hole(1))));
  EXPECT_FALSE(std::isfinite(map["upper_bound"](hole(0), hole(1))));
  EXPECT_FALSE(std::isfinite(map["lower_bound"](hole(0), hole(1))));
}

TEST(SmallHoleFillerTest, LeavesExistingElevationsUnchanged) {
  auto map = makeMap();
  addCardinalSupport(map);
  setSample(map, 2.0, 2.0, 9.0f, 90.0f, -90.0f);

  EXPECT_EQ(fillSmallElevationHoles(map, defaultParameters()), 1u);

  const auto existing = indexAt(map, 2.0, 2.0);
  EXPECT_FLOAT_EQ(map["elevation"](existing(0), existing(1)), 9.0f);
  EXPECT_FLOAT_EQ(map["upper_bound"](existing(0), existing(1)), 90.0f);
  EXPECT_FLOAT_EQ(map["lower_bound"](existing(0), existing(1)), -90.0f);
}

}  // namespace
}  // namespace elevation_mapping
