#include "elevation_mapping/PiecewisePlanarProcessor.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace elevation_mapping {
namespace {

constexpr float kResolution = 0.04f;

grid_map::GridMap makeMap() {
  grid_map::GridMap map({"elevation", "upper_bound", "lower_bound"});
  map.setGeometry(grid_map::Length(0.2, 0.2), kResolution,
                  grid_map::Position(0.0, 0.0));
  return map;
}

grid_map::Index indexAt(const grid_map::GridMap& map, double x, double y) {
  grid_map::Index index;
  EXPECT_TRUE(map.getIndex(grid_map::Position(x, y), index));
  return index;
}

float planeHeight(double x, double y) {
  return static_cast<float>(0.02 * x - 0.01 * y + 0.4);
}

void setSample(grid_map::GridMap& map, const grid_map::Index& index,
               float elevation) {
  map.at("elevation", index) = elevation;
  map.at("upper_bound", index) = elevation + 0.03f;
  map.at("lower_bound", index) = elevation - 0.03f;
}

void addNoisyPlane(grid_map::GridMap& map) {
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(map.getPosition(*iterator, position));
    const float noise = (((*iterator)(0) + (*iterator)(1)) % 2 == 0)
                            ? -0.02f
                            : 0.02f;
    setSample(map, *iterator, planeHeight(position.x(), position.y()) + noise);
  }
}

PiecewisePlanarParameters defaultParameters() {
  return {0.05f, 4u, 0.03f, 0.03f, 0.1f, 1u, 0.1f};
}

void expectMapEqual(const grid_map::GridMap& expected,
                    const grid_map::GridMap& actual) {
  for (grid_map::GridMapIterator iterator(expected); !iterator.isPastEnd();
       ++iterator) {
    EXPECT_FLOAT_EQ(expected.at("elevation", *iterator),
                    actual.at("elevation", *iterator));
    EXPECT_FLOAT_EQ(expected.at("upper_bound", *iterator),
                    actual.at("upper_bound", *iterator));
    EXPECT_FLOAT_EQ(expected.at("lower_bound", *iterator),
                    actual.at("lower_bound", *iterator));
  }
}

TEST(PiecewisePlanarProcessorTest, DefaultConfigurationIsDisabled) {
  EXPECT_FALSE(defaultPiecewisePlanarRegularizationEnabled());
}

TEST(PiecewisePlanarProcessorTest, DisabledLeavesOutputUnchanged) {
  auto support = makeMap();
  addNoisyPlane(support);
  auto output = support;
  const auto before = output;

  const auto result = processPiecewisePlanarElevationIfEnabled(
      support, output, false, defaultParameters());

  EXPECT_EQ(result.acceptedPlanes, 0u);
  EXPECT_EQ(result.regularizedCells, 0u);
  EXPECT_EQ(result.inferredCells, 0u);
  expectMapEqual(before, output);
}

TEST(PiecewisePlanarProcessorTest, FlattensNoisyPlaneAndPreservesBoundWidth) {
  auto support = makeMap();
  addNoisyPlane(support);
  auto output = support;

  const auto result =
      processPiecewisePlanarElevation(support, output, defaultParameters());

  const auto center = indexAt(output, 0.0, 0.0);
  EXPECT_EQ(result.acceptedPlanes, 1u);
  EXPECT_EQ(result.regularizedCells, 25u);
  EXPECT_NEAR(output.at("elevation", center), 0.4, 1.0e-3);
  EXPECT_NEAR(output.at("upper_bound", center) -
                  output.at("lower_bound", center),
              0.06, 1.0e-5);
}

TEST(PiecewisePlanarProcessorTest, RetainsSlopeInsteadOfForcingHorizontal) {
  auto support = makeMap();
  addNoisyPlane(support);
  auto output = support;

  const auto result =
      processPiecewisePlanarElevation(support, output, defaultParameters());

  const auto left = indexAt(output, -0.08, 0.0);
  const auto right = indexAt(output, 0.08, 0.0);
  EXPECT_EQ(result.acceptedPlanes, 1u);
  EXPECT_GT(output.at("elevation", right), output.at("elevation", left));
  EXPECT_NEAR(output.at("elevation", right) - output.at("elevation", left),
              0.0032, 1.0e-3);
}

TEST(PiecewisePlanarProcessorTest, DoesNotModifyLargeResidualObstacle) {
  auto support = makeMap();
  addNoisyPlane(support);
  const auto obstacle = indexAt(support, 0.0, 0.0);
  support.at("elevation", obstacle) += 0.20f;
  support.at("upper_bound", obstacle) += 0.20f;
  support.at("lower_bound", obstacle) += 0.20f;
  auto output = support;
  const float obstacleElevation = output.at("elevation", obstacle);
  const float obstacleUpper = output.at("upper_bound", obstacle);
  const float obstacleLower = output.at("lower_bound", obstacle);
  auto parameters = defaultParameters();
  parameters.neighborHeightTolerance = 0.3f;

  const auto result = processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.acceptedPlanes, 1u);
  EXPECT_EQ(output.at("elevation", obstacle), obstacleElevation);
  EXPECT_EQ(output.at("upper_bound", obstacle), obstacleUpper);
  EXPECT_EQ(output.at("lower_bound", obstacle), obstacleLower);
}

TEST(PiecewisePlanarProcessorTest, DoesNotModifySupportMap) {
  auto support = makeMap();
  addNoisyPlane(support);
  const auto before = support;
  auto output = support;

  processPiecewisePlanarElevation(support, output, defaultParameters());

  expectMapEqual(before, support);
}

}  // namespace
}  // namespace elevation_mapping
