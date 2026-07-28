#include "elevation_mapping/PiecewisePlanarProcessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
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

grid_map::GridMap makeMap(const int xCells, const int yCells) {
  grid_map::GridMap map({"elevation", "upper_bound", "lower_bound"});
  map.setGeometry(grid_map::Length(kResolution * xCells,
                                   kResolution * yCells),
                  kResolution, grid_map::Position(0.0, 0.0));
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

PiecewisePlanarParameters occlusionParameters() {
  auto parameters = defaultParameters();
  parameters.inferredHalfRange = 0.05f;
  return parameters;
}

void addFlatPatch(grid_map::GridMap& map, const int firstX, const int lastX,
                  const int firstY, const int lastY, const float elevation) {
  for (int x = firstX; x <= lastX; ++x) {
    for (int y = firstY; y <= lastY; ++y) {
      setSample(map, grid_map::Index(x, y), elevation);
    }
  }
}

void expectInferredCellAtOneOfHeights(const grid_map::GridMap& output,
                                      const grid_map::Index& index,
                                      const float firstHeight,
                                      const float secondHeight) {
  const float elevation = output.at("elevation", index);
  EXPECT_TRUE(std::abs(elevation - firstHeight) < 1.0e-4f ||
              std::abs(elevation - secondHeight) < 1.0e-4f);
  EXPECT_FALSE(elevation > firstHeight + 1.0e-4f &&
               elevation < secondHeight - 1.0e-4f);
  EXPECT_NEAR(output.at("upper_bound", index) - elevation, 0.05, 1.0e-5);
  EXPECT_NEAR(elevation - output.at("lower_bound", index), 0.05, 1.0e-5);
}

double nearestOriginalSupportDistance(const grid_map::GridMap& support,
                                      const grid_map::Index& index) {
  grid_map::Position position;
  EXPECT_TRUE(support.getPosition(index, position));
  double nearestDistance = std::numeric_limits<double>::infinity();
  const std::vector<std::string> layers{
      "elevation", "upper_bound", "lower_bound"};
  for (grid_map::GridMapIterator iterator(support); !iterator.isPastEnd();
       ++iterator) {
    if (!support.isValid(*iterator, layers)) {
      continue;
    }
    grid_map::Position supportPosition;
    EXPECT_TRUE(support.getPosition(*iterator, supportPosition));
    nearestDistance = std::min(nearestDistance,
                               (supportPosition - position).norm());
  }
  return nearestDistance;
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

TEST(PiecewisePlanarProcessorTest,
     FillsThinBandWithOnlyBorderingPlaneHeights) {
  auto support = makeMap(10, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 6, 9, 0, 4, 0.30f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_EQ(result.inferredCells, 10u);
  for (int x = 4; x <= 5; ++x) {
    for (int y = 0; y <= 4; ++y) {
      expectInferredCellAtOneOfHeights(output, grid_map::Index(x, y), 0.0f,
                                       0.30f);
    }
  }
}

TEST(PiecewisePlanarProcessorTest, SupportsThreeIndependentHeightLevels) {
  auto support = makeMap(16, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 6, 9, 0, 4, 0.20f);
  addFlatPatch(support, 12, 15, 0, 4, 0.45f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_EQ(result.inferredCells, 20u);
  for (int x = 4; x <= 5; ++x) {
    for (int y = 0; y <= 4; ++y) {
      expectInferredCellAtOneOfHeights(output, grid_map::Index(x, y), 0.0f,
                                       0.20f);
    }
  }
  for (int x = 10; x <= 11; ++x) {
    for (int y = 0; y <= 4; ++y) {
      expectInferredCellAtOneOfHeights(output, grid_map::Index(x, y), 0.20f,
                                       0.45f);
    }
  }
}

TEST(PiecewisePlanarProcessorTest, DoesNotExtrapolateFromOnePlane) {
  auto support = makeMap(10, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_EQ(result.inferredCells, 0u);
  for (int x = 4; x <= 9; ++x) {
    for (int y = 0; y <= 4; ++y) {
      EXPECT_FALSE(std::isfinite(output.at("elevation", grid_map::Index(x, y))));
    }
  }
}

TEST(PiecewisePlanarProcessorTest, LeavesWideUnknownInteriorInvalid) {
  auto support = makeMap(14, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 10, 13, 0, 4, 0.30f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_EQ(result.inferredCells, 0u);
  EXPECT_FALSE(std::isfinite(output.at("elevation", grid_map::Index(6, 2))));
  EXPECT_FALSE(std::isfinite(output.at("elevation", grid_map::Index(7, 2))));
}

TEST(PiecewisePlanarProcessorTest, InferredCellsDoNotPropagate) {
  auto support = makeMap(16, 9);
  addFlatPatch(support, 4, 7, 2, 6, 0.0f);
  addFlatPatch(support, 10, 13, 2, 6, 0.30f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_GT(result.inferredCells, 0u);
  for (grid_map::GridMapIterator iterator(support); !iterator.isPastEnd();
       ++iterator) {
    if (support.isValid(*iterator, {"elevation", "upper_bound", "lower_bound"}) ||
        !std::isfinite(output.at("elevation", *iterator))) {
      continue;
    }
    EXPECT_LE(nearestOriginalSupportDistance(support, *iterator), 0.1 + 1.0e-6);
  }
  const grid_map::Index distantUnknown(0, 0);
  EXPECT_GT(nearestOriginalSupportDistance(support, distantUnknown), 0.12);
  EXPECT_FALSE(std::isfinite(output.at("elevation", distantUnknown)));
}

}  // namespace
}  // namespace elevation_mapping
