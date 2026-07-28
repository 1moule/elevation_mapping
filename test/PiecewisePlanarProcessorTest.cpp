#include "elevation_mapping/PiecewisePlanarProcessor.hpp"
#include "elevation_mapping/SmallHoleFiller.hpp"

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

grid_map::GridMap makeMap(const int xCells, const int yCells,
                          const grid_map::Position& center) {
  grid_map::GridMap map({"elevation", "upper_bound", "lower_bound"});
  map.setGeometry(grid_map::Length(kResolution * xCells,
                                   kResolution * yCells),
                  kResolution, center);
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
  return {0.05f, 4u, 0.03f, 0.03f, 0.1f, 1u, 0.1f, false, 0.0f};
}

PiecewisePlanarParameters occlusionParameters() {
  auto parameters = defaultParameters();
  parameters.inferredHalfRange = 0.05f;
  return parameters;
}

PiecewisePlanarParameters directionalParameters() {
  auto parameters = occlusionParameters();
  parameters.maxOcclusionDistance = kResolution;
  parameters.enableDirectionalGroundCompletion = true;
  parameters.directionalGroundMaxGapWidth = 0.40f;
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

void addFlatPatchInRectangle(grid_map::GridMap& map, const double minimumX,
                             const double maximumX, const double minimumY,
                             const double maximumY, const float elevation) {
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(map.getPosition(*iterator, position));
    constexpr double kBoundaryTolerance = 1.0e-9;
    if (position.x() < minimumX - kBoundaryTolerance ||
        position.x() > maximumX + kBoundaryTolerance ||
        position.y() < minimumY - kBoundaryTolerance ||
        position.y() > maximumY + kBoundaryTolerance) {
      continue;
    }
    setSample(map, *iterator, elevation);
  }
}

void copyLayersByPosition(const grid_map::GridMap& source,
                          grid_map::GridMap& destination) {
  const std::vector<std::string> layers{
      "elevation", "upper_bound", "lower_bound"};
  for (grid_map::GridMapIterator iterator(source); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(source.getPosition(*iterator, position));
    grid_map::Index destinationIndex;
    ASSERT_TRUE(destination.getIndex(position, destinationIndex));
    for (const auto& layer : layers) {
      destination.at(layer, destinationIndex) = source.at(layer, *iterator);
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

void expectCellsInRectangleHaveHeight(const grid_map::GridMap& map,
                                      const double minimumX,
                                      const double maximumX,
                                      const double minimumY,
                                      const double maximumY,
                                      const float elevation) {
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(map.getPosition(*iterator, position));
    if (position.x() < minimumX || position.x() > maximumX ||
        position.y() < minimumY || position.y() > maximumY) {
      continue;
    }
    EXPECT_NEAR(map.at("elevation", *iterator), elevation, 1.0e-5);
  }
}

void expectCellsInRectangleInvalid(const grid_map::GridMap& map,
                                   const double minimumX,
                                   const double maximumX,
                                   const double minimumY,
                                   const double maximumY) {
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(map.getPosition(*iterator, position));
    if (position.x() < minimumX || position.x() > maximumX ||
        position.y() < minimumY || position.y() > maximumY) {
      continue;
    }
    EXPECT_FALSE(std::isfinite(map.at("elevation", *iterator)));
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
     UsesPreHoleFillSupportForOcclusionCompletion) {
  auto support = makeMap(5, 5);
  addFlatPatch(support, 0, 1, 0, 4, 0.0f);
  addFlatPatch(support, 3, 4, 0, 4, 0.05f);
  setSample(support, grid_map::Index(2, 0), 0.025f);
  setSample(support, grid_map::Index(2, 1), 0.025f);
  setSample(support, grid_map::Index(2, 3), 0.025f);
  setSample(support, grid_map::Index(2, 4), 0.025f);
  auto output = support;
  const auto planarSupport = output;
  const grid_map::Index hole(2, 2);

  SmallHoleFillingParameters holeParameters{1u, 4u, 0.05f};
  EXPECT_EQ(fillSmallElevationHoles(output, holeParameters), 1u);
  EXPECT_NEAR(output.at("elevation", hole), 0.025, 1.0e-5);

  auto parameters = occlusionParameters();
  parameters.minRegionSize = 4u;
  parameters.neighborHeightTolerance = 0.01f;
  parameters.maxOcclusionDistance = 0.05f;
  parameters.minOcclusionSupport = 1u;
  const auto result = processPiecewisePlanarElevation(
      planarSupport, output, parameters);

  EXPECT_EQ(result.inferredCells, 1u);
  expectInferredCellAtOneOfHeights(output, hole, 0.0f, 0.05f);
  EXPECT_FALSE(std::isfinite(planarSupport.at("elevation", hole)));
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

TEST(PiecewisePlanarProcessorTest,
     WritesInferredCellsAtPhysicalPositionsWithDifferentStartIndices) {
  auto support = makeMap(12, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 6, 9, 0, 4, 0.30f);
  auto output = support;
  const grid_map::Position originalPosition = output.getPosition();
  ASSERT_TRUE(output.move(originalPosition +
                          grid_map::Position(kResolution, 0.0)));
  output.setPosition(originalPosition);
  ASSERT_FALSE((support.getStartIndex() == output.getStartIndex()).all());
  ASSERT_TRUE(support.getPosition() == output.getPosition());
  ASSERT_TRUE((support.getLength() == output.getLength()).all());
  EXPECT_FLOAT_EQ(support.getResolution(), output.getResolution());
  copyLayersByPosition(support, output);

  const auto result = processPiecewisePlanarElevation(
      support, output, occlusionParameters());

  EXPECT_EQ(result.inferredCells, 10u);
  for (grid_map::GridMapIterator iterator(support); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(support.getPosition(*iterator, position));
    grid_map::Index outputIndex;
    ASSERT_TRUE(output.getIndex(position, outputIndex));
    if ((*iterator)(0) >= 4 && (*iterator)(0) <= 5) {
      expectInferredCellAtOneOfHeights(output, outputIndex, 0.0f, 0.30f);
      continue;
    }
    if ((*iterator)(0) >= 10) {
      EXPECT_FALSE(std::isfinite(output.at("elevation", outputIndex)));
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

TEST(PiecewisePlanarProcessorTest,
     DisabledDirectionalCompletionPreservesNarrowAcceptedPlane) {
  auto support = makeMap(12, 1);
  addFlatPatch(support, 0, 11, 0, 0, 0.20f);
  auto output = support;
  auto parameters = defaultParameters();
  parameters.enableDirectionalGroundCompletion = false;

  const auto result = processPiecewisePlanarElevation(
      support, output, parameters);

  EXPECT_EQ(result.acceptedPlanes, 1u);
  EXPECT_EQ(result.regularizedCells, 12u);
  for (int x = 0; x < 12; ++x) {
    EXPECT_NEAR(output.at("elevation", grid_map::Index(x, 0)), 0.20, 1.0e-6);
    EXPECT_NEAR(output.at("upper_bound", grid_map::Index(x, 0)), 0.23,
                1.0e-6);
    EXPECT_NEAR(output.at("lower_bound", grid_map::Index(x, 0)), 0.17,
                1.0e-6);
  }
}

TEST(PiecewisePlanarProcessorTest,
     DisabledDirectionalCompletionPreservesNearCollinearAcceptedPlane) {
  auto support = makeMap(14, 14);
  for (int diagonal = 1; diagonal <= 12; ++diagonal) {
    setSample(support, grid_map::Index(diagonal, diagonal), 0.20f);
  }
  setSample(support, grid_map::Index(6, 7), 0.20f);
  auto output = support;
  auto parameters = defaultParameters();
  parameters.minRegionSize = 10u;
  parameters.enableDirectionalGroundCompletion = false;

  const auto result =
      processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.acceptedPlanes, 1u);
  EXPECT_EQ(result.regularizedCells, 13u);
  for (int diagonal = 1; diagonal <= 12; ++diagonal) {
    EXPECT_NEAR(output.at("elevation",
                          grid_map::Index(diagonal, diagonal)),
                0.20, 1.0e-6);
  }
  EXPECT_NEAR(output.at("elevation", grid_map::Index(6, 7)), 0.20, 1.0e-6);
}

TEST(PiecewisePlanarProcessorTest,
     LeavesCellsWithOnlyOneLocalPlaneInvalid) {
  auto support = makeMap(12, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 8, 11, 0, 4, 0.30f);
  auto output = support;
  auto parameters = occlusionParameters();
  parameters.maxOcclusionDistance = 0.081f;

  const auto result =
      processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.inferredCells, 0u);
  for (int x = 4; x <= 7; ++x) {
    for (int y = 0; y <= 4; ++y) {
      EXPECT_FALSE(std::isfinite(output.at("elevation", grid_map::Index(x, y))));
    }
  }
}

TEST(PiecewisePlanarProcessorTest,
     DisabledDirectionalCompletionPreservesBaseConservativeExactTie) {
  auto canonicalSupport = makeMap(9, 5);
  addFlatPatch(canonicalSupport, 0, 3, 0, 4, 0.0f);
  addFlatPatch(canonicalSupport, 5, 8, 0, 4, 0.30f);
  auto rolledSupport = canonicalSupport;
  const grid_map::Position originalPosition = rolledSupport.getPosition();
  ASSERT_TRUE(rolledSupport.move(
      originalPosition + grid_map::Position(4.0 * kResolution, 0.0)));
  rolledSupport.setPosition(originalPosition);
  ASSERT_FALSE(
      (canonicalSupport.getStartIndex() == rolledSupport.getStartIndex()).all());
  copyLayersByPosition(canonicalSupport, rolledSupport);
  auto canonicalOutput = canonicalSupport;
  auto rolledOutput = rolledSupport;
  auto parameters = occlusionParameters();
  parameters.maxOcclusionDistance = 0.05f;
  parameters.enableDirectionalGroundCompletion = false;

  const auto canonicalResult = processPiecewisePlanarElevation(
      canonicalSupport, canonicalOutput, parameters);
  const auto rolledResult = processPiecewisePlanarElevation(
      rolledSupport, rolledOutput, parameters);

  const grid_map::Index canonicalTarget(4, 2);
  grid_map::Position targetPosition;
  ASSERT_TRUE(canonicalSupport.getPosition(canonicalTarget, targetPosition));
  grid_map::Index rolledTarget;
  ASSERT_TRUE(rolledSupport.getIndex(targetPosition, rolledTarget));
  EXPECT_EQ(canonicalResult.inferredCells, 5u);
  EXPECT_EQ(rolledResult.inferredCells, 5u);
  EXPECT_FLOAT_EQ(canonicalOutput.at("elevation", canonicalTarget), 0.0f);
  EXPECT_FLOAT_EQ(rolledOutput.at("elevation", rolledTarget), 0.30f);
}

TEST(PiecewisePlanarProcessorTest, HandlesExtremeFiniteSearchRadius) {
  auto support = makeMap(9, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 5, 8, 0, 4, 0.30f);
  auto output = support;
  auto parameters = occlusionParameters();
  parameters.maxOcclusionDistance = std::numeric_limits<float>::max();

  const auto result =
      processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.inferredCells, 5u);
  for (int y = 0; y <= 4; ++y) {
    expectInferredCellAtOneOfHeights(output, grid_map::Index(4, y), 0.0f,
                                     0.30f);
  }
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionFillsDescendingGapWithLowerPlane) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.inferredCells, 20u);
  for (grid_map::GridMapIterator iterator(support); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(support.getPosition(*iterator, position));
    if (position.x() < 0.20 || position.x() > 0.28) {
      continue;
    }
    grid_map::Index outputIndex;
    ASSERT_TRUE(output.getIndex(position, outputIndex));
    EXPECT_NEAR(output.at("elevation", outputIndex), 0.10, 1.0e-5);
    EXPECT_NEAR(output.at("upper_bound", outputIndex) -
                    output.at("elevation", outputIndex),
                0.05, 1.0e-5);
    EXPECT_NEAR(output.at("elevation", outputIndex) -
                    output.at("lower_bound", outputIndex),
                0.05, 1.0e-5);
  }
}

TEST(PiecewisePlanarProcessorTest,
     DisabledDirectionalCompletionPreservesConservativeResult) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  auto output = support;
  auto parameters = directionalParameters();
  parameters.enableDirectionalGroundCompletion = false;

  const auto result = processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.inferredCells, 0u);
  for (grid_map::GridMapIterator iterator(support); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    ASSERT_TRUE(support.getPosition(*iterator, position));
    if (position.x() < 0.20 || position.x() > 0.28) {
      continue;
    }
    grid_map::Index outputIndex;
    ASSERT_TRUE(output.getIndex(position, outputIndex));
    EXPECT_FALSE(std::isfinite(output.at("elevation", outputIndex)));
  }
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionDoesNotTriggerWhenAscending) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.10f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.30f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.inferredCells, 0u);
  expectCellsInRectangleInvalid(output, 0.20, 0.32, -0.08, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionUsesMapCenterAsObserverXY) {
  auto nearSupport = makeMap(30, 5, grid_map::Position(0.0, 0.0));
  auto farSupport = makeMap(30, 5, grid_map::Position(0.50, 0.0));
  for (auto* support : {&nearSupport, &farSupport}) {
    addFlatPatchInRectangle(*support, 0.06, 0.18, -0.08, 0.08, 0.30f);
    addFlatPatchInRectangle(*support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  }
  auto nearOutput = nearSupport;
  auto farOutput = farSupport;

  const auto nearResult = processPiecewisePlanarElevation(
      nearSupport, nearOutput, directionalParameters());
  const auto farResult = processPiecewisePlanarElevation(
      farSupport, farOutput, directionalParameters());

  EXPECT_GT(nearResult.inferredCells, 0u);
  expectCellsInRectangleHaveHeight(nearOutput, 0.20, 0.32, -0.08, 0.08,
                                    0.10f);
  EXPECT_EQ(farResult.inferredCells, 0u);
  expectCellsInRectangleInvalid(farOutput, 0.20, 0.32, -0.08, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRejectsGapWiderThanMaximum) {
  auto support = makeMap(50, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.70, 0.82, -0.08, 0.08, 0.10f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.inferredCells, 0u);
  expectCellsInRectangleInvalid(output, 0.34, 0.62, -0.08, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRequiresObservedLowerPlane) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  auto output = support;

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.inferredCells, 0u);
  expectCellsInRectangleInvalid(output, 0.20, 0.46, -0.08, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRejectsCellsBesideSupportSegment) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, -0.04, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, -0.04, 0.10f);
  auto output = support;

  processPiecewisePlanarElevation(support, output, directionalParameters());

  expectCellsInRectangleInvalid(output, 0.20, 0.32, 0.04, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionDoesNotPropagateInferredCells) {
  auto support = makeMap(60, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  auto output = support;
  addFlatPatchInRectangle(output, 0.62, 0.74, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(output, 0.90, 1.02, -0.08, 0.08, 0.10f);

  const auto outputOnlyHigh = indexAt(output, 0.70, 0.0);
  const auto outputOnlyLow = indexAt(output, 0.98, 0.0);
  ASSERT_FALSE(std::isfinite(support.at("elevation", outputOnlyHigh)));
  ASSERT_FALSE(std::isfinite(support.at("elevation", outputOnlyLow)));
  ASSERT_TRUE(std::isfinite(output.at("elevation", outputOnlyHigh)));
  ASSERT_TRUE(std::isfinite(output.at("elevation", outputOnlyLow)));

  processPiecewisePlanarElevation(support, output, directionalParameters());

  expectCellsInRectangleHaveHeight(output, 0.20, 0.32, -0.08, 0.08, 0.10f);
  expectCellsInRectangleInvalid(output, 0.76, 0.88, -0.08, 0.08);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionHandlesTwoPlatformsInWideView) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, -0.04, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, -0.04, 0.10f);
  addFlatPatchInRectangle(support, 0.06, 0.18, 0.04, 0.08, 0.50f);
  addFlatPatchInRectangle(support, 0.34, 0.46, 0.04, 0.08, 0.20f);
  auto output = support;

  processPiecewisePlanarElevation(support, output, directionalParameters());

  expectCellsInRectangleHaveHeight(output, 0.20, 0.32, -0.08, -0.04, 0.10f);
  expectCellsInRectangleHaveHeight(output, 0.20, 0.32, 0.04, 0.08, 0.20f);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionKeepsMultipleStairLevelsSeparate) {
  auto support = makeMap(40, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.50f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.62, 0.74, -0.08, 0.08, 0.10f);
  auto output = support;

  processPiecewisePlanarElevation(support, output, directionalParameters());

  expectCellsInRectangleHaveHeight(output, 0.20, 0.32, -0.08, 0.08, 0.30f);
  expectCellsInRectangleHaveHeight(output, 0.48, 0.60, -0.08, 0.08, 0.10f);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionPairTieUsesStableIdsAcrossCircularBufferRoll) {
  auto support = makeMap(30, 7);
  const auto highFirst = indexAt(support, 0.06, 0.0);
  const auto highLast = indexAt(support, 0.18, 0.0);
  const auto lowFirst = indexAt(support, 0.34, 0.0);
  const auto lowLast = indexAt(support, 0.46, 0.0);
  const auto target = indexAt(support, 0.26, 0.0);
  ASSERT_GE(target(1), 2);
  ASSERT_LT(target(1) + 2, support.getSize()(1));
  addFlatPatch(support, std::min(highFirst(0), highLast(0)),
               std::max(highFirst(0), highLast(0)), target(1) + 1,
               target(1) + 2, 0.30f);
  addFlatPatch(support, std::min(lowFirst(0), lowLast(0)),
               std::max(lowFirst(0), lowLast(0)), target(1) - 2,
               target(1) - 1, 0.10f);
  addFlatPatch(support, std::min(lowFirst(0), lowLast(0)),
               std::max(lowFirst(0), lowLast(0)), target(1) + 1,
               target(1) + 2, 0.05f);
  auto output = support;

  grid_map::Position targetPosition;
  grid_map::Position firstLowPosition;
  grid_map::Position secondLowPosition;
  ASSERT_TRUE(support.getPosition(target, targetPosition));
  ASSERT_TRUE(support.getPosition(
      grid_map::Index(lowFirst(0), target(1) - 1), firstLowPosition));
  ASSERT_TRUE(support.getPosition(
      grid_map::Index(lowFirst(0), target(1) + 1), secondLowPosition));
  EXPECT_DOUBLE_EQ((targetPosition - firstLowPosition).squaredNorm(),
                   (targetPosition - secondLowPosition).squaredNorm());
  EXPECT_LT(secondLowPosition.y(), firstLowPosition.y());

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.acceptedPlanes, 3u);
  EXPECT_NEAR(output.at("elevation", target), 0.05, 1.0e-5);

  auto rolledSupport = support;
  const grid_map::Position originalPosition = rolledSupport.getPosition();
  ASSERT_TRUE(rolledSupport.move(
      originalPosition + grid_map::Position(4.0 * kResolution, 0.0)));
  rolledSupport.setPosition(originalPosition);
  ASSERT_FALSE((support.getStartIndex() == rolledSupport.getStartIndex()).all());
  copyLayersByPosition(support, rolledSupport);
  auto rolledOutput = rolledSupport;
  grid_map::Index rolledTarget;
  ASSERT_TRUE(rolledSupport.getIndex(targetPosition, rolledTarget));

  const auto rolledResult = processPiecewisePlanarElevation(
      rolledSupport, rolledOutput, directionalParameters());

  EXPECT_EQ(rolledResult.acceptedPlanes, 3u);
  EXPECT_NEAR(rolledOutput.at("elevation", rolledTarget), 0.05, 1.0e-5);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionWritesByPhysicalPosition) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  auto output = support;
  const grid_map::Position originalPosition = output.getPosition();
  ASSERT_TRUE(output.move(originalPosition +
                          grid_map::Position(kResolution, 0.0)));
  output.setPosition(originalPosition);
  ASSERT_FALSE((support.getStartIndex() == output.getStartIndex()).all());
  copyLayersByPosition(support, output);

  const auto result = processPiecewisePlanarElevation(
      support, output, directionalParameters());

  EXPECT_EQ(result.inferredCells, 20u);
  expectCellsInRectangleHaveHeight(output, 0.20, 0.32, -0.08, 0.08, 0.10f);
}

TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionCountsConservativeOverlapOnce) {
  auto support = makeMap(30, 5);
  const auto highFirst = indexAt(support, 0.06, 0.0);
  const auto highLast = indexAt(support, 0.18, 0.0);
  const auto lowFirst = indexAt(support, 0.26, 0.0);
  const auto lowLast = indexAt(support, 0.38, 0.0);
  addFlatPatch(support, std::min(highFirst(0), highLast(0)),
               std::max(highFirst(0), highLast(0)), 0, 4, 0.30f);
  addFlatPatch(support, std::min(lowFirst(0), lowLast(0)),
               std::max(lowFirst(0), lowLast(0)), 0, 4, 0.10f);
  auto output = support;
  auto parameters = directionalParameters();
  parameters.maxOcclusionDistance = 0.10f;

  const auto result = processPiecewisePlanarElevation(support, output, parameters);

  EXPECT_EQ(result.inferredCells, 5u);
  const auto gap = indexAt(output, 0.22, 0.0);
  EXPECT_NEAR(output.at("elevation", gap), 0.10, 1.0e-5);
}

TEST(PiecewisePlanarProcessorTest,
     InvalidDirectionalWidthsDoNotCompleteDirectionalGaps) {
  auto support = makeMap(30, 5);
  addFlatPatchInRectangle(support, 0.06, 0.18, -0.08, 0.08, 0.30f);
  addFlatPatchInRectangle(support, 0.34, 0.46, -0.08, 0.08, 0.10f);
  const auto noisyHighSample = indexAt(support, 0.10, 0.0);
  setSample(support, noisyHighSample, 0.32f);
  const std::vector<float> invalidWidths{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(), 0.0f, -0.40f};

  for (const float invalidWidth : invalidWidths) {
    auto output = support;
    auto parameters = directionalParameters();
    parameters.directionalGroundMaxGapWidth = invalidWidth;

    const auto result =
        processPiecewisePlanarElevation(support, output, parameters);

    EXPECT_EQ(result.inferredCells, 0u);
    EXPECT_GT(result.regularizedCells, 0u);
    EXPECT_LT(output.at("elevation", noisyHighSample), 0.31f);
    expectCellsInRectangleInvalid(output, 0.20, 0.32, -0.08, 0.08);
  }
}

TEST(PiecewisePlanarProcessorTest,
     InvalidDirectionalWidthsPreserveConservativeCompletion) {
  auto support = makeMap(10, 5);
  addFlatPatch(support, 0, 3, 0, 4, 0.0f);
  addFlatPatch(support, 6, 9, 0, 4, 0.30f);
  const std::vector<float> invalidWidths{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(), 0.0f, -0.40f};

  for (const float invalidWidth : invalidWidths) {
    auto output = support;
    auto parameters = occlusionParameters();
    parameters.enableDirectionalGroundCompletion = true;
    parameters.directionalGroundMaxGapWidth = invalidWidth;

    const auto result =
        processPiecewisePlanarElevation(support, output, parameters);

    EXPECT_EQ(result.inferredCells, 10u);
    for (int x = 4; x <= 5; ++x) {
      for (int y = 0; y <= 4; ++y) {
        expectInferredCellAtOneOfHeights(output, grid_map::Index(x, y), 0.0f,
                                         0.30f);
      }
    }
  }
}

}  // namespace
}  // namespace elevation_mapping
