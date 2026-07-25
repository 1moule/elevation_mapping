#include "elevation_mapping/MultimodalElevation.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace elevation_mapping {
namespace {

MultimodalConfig config() {
  return MultimodalConfig{};
}

MultimodalPoint point(float x, float y, float z) {
  return {x, y, z, 1e-4f, 0.0f};
}

SurfaceObservation surface(float height, std::uint16_t mask,
                           bool centerOccupied) {
  return {height, 1e-4f, 0.0f, height + 0.03f, 3u, mask, centerOccupied};
}

CellObservation observation(
    std::initializer_list<SurfaceObservation> surfaces) {
  CellObservation result{};
  result.modeCount = std::min<std::size_t>(surfaces.size(), result.modes.size());
  std::size_t index = 0;
  for (const auto& value : surfaces) {
    if (index == result.modeCount) {
      break;
    }
    result.modes[index++] = value;
  }
  return result;
}

MultimodalCellState lowerPrimaryState() {
  MultimodalCellState state;
  state.modes[0] = {true, -0.30f, 1e-4f, 0.0f, -0.27f, 1.0f, 0.0,
                    2u, 1u, false};
  state.modes[1] = {true, 0.20f, 1e-4f, 0.0f, 0.23f, 1.0f, 0.0,
                    2u, 1u, false};
  state.primaryIndex = 0;
  return state;
}

MultimodalCellState twoModeState() {
  return lowerPrimaryState();
}

CellObservation observationWithCenterOnUpper() {
  return observation({surface(-0.30f, (1u << 0) | (1u << 1), false),
                      surface(0.20f, (1u << 4) | (1u << 5), true)});
}

CellObservation upperCoverageWinner() {
  return observation({surface(-0.30f, (1u << 0) | (1u << 1), false),
                      surface(0.20f,
                              (1u << 2) | (1u << 4) | (1u << 5) | (1u << 8),
                              true)});
}

CellObservation equalCoverageObservation() {
  return observation({surface(-0.30f, (1u << 0) | (1u << 1), false),
                      surface(0.20f, (1u << 7) | (1u << 8), false)});
}

CellObservation lowerOnlyObservation() {
  return observation({surface(-0.30f, (1u << 0) | (1u << 1), false)});
}

void expectInsideTargetCell(const std::vector<MultimodalPoint>& points) {
  for (const auto& source : points) {
    EXPECT_LT(std::fabs(source.x), 0.02f);
    EXPECT_LT(std::fabs(source.y), 0.02f);
  }
}

std::vector<MultimodalPoint> makeTwoSurfacePoints(bool highFirst) {
  const std::vector<MultimodalPoint> lower{
      point(-0.018f, -0.018f, -0.30f),
      point(-0.006f, -0.018f, -0.30f),
      point(-0.018f, -0.006f, -0.30f),
  };
  const std::vector<MultimodalPoint> upper{
      point(0.006f, 0.006f, 0.20f),
      point(0.018f, 0.006f, 0.20f),
      point(0.006f, 0.018f, 0.20f),
  };

  std::vector<MultimodalPoint> points;
  points.reserve(lower.size() + upper.size());
  if (highFirst) {
    points.insert(points.end(), upper.begin(), upper.end());
    points.insert(points.end(), lower.begin(), lower.end());
  } else {
    points.insert(points.end(), lower.begin(), lower.end());
    points.insert(points.end(), upper.begin(), upper.end());
  }
  expectInsideTargetCell(points);
  return points;
}

std::vector<MultimodalPoint> makeCenterSupportedUpperSurface() {
  const std::vector<MultimodalPoint> points{
      point(-0.018f, -0.018f, -0.30f),
      point(0.018f, -0.018f, -0.30f),
      point(-0.018f, 0.018f, -0.30f),
      point(0.0f, 0.0f, 0.20f),
      point(0.018f, 0.0f, 0.20f),
      point(0.0f, 0.018f, 0.20f),
  };
  expectInsideTargetCell(points);
  return points;
}

std::vector<MultimodalPoint> makeUpperLowerAndVerticalFacePoints() {
  auto points = makeTwoSurfacePoints(false);
  points.push_back(point(0.018f, -0.018f, -0.15f));
  points.push_back(point(0.018f, -0.018f, -0.14f));
  points.push_back(point(0.018f, -0.018f, -0.13f));
  expectInsideTargetCell(points);
  return points;
}

TEST(MultimodalObservation, IsIndependentOfPointOrder) {
  const auto highFirst = buildCellObservation(
      makeTwoSurfacePoints(true), 0.0f, 0.0f, 0.04f, config());
  const auto lowFirst = buildCellObservation(
      makeTwoSurfacePoints(false), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(highFirst.modeCount, 2u);
  ASSERT_EQ(lowFirst.modeCount, 2u);
  EXPECT_NEAR(highFirst.modes[0].height, lowFirst.modes[0].height, 1e-5f);
  EXPECT_NEAR(highFirst.modes[1].height, lowFirst.modes[1].height, 1e-5f);
}

TEST(MultimodalObservation, MarksModeOccupyingCenterBin) {
  const auto observation = buildCellObservation(
      makeCenterSupportedUpperSurface(), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(observation.modeCount, 2u);
  EXPECT_FALSE(observation.modes[0].centerOccupied);
  EXPECT_TRUE(observation.modes[1].centerOccupied);
}

TEST(MultimodalObservation, KeepsTwoModesWithLargestHorizontalCoverage) {
  const auto observation = buildCellObservation(
      makeUpperLowerAndVerticalFacePoints(), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(observation.modeCount, 2u);
  EXPECT_NEAR(observation.modes[0].height, -0.30f, 0.01f);
  EXPECT_NEAR(observation.modes[1].height, 0.20f, 0.01f);
}

TEST(MultimodalObservation, RejectsInvalidConfiguration) {
  const auto points = makeTwoSurfacePoints(false);
  std::vector<MultimodalConfig> invalidConfigs;

  auto invalid = config();
  invalid.modeSeparation = 0.0f;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minPoints = 1;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minPoints = 2;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minPoints = 0;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minBins = 1;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minBins = 0;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.minBins = 10;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.switchMarginBins = 10;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.switchConfirmations = 0;
  invalidConfigs.push_back(invalid);
  invalid = config();
  invalid.staleTimeout = 0.0;
  invalidConfigs.push_back(invalid);

  for (const auto& invalidConfig : invalidConfigs) {
    EXPECT_FALSE(isValidMultimodalConfig(invalidConfig));
    EXPECT_EQ(buildCellObservation(points, 0.0f, 0.0f, 0.04f, invalidConfig).modeCount, 0u);
  }
}

TEST(MultimodalObservation, RejectsNonFinitePoints) {
  std::vector<MultimodalPoint> points{
      point(-0.018f, -0.018f, -0.30f),
      point(-0.006f, -0.018f, -0.30f),
      point(-0.018f, -0.006f, -0.30f),
  };
  points[0].height = std::numeric_limits<float>::quiet_NaN();
  points[1].variance = std::numeric_limits<float>::infinity();
  points[2].x = -std::numeric_limits<float>::infinity();

  EXPECT_EQ(buildCellObservation(points, 0.0f, 0.0f, 0.04f, config()).modeCount, 0u);
}

TEST(MultimodalObservation, RejectsGroupsWithFewerThanThreePoints) {
  const std::vector<MultimodalPoint> points{
      point(-0.018f, -0.018f, -0.30f),
      point(0.018f, 0.018f, -0.30f),
  };

  EXPECT_EQ(buildCellObservation(points, 0.0f, 0.0f, 0.04f, config()).modeCount, 0u);
}

TEST(MultimodalObservation, RejectsGroupsWithFewerThanTwoOccupiedBins) {
  const std::vector<MultimodalPoint> points{
      point(0.018f, -0.018f, -0.30f),
      point(0.018f, -0.018f, -0.30f),
      point(0.018f, -0.018f, -0.30f),
  };

  EXPECT_EQ(buildCellObservation(points, 0.0f, 0.0f, 0.04f, config()).modeCount, 0u);
}

TEST(MultimodalObservation, CountsOccupiedBins) {
  EXPECT_EQ(countOccupiedBins(0b101001001), 4u);
}

TEST(MultimodalSelection, PrefersCenterSupportedMode) {
  MultimodalCellState state;
  const auto result = updateMultimodalCell(
      state, observationWithCenterOnUpper(), 0.0, config());
  ASSERT_TRUE(result.handled);
  EXPECT_NEAR(result.primary.height, 0.20f, 0.01f);
}

TEST(MultimodalSelection, RequiresThreeConsecutiveWinsToSwitch) {
  MultimodalCellState state = lowerPrimaryState();
  const auto challenger = upperCoverageWinner();
  EXPECT_FALSE(
      updateMultimodalCell(state, challenger, 0.1, config()).primarySwitched);
  EXPECT_FALSE(
      updateMultimodalCell(state, challenger, 0.2, config()).primarySwitched);
  const auto third = updateMultimodalCell(state, challenger, 0.3, config());
  EXPECT_TRUE(third.primarySwitched);
  EXPECT_NEAR(third.primary.height, 0.20f, 0.01f);
}

TEST(MultimodalSelection, CountsChallengerOncePerTimestamp) {
  MultimodalCellState state = lowerPrimaryState();
  const auto challenger = upperCoverageWinner();
  EXPECT_FALSE(
      updateMultimodalCell(state, challenger, 0.1, config()).primarySwitched);
  EXPECT_FALSE(
      updateMultimodalCell(state, challenger, 0.1, config()).primarySwitched);
  EXPECT_FALSE(
      updateMultimodalCell(state, challenger, 0.2, config()).primarySwitched);
  EXPECT_TRUE(
      updateMultimodalCell(state, challenger, 0.3, config()).primarySwitched);
}

TEST(MultimodalSelection, RetainsIncumbentWhenSpatialSupportTies) {
  MultimodalCellState state = lowerPrimaryState();
  const auto result = updateMultimodalCell(
      state, equalCoverageObservation(), 0.1, config());
  EXPECT_NEAR(result.primary.height, -0.30f, 0.01f);
}

TEST(MultimodalSelection, ExpiresUnobservedSecondaryMode) {
  MultimodalCellState state = twoModeState();
  EXPECT_TRUE(expireStaleSecondaryMode(state, 0.7, config()));
  EXPECT_TRUE(state.modes[state.primaryIndex].valid);
  EXPECT_EQ(countValidModes(state), 1u);
}

TEST(MultimodalSelection, MatchesModesByHeightWhenObservationOrderChanges) {
  MultimodalCellState state = lowerPrimaryState();
  const auto reversed = observation({
      surface(0.22f, (1u << 4) | (1u << 5), true),
      surface(-0.28f, (1u << 0) | (1u << 1), false),
  });

  const auto result = updateMultimodalCell(state, reversed, 0.1, config());

  ASSERT_TRUE(result.handled);
  EXPECT_NEAR(state.modes[0].height, -0.29f, 0.01f);
  EXPECT_NEAR(state.modes[1].height, 0.21f, 0.01f);
  EXPECT_NEAR(state.modes[0].lowestPointHeight, -0.25f, 0.01f);
  EXPECT_NEAR(state.modes[1].lowestPointHeight, 0.25f, 0.01f);
}

TEST(MultimodalSelection, DoesNotHandleSingleModeWithoutExistingState) {
  MultimodalCellState state;
  const auto result = updateMultimodalCell(
      state, lowerOnlyObservation(), 0.0, config());

  EXPECT_FALSE(result.handled);
  EXPECT_EQ(countValidModes(state), 0u);
}

TEST(MultimodalSelection, DoesNotExpireUnobservedPrimary) {
  MultimodalCellState state = twoModeState();

  EXPECT_TRUE(expireStaleSecondaryMode(state, 0.7, config()));
  EXPECT_TRUE(state.modes[state.primaryIndex].valid);
  EXPECT_NEAR(state.modes[state.primaryIndex].height, -0.30f, 0.01f);
}

}  // namespace
}  // namespace elevation_mapping
