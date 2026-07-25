#include "elevation_mapping/EdgeAwareMapUtils.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "grid_map_core/GridMap.hpp"

namespace elevation_mapping {
namespace {

grid_map::GridMap makeMap() {
  grid_map::GridMap map({"elevation", "lower_candidate"});
  map.setGeometry(grid_map::Length(0.6, 0.6), 0.04, grid_map::Position(0.0, 0.0));
  map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
  map["lower_candidate"].setZero();
  return map;
}

grid_map::Index indexAt(const grid_map::GridMap& map, double x, double y) {
  grid_map::Index index;
  EXPECT_TRUE(map.getIndex(grid_map::Position(x, y), index));
  return index;
}

void setHeight(grid_map::GridMap& map, double x, double y, float height) {
  map.at("elevation", indexAt(map, x, y)) = height;
}

void setLowerCandidate(grid_map::GridMap& map, double x, double y) {
  map.at("lower_candidate", indexAt(map, x, y)) = 1.0f;
}

TEST(EdgeAwareMapUtils, ConsistentHeightSupportRequiresCluster) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, 0.0, 0.0, -0.30f);

  EXPECT_FALSE(hasConsistentHeightSupport(map, "elevation", center, -0.30f, 0.12, 0.05, 4));

  setHeight(map, 0.04, 0.0, -0.31f);
  setHeight(map, 0.08, 0.0, -0.29f);
  setHeight(map, 0.04, 0.04, -0.30f);

  EXPECT_TRUE(hasConsistentHeightSupport(map, "elevation", center, -0.30f, 0.12, 0.05, 4));
}

TEST(EdgeAwareMapUtils, HoleFillRejectsOneSidedSupport) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, 0.04, 0.0, 0.20f);
  setHeight(map, 0.08, 0.0, 0.20f);
  setHeight(map, 0.04, 0.04, 0.21f);
  setHeight(map, 0.08, 0.04, 0.19f);

  grid_map::Index source;
  EXPECT_FALSE(findEdgeSafeHoleFillSource(map, "elevation", center, 0.12, 0.05, 4, source));
}

TEST(EdgeAwareMapUtils, HoleFillRejectsRotatedOneSidedSupport) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, -0.04, 0.04, 0.20f);
  setHeight(map, 0.04, 0.04, 0.20f);
  setHeight(map, -0.08, 0.08, 0.21f);
  setHeight(map, 0.08, 0.08, 0.19f);

  grid_map::Index source;
  EXPECT_FALSE(findEdgeSafeHoleFillSource(map, "elevation", center, 0.12, 0.05, 4, source));
}

TEST(EdgeAwareMapUtils, HoleFillRejectsHeightDiscontinuity) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, -0.04, 0.0, 0.20f);
  setHeight(map, -0.08, 0.0, 0.20f);
  setHeight(map, 0.04, 0.0, -0.30f);
  setHeight(map, 0.08, 0.0, -0.30f);

  grid_map::Index source;
  EXPECT_FALSE(findEdgeSafeHoleFillSource(map, "elevation", center, 0.12, 0.05, 4, source));
}

TEST(EdgeAwareMapUtils, HoleFillAcceptsSurroundedFlatHole) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, -0.04, 0.0, 0.20f);
  setHeight(map, 0.04, 0.0, 0.21f);
  setHeight(map, 0.0, -0.04, 0.19f);
  setHeight(map, 0.0, 0.04, 0.20f);

  grid_map::Index source;
  ASSERT_TRUE(findEdgeSafeHoleFillSource(map, "elevation", center, 0.12, 0.05, 4, source));
  EXPECT_TRUE(std::isfinite(map.at("elevation", source)));
}

TEST(EdgeAwareMapUtils, HoleFillWorksAfterCircularBufferMove) {
  auto map = makeMap();
  std::vector<grid_map::BufferRegion> newRegions;
  ASSERT_TRUE(map.move(grid_map::Position(0.08, 0.08), newRegions));
  const auto center = indexAt(map, 0.08, 0.08);
  setHeight(map, 0.04, 0.08, 0.20f);
  setHeight(map, 0.12, 0.08, 0.20f);
  setHeight(map, 0.08, 0.04, 0.20f);
  setHeight(map, 0.08, 0.12, 0.20f);

  grid_map::Index source;
  EXPECT_TRUE(findEdgeSafeHoleFillSource(map, "elevation", center, 0.12, 0.05, 4, source));
}

TEST(EdgeAwareMapUtils, LowerSurfaceSupportRequiresMultipleLowerCandidates) {
  auto map = makeMap();
  const auto center = indexAt(map, 0.0, 0.0);
  setHeight(map, 0.0, 0.0, -0.30f);
  setHeight(map, 0.04, 0.0, -0.31f);
  setHeight(map, 0.08, 0.0, -0.29f);
  setHeight(map, 0.04, 0.04, -0.30f);
  setLowerCandidate(map, 0.0, 0.0);

  EXPECT_FALSE(hasConsistentLowerSurfaceSupport(
      map, "elevation", "lower_candidate", center, -0.30f, 0.12, 0.05, 4, 2));

  setLowerCandidate(map, 0.04, 0.0);
  EXPECT_TRUE(hasConsistentLowerSurfaceSupport(
      map, "elevation", "lower_candidate", center, -0.30f, 0.12, 0.05, 4, 2));
}

TEST(EdgeAwareMapUtils, AdaptiveLowerSurfaceRequiresTwoSupportedScans) {
  AdaptiveLowerSurfaceState state;

  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.0, false, 0.05, 0.25, 2));
  EXPECT_EQ(state.count, 0);
  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.1, true, 0.05, 0.25, 2));
  EXPECT_EQ(state.count, 1);
  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.2, false, 0.05, 0.25, 2));
  EXPECT_EQ(state.count, 0);
  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.3, true, 0.05, 0.25, 2));
  EXPECT_TRUE(updateAdaptiveLowerSurfaceState(state, -0.31f, 0.4, true, 0.05, 0.25, 2));
}

TEST(EdgeAwareMapUtils, AdaptiveLowerSurfaceResetsOnGapOrHeightChange) {
  AdaptiveLowerSurfaceState state;

  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.0, true, 0.05, 0.25, 2));
  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.30f, 0.4, true, 0.05, 0.25, 2));
  EXPECT_EQ(state.count, 1);
  EXPECT_FALSE(updateAdaptiveLowerSurfaceState(state, -0.45f, 0.5, true, 0.05, 0.25, 2));
  EXPECT_EQ(state.count, 1);
  EXPECT_TRUE(updateAdaptiveLowerSurfaceState(state, -0.44f, 0.6, true, 0.05, 0.25, 2));
}

}  // namespace
}  // namespace elevation_mapping
