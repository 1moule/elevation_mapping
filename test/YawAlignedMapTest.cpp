#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "elevation_mapping/YawAlignedMap.hpp"

namespace elevation_mapping {
namespace {

grid_map::GridMap makeWorldMap() {
  grid_map::GridMap map({"elevation", "variance"});
  map.setFrameId("world");
  map.setTimestamp(123456789);
  map.setGeometry(grid_map::Length(6.0, 6.0), 1.0,
                  grid_map::Position(2.0, 3.0));

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position position;
    EXPECT_TRUE(map.getPosition(*iterator, position));
    map.at("elevation", *iterator) =
        static_cast<float>(10.0 * position.x() + position.y());
    map.at("variance", *iterator) =
        static_cast<float>(position.x() - position.y());
  }
  return map;
}

void expectOutputAtLocalPositionMatchesWorld(
    const grid_map::GridMap& source, const grid_map::GridMap& output,
    const grid_map::Position& localPosition,
    const grid_map::Position& expectedWorldPosition) {
  grid_map::Index outputIndex;
  grid_map::Index sourceIndex;
  ASSERT_TRUE(output.getIndex(localPosition, outputIndex));
  ASSERT_TRUE(source.getIndex(expectedWorldPosition, sourceIndex));
  EXPECT_FLOAT_EQ(output.at("elevation", outputIndex),
                  source.at("elevation", sourceIndex));
  EXPECT_FLOAT_EQ(output.at("variance", outputIndex),
                  source.at("variance", sourceIndex));
}

TEST(YawAlignedMapTest, ZeroYawTranslatesLocalCellsToBodyWorldPosition) {
  const auto source = makeWorldMap();
  const grid_map::Position bodyWorldPosition(2.0, 3.0);

  const auto output =
      createYawAlignedMap(source, bodyWorldPosition, 0.0, "elevation_map_body");

  EXPECT_EQ(output.getFrameId(), "elevation_map_body");
  EXPECT_EQ(output.getTimestamp(), source.getTimestamp());
  EXPECT_TRUE(output.getPosition().isApprox(grid_map::Position::Zero()));
  EXPECT_TRUE(output.getLength().isApprox(source.getLength()));
  EXPECT_DOUBLE_EQ(output.getResolution(), source.getResolution());
  expectOutputAtLocalPositionMatchesWorld(
      source, output, grid_map::Position(1.5, -0.5),
      grid_map::Position(3.5, 2.5));
}

TEST(YawAlignedMapTest, QuarterTurnRotatesLocalCellsBeforeWorldLookup) {
  const auto source = makeWorldMap();
  const grid_map::Position bodyWorldPosition(2.0, 3.0);
  constexpr double kHalfPi = 1.5707963267948966;

  const auto output = createYawAlignedMap(source, bodyWorldPosition, kHalfPi,
                                          "elevation_map_body");

  expectOutputAtLocalPositionMatchesWorld(
      source, output, grid_map::Position(1.5, -0.5),
      grid_map::Position(2.5, 4.5));
}

}  // namespace
}  // namespace elevation_mapping
