#pragma once

#include <cstddef>

#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

struct SmallHoleFillingParameters {
  std::size_t maxHoleSize;
  std::size_t minSupport;
  float maxHeightRange;
};

std::size_t fillSmallElevationHoles(
    grid_map::GridMap& map,
    const SmallHoleFillingParameters& parameters);

}  // namespace elevation_mapping
