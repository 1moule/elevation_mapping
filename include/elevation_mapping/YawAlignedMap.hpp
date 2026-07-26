#pragma once

#include <string>

#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

grid_map::GridMap createYawAlignedMap(
    const grid_map::GridMap& source,
    const grid_map::Position& bodyWorldPosition, double bodyYaw,
    const std::string& frameId);

}  // namespace elevation_mapping
