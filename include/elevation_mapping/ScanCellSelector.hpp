#pragma once

#include <cstddef>
#include <vector>

#include <grid_map_core/GridMap.hpp>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

// Representatives minimize squared distance to the grid-cell center. Exact
// distance ties prefer finite, lower elevations, then compare confidence,
// color, x, and y lexicographically; non-finite values sort after finite ones.
std::vector<std::size_t> selectScanCellRepresentatives(
    const grid_map::GridMap& map, const PointCloudType& pointCloud);

float sanitizePointVariance(float pointVariance, double minVariance);

}  // namespace elevation_mapping
