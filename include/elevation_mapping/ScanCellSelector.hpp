#pragma once

#include <cstddef>
#include <vector>

#include <grid_map_core/GridMap.hpp>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

std::vector<std::size_t> selectScanCellRepresentatives(
    const grid_map::GridMap& map, const PointCloudType& pointCloud);

float sanitizePointVariance(float pointVariance, double minVariance);

}  // namespace elevation_mapping
