#include "elevation_mapping/ScanCellSelector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace elevation_mapping {

std::vector<std::size_t> selectScanCellRepresentatives(
    const grid_map::GridMap& map, const PointCloudType& pointCloud) {
  const auto size = map.getSize();
  if ((size <= 0).any()) {
    return {};
  }

  const std::size_t cellCount =
      static_cast<std::size_t>(size(0) * size(1));
  const std::size_t noPoint = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> selectedByCell(cellCount, noPoint);
  std::vector<double> squaredDistanceByCell(
      cellCount, std::numeric_limits<double>::infinity());

  for (std::size_t pointIndex = 0; pointIndex < pointCloud.size();
       ++pointIndex) {
    const auto& point = pointCloud.points[pointIndex];
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    const grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (!map.getIndex(position, index)) {
      continue;
    }

    grid_map::Position cellCenter;
    if (!map.getPosition(index, cellCenter)) {
      continue;
    }
    const double squaredDistance = (position - cellCenter).squaredNorm();
    const std::size_t cellIndex =
        static_cast<std::size_t>(index(0) * size(1) + index(1));
    if (squaredDistance < squaredDistanceByCell[cellIndex]) {
      squaredDistanceByCell[cellIndex] = squaredDistance;
      selectedByCell[cellIndex] = pointIndex;
    }
  }

  std::vector<std::size_t> selected;
  selected.reserve(cellCount);
  for (const auto pointIndex : selectedByCell) {
    if (pointIndex != noPoint) {
      selected.push_back(pointIndex);
    }
  }
  std::sort(selected.begin(), selected.end());
  return selected;
}

float sanitizePointVariance(
    const float pointVariance, const double minVariance) {
  if (!std::isfinite(pointVariance) ||
      pointVariance < static_cast<float>(minVariance)) {
    return static_cast<float>(minVariance);
  }
  return pointVariance;
}

}  // namespace elevation_mapping
