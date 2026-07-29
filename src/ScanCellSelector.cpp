#include "elevation_mapping/ScanCellSelector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace elevation_mapping {
namespace {

int compareFiniteSafeFloat(float left, float right) {
  const bool leftIsFinite = std::isfinite(left);
  const bool rightIsFinite = std::isfinite(right);
  if (leftIsFinite != rightIsFinite) {
    return leftIsFinite ? -1 : 1;
  }
  if (leftIsFinite) {
    if (left < right) {
      return -1;
    }
    if (right < left) {
      return 1;
    }
  }

  std::uint32_t leftBits;
  std::uint32_t rightBits;
  std::memcpy(&leftBits, &left, sizeof(leftBits));
  std::memcpy(&rightBits, &right, sizeof(rightBits));
  if (leftBits < rightBits) {
    return -1;
  }
  if (rightBits < leftBits) {
    return 1;
  }
  return 0;
}

bool pointDataLess(const pcl::PointXYZRGBConfidenceRatio& left,
                   const pcl::PointXYZRGBConfidenceRatio& right) {
  const int zComparison = compareFiniteSafeFloat(left.z, right.z);
  if (zComparison != 0) {
    return zComparison < 0;
  }
  const int confidenceComparison =
      compareFiniteSafeFloat(left.confidence_ratio, right.confidence_ratio);
  if (confidenceComparison != 0) {
    return confidenceComparison < 0;
  }
  if (left.rgba != right.rgba) {
    return left.rgba < right.rgba;
  }
  const int xComparison = compareFiniteSafeFloat(left.x, right.x);
  if (xComparison != 0) {
    return xComparison < 0;
  }
  return compareFiniteSafeFloat(left.y, right.y) < 0;
}

}  // namespace

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
    const auto selectedPointIndex = selectedByCell[cellIndex];
    if (squaredDistance < squaredDistanceByCell[cellIndex] ||
        (squaredDistance == squaredDistanceByCell[cellIndex] &&
         selectedPointIndex != noPoint &&
         pointDataLess(point, pointCloud.points[selectedPointIndex]))) {
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
