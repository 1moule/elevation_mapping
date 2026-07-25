#include "elevation_mapping/MultimodalElevation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace elevation_mapping {
namespace {

struct ProvisionalGroup {
  std::vector<MultimodalPoint> points;
  SurfaceObservation observation{};
};

bool isFinitePoint(const MultimodalPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.height) && std::isfinite(point.variance) &&
         std::isfinite(point.color);
}

bool pointLessByHeight(const MultimodalPoint& left, const MultimodalPoint& right) {
  if (left.height != right.height) {
    return left.height < right.height;
  }
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  if (left.variance != right.variance) {
    return left.variance < right.variance;
  }
  return left.color < right.color;
}

int microBin(float offset, float cellResolution) {
  const int bin = static_cast<int>(
      std::floor((offset / cellResolution + 0.5f) * 3.0f));
  return std::max(0, std::min(2, bin));
}

SurfaceObservation makeObservation(const std::vector<MultimodalPoint>& points,
                                   float cellCenterX, float cellCenterY,
                                   float cellResolution) {
  SurfaceObservation observation{};
  observation.pointCount = points.size();
  observation.lowestPointHeight = std::numeric_limits<float>::infinity();

  double totalWeight = 0.0;
  double weightedHeight = 0.0;
  for (const auto& point : points) {
    const float variance = std::max(point.variance, 1e-9f);
    const double weight = 1.0 / static_cast<double>(variance);
    totalWeight += weight;
    weightedHeight += weight * static_cast<double>(point.height);

    const int column = microBin(point.x - cellCenterX, cellResolution);
    const int row = microBin(point.y - cellCenterY, cellResolution);
    observation.xyMask |= static_cast<std::uint16_t>(1u << (row * 3 + column));
    observation.lowestPointHeight = std::min(
        observation.lowestPointHeight,
        point.height + 3.0f * std::sqrt(variance));
  }

  observation.height = static_cast<float>(weightedHeight / totalWeight);
  observation.variance = static_cast<float>(1.0 / totalWeight);
  observation.centerOccupied = (observation.xyMask & (1u << 4)) != 0;

  const MultimodalPoint* colorPoint = &points.front();
  float closestDistance = std::fabs(colorPoint->height - observation.height);
  for (const auto& point : points) {
    const float distance = std::fabs(point.height - observation.height);
    if (distance < closestDistance ||
        (distance == closestDistance && pointLessByHeight(point, *colorPoint))) {
      closestDistance = distance;
      colorPoint = &point;
    }
  }
  observation.color = colorPoint->color;
  return observation;
}

bool hasHigherRank(const ProvisionalGroup& left, const ProvisionalGroup& right) {
  const std::size_t leftBins = countOccupiedBins(left.observation.xyMask);
  const std::size_t rightBins = countOccupiedBins(right.observation.xyMask);
  if (leftBins != rightBins) {
    return leftBins > rightBins;
  }
  if (left.observation.pointCount != right.observation.pointCount) {
    return left.observation.pointCount > right.observation.pointCount;
  }
  if (left.observation.variance != right.observation.variance) {
    return left.observation.variance < right.observation.variance;
  }
  return left.observation.height < right.observation.height;
}

}  // namespace

bool isValidMultimodalConfig(const MultimodalConfig& config) {
  return std::isfinite(config.modeSeparation) && config.modeSeparation > 0.0f &&
         config.minPoints >= 3 && config.minBins >= 2 && config.minBins <= 9 &&
         config.switchMarginBins <= 9 && config.switchConfirmations > 0 &&
         std::isfinite(config.staleTimeout) && config.staleTimeout > 0.0;
}

std::size_t countOccupiedBins(std::uint16_t mask) {
  std::size_t count = 0;
  while (mask != 0) {
    count += mask & 1u;
    mask >>= 1;
  }
  return count;
}

CellObservation buildCellObservation(
    const std::vector<MultimodalPoint>& points, float cellCenterX,
    float cellCenterY, float cellResolution, const MultimodalConfig& config) {
  CellObservation result{};
  if (!isValidMultimodalConfig(config) || !std::isfinite(cellCenterX) ||
      !std::isfinite(cellCenterY) || !std::isfinite(cellResolution) ||
      cellResolution <= 0.0f) {
    return result;
  }

  std::vector<MultimodalPoint> accepted;
  accepted.reserve(points.size());
  for (const auto& point : points) {
    if (isFinitePoint(point)) {
      accepted.push_back(point);
    }
  }
  std::sort(accepted.begin(), accepted.end(), pointLessByHeight);

  std::vector<ProvisionalGroup> groups;
  for (const auto& point : accepted) {
    if (groups.empty() ||
        point.height - groups.back().points.front().height > config.modeSeparation) {
      groups.push_back(ProvisionalGroup{});
    }
    groups.back().points.push_back(point);
  }

  std::vector<ProvisionalGroup> supportedGroups;
  supportedGroups.reserve(groups.size());
  for (auto& group : groups) {
    if (group.points.size() < config.minPoints) {
      continue;
    }

    group.observation = makeObservation(group.points, cellCenterX, cellCenterY,
                                        cellResolution);
    if (countOccupiedBins(group.observation.xyMask) < config.minBins) {
      continue;
    }
    supportedGroups.push_back(std::move(group));
  }

  std::sort(supportedGroups.begin(), supportedGroups.end(), hasHigherRank);
  const std::size_t selectedCount = std::min<std::size_t>(2, supportedGroups.size());
  for (std::size_t index = 0; index < selectedCount; ++index) {
    result.modes[index] = supportedGroups[index].observation;
  }
  result.modeCount = selectedCount;
  std::sort(result.modes.begin(), result.modes.begin() + result.modeCount,
            [](const SurfaceObservation& left, const SurfaceObservation& right) {
              return left.height < right.height;
            });
  return result;
}

}  // namespace elevation_mapping
