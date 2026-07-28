#include "elevation_mapping/SmallHoleFiller.hpp"

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace elevation_mapping {
namespace {

using IndexKey = std::pair<int, int>;
using IndexSet = std::set<IndexKey>;

IndexKey makeKey(const grid_map::Index& index) {
  return {index(0), index(1)};
}

std::vector<grid_map::Index> getNeighbors(
    const grid_map::GridMap& map, const grid_map::Index& index,
    bool includeDiagonals) {
  grid_map::Position position;
  if (!map.getPosition(index, position)) {
    return {};
  }

  std::vector<grid_map::Index> neighbors;
  const double resolution = map.getResolution();
  for (int xOffset = -1; xOffset <= 1; ++xOffset) {
    for (int yOffset = -1; yOffset <= 1; ++yOffset) {
      if (xOffset == 0 && yOffset == 0) {
        continue;
      }
      if (!includeDiagonals && xOffset != 0 && yOffset != 0) {
        continue;
      }
      grid_map::Index neighbor;
      const grid_map::Position neighborPosition =
          position + grid_map::Position(xOffset * resolution,
                                        yOffset * resolution);
      if (map.getIndex(neighborPosition, neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}

std::vector<grid_map::Index> collectInvalidComponent(
    const grid_map::GridMap& map, const grid_map::Index& start,
    IndexSet& visited) {
  std::vector<grid_map::Index> component;
  std::vector<grid_map::Index> queue{start};
  visited.insert(makeKey(start));

  while (!queue.empty()) {
    const grid_map::Index index = queue.back();
    queue.pop_back();
    component.push_back(index);

    for (const auto& neighbor : getNeighbors(map, index, false)) {
      if (map.isValid(neighbor, "elevation") ||
          !visited.insert(makeKey(neighbor)).second) {
        continue;
      }
      queue.push_back(neighbor);
    }
  }

  return component;
}

float median(std::vector<float> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 != 0) {
    return values[middle];
  }
  return (values[middle - 1] + values[middle]) / 2.0f;
}

}  // namespace

std::size_t fillSmallElevationHoles(
    grid_map::GridMap& map,
    const SmallHoleFillingParameters& parameters) {
  if (!map.exists("elevation") || !map.exists("upper_bound") ||
      !map.exists("lower_bound")) {
    return 0u;
  }

  const grid_map::GridMap originalMap = map;
  std::vector<std::vector<grid_map::Index>> components;
  IndexSet visited;
  for (grid_map::GridMapIterator iterator(originalMap);
       !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    if (originalMap.isValid(index, "elevation") ||
        visited.count(makeKey(index)) != 0u) {
      continue;
    }
    components.push_back(collectInvalidComponent(originalMap, index, visited));
  }

  std::size_t filled = 0u;
  for (const auto& component : components) {
    if (component.size() > parameters.maxHoleSize) {
      continue;
    }

    IndexSet supportKeys;
    std::vector<grid_map::Index> support;
    for (const auto& index : component) {
      for (const auto& neighbor : getNeighbors(originalMap, index, true)) {
        if (!originalMap.isValid(neighbor, "elevation") ||
            !originalMap.isValid(neighbor, "upper_bound") ||
            !originalMap.isValid(neighbor, "lower_bound") ||
            !supportKeys.insert(makeKey(neighbor)).second) {
          continue;
        }
        support.push_back(neighbor);
      }
    }
    if (support.empty() || support.size() < parameters.minSupport) {
      continue;
    }

    std::vector<float> elevations;
    std::vector<float> upperBounds;
    std::vector<float> lowerBounds;
    elevations.reserve(support.size());
    upperBounds.reserve(support.size());
    lowerBounds.reserve(support.size());
    for (const auto& index : support) {
      elevations.push_back(originalMap.at("elevation", index));
      upperBounds.push_back(originalMap.at("upper_bound", index));
      lowerBounds.push_back(originalMap.at("lower_bound", index));
    }
    const auto elevationRange =
        std::minmax_element(elevations.begin(), elevations.end());
    if (*elevationRange.second - *elevationRange.first >
        parameters.maxHeightRange) {
      continue;
    }

    const float elevation = median(elevations);
    const float upperBound = median(upperBounds);
    const float lowerBound = median(lowerBounds);
    for (const auto& index : component) {
      map.at("elevation", index) = elevation;
      map.at("upper_bound", index) = upperBound;
      map.at("lower_bound", index) = lowerBound;
    }
    filled += component.size();
  }

  return filled;
}

}  // namespace elevation_mapping
