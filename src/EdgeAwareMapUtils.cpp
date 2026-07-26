#include "elevation_mapping/EdgeAwareMapUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <grid_map_core/iterators/CircleIterator.hpp>

namespace elevation_mapping {

bool hasConsistentHeightSupport(const grid_map::GridMap& map, const std::string& layer,
                                const grid_map::Index& centerIndex, float referenceHeight,
                                double radius, double heightThreshold, std::size_t minSupport) {
  if (!map.exists(layer) || radius <= 0.0 || heightThreshold < 0.0 || minSupport == 0 ||
      !std::isfinite(referenceHeight)) {
    return false;
  }

  grid_map::Position center;
  if (!map.getPosition(centerIndex, center)) {
    return false;
  }

  std::size_t support = 0;
  for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {
    const float height = map.at(layer, *iterator);
    if (!std::isfinite(height) || std::fabs(height - referenceHeight) > heightThreshold) {
      continue;
    }
    if (++support >= minSupport) {
      return true;
    }
  }
  return false;
}

bool hasConsistentLowerSurfaceSupport(
    const grid_map::GridMap& map, const std::string& heightLayer,
    const std::string& candidateLayer, const grid_map::Index& centerIndex,
    float referenceHeight, double radius, double heightThreshold,
    std::size_t minSupport, std::size_t minCandidateSupport) {
  if (!map.exists(heightLayer) || !map.exists(candidateLayer) || radius <= 0.0 ||
      heightThreshold < 0.0 || minSupport == 0 || minCandidateSupport == 0 ||
      !std::isfinite(referenceHeight)) {
    return false;
  }

  grid_map::Position center;
  if (!map.getPosition(centerIndex, center)) {
    return false;
  }

  std::size_t support = 0;
  std::size_t candidateSupport = 0;
  for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {
    const float height = map.at(heightLayer, *iterator);
    if (!std::isfinite(height) || std::fabs(height - referenceHeight) > heightThreshold) {
      continue;
    }
    ++support;
    if (map.at(candidateLayer, *iterator) > 0.5f) {
      ++candidateSupport;
    }
  }

  return support >= minSupport && candidateSupport >= minCandidateSupport;
}

void resetAdaptiveLowerSurfaceState(AdaptiveLowerSurfaceState& state) {
  state.height = std::numeric_limits<float>::quiet_NaN();
  state.timestamp = std::numeric_limits<double>::quiet_NaN();
  state.count = 0;
}

bool updateAdaptiveLowerSurfaceState(AdaptiveLowerSurfaceState& state, float candidateHeight,
                                     double timestamp, bool hasSurfaceSupport,
                                     double heightThreshold, double maxTimeGap,
                                     int recoveryCount) {
  if (!hasSurfaceSupport || !std::isfinite(candidateHeight) || !std::isfinite(timestamp) ||
      heightThreshold < 0.0 || maxTimeGap <= 0.0 || recoveryCount < 2) {
    resetAdaptiveLowerSurfaceState(state);
    return false;
  }

  const double timeDelta = timestamp - state.timestamp;
  const bool continuous = std::isfinite(state.timestamp) && timeDelta >= 0.0 &&
                          timeDelta <= maxTimeGap;
  const bool sameHeight = std::isfinite(state.height) &&
                          std::fabs(candidateHeight - state.height) <= heightThreshold;
  if (!continuous || !sameHeight) {
    state.height = candidateHeight;
    state.timestamp = timestamp;
    state.count = 1;
    return false;
  }

  if (timeDelta > 0.0001) {
    state.height =
        (state.height * static_cast<float>(state.count) + candidateHeight) /
        static_cast<float>(state.count + 1);
    state.timestamp = timestamp;
    ++state.count;
  }
  return state.count >= recoveryCount;
}

bool findEdgeSafeHoleFillSource(const grid_map::GridMap& map, const std::string& layer,
                                const std::string& modeCountLayer,
                                const std::string& modeSeparationLayer,
                                const grid_map::Index& centerIndex,
                                double radius, double heightThreshold,
                                std::size_t minSupport,
                                grid_map::Index& sourceIndex) {
  if (!map.exists(layer) || radius <= 0.0 || heightThreshold < 0.0 || minSupport == 0) {
    return false;
  }

  grid_map::Position center;
  if (!map.getPosition(centerIndex, center)) {
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double directionEpsilon = 0.25 * map.getResolution();
  double nearestDistanceSquared = std::numeric_limits<double>::infinity();
  std::size_t support = 0;
  std::vector<double> supportAngles;
  (void)modeCountLayer;
  (void)modeSeparationLayer;

  for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {
    const float height = map.at(layer, *iterator);
    if (!std::isfinite(height)) {
      continue;
    }

    grid_map::Position neighbor;
    map.getPosition(*iterator, neighbor);
    const grid_map::Position offset = neighbor - center;
    const double distanceSquared = offset.squaredNorm();
    const bool closer = distanceSquared < nearestDistanceSquared;
    const bool sameDistanceWithLowerIndex =
        distanceSquared == nearestDistanceSquared &&
        ((*iterator)(0) < sourceIndex(0) ||
         ((*iterator)(0) == sourceIndex(0) &&
          (*iterator)(1) < sourceIndex(1)));
    if (closer || sameDistanceWithLowerIndex) {
      nearestDistanceSquared = distanceSquared;
      sourceIndex = *iterator;
    }

    ++support;
    if (distanceSquared > directionEpsilon * directionEpsilon) {
      supportAngles.push_back(std::atan2(offset.y(), offset.x()));
    }
  }

  if (support < minSupport || supportAngles.size() < 2) {
    return false;
  }

  std::sort(supportAngles.begin(), supportAngles.end());
  double largestAngularGap = supportAngles.front() + 2.0 * kPi - supportAngles.back();
  for (std::size_t i = 1; i < supportAngles.size(); ++i) {
    largestAngularGap =
        std::max(largestAngularGap, supportAngles[i] - supportAngles[i - 1]);
  }
  return largestAngularGap <= kPi + 1e-6;
}

}  // namespace elevation_mapping
