#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

struct AdaptiveLowerSurfaceState {
  float height{std::numeric_limits<float>::quiet_NaN()};
  double timestamp{std::numeric_limits<double>::quiet_NaN()};
  int count{0};
};

bool hasConsistentHeightSupport(const grid_map::GridMap& map, const std::string& layer,
                                const grid_map::Index& centerIndex, float referenceHeight,
                                double radius, double heightThreshold, std::size_t minSupport);

bool hasConsistentLowerSurfaceSupport(
    const grid_map::GridMap& map, const std::string& heightLayer,
    const std::string& candidateLayer, const grid_map::Index& centerIndex,
    float referenceHeight, double radius, double heightThreshold,
    std::size_t minSupport, std::size_t minCandidateSupport);

bool updateAdaptiveLowerSurfaceState(AdaptiveLowerSurfaceState& state, float candidateHeight,
                                     double timestamp, bool hasSurfaceSupport,
                                     double heightThreshold, double maxTimeGap,
                                     int recoveryCount);

void resetAdaptiveLowerSurfaceState(AdaptiveLowerSurfaceState& state);

bool findEdgeSafeHoleFillSource(const grid_map::GridMap& map, const std::string& layer,
                                const grid_map::Index& centerIndex, double radius,
                                double heightThreshold, std::size_t minSupport,
                                grid_map::Index& sourceIndex);

}  // namespace elevation_mapping
