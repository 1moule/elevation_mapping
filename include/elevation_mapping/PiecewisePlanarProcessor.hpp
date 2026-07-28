#pragma once

#include <cstddef>

#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

struct PiecewisePlanarParameters {
  float neighborHeightTolerance;
  std::size_t minRegionSize;
  float maxRegionMad;
  float maxRegularizationResidual;
  float maxOcclusionDistance;
  std::size_t minOcclusionSupport;
  float inferredHalfRange;
};

struct PiecewisePlanarResult {
  std::size_t acceptedPlanes;
  std::size_t regularizedCells;
  std::size_t inferredCells;
};

constexpr bool defaultPiecewisePlanarRegularizationEnabled() {
  return false;
}

PiecewisePlanarResult processPiecewisePlanarElevationIfEnabled(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    bool enabled,
    const PiecewisePlanarParameters& parameters);

PiecewisePlanarResult processPiecewisePlanarElevation(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    const PiecewisePlanarParameters& parameters);

}  // namespace elevation_mapping
