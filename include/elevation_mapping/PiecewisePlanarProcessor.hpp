#pragma once

#include <cstddef>

#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

enum class PiecewisePlanarConfidence : int {
  None = 0,
  Weak = 1,
  StablePlane = 2,
  HorizontalPlane = 3,
  DirectionalCompletion = 4,
};

struct PiecewisePlanarParameters {
  float neighborHeightTolerance;
  std::size_t minRegionSize;
  float maxRegionMad;
  float maxRegularizationResidual;
  float maxOcclusionDistance;
  std::size_t minOcclusionSupport;
  float inferredHalfRange;
  bool enableDirectionalGroundCompletion;
  float directionalGroundMaxGapWidth;
  float horizontalSnapMaxSlope;
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
    const PiecewisePlanarParameters& parameters,
    grid_map::Matrix* confidence = nullptr);

PiecewisePlanarResult processPiecewisePlanarElevation(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    const PiecewisePlanarParameters& parameters,
    grid_map::Matrix* confidence = nullptr);

}  // namespace elevation_mapping
