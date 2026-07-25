#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace elevation_mapping {

struct MultimodalConfig {
  float modeSeparation{0.05f};
  std::size_t minPoints{3};
  std::size_t minBins{2};
  std::size_t switchMarginBins{1};
  int switchConfirmations{3};
  double staleTimeout{0.5};
};

struct MultimodalPoint {
  float x;
  float y;
  float height;
  float variance;
  float color;
};

struct SurfaceObservation {
  float height;
  float variance;
  float color;
  float lowestPointHeight;
  std::size_t pointCount;
  std::uint16_t xyMask;
  bool centerOccupied;
};

struct CellObservation {
  std::array<SurfaceObservation, 2> modes;
  std::size_t modeCount;
};

bool isValidMultimodalConfig(const MultimodalConfig& config);
std::size_t countOccupiedBins(std::uint16_t mask);
CellObservation buildCellObservation(
    const std::vector<MultimodalPoint>& points, float cellCenterX,
    float cellCenterY, float cellResolution, const MultimodalConfig& config);

}  // namespace elevation_mapping
