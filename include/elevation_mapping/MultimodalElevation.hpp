#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
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

struct SurfaceModeState {
  bool valid{false};
  float height{std::numeric_limits<float>::quiet_NaN()};
  float variance{std::numeric_limits<float>::quiet_NaN()};
  float color{std::numeric_limits<float>::quiet_NaN()};
  float lowestPointHeight{std::numeric_limits<float>::quiet_NaN()};
  float confidence{0.0f};
  double lastSeen{std::numeric_limits<double>::quiet_NaN()};
  std::size_t coverageBins{0};
  std::size_t consecutiveObservations{0};
  bool centerOccupied{false};
};

struct MultimodalCellState {
  std::array<SurfaceModeState, 2> modes;
  int primaryIndex{-1};
  int challengerIndex{-1};
  int challengerCount{0};
};

struct MultimodalUpdateResult {
  bool handled{false};
  bool primarySwitched{false};
  SurfaceModeState primary;
  SurfaceModeState secondary;
  std::size_t modeCount{0};
};

bool isValidMultimodalConfig(const MultimodalConfig& config);
std::size_t countOccupiedBins(std::uint16_t mask);
CellObservation buildCellObservation(
    const std::vector<MultimodalPoint>& points, float cellCenterX,
    float cellCenterY, float cellResolution, const MultimodalConfig& config);
MultimodalUpdateResult updateMultimodalCell(
    MultimodalCellState& state, const CellObservation& observation,
    double timestamp, const MultimodalConfig& config);
std::size_t countValidModes(const MultimodalCellState& state);
bool expireStaleModes(
    MultimodalCellState& state, double timestamp,
    const MultimodalConfig& config);

}  // namespace elevation_mapping
