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

bool isModeIndex(int index) {
  return index >= 0 && index < 2;
}

bool isFiniteObservation(const SurfaceObservation& observation) {
  return std::isfinite(observation.height) &&
         std::isfinite(observation.variance) && observation.variance > 0.0f &&
         std::isfinite(observation.color) &&
         std::isfinite(observation.lowestPointHeight);
}

void clearMode(SurfaceModeState& mode) {
  mode = SurfaceModeState{};
}

bool modeHasHigherRank(const SurfaceModeState& left,
                       const SurfaceModeState& right) {
  if (left.centerOccupied != right.centerOccupied) {
    return left.centerOccupied;
  }
  return left.coverageBins > right.coverageBins;
}

bool challengerWins(const SurfaceModeState& challenger,
                    const SurfaceModeState& incumbent,
                    const MultimodalConfig& config) {
  if (challenger.centerOccupied != incumbent.centerOccupied) {
    return challenger.centerOccupied;
  }
  return challenger.coverageBins >=
         incumbent.coverageBins + config.switchMarginBins;
}

bool observationLess(const SurfaceObservation& left,
                     const SurfaceObservation& right) {
  if (left.height != right.height) {
    return left.height < right.height;
  }
  if (left.variance != right.variance) {
    return left.variance < right.variance;
  }
  if (left.color != right.color) {
    return left.color < right.color;
  }
  if (left.lowestPointHeight != right.lowestPointHeight) {
    return left.lowestPointHeight < right.lowestPointHeight;
  }
  if (left.pointCount != right.pointCount) {
    return left.pointCount < right.pointCount;
  }
  if (left.xyMask != right.xyMask) {
    return left.xyMask < right.xyMask;
  }
  return left.centerOccupied < right.centerOccupied;
}

struct ModeAssignment {
  std::array<int, 2> observationToMode{{-1, -1}};
  std::size_t matchCount{0};
  float totalDistance{0.0f};
};

bool isBetterAssignment(const ModeAssignment& candidate,
                        const ModeAssignment& best,
                        const std::array<int, 2>& observationOrder,
                        std::size_t observationCount) {
  if (candidate.matchCount != best.matchCount) {
    return candidate.matchCount > best.matchCount;
  }
  if (candidate.totalDistance != best.totalDistance) {
    return candidate.totalDistance < best.totalDistance;
  }

  for (std::size_t index = 0; index < observationCount; ++index) {
    const int observationIndex = observationOrder[index];
    const int candidateMode = candidate.observationToMode[observationIndex] < 0
                                  ? 2
                                  : candidate.observationToMode[observationIndex];
    const int bestMode = best.observationToMode[observationIndex] < 0
                             ? 2
                             : best.observationToMode[observationIndex];
    if (candidateMode != bestMode) {
      return candidateMode < bestMode;
    }
  }
  return false;
}

void findBestAssignment(const MultimodalCellState& state,
                        const CellObservation& observation,
                        const MultimodalConfig& config,
                        const std::array<int, 2>& observationOrder,
                        std::size_t observationCount,
                        std::size_t observationPosition,
                        std::array<bool, 2>& usedModes,
                        ModeAssignment& candidate, ModeAssignment& best) {
  if (observationPosition == observationCount) {
    if (isBetterAssignment(candidate, best, observationOrder, observationCount)) {
      best = candidate;
    }
    return;
  }

  const int observationIndex = observationOrder[observationPosition];
  findBestAssignment(state, observation, config, observationOrder,
                     observationCount, observationPosition + 1, usedModes,
                     candidate, best);

  for (int modeIndex = 0; modeIndex < 2; ++modeIndex) {
    if (!state.modes[modeIndex].valid || usedModes[modeIndex]) {
      continue;
    }
    const float distance = std::fabs(
        state.modes[modeIndex].height - observation.modes[observationIndex].height);
    if (distance > config.modeSeparation) {
      continue;
    }

    usedModes[modeIndex] = true;
    candidate.observationToMode[observationIndex] = modeIndex;
    ++candidate.matchCount;
    candidate.totalDistance += distance;
    findBestAssignment(state, observation, config, observationOrder,
                       observationCount, observationPosition + 1, usedModes,
                       candidate, best);
    candidate.totalDistance -= distance;
    --candidate.matchCount;
    candidate.observationToMode[observationIndex] = -1;
    usedModes[modeIndex] = false;
  }
}

ModeAssignment findBestAssignment(const MultimodalCellState& state,
                                  const CellObservation& observation,
                                  const MultimodalConfig& config) {
  std::array<int, 2> observationOrder{{-1, -1}};
  std::size_t observationCount = 0;
  for (std::size_t index = 0; index < observation.modeCount; ++index) {
    if (isFiniteObservation(observation.modes[index])) {
      observationOrder[observationCount++] = static_cast<int>(index);
    }
  }
  std::sort(observationOrder.begin(), observationOrder.begin() + observationCount,
            [&observation](int left, int right) {
              return observationLess(observation.modes[left],
                                     observation.modes[right]);
            });

  ModeAssignment candidate;
  ModeAssignment best;
  std::array<bool, 2> usedModes{{false, false}};
  findBestAssignment(state, observation, config, observationOrder,
                     observationCount, 0, usedModes, candidate, best);
  return best;
}

void clearChallengerIfNeeded(MultimodalCellState& state, int modeIndex) {
  if (state.challengerIndex == modeIndex) {
    state.challengerIndex = -1;
    state.challengerCount = 0;
  }
}

}  // namespace

bool isValidMultimodalConfig(const MultimodalConfig& config) {
  return std::isfinite(config.modeSeparation) && config.modeSeparation > 0.0f &&
         config.minPoints >= 3 && config.minBins >= 2 && config.minBins <= 9 &&
         config.switchMarginBins >= 1 && config.switchMarginBins <= 9 &&
         config.switchConfirmations >= 2 &&
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

std::size_t countValidModes(const MultimodalCellState& state) {
  std::size_t count = 0;
  for (const auto& mode : state.modes) {
    count += mode.valid ? 1u : 0u;
  }
  return count;
}

bool expireStaleModes(MultimodalCellState& state, double timestamp,
                      const MultimodalConfig& config) {
  if (!isValidMultimodalConfig(config) || !std::isfinite(timestamp)) {
    return false;
  }

  bool expired = false;
  for (int index = 0; index < 2; ++index) {
    if (!state.modes[index].valid) {
      continue;
    }
    const double lastSeen = state.modes[index].lastSeen;
    if (!std::isfinite(lastSeen) || timestamp - lastSeen > config.staleTimeout) {
      clearMode(state.modes[index]);
      clearChallengerIfNeeded(state, index);
      expired = true;
    }
  }
  if (expired &&
      (!isModeIndex(state.primaryIndex) ||
       !state.modes[state.primaryIndex].valid)) {
    state.primaryIndex = -1;
    for (int index = 0; index < 2; ++index) {
      if (state.modes[index].valid &&
          (state.primaryIndex == -1 ||
           modeHasHigherRank(state.modes[index],
                             state.modes[state.primaryIndex]))) {
        state.primaryIndex = index;
      }
    }
    state.challengerIndex = -1;
    state.challengerCount = 0;
  }
  return expired;
}

MultimodalUpdateResult updateMultimodalCell(
    MultimodalCellState& state, const CellObservation& observation,
    double timestamp, const MultimodalConfig& config) {
  MultimodalUpdateResult result;
  if (!isValidMultimodalConfig(config) || !std::isfinite(timestamp) ||
      observation.modeCount > observation.modes.size()) {
    return result;
  }

  const std::size_t validBefore = countValidModes(state);
  if (observation.modeCount == 1 && validBefore == 0) {
    return result;
  }

  if (observation.modeCount == 0) {
    state.challengerIndex = -1;
    state.challengerCount = 0;
    for (auto& mode : state.modes) {
      if (!mode.valid) {
        continue;
      }
      mode.confidence = std::max(0.0f, mode.confidence - 0.25f);
      mode.consecutiveObservations = 0;
    }
    expireStaleModes(state, timestamp, config);
    const std::size_t modeCount = countValidModes(state);
    if (modeCount == 0) {
      return result;
    }
    if (!isModeIndex(state.primaryIndex) ||
        !state.modes[state.primaryIndex].valid) {
      state.primaryIndex = state.modes[0].valid ? 0 : 1;
    }
    result.handled = true;
    result.modeCount = modeCount;
    result.primary = state.modes[state.primaryIndex];
    if (modeCount == 2) {
      const int secondaryIndex = state.primaryIndex == 0 ? 1 : 0;
      result.secondary = state.modes[secondaryIndex];
    }
    return result;
  }

  const bool seenAtTimestamp[] = {
      state.modes[0].valid && state.modes[0].lastSeen == timestamp,
      state.modes[1].valid && state.modes[1].lastSeen == timestamp,
  };
  expireStaleModes(state, timestamp, config);

  const ModeAssignment assignment = findBestAssignment(state, observation, config);
  int observationMatches[] = {assignment.observationToMode[0],
                              assignment.observationToMode[1]};
  bool modeObserved[] = {false, false};

  for (std::size_t observationIndex = 0;
       observationIndex < observation.modeCount; ++observationIndex) {
    const auto& current = observation.modes[observationIndex];
    if (!isFiniteObservation(current)) {
      continue;
    }

    int modeIndex = observationMatches[observationIndex];
    if (modeIndex == -1) {
      for (int candidate = 0; candidate < 2; ++candidate) {
        if (!state.modes[candidate].valid) {
          modeIndex = candidate;
          break;
        }
      }
      if (modeIndex == -1) {
        continue;
      }
      state.modes[modeIndex].valid = true;
      state.modes[modeIndex].height = current.height;
      state.modes[modeIndex].variance = current.variance;
      state.modes[modeIndex].confidence = 0.0f;
      state.modes[modeIndex].consecutiveObservations = 0;
    } else {
      auto& mode = state.modes[modeIndex];
      const float gain = mode.variance / (mode.variance + current.variance);
      mode.height += gain * (current.height - mode.height);
      mode.variance *= 1.0f - gain;
    }

    auto& mode = state.modes[modeIndex];
    mode.color = current.color;
    mode.lowestPointHeight = current.lowestPointHeight;
    mode.confidence = std::min(1.0f, mode.confidence + 0.25f);
    mode.lastSeen = timestamp;
    mode.coverageBins = countOccupiedBins(current.xyMask);
    mode.centerOccupied = current.centerOccupied;
    mode.sensorX = current.sensorX;
    mode.sensorY = current.sensorY;
    mode.sensorZ = current.sensorZ;
    mode.consecutiveObservations += 1;
    modeObserved[modeIndex] = true;
  }

  for (int modeIndex = 0; modeIndex < 2; ++modeIndex) {
    auto& mode = state.modes[modeIndex];
    if (!mode.valid || modeObserved[modeIndex]) {
      continue;
    }
    mode.confidence = std::max(0.0f, mode.confidence - 0.25f);
    mode.consecutiveObservations = 0;
  }
  expireStaleModes(state, timestamp, config);

  const std::size_t modeCount = countValidModes(state);
  if (modeCount == 0) {
    return result;
  }

  if (!isModeIndex(state.primaryIndex) ||
      !state.modes[state.primaryIndex].valid) {
    int primary = -1;
    for (int index = 0; index < 2; ++index) {
      if (state.modes[index].valid &&
          (primary == -1 || modeHasHigherRank(state.modes[index],
                                               state.modes[primary]))) {
        primary = index;
      }
    }
    state.primaryIndex = primary;
    state.challengerIndex = -1;
    state.challengerCount = 0;
  } else if (modeCount == 2) {
    const int challengerIndex = state.primaryIndex == 0 ? 1 : 0;
    if (modeObserved[challengerIndex] &&
        challengerWins(state.modes[challengerIndex],
                       state.modes[state.primaryIndex], config)) {
      if (state.challengerIndex != challengerIndex) {
        state.challengerIndex = challengerIndex;
        state.challengerCount = seenAtTimestamp[challengerIndex] ? 0 : 1;
      } else if (!seenAtTimestamp[challengerIndex]) {
        ++state.challengerCount;
      }
      if (state.challengerCount >= config.switchConfirmations) {
        state.primaryIndex = challengerIndex;
        state.challengerIndex = -1;
        state.challengerCount = 0;
        result.primarySwitched = true;
      }
    } else {
      state.challengerIndex = -1;
      state.challengerCount = 0;
    }
  } else {
    state.challengerIndex = -1;
    state.challengerCount = 0;
  }

  result.handled = true;
  result.modeCount = modeCount;
  result.primary = state.modes[state.primaryIndex];
  if (modeCount == 2) {
    const int secondaryIndex = state.primaryIndex == 0 ? 1 : 0;
    result.secondary = state.modes[secondaryIndex];
  }
  return result;
}

}  // namespace elevation_mapping
