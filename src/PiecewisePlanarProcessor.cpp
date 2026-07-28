#include "elevation_mapping/PiecewisePlanarProcessor.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace elevation_mapping {
namespace {

using IndexKey = std::pair<int, int>;

struct SupportCell {
  grid_map::Index index;
  double x;
  double y;
  float elevation;
  float upperBound;
  float lowerBound;
};

struct Plane {
  Eigen::Vector3d coefficients;
  double residualMad;
};

struct AcceptedPlane {
  std::size_t regionId;
  Plane plane;
  std::vector<SupportCell> cells;
};

struct PlaneCandidate {
  std::size_t planeId;
  std::size_t consistentSupportCount;
  double nearestSquaredDistance;
  float predictedHeight;
};

struct CellOffset {
  grid_map::Position offset;
  double squaredDistance;
};

IndexKey makeKey(const grid_map::Index& index) {
  return {index(0), index(1)};
}

PiecewisePlanarResult emptyResult() {
  return {0u, 0u, 0u};
}

bool hasRequiredLayers(const grid_map::GridMap& map) {
  return map.exists("elevation") && map.exists("upper_bound") &&
         map.exists("lower_bound");
}

bool hasIdenticalGeometry(const grid_map::GridMap& first,
                          const grid_map::GridMap& second) {
  return first.getResolution() == second.getResolution() &&
         (first.getLength() == second.getLength()).all() &&
         first.getPosition() == second.getPosition() &&
         (first.getSize() == second.getSize()).all();
}

bool hasValidParameters(const PiecewisePlanarParameters& parameters) {
  return parameters.minRegionSize > 0u && parameters.minOcclusionSupport > 0u &&
         std::isfinite(parameters.neighborHeightTolerance) &&
         parameters.neighborHeightTolerance > 0.0f &&
         std::isfinite(parameters.maxRegionMad) &&
         parameters.maxRegionMad > 0.0f &&
         std::isfinite(parameters.maxRegularizationResidual) &&
         parameters.maxRegularizationResidual > 0.0f &&
         std::isfinite(parameters.maxOcclusionDistance) &&
         parameters.maxOcclusionDistance > 0.0f &&
         std::isfinite(parameters.inferredHalfRange) &&
         parameters.inferredHalfRange > 0.0f;
}

std::vector<grid_map::Index> getNeighbors(const grid_map::GridMap& map,
                                          const grid_map::Index& index) {
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

std::vector<grid_map::Index> getFourConnectedNeighbors(
    const grid_map::GridMap& map, const grid_map::Index& index) {
  grid_map::Position position;
  if (!map.getPosition(index, position)) {
    return {};
  }

  const double resolution = map.getResolution();
  const std::vector<grid_map::Position> offsets{
      {resolution, 0.0}, {-resolution, 0.0},
      {0.0, resolution}, {0.0, -resolution}};
  std::vector<grid_map::Index> neighbors;
  for (const auto& offset : offsets) {
    grid_map::Index neighbor;
    if (map.getIndex(position + offset, neighbor)) {
      neighbors.push_back(neighbor);
    }
  }
  return neighbors;
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2u;
  if (values.size() % 2u != 0u) {
    return values[middle];
  }
  return (values[middle - 1u] + values[middle]) / 2.0;
}

double residualMad(const std::vector<SupportCell>& cells,
                   const Eigen::Vector3d& coefficients) {
  std::vector<double> absoluteResiduals;
  absoluteResiduals.reserve(cells.size());
  for (const auto& cell : cells) {
    const double prediction =
        coefficients(0) * cell.x + coefficients(1) * cell.y + coefficients(2);
    absoluteResiduals.push_back(std::abs(cell.elevation - prediction));
  }
  return median(std::move(absoluteResiduals));
}

bool solveWeightedPlane(const std::vector<SupportCell>& cells,
                        const std::vector<double>& weights,
                        Eigen::Vector3d& coefficients) {
  Eigen::Matrix3d normalMatrix = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rightHandSide = Eigen::Vector3d::Zero();
  for (std::size_t index = 0u; index < cells.size(); ++index) {
    const auto& cell = cells[index];
    const Eigen::Vector3d row(cell.x, cell.y, 1.0);
    normalMatrix.noalias() += weights[index] * row * row.transpose();
    rightHandSide.noalias() += weights[index] * row * cell.elevation;
  }

  const Eigen::LDLT<Eigen::Matrix3d> decomposition(normalMatrix);
  if (decomposition.info() != Eigen::Success) {
    return false;
  }
  coefficients = decomposition.solve(rightHandSide);
  return decomposition.info() == Eigen::Success && coefficients.allFinite();
}

bool fitPlane(const std::vector<SupportCell>& cells, Plane& plane) {
  std::vector<double> weights(cells.size(), 1.0);
  if (!solveWeightedPlane(cells, weights, plane.coefficients)) {
    return false;
  }

  for (int iteration = 0; iteration < 5; ++iteration) {
    const double scale = std::max(1.4826 * residualMad(cells, plane.coefficients),
                                  1.0e-6);
    const double cutoff = 1.345 * scale;
    for (std::size_t index = 0u; index < cells.size(); ++index) {
      const auto& cell = cells[index];
      const double prediction = plane.coefficients(0) * cell.x +
                                plane.coefficients(1) * cell.y +
                                plane.coefficients(2);
      const double absoluteResidual = std::abs(cell.elevation - prediction);
      weights[index] = absoluteResidual <= cutoff ? 1.0 : cutoff / absoluteResidual;
    }
    if (!solveWeightedPlane(cells, weights, plane.coefficients)) {
      return false;
    }
  }

  plane.residualMad = residualMad(cells, plane.coefficients);
  return std::isfinite(plane.residualMad);
}

std::vector<SupportCell> snapshotSupportCells(
    const grid_map::GridMap& supportMap) {
  const std::vector<std::string> requiredLayers{
      "elevation", "upper_bound", "lower_bound"};
  std::vector<SupportCell> cells;
  for (grid_map::GridMapIterator iterator(supportMap); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index = *iterator;
    if (!supportMap.isValid(index, requiredLayers)) {
      continue;
    }
    grid_map::Position position;
    if (!supportMap.getPosition(index, position)) {
      continue;
    }
    cells.push_back({index, position.x(), position.y(),
                     supportMap.at("elevation", index),
                     supportMap.at("upper_bound", index),
                     supportMap.at("lower_bound", index)});
  }
  return cells;
}

std::vector<AcceptedPlane> findAcceptedPlanes(
    const grid_map::GridMap& supportMap,
    const std::vector<SupportCell>& supportCells,
    const PiecewisePlanarParameters& parameters) {
  std::map<IndexKey, std::size_t> cellsByIndex;
  for (std::size_t index = 0u; index < supportCells.size(); ++index) {
    cellsByIndex.emplace(makeKey(supportCells[index].index), index);
  }

  std::set<IndexKey> visited;
  std::vector<AcceptedPlane> acceptedPlanes;
  std::size_t regionId = 0u;
  for (const auto& start : supportCells) {
    if (!visited.insert(makeKey(start.index)).second) {
      continue;
    }

    std::vector<std::size_t> regionIndices;
    std::vector<std::size_t> queue{cellsByIndex.at(makeKey(start.index))};
    while (!queue.empty()) {
      const std::size_t currentIndex = queue.back();
      queue.pop_back();
      regionIndices.push_back(currentIndex);
      const auto& current = supportCells[currentIndex];
      for (const auto& neighborIndex : getNeighbors(supportMap, current.index)) {
        const auto found = cellsByIndex.find(makeKey(neighborIndex));
        if (found == cellsByIndex.end()) {
          continue;
        }
        const auto& neighbor = supportCells[found->second];
        if (std::abs(current.elevation - neighbor.elevation) >
                parameters.neighborHeightTolerance ||
            !visited.insert(found->first).second) {
          continue;
        }
        queue.push_back(found->second);
      }
    }

    if (regionIndices.size() < parameters.minRegionSize) {
      ++regionId;
      continue;
    }

    std::vector<SupportCell> region;
    region.reserve(regionIndices.size());
    for (const auto index : regionIndices) {
      region.push_back(supportCells[index]);
    }
    Plane plane;
    if (fitPlane(region, plane) && plane.residualMad <= parameters.maxRegionMad) {
      acceptedPlanes.push_back({regionId, plane, std::move(region)});
    }
    ++regionId;
  }
  return acceptedPlanes;
}

Eigen::MatrixXi buildSupportPlaneIds(
    const grid_map::GridMap& supportMap,
    const std::vector<AcceptedPlane>& acceptedPlanes,
    const PiecewisePlanarParameters& parameters) {
  const grid_map::Size size = supportMap.getSize();
  Eigen::MatrixXi supportPlaneIds =
      Eigen::MatrixXi::Constant(size(0), size(1), -1);
  for (std::size_t planeIndex = 0u; planeIndex < acceptedPlanes.size();
       ++planeIndex) {
    const auto& accepted = acceptedPlanes[planeIndex];
    for (const auto& cell : accepted.cells) {
      const double predictedHeight =
          accepted.plane.coefficients(0) * cell.x +
          accepted.plane.coefficients(1) * cell.y +
          accepted.plane.coefficients(2);
      if (std::abs(cell.elevation - predictedHeight) <=
          parameters.maxRegularizationResidual) {
        supportPlaneIds(cell.index(0), cell.index(1)) =
            static_cast<int>(planeIndex);
      }
    }
  }
  return supportPlaneIds;
}

std::vector<CellOffset> buildSearchOffsets(
    const grid_map::GridMap& map,
    const PiecewisePlanarParameters& parameters) {
  const double resolution = map.getResolution();
  const int cellRadius = static_cast<int>(
      std::ceil(parameters.maxOcclusionDistance / resolution));
  const double maximumSquaredDistance =
      parameters.maxOcclusionDistance * parameters.maxOcclusionDistance;
  std::vector<CellOffset> offsets;
  for (int xOffset = -cellRadius; xOffset <= cellRadius; ++xOffset) {
    for (int yOffset = -cellRadius; yOffset <= cellRadius; ++yOffset) {
      const grid_map::Position offset(xOffset * resolution,
                                      yOffset * resolution);
      const double squaredDistance = offset.squaredNorm();
      if (squaredDistance <= maximumSquaredDistance) {
        offsets.push_back({offset, squaredDistance});
      }
    }
  }
  return offsets;
}

std::vector<PlaneCandidate> findEligibleCandidates(
    const grid_map::GridMap& supportMap,
    const grid_map::Index& index,
    const Eigen::MatrixXi& supportPlaneIds,
    const std::vector<AcceptedPlane>& acceptedPlanes,
    const std::vector<CellOffset>& offsets,
    const PiecewisePlanarParameters& parameters) {
  grid_map::Position position;
  if (!supportMap.getPosition(index, position)) {
    return {};
  }

  std::map<std::size_t, PlaneCandidate> candidatesByPlane;
  for (const auto& offset : offsets) {
    grid_map::Index supportIndex;
    if (!supportMap.getIndex(position + offset.offset, supportIndex)) {
      continue;
    }
    const int planeIndex = supportPlaneIds(supportIndex(0), supportIndex(1));
    if (planeIndex < 0) {
      continue;
    }

    const std::size_t stablePlaneIndex = static_cast<std::size_t>(planeIndex);
    const auto found = candidatesByPlane.find(stablePlaneIndex);
    if (found == candidatesByPlane.end()) {
      candidatesByPlane.emplace(
          stablePlaneIndex,
          PlaneCandidate{acceptedPlanes[stablePlaneIndex].regionId, 1u,
                         offset.squaredDistance, 0.0f});
      continue;
    }
    ++found->second.consistentSupportCount;
    found->second.nearestSquaredDistance = std::min(
        found->second.nearestSquaredDistance, offset.squaredDistance);
  }

  std::vector<PlaneCandidate> candidates;
  candidates.reserve(candidatesByPlane.size());
  for (const auto& entry : candidatesByPlane) {
    const std::size_t planeIndex = entry.first;
    PlaneCandidate candidate = entry.second;
    if (candidate.consistentSupportCount < parameters.minOcclusionSupport) {
      continue;
    }
    const auto& plane = acceptedPlanes[planeIndex].plane;
    candidate.predictedHeight = static_cast<float>(
        plane.coefficients(0) * position.x() +
        plane.coefficients(1) * position.y() + plane.coefficients(2));
    candidates.push_back(candidate);
  }
  return candidates;
}

bool candidatesHaveDistinctHeights(const PlaneCandidate& first,
                                   const PlaneCandidate& second,
                                   const float heightTolerance) {
  return std::abs(first.predictedHeight - second.predictedHeight) >=
         heightTolerance;
}

std::vector<std::size_t> distinctHeightCandidateIndices(
    const std::vector<PlaneCandidate>& candidates,
    const float heightTolerance) {
  std::vector<std::size_t> indices;
  for (std::size_t firstIndex = 0u; firstIndex < candidates.size();
       ++firstIndex) {
    for (std::size_t secondIndex = firstIndex + 1u;
         secondIndex < candidates.size(); ++secondIndex) {
      if (!candidatesHaveDistinctHeights(candidates[firstIndex],
                                         candidates[secondIndex],
                                         heightTolerance)) {
        continue;
      }
      if (std::find(indices.begin(), indices.end(), firstIndex) == indices.end()) {
        indices.push_back(firstIndex);
      }
      if (std::find(indices.begin(), indices.end(), secondIndex) == indices.end()) {
        indices.push_back(secondIndex);
      }
    }
  }
  return indices;
}

std::size_t completeThinOcclusionBands(
    const grid_map::GridMap& supportMap, grid_map::GridMap& outputMap,
    const std::vector<AcceptedPlane>& acceptedPlanes,
    const PiecewisePlanarParameters& parameters) {
  const grid_map::Size size = supportMap.getSize();
  const Eigen::MatrixXi supportPlaneIds =
      buildSupportPlaneIds(supportMap, acceptedPlanes, parameters);
  const auto offsets = buildSearchOffsets(supportMap, parameters);
  Eigen::MatrixXi candidateMask = Eigen::MatrixXi::Zero(size(0), size(1));
  std::map<IndexKey, std::vector<PlaneCandidate>> candidatesByCell;
  const std::vector<std::string> requiredLayers{
      "elevation", "upper_bound", "lower_bound"};
  for (grid_map::GridMapIterator iterator(supportMap); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index = *iterator;
    if (supportMap.isValid(index, requiredLayers)) {
      continue;
    }
    auto candidates = findEligibleCandidates(
        supportMap, index, supportPlaneIds, acceptedPlanes, offsets, parameters);
    if (candidates.empty()) {
      continue;
    }
    candidateMask(index(0), index(1)) = 1;
    candidatesByCell.emplace(makeKey(index), std::move(candidates));
  }

  std::set<IndexKey> visited;
  std::size_t inferredCells = 0u;
  for (const auto& entry : candidatesByCell) {
    const grid_map::Index start(entry.first.first, entry.first.second);
    if (!visited.insert(entry.first).second) {
      continue;
    }

    std::vector<grid_map::Index> component;
    std::vector<grid_map::Index> queue{start};
    while (!queue.empty()) {
      const grid_map::Index current = queue.back();
      queue.pop_back();
      component.push_back(current);
      for (const auto& neighbor : getFourConnectedNeighbors(supportMap, current)) {
        if (candidateMask(neighbor(0), neighbor(1)) == 0 ||
            !visited.insert(makeKey(neighbor)).second) {
          continue;
        }
        queue.push_back(neighbor);
      }
    }

    bool hasDistinctHeightPair = false;
    for (const auto& index : component) {
      const auto& candidates = candidatesByCell.at(makeKey(index));
      if (!distinctHeightCandidateIndices(candidates,
                                          parameters.neighborHeightTolerance)
               .empty()) {
        hasDistinctHeightPair = true;
        break;
      }
    }
    if (!hasDistinctHeightPair) {
      continue;
    }

    for (const auto& index : component) {
      const auto& candidates = candidatesByCell.at(makeKey(index));
      const auto participatingCandidates = distinctHeightCandidateIndices(
          candidates, parameters.neighborHeightTolerance);
      if (participatingCandidates.empty()) {
        continue;
      }
      std::size_t selectedIndex = participatingCandidates.front();
      for (const auto candidateIndex : participatingCandidates) {
        const auto& selected = candidates[selectedIndex];
        const auto& candidate = candidates[candidateIndex];
        if (candidate.nearestSquaredDistance < selected.nearestSquaredDistance ||
            (candidate.nearestSquaredDistance == selected.nearestSquaredDistance &&
             candidate.planeId < selected.planeId)) {
          selectedIndex = candidateIndex;
        }
      }
      const auto& selected = candidates[selectedIndex];
      outputMap.at("elevation", index) = selected.predictedHeight;
      outputMap.at("lower_bound", index) =
          selected.predictedHeight - parameters.inferredHalfRange;
      outputMap.at("upper_bound", index) =
          selected.predictedHeight + parameters.inferredHalfRange;
      ++inferredCells;
    }
  }
  return inferredCells;
}

}  // namespace

PiecewisePlanarResult processPiecewisePlanarElevationIfEnabled(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    const bool enabled,
    const PiecewisePlanarParameters& parameters) {
  if (!enabled) {
    return emptyResult();
  }
  return processPiecewisePlanarElevation(supportMap, outputMap, parameters);
}

PiecewisePlanarResult processPiecewisePlanarElevation(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    const PiecewisePlanarParameters& parameters) {
  if (!hasRequiredLayers(supportMap) || !hasRequiredLayers(outputMap) ||
      !hasIdenticalGeometry(supportMap, outputMap) ||
      !hasValidParameters(parameters)) {
    return emptyResult();
  }

  const auto supportCells = snapshotSupportCells(supportMap);
  const auto acceptedPlanes =
      findAcceptedPlanes(supportMap, supportCells, parameters);

  PiecewisePlanarResult result{acceptedPlanes.size(), 0u, 0u};
  for (const auto& accepted : acceptedPlanes) {
    for (const auto& cell : accepted.cells) {
      const float predictedHeight = static_cast<float>(
          accepted.plane.coefficients(0) * cell.x +
          accepted.plane.coefficients(1) * cell.y +
          accepted.plane.coefficients(2));
      if (std::abs(cell.elevation - predictedHeight) >
          parameters.maxRegularizationResidual) {
        continue;
      }
      grid_map::Index outputIndex;
      if (!outputMap.getIndex(grid_map::Position(cell.x, cell.y), outputIndex)) {
        continue;
      }
      const float correction = predictedHeight - cell.elevation;
      outputMap.at("elevation", outputIndex) = predictedHeight;
      outputMap.at("upper_bound", outputIndex) = cell.upperBound + correction;
      outputMap.at("lower_bound", outputIndex) = cell.lowerBound + correction;
      ++result.regularizedCells;
    }
  }
  result.inferredCells = completeThinOcclusionBands(
      supportMap, outputMap, acceptedPlanes, parameters);
  return result;
}

}  // namespace elevation_mapping
