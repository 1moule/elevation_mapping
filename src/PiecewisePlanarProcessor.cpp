#include "elevation_mapping/PiecewisePlanarProcessor.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
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
  grid_map::Position nearestSupportPosition;
  float predictedHeight;
};

struct CellOffset {
  grid_map::Position offset;
  double squaredDistance;
};

struct ComponentContact {
  grid_map::Position componentPosition;
  grid_map::Position supportPosition;
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

Eigen::Vector2d supportCentroid(const std::vector<SupportCell>& cells) {
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& cell : cells) {
    centroid += Eigen::Vector2d(cell.x, cell.y);
  }
  return centroid / static_cast<double>(cells.size());
}

bool hasSufficientTwoDimensionalSpan(
    const std::vector<SupportCell>& cells, const double resolution,
    const Eigen::Vector2d& centroid) {
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& cell : cells) {
    const Eigen::Vector2d centered(cell.x - centroid.x(),
                                   cell.y - centroid.y());
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(cells.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigenSolver(covariance);
  if (eigenSolver.info() != Eigen::Success ||
      !eigenSolver.eigenvalues().allFinite()) {
    return false;
  }
  constexpr double kMinimumVarianceInResolutionSquared = 0.2;
  return eigenSolver.eigenvalues().minCoeff() >=
         kMinimumVarianceInResolutionSquared * resolution * resolution;
}

bool solveWeightedPlane(const std::vector<SupportCell>& cells,
                        const std::vector<double>& weights,
                        const Eigen::Vector2d& centroid,
                        Eigen::Vector3d& coefficients) {
  Eigen::MatrixXd weightedDesign(cells.size(), 3);
  Eigen::VectorXd weightedElevation(cells.size());
  for (std::size_t index = 0u; index < cells.size(); ++index) {
    const auto& cell = cells[index];
    const double squareRootWeight = std::sqrt(weights[index]);
    weightedDesign.row(index) =
        squareRootWeight *
        Eigen::RowVector3d(cell.x - centroid.x(),
                           cell.y - centroid.y(), 1.0);
    weightedElevation(index) = squareRootWeight * cell.elevation;
  }

  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(
      weightedDesign);
  if (decomposition.rank() != 3) {
    return false;
  }
  const Eigen::Vector3d centeredCoefficients =
      decomposition.solve(weightedElevation);
  if (!centeredCoefficients.allFinite()) {
    return false;
  }
  coefficients(0) = centeredCoefficients(0);
  coefficients(1) = centeredCoefficients(1);
  coefficients(2) = centeredCoefficients(2) -
                    centeredCoefficients(0) * centroid.x() -
                    centeredCoefficients(1) * centroid.y();
  return coefficients.allFinite();
}

bool fitPlane(const std::vector<SupportCell>& cells, const double resolution,
              Plane& plane) {
  const Eigen::Vector2d centroid = supportCentroid(cells);
  if (!hasSufficientTwoDimensionalSpan(cells, resolution, centroid)) {
    return false;
  }
  std::vector<double> weights(cells.size(), 1.0);
  if (!solveWeightedPlane(cells, weights, centroid, plane.coefficients)) {
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
    if (!solveWeightedPlane(cells, weights, centroid, plane.coefficients)) {
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
    if (fitPlane(region, supportMap.getResolution(), plane) &&
        plane.residualMad <= parameters.maxRegionMad) {
      acceptedPlanes.push_back({regionId, plane, std::move(region)});
    }
    ++regionId;
  }

  const auto canonicalCell = [](const AcceptedPlane& accepted) {
    return std::min_element(
        accepted.cells.begin(), accepted.cells.end(),
        [](const SupportCell& first, const SupportCell& second) {
          return first.x < second.x ||
                 (first.x == second.x && first.y < second.y);
        });
  };
  std::sort(acceptedPlanes.begin(), acceptedPlanes.end(),
            [&canonicalCell](const AcceptedPlane& first,
                             const AcceptedPlane& second) {
              const auto firstCell = canonicalCell(first);
              const auto secondCell = canonicalCell(second);
              return firstCell->x < secondCell->x ||
                     (firstCell->x == secondCell->x &&
                      firstCell->y < secondCell->y);
            });
  for (std::size_t planeId = 0u; planeId < acceptedPlanes.size(); ++planeId) {
    acceptedPlanes[planeId].regionId = planeId;
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
    const grid_map::GridMap& map, const float maximumDistance) {
  const double resolution = map.getResolution();
  const double requestedCellRadius =
      std::ceil(static_cast<double>(maximumDistance) / resolution);
  const grid_map::Size mapSize = map.getSize();
  const int xCellRadius = static_cast<int>(std::min(
      requestedCellRadius,
      static_cast<double>(std::max(0, mapSize(0) - 1))));
  const int yCellRadius = static_cast<int>(std::min(
      requestedCellRadius,
      static_cast<double>(std::max(0, mapSize(1) - 1))));
  const double maximumSquaredDistance =
      static_cast<double>(maximumDistance) *
      static_cast<double>(maximumDistance);
  std::vector<CellOffset> offsets;
  for (int xOffset = -xCellRadius; xOffset <= xCellRadius; ++xOffset) {
    for (int yOffset = -yCellRadius; yOffset <= yCellRadius; ++yOffset) {
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

bool isLexicographicallySmaller(const grid_map::Position& first,
                                const grid_map::Position& second) {
  return first.x() < second.x() ||
         (first.x() == second.x() && first.y() < second.y());
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
    grid_map::Position supportPosition;
    if (!supportMap.getPosition(supportIndex, supportPosition)) {
      continue;
    }

    const std::size_t stablePlaneIndex = static_cast<std::size_t>(planeIndex);
    const auto found = candidatesByPlane.find(stablePlaneIndex);
    if (found == candidatesByPlane.end()) {
      candidatesByPlane.emplace(
          stablePlaneIndex,
          PlaneCandidate{acceptedPlanes[stablePlaneIndex].regionId, 1u,
                         offset.squaredDistance, supportPosition, 0.0f});
      continue;
    }
    ++found->second.consistentSupportCount;
    if (offset.squaredDistance < found->second.nearestSquaredDistance ||
        (offset.squaredDistance == found->second.nearestSquaredDistance &&
         isLexicographicallySmaller(supportPosition,
                                    found->second.nearestSupportPosition))) {
      found->second.nearestSquaredDistance = offset.squaredDistance;
      found->second.nearestSupportPosition = supportPosition;
    }
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

bool contactsBracketFromOpposingDirections(
    const std::vector<ComponentContact>& firstContacts,
    const std::vector<ComponentContact>& secondContacts) {
  constexpr double kOpposingDirectionTolerance = 1.0e-9;
  for (const auto& first : firstContacts) {
    const grid_map::Position firstDirection =
        first.supportPosition - first.componentPosition;
    for (const auto& second : secondContacts) {
      const grid_map::Position secondDirection =
          second.supportPosition - second.componentPosition;
      const double normProduct =
          firstDirection.norm() * secondDirection.norm();
      if (normProduct > 0.0 &&
          firstDirection.dot(secondDirection) <
              -kOpposingDirectionTolerance * normProduct) {
        return true;
      }
    }
  }
  return false;
}

float predictHeightAt(const AcceptedPlane& accepted,
                      const grid_map::Position& position) {
  return static_cast<float>(
      accepted.plane.coefficients(0) * position.x() +
      accepted.plane.coefficients(1) * position.y() +
      accepted.plane.coefficients(2));
}

void writeInferredCell(const grid_map::GridMap& supportMap,
                       grid_map::GridMap& outputMap,
                       const grid_map::Index& supportIndex,
                       const float predictedHeight,
                       const PiecewisePlanarParameters& parameters) {
  grid_map::Position position;
  if (!supportMap.getPosition(supportIndex, position)) {
    return;
  }
  grid_map::Index outputIndex;
  if (!outputMap.getIndex(position, outputIndex)) {
    return;
  }
  outputMap.at("elevation", outputIndex) = predictedHeight;
  outputMap.at("lower_bound", outputIndex) =
      predictedHeight - parameters.inferredHalfRange;
  outputMap.at("upper_bound", outputIndex) =
      predictedHeight + parameters.inferredHalfRange;
}

void completeThinOcclusionBands(
    const grid_map::GridMap& supportMap, grid_map::GridMap& outputMap,
    const std::vector<AcceptedPlane>& acceptedPlanes,
    const PiecewisePlanarParameters& parameters,
    std::set<IndexKey>& inferredSupportIndices) {
  const grid_map::Size size = supportMap.getSize();
  const Eigen::MatrixXi supportPlaneIds =
      buildSupportPlaneIds(supportMap, acceptedPlanes, parameters);
  const auto offsets =
      buildSearchOffsets(supportMap, parameters.maxOcclusionDistance);
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

    std::sort(component.begin(), component.end(),
              [&supportMap](const grid_map::Index& first,
                            const grid_map::Index& second) {
                grid_map::Position firstPosition;
                grid_map::Position secondPosition;
                supportMap.getPosition(first, firstPosition);
                supportMap.getPosition(second, secondPosition);
                return isLexicographicallySmaller(firstPosition,
                                                  secondPosition);
              });

    std::map<std::size_t, std::vector<ComponentContact>> contactsByPlane;
    grid_map::Position componentCenter = grid_map::Position::Zero();
    for (const auto& index : component) {
      grid_map::Position componentPosition;
      if (!supportMap.getPosition(index, componentPosition)) {
        continue;
      }
      componentCenter += componentPosition;
      const auto& candidates = candidatesByCell.at(makeKey(index));
      for (const auto& candidate : candidates) {
        const ComponentContact contact{
            componentPosition, candidate.nearestSupportPosition,
            candidate.nearestSquaredDistance};
        auto& contacts = contactsByPlane[candidate.planeId];
        if (contacts.empty() ||
            contact.squaredDistance < contacts.front().squaredDistance) {
          contacts.clear();
          contacts.push_back(contact);
        } else if (contact.squaredDistance ==
                   contacts.front().squaredDistance) {
          contacts.push_back(contact);
        }
      }
    }
    componentCenter /= static_cast<double>(component.size());

    std::set<std::size_t> participatingPlaneIds;
    for (auto first = contactsByPlane.begin(); first != contactsByPlane.end();
         ++first) {
      for (auto second = std::next(first); second != contactsByPlane.end();
           ++second) {
        const PlaneCandidate firstAtCenter{
            first->first, 0u, 0.0, grid_map::Position::Zero(),
            predictHeightAt(acceptedPlanes[first->first], componentCenter)};
        const PlaneCandidate secondAtCenter{
            second->first, 0u, 0.0, grid_map::Position::Zero(),
            predictHeightAt(acceptedPlanes[second->first], componentCenter)};
        if (!candidatesHaveDistinctHeights(
                firstAtCenter, secondAtCenter,
                parameters.neighborHeightTolerance) ||
            !contactsBracketFromOpposingDirections(
                first->second, second->second)) {
          continue;
        }
        participatingPlaneIds.insert(first->first);
        participatingPlaneIds.insert(second->first);
      }
    }
    if (participatingPlaneIds.empty()) {
      continue;
    }

    for (const auto& index : component) {
      const auto& candidates = candidatesByCell.at(makeKey(index));
      std::size_t selectedIndex = candidates.size();
      for (std::size_t candidateIndex = 0u;
           candidateIndex < candidates.size(); ++candidateIndex) {
        if (participatingPlaneIds.count(
                candidates[candidateIndex].planeId) == 0u) {
          continue;
        }
        if (selectedIndex == candidates.size()) {
          selectedIndex = candidateIndex;
          continue;
        }
        const auto& selected = candidates[selectedIndex];
        const auto& candidate = candidates[candidateIndex];
        if (candidate.nearestSquaredDistance < selected.nearestSquaredDistance ||
            (candidate.nearestSquaredDistance == selected.nearestSquaredDistance &&
             candidate.planeId < selected.planeId)) {
          selectedIndex = candidateIndex;
        }
      }
      if (selectedIndex == candidates.size()) {
        continue;
      }
      const auto& selected = candidates[selectedIndex];
      writeInferredCell(supportMap, outputMap, index, selected.predictedHeight,
                        parameters);
      inferredSupportIndices.insert(makeKey(index));
    }
  }
}

void completeDirectionalGroundGaps(
    const grid_map::GridMap& supportMap, grid_map::GridMap& outputMap,
    const std::vector<AcceptedPlane>& acceptedPlanes,
    const PiecewisePlanarParameters& parameters,
    std::set<IndexKey>& inferredSupportIndices) {
  if (!parameters.enableDirectionalGroundCompletion ||
      !std::isfinite(parameters.directionalGroundMaxGapWidth) ||
      parameters.directionalGroundMaxGapWidth <= 0.0f) {
    return;
  }

  const Eigen::MatrixXi supportPlaneIds =
      buildSupportPlaneIds(supportMap, acceptedPlanes, parameters);
  const auto offsets = buildSearchOffsets(
      supportMap, parameters.directionalGroundMaxGapWidth);
  const grid_map::Position observer = supportMap.getPosition();
  const double maximumSquaredGap =
      parameters.directionalGroundMaxGapWidth *
      parameters.directionalGroundMaxGapWidth;
  const double maximumLateralDistance =
      std::sqrt(2.0) * supportMap.getResolution();
  const std::vector<std::string> requiredLayers{
      "elevation", "upper_bound", "lower_bound"};

  for (grid_map::GridMapIterator iterator(supportMap); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index = *iterator;
    if (supportMap.isValid(index, requiredLayers)) {
      continue;
    }
    grid_map::Position cellPosition;
    if (!supportMap.getPosition(index, cellPosition)) {
      continue;
    }
    const auto candidates = findEligibleCandidates(
        supportMap, index, supportPlaneIds, acceptedPlanes, offsets, parameters);

    const PlaneCandidate* selectedHigh = nullptr;
    const PlaneCandidate* selectedLow = nullptr;
    double selectedDistance = 0.0;
    for (std::size_t firstIndex = 0u; firstIndex < candidates.size();
         ++firstIndex) {
      for (std::size_t secondIndex = firstIndex + 1u;
           secondIndex < candidates.size(); ++secondIndex) {
        const auto& first = candidates[firstIndex];
        const auto& second = candidates[secondIndex];
        if (!candidatesHaveDistinctHeights(
                first, second, parameters.neighborHeightTolerance)) {
          continue;
        }
        const PlaneCandidate& high =
            first.predictedHeight > second.predictedHeight ? first : second;
        const PlaneCandidate& low =
            first.predictedHeight > second.predictedHeight ? second : first;
        const grid_map::Position delta =
            low.nearestSupportPosition - high.nearestSupportPosition;
        const double squaredGap = delta.squaredNorm();
        if (squaredGap <= 0.0 || squaredGap > maximumSquaredGap) {
          continue;
        }
        const double observerProjection =
            (observer - high.nearestSupportPosition).dot(delta) / squaredGap;
        const double cellProjection =
            (cellPosition - high.nearestSupportPosition).dot(delta) / squaredGap;
        const grid_map::Position projected =
            high.nearestSupportPosition + cellProjection * delta;
        const double lateralDistance = (cellPosition - projected).norm();
        if (observerProjection >= 0.0 || cellProjection < 0.0 ||
            cellProjection > 1.0 || lateralDistance > maximumLateralDistance) {
          continue;
        }

        const double distance = std::sqrt(high.nearestSquaredDistance) +
                                std::sqrt(low.nearestSquaredDistance);
        if (selectedHigh == nullptr || distance < selectedDistance ||
            (distance == selectedDistance &&
             (high.planeId < selectedHigh->planeId ||
              (high.planeId == selectedHigh->planeId &&
               low.planeId < selectedLow->planeId)))) {
          selectedHigh = &high;
          selectedLow = &low;
          selectedDistance = distance;
        }
      }
    }
    if (selectedLow == nullptr) {
      continue;
    }
    writeInferredCell(supportMap, outputMap, index,
                      selectedLow->predictedHeight, parameters);
    inferredSupportIndices.insert(makeKey(index));
  }
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
  std::set<IndexKey> inferredSupportIndices;
  completeThinOcclusionBands(supportMap, outputMap, acceptedPlanes, parameters,
                             inferredSupportIndices);
  completeDirectionalGroundGaps(supportMap, outputMap, acceptedPlanes,
                                parameters, inferredSupportIndices);
  result.inferredCells = inferredSupportIndices.size();
  return result;
}

}  // namespace elevation_mapping
