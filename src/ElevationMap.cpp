/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/EdgeAwareMapUtils.hpp"
#include "elevation_mapping/MultimodalElevation.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ScanCellSelector.hpp"
#include "elevation_mapping/SmallHoleFiller.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

namespace {
/**
 * Store an unsigned integer value in a float
 * @param input integer
 * @return A float with the bit pattern of the input integer
 */
float intAsFloat(const uint32_t input) {
  float output;
  std::memcpy(&output, &input, sizeof(uint32_t));
  return output;
}
}  // namespace

namespace elevation_mapping {
namespace {

struct ModeLayerNames {
  const char* elevation;
  const char* variance;
  const char* color;
  const char* lowest;
  const char* confidence;
  const char* time;
  const char* coverage;
  const char* observations;
  const char* center;
  const char* sensorX;
  const char* sensorY;
  const char* sensorZ;
};

const std::array<ModeLayerNames, 2> kModeLayers{{
    {"mode0_elevation", "mode0_variance", "mode0_color", "mode0_lowest",
     "mode0_confidence", "mode0_time", "mode0_coverage",
     "mode0_observations", "mode0_center", "mode0_sensor_x",
     "mode0_sensor_y", "mode0_sensor_z"},
    {"mode1_elevation", "mode1_variance", "mode1_color", "mode1_lowest",
     "mode1_confidence", "mode1_time", "mode1_coverage",
     "mode1_observations", "mode1_center", "mode1_sensor_x",
     "mode1_sensor_y", "mode1_sensor_z"},
}};

constexpr const char* kPrimaryModeIndexLayer = "primary_mode_index";
constexpr const char* kChallengerModeIndexLayer = "challenger_mode_index";
constexpr const char* kChallengerCountLayer = "challenger_count";
constexpr std::array<const char*, 4> kMultimodalDiagnosticLayers{{
    "secondary_elevation", "height_mode_count", "height_mode_separation",
    "primary_mode_confidence",
}};

std::size_t cellKey(
    const grid_map::Index& index, const grid_map::Size& size) {
  return static_cast<std::size_t>(index(0) * size(1) + index(1));
}

void resetModeAt(
    grid_map::GridMap& map, const grid_map::Index& index,
    const ModeLayerNames& layers) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  map.at(layers.elevation, index) = nan;
  map.at(layers.variance, index) = nan;
  map.at(layers.color, index) = nan;
  map.at(layers.lowest, index) = nan;
  map.at(layers.confidence, index) = 0.0f;
  map.at(layers.time, index) = nan;
  map.at(layers.coverage, index) = 0.0f;
  map.at(layers.observations, index) = 0.0f;
  map.at(layers.center, index) = 0.0f;
  map.at(layers.sensorX, index) = nan;
  map.at(layers.sensorY, index) = nan;
  map.at(layers.sensorZ, index) = nan;
}

void resetMultimodalStateAt(
    grid_map::GridMap& map, const grid_map::Index& index) {
  for (const auto& layers : kModeLayers) {
    resetModeAt(map, index, layers);
  }
  map.at(kPrimaryModeIndexLayer, index) = -1.0f;
  map.at(kChallengerModeIndexLayer, index) = -1.0f;
  map.at(kChallengerCountLayer, index) = 0.0f;
}

void resetMultimodalState(grid_map::GridMap& map) {
  if (map.getSize().prod() == 0) {
    return;
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    resetMultimodalStateAt(map, *iterator);
  }
}

void resetMultimodalState(
    grid_map::GridMap& map,
    const std::vector<grid_map::BufferRegion>& regions) {
  for (const auto& region : regions) {
    for (grid_map::SubmapIterator iterator(map, region);
         !iterator.isPastEnd(); ++iterator) {
      resetMultimodalStateAt(map, *iterator);
    }
  }
}

void resetRawDiagnostics(grid_map::GridMap& map) {
  for (const char* layer : kMultimodalDiagnosticLayers) {
    if (!map.exists(layer)) {
      map.add(layer);
    }
  }
  map["secondary_elevation"].setConstant(
      std::numeric_limits<float>::quiet_NaN());
  map["height_mode_count"].setZero();
  map["height_mode_separation"].setConstant(
      std::numeric_limits<float>::quiet_NaN());
  map["primary_mode_confidence"].setConstant(
      std::numeric_limits<float>::quiet_NaN());
}

bool hasValidRawMapGeometry(const grid_map::GridMap& map) {
  const auto& size = map.getSize();
  const auto& startIndex = map.getStartIndex();
  return size(0) > 0 && size(1) > 0 &&
         std::isfinite(map.getResolution()) && map.getResolution() > 0.0 &&
         map.getLength().allFinite() && (map.getLength() > 0.0).all() &&
         map.getPosition().allFinite() &&
         startIndex(0) >= 0 && startIndex(0) < size(0) &&
         startIndex(1) >= 0 && startIndex(1) < size(1);
}

void resetRawDiagnostics(
    grid_map::GridMap& map,
    const std::vector<grid_map::BufferRegion>& regions) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const auto& region : regions) {
    for (grid_map::SubmapIterator iterator(map, region);
         !iterator.isPastEnd(); ++iterator) {
      map.at("secondary_elevation", *iterator) = nan;
      map.at("height_mode_count", *iterator) = 0.0f;
      map.at("height_mode_separation", *iterator) = nan;
      map.at("primary_mode_confidence", *iterator) = nan;
    }
  }
}

std::size_t decodeCount(float value) {
  if (!std::isfinite(value) || value <= 0.0f) {
    return 0;
  }
  return static_cast<std::size_t>(std::lround(value));
}

int decodeIndex(float value) {
  if (!std::isfinite(value)) {
    return -1;
  }
  return static_cast<int>(std::lround(value));
}

SurfaceModeState decodeMode(
    const grid_map::GridMap& map, const grid_map::Index& index,
    const ModeLayerNames& layers) {
  SurfaceModeState mode;
  mode.height = map.at(layers.elevation, index);
  mode.valid = std::isfinite(mode.height);
  if (!mode.valid) {
    return mode;
  }
  mode.variance = map.at(layers.variance, index);
  mode.color = map.at(layers.color, index);
  mode.lowestPointHeight = map.at(layers.lowest, index);
  mode.confidence = map.at(layers.confidence, index);
  mode.lastSeen = map.at(layers.time, index);
  mode.coverageBins = decodeCount(map.at(layers.coverage, index));
  mode.consecutiveObservations =
      decodeCount(map.at(layers.observations, index));
  mode.centerOccupied = map.at(layers.center, index) > 0.5f;
  mode.sensorX = map.at(layers.sensorX, index);
  mode.sensorY = map.at(layers.sensorY, index);
  mode.sensorZ = map.at(layers.sensorZ, index);
  return mode;
}

MultimodalCellState decodeMultimodalState(
    const grid_map::GridMap& map, const grid_map::Index& index) {
  MultimodalCellState state;
  for (std::size_t modeIndex = 0; modeIndex < state.modes.size();
       ++modeIndex) {
    state.modes[modeIndex] =
        decodeMode(map, index, kModeLayers[modeIndex]);
  }
  state.primaryIndex = decodeIndex(map.at(kPrimaryModeIndexLayer, index));
  state.challengerIndex =
      decodeIndex(map.at(kChallengerModeIndexLayer, index));
  state.challengerCount =
      std::max(0, decodeIndex(map.at(kChallengerCountLayer, index)));
  return state;
}

void encodeMode(
    grid_map::GridMap& map, const grid_map::Index& index,
    const ModeLayerNames& layers, const SurfaceModeState& mode) {
  if (!mode.valid) {
    resetModeAt(map, index, layers);
    return;
  }
  map.at(layers.elevation, index) = mode.height;
  map.at(layers.variance, index) = mode.variance;
  map.at(layers.color, index) = mode.color;
  map.at(layers.lowest, index) = mode.lowestPointHeight;
  map.at(layers.confidence, index) = mode.confidence;
  map.at(layers.time, index) = static_cast<float>(mode.lastSeen);
  map.at(layers.coverage, index) = static_cast<float>(mode.coverageBins);
  map.at(layers.observations, index) =
      static_cast<float>(mode.consecutiveObservations);
  map.at(layers.center, index) = mode.centerOccupied ? 1.0f : 0.0f;
  map.at(layers.sensorX, index) = mode.sensorX;
  map.at(layers.sensorY, index) = mode.sensorY;
  map.at(layers.sensorZ, index) = mode.sensorZ;
}

void encodeMultimodalState(
    grid_map::GridMap& map, const grid_map::Index& index,
    const MultimodalCellState& state) {
  for (std::size_t modeIndex = 0; modeIndex < state.modes.size();
       ++modeIndex) {
    encodeMode(map, index, kModeLayers[modeIndex], state.modes[modeIndex]);
  }
  map.at(kPrimaryModeIndexLayer, index) =
      static_cast<float>(state.primaryIndex);
  map.at(kChallengerModeIndexLayer, index) =
      static_cast<float>(state.challengerIndex);
  map.at(kChallengerCountLayer, index) =
      static_cast<float>(state.challengerCount);
}

void refreshMultimodalDiagnostics(
    grid_map::GridMap& rawMap, const grid_map::Index& index,
    const MultimodalCellState& state) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto& secondaryElevation = rawMap.at("secondary_elevation", index);
  auto& modeCountLayer = rawMap.at("height_mode_count", index);
  auto& modeSeparation = rawMap.at("height_mode_separation", index);
  auto& primaryConfidence = rawMap.at("primary_mode_confidence", index);
  secondaryElevation = nan;
  modeSeparation = nan;
  primaryConfidence = nan;

  const std::size_t modeCount = countValidModes(state);
  modeCountLayer = static_cast<float>(modeCount);
  if (modeCount == 0) {
    return;
  }

  int primaryIndex = state.primaryIndex;
  if (primaryIndex < 0 || primaryIndex >= 2 ||
      !state.modes[primaryIndex].valid) {
    primaryIndex = state.modes[0].valid ? 0 : 1;
  }
  const auto& primary = state.modes[primaryIndex];
  primaryConfidence = primary.confidence;
  if (modeCount == 1) {
    return;
  }

  const int secondaryIndex = primaryIndex == 0 ? 1 : 0;
  const auto& secondary = state.modes[secondaryIndex];
  secondaryElevation = secondary.height;
  modeSeparation = std::fabs(primary.height - secondary.height);
}

void applyMultimodalPrimary(
    grid_map::GridMap& rawMap, const grid_map::Index& index,
    const SurfaceModeState& primary, float minHorizontalVariance,
    float dynamicTime) {
  rawMap.at("elevation", index) = primary.height;
  rawMap.at("variance", index) = primary.variance;
  rawMap.at("horizontal_variance_x", index) = minHorizontalVariance;
  rawMap.at("horizontal_variance_y", index) = minHorizontalVariance;
  rawMap.at("horizontal_variance_xy", index) = 0.0f;
  rawMap.at("color", index) = primary.color;
  rawMap.at("time", index) = static_cast<float>(primary.lastSeen);
  rawMap.at("dynamic_time", index) = dynamicTime;
  rawMap.at("lowest_scan_point", index) = primary.lowestPointHeight;
  rawMap.at("sensor_x_at_lowest_scan", index) = primary.sensorX;
  rawMap.at("sensor_y_at_lowest_scan", index) = primary.sensorY;
  rawMap.at("sensor_z_at_lowest_scan", index) = primary.sensorZ;
  rawMap.at("lower_point_height", index) = NAN;
  rawMap.at("lower_point_time", index) = NAN;
  rawMap.at("lower_point_count", index) = 0.0f;
  rawMap.at("adaptive_lower_point_height", index) = NAN;
  rawMap.at("adaptive_lower_point_time", index) = NAN;
  rawMap.at("adaptive_lower_point_count", index) = 0.0f;
}

}  // namespace

ElevationMap::ElevationMap(std::shared_ptr<rclcpp::Node> nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", 
      "horizontal_variance_xy", "color", "time","dynamic_time", "lowest_scan_point", 
      "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan",
      "lower_point_height", "lower_point_time", "lower_point_count",
      "adaptive_lower_point_height", "adaptive_lower_point_time", "adaptive_lower_point_count",
      "secondary_elevation", "height_mode_count", "height_mode_separation",
      "primary_mode_confidence"}),
      multimodalStateMap_(
          {"mode0_elevation", "mode0_variance", "mode0_color", "mode0_lowest",
           "mode0_confidence", "mode0_time", "mode0_coverage",
           "mode0_observations", "mode0_center", "mode0_sensor_x",
           "mode0_sensor_y", "mode0_sensor_z", "mode1_elevation",
           "mode1_variance", "mode1_color", "mode1_lowest",
           "mode1_confidence", "mode1_time", "mode1_coverage",
           "mode1_observations", "mode1_center", "mode1_sensor_x",
           "mode1_sensor_y", "mode1_sensor_z", "primary_mode_index",
           "challenger_mode_index", "challenger_count"}),
      fusedMap_({"elevation", "upper_bound", "lower_bound", "color",
                 "secondary_elevation", "height_mode_count",
                 "height_mode_separation", "primary_mode_confidence"}),
      // FIXME: Postprocessor num threads should be same as number of filters
      postprocessorPool_(nodeHandle_->get_parameter("postprocessor_num_threads").as_int(), nodeHandle_),
      hasUnderlyingMap_(false),
      initialTime_(0, 0, nodeHandle->get_clock()->get_clock_type()),
      isInitialTimeSet_(false),
      minVariance_(0.000009),
      maxVariance_(0.0009),
      mahalanobisDistanceThreshold_(1.5),
      multiHeightNoise_(0.000009),
      minHorizontalVariance_(0.0001),
      maxHorizontalVariance_(0.05),
      edgeAwareFusion_(true),
      enableMultimodalCells_(false),
      multimodalConfig_(),
      enableSkipLowerPoints_(false),
      skipLowerPointsDuration_(0.5),
      lowerPointRecoveryCount_(5),
      enableAdaptiveLowerSurface_(false),
      lowerSurfaceNeighborRadius_(0.12),
      lowerSurfaceMinSupport_(4),
      lowerSurfaceMinCandidateSupport_(2),
      lowerSurfaceRecoveryCount_(2),
      lowerSurfaceHeightThreshold_(0.05),
      lowerSurfaceMaxTimeGap_(0.25),
      enableFusedMapHoleFilling_(false),
      fusedMapHoleFillingRadius_(0.08),
      fusedMapHoleFillingMinSupport_(4),
      fusedMapHoleFillingHeightThreshold_(0.05),
      fusionHeightDifferenceThreshold_(0.08),
      enableVisibilityCleanup_(true),
      enableContinuousCleanup_(false),
      visibilityCleanupDuration_(0.0),
      scanningDuration_(1.0),
      enableSmallHoleFilling_(defaultSmallHoleFillingEnabled()),
      smallHoleFillingParameters_{4u, 4u, 0.05f},
      enablePiecewisePlanarRegularization_(
          defaultPiecewisePlanarRegularizationEnabled()),
      piecewisePlanarParameters_{
          0.06f, 12u, 0.03f, 0.05f, 0.12f, 4u, 0.05f, false, 0.40f} {
  nodeHandle_->declare_parameter(
      "enable_small_hole_filling", defaultSmallHoleFillingEnabled());
  nodeHandle_->declare_parameter("small_hole_max_size", 4);
  nodeHandle_->declare_parameter("small_hole_min_support", 4);
  nodeHandle_->declare_parameter("small_hole_max_height_range", 0.05);
  nodeHandle_->get_parameter("enable_small_hole_filling", enableSmallHoleFilling_);
  int maxHoleSize;
  int minSupport;
  nodeHandle_->get_parameter("small_hole_max_size", maxHoleSize);
  nodeHandle_->get_parameter("small_hole_min_support", minSupport);
  nodeHandle_->get_parameter("small_hole_max_height_range", smallHoleFillingParameters_.maxHeightRange);
  smallHoleFillingParameters_.maxHoleSize = static_cast<std::size_t>(maxHoleSize);
  smallHoleFillingParameters_.minSupport = static_cast<std::size_t>(minSupport);
  nodeHandle_->declare_parameter(
      "enable_piecewise_planar_regularization",
      defaultPiecewisePlanarRegularizationEnabled());
  nodeHandle_->declare_parameter("planar_neighbor_height_tolerance", 0.06);
  nodeHandle_->declare_parameter("planar_min_region_size", 12);
  nodeHandle_->declare_parameter("planar_max_region_mad", 0.03);
  nodeHandle_->declare_parameter("planar_max_regularization_residual", 0.05);
  nodeHandle_->declare_parameter("planar_max_occlusion_distance", 0.12);
  nodeHandle_->declare_parameter("planar_min_occlusion_support", 4);
  nodeHandle_->declare_parameter("planar_inferred_half_range", 0.05);
  nodeHandle_->declare_parameter("enable_directional_ground_completion", false);
  nodeHandle_->declare_parameter("directional_ground_max_gap_width", 0.40);
  nodeHandle_->get_parameter(
      "enable_piecewise_planar_regularization",
      enablePiecewisePlanarRegularization_);
  int minRegionSize;
  int minOcclusionSupport;
  nodeHandle_->get_parameter(
      "planar_neighbor_height_tolerance",
      piecewisePlanarParameters_.neighborHeightTolerance);
  nodeHandle_->get_parameter("planar_min_region_size", minRegionSize);
  nodeHandle_->get_parameter(
      "planar_max_region_mad", piecewisePlanarParameters_.maxRegionMad);
  nodeHandle_->get_parameter(
      "planar_max_regularization_residual",
      piecewisePlanarParameters_.maxRegularizationResidual);
  nodeHandle_->get_parameter(
      "planar_max_occlusion_distance",
      piecewisePlanarParameters_.maxOcclusionDistance);
  nodeHandle_->get_parameter(
      "planar_min_occlusion_support", minOcclusionSupport);
  nodeHandle_->get_parameter(
      "planar_inferred_half_range",
      piecewisePlanarParameters_.inferredHalfRange);
  nodeHandle_->get_parameter(
      "enable_directional_ground_completion",
      piecewisePlanarParameters_.enableDirectionalGroundCompletion);
  nodeHandle_->get_parameter(
      "directional_ground_max_gap_width",
      piecewisePlanarParameters_.directionalGroundMaxGapWidth);
  piecewisePlanarParameters_.minRegionSize =
      static_cast<std::size_t>(minRegionSize < 0 ? 0 : minRegionSize);
  piecewisePlanarParameters_.minOcclusionSupport =
      static_cast<std::size_t>(minOcclusionSupport < 0 ? 0 : minOcclusionSupport);
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  clear();

  elevationMapFusedPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) {
    underlyingMapSubscriber_ = nodeHandle_->create_subscription<grid_map_msgs::msg::GridMap>(underlyingMapTopic_, 1, std::bind(&ElevationMap::underlyingMapCallback, this, std::placeholders::_1));
  }
  // TODO(max): if (enableVisibilityCleanup_) when parameter cleanup is ready.
  visibilityCleanupMapPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>("visibility_cleanup_map", 1);

}

ElevationMap::~ElevationMap() = default;

bool ElevationMap::add(const PointCloudType::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const rclcpp::Time& timestamp,
                       const Eigen::Affine3d& transformationSensorToMap) {
  if (static_cast<unsigned int>(pointCloud->size()) != static_cast<unsigned int>(pointCloudVariances.size())) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.", (int)pointCloud->size(),
              (int)pointCloudVariances.size());
    return false;
  }
  // RCLCPP_INFO(nodeHandle_->get_logger(), "Point cloud variances: %f, %f, %f", pointCloudVariances(0), pointCloudVariances(1), pointCloudVariances(2)) ;

  // Initialization for time calculation.
  // RCLCPP_INFO(nodeHandle_->get_logger(), "ElevationMap::add: Initializing map.");
  const auto methodStartTime = std::chrono::system_clock::now();
  const rclcpp::Time currentTime = nodeHandle_->get_clock()->now();
  const float currentTimeSecondsPattern{intAsFloat(static_cast<uint32_t>(static_cast<uint64_t>(currentTime.seconds())))};
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  // Update initial time if it is not initialized.
  if (!isInitialTimeSet_) {
    initialTime_ = timestamp;
    isInitialTimeSet_ = true;
  }
  const float scanTimeSinceInitialization = (timestamp - initialTime_).seconds();
  const auto rawMapSize = rawMap_.getSize();
  const std::size_t cellCount = static_cast<std::size_t>(rawMapSize.prod());
  std::vector<std::vector<MultimodalPoint>> pointsByCell;
  std::vector<unsigned char> multimodalHandled;
  std::vector<unsigned char> multimodalExpired;
  std::vector<unsigned char> multimodalPrimaryUpdates;
  std::vector<MultimodalUpdateResult> multimodalResults;
  std::vector<std::size_t> activeChallengerKeys;

  if (enableMultimodalCells_) {
    pointsByCell.resize(cellCount);
    multimodalHandled.assign(cellCount, 0);
    multimodalExpired.assign(cellCount, 0);
    multimodalPrimaryUpdates.assign(cellCount, 0);
    multimodalResults.resize(cellCount);
    activeChallengerKeys.reserve(cellCount);

    for (grid_map::GridMapIterator iterator(multimodalStateMap_);
         !iterator.isPastEnd(); ++iterator) {
      auto state = decodeMultimodalState(multimodalStateMap_, *iterator);
      const std::size_t key = cellKey(*iterator, rawMapSize);
      if (expireStaleModes(
              state, scanTimeSinceInitialization, multimodalConfig_)) {
        multimodalExpired[key] = 1;
        encodeMultimodalState(multimodalStateMap_, *iterator, state);
        refreshMultimodalDiagnostics(rawMap_, *iterator, state);
        if (state.primaryIndex >= 0 && state.primaryIndex < 2 &&
            state.modes[state.primaryIndex].valid) {
          multimodalPrimaryUpdates[key] = 1;
          multimodalResults[key].primary = state.modes[state.primaryIndex];
          multimodalResults[key].modeCount = countValidModes(state);
        }
      }
      if (state.challengerIndex != -1 || state.challengerCount != 0) {
        activeChallengerKeys.push_back(key);
      }
    }

    for (unsigned int i = 0; i < pointCloud->size(); ++i) {
      const auto& point = pointCloud->points[i];
      grid_map::Index index;
      if (!rawMap_.getIndex(grid_map::Position(point.x, point.y), index)) {
        continue;
      }

      float color;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      const float variance = std::min(
          std::max(
              static_cast<float>(pointCloudVariances(i)),
              static_cast<float>(minVariance_)),
          static_cast<float>(maxVariance_));
      pointsByCell[cellKey(index, rawMapSize)].push_back(
          {point.x, point.y, point.z, variance, color});
    }

    for (const std::size_t key : activeChallengerKeys) {
      if (!pointsByCell[key].empty()) {
        continue;
      }
      const grid_map::Index index(
          static_cast<int>(key / static_cast<std::size_t>(rawMapSize(1))),
          static_cast<int>(key % static_cast<std::size_t>(rawMapSize(1))));
      auto state = decodeMultimodalState(multimodalStateMap_, index);
      if (state.challengerIndex != -1 || state.challengerCount != 0) {
        state.challengerIndex = -1;
        state.challengerCount = 0;
        encodeMultimodalState(multimodalStateMap_, index, state);
      }
    }

    for (std::size_t key = 0; key < pointsByCell.size(); ++key) {
      if (pointsByCell[key].empty()) {
        continue;
      }
      multimodalPrimaryUpdates[key] = 0;

      const grid_map::Index index(
          static_cast<int>(key / static_cast<std::size_t>(rawMapSize(1))),
          static_cast<int>(key % static_cast<std::size_t>(rawMapSize(1))));
      grid_map::Position cellCenter;
      rawMap_.getPosition(index, cellCenter);
      auto observation = buildCellObservation(
          pointsByCell[key], static_cast<float>(cellCenter.x()),
          static_cast<float>(cellCenter.y()),
          static_cast<float>(rawMap_.getResolution()), multimodalConfig_);
      for (std::size_t modeIndex = 0; modeIndex < observation.modeCount;
           ++modeIndex) {
        observation.modes[modeIndex].sensorX =
            static_cast<float>(transformationSensorToMap.translation().x());
        observation.modes[modeIndex].sensorY =
            static_cast<float>(transformationSensorToMap.translation().y());
        observation.modes[modeIndex].sensorZ =
            static_cast<float>(transformationSensorToMap.translation().z());
      }
      auto state = decodeMultimodalState(multimodalStateMap_, index);
      if (countValidModes(state) == 0 && multimodalExpired[key] == 0) {
        const float legacyHeight = rawMap_.at("elevation", index);
        const float legacyVariance = rawMap_.at("variance", index);
        const bool separatedSupportedReturn =
            observation.modeCount == 1 &&
            std::fabs(legacyHeight - observation.modes[0].height) >
                multimodalConfig_.modeSeparation;
        const bool separatedUnsupportedReturn =
            observation.modeCount == 0 &&
            std::any_of(
                pointsByCell[key].begin(), pointsByCell[key].end(),
                [&](const MultimodalPoint& point) {
                  return std::isfinite(point.height) &&
                         std::fabs(legacyHeight - point.height) >
                             multimodalConfig_.modeSeparation;
                });
        if (std::isfinite(legacyHeight) && std::isfinite(legacyVariance) &&
            legacyVariance > 0.0f &&
            (separatedSupportedReturn || separatedUnsupportedReturn)) {
          auto& incumbent = state.modes[0];
          incumbent.valid = true;
          incumbent.height = legacyHeight;
          incumbent.variance = legacyVariance;
          incumbent.color = rawMap_.at("color", index);
          if (!std::isfinite(incumbent.color)) {
            incumbent.color = observation.modeCount == 1
                                  ? observation.modes[0].color
                                  : pointsByCell[key].front().color;
          }
          incumbent.lowestPointHeight =
              rawMap_.at("lowest_scan_point", index);
          if (!std::isfinite(incumbent.lowestPointHeight)) {
            incumbent.lowestPointHeight =
                legacyHeight + 3.0f * std::sqrt(legacyVariance);
          }
          incumbent.confidence = 1.0f;
          incumbent.lastSeen = scanTimeSinceInitialization;
          incumbent.coverageBins = 0;
          incumbent.consecutiveObservations = 1;
          incumbent.centerOccupied = false;
          incumbent.sensorX =
              rawMap_.at("sensor_x_at_lowest_scan", index);
          incumbent.sensorY =
              rawMap_.at("sensor_y_at_lowest_scan", index);
          incumbent.sensorZ =
              rawMap_.at("sensor_z_at_lowest_scan", index);
          state.primaryIndex = 0;
        }
      }
      auto result = updateMultimodalCell(
          state, observation, scanTimeSinceInitialization, multimodalConfig_);
      encodeMultimodalState(multimodalStateMap_, index, state);
      refreshMultimodalDiagnostics(rawMap_, index, state);
      if (result.handled) {
        multimodalHandled[key] = 1;
        multimodalPrimaryUpdates[key] = 1;
        multimodalResults[key] = result;
      }
    }
  }

  const auto isMultimodalHandled = [&](const grid_map::Index& index) {
    return enableMultimodalCells_ &&
           multimodalHandled[cellKey(index, rawMapSize)] != 0;
  };

  // Store references for efficient interaction.
  // RCLCPP_INFO(nodeHandle_->get_logger(), "ElevationMap::add: Storing references.");
  auto& elevationLayer = rawMap_["elevation"];
  auto& varianceLayer = rawMap_["variance"];
  auto& horizontalVarianceXLayer = rawMap_["horizontal_variance_x"];
  auto& horizontalVarianceYLayer = rawMap_["horizontal_variance_y"];
  auto& horizontalVarianceXYLayer = rawMap_["horizontal_variance_xy"];
  auto& colorLayer = rawMap_["color"];
  auto& timeLayer = rawMap_["time"];
  auto& dynamicTimeLayer = rawMap_["dynamic_time"];
  auto& lowestScanPointLayer = rawMap_["lowest_scan_point"];
  auto& sensorXatLowestScanLayer = rawMap_["sensor_x_at_lowest_scan"];
  auto& sensorYatLowestScanLayer = rawMap_["sensor_y_at_lowest_scan"];
  auto& sensorZatLowestScanLayer = rawMap_["sensor_z_at_lowest_scan"];
  auto& lowerPointHeightLayer = rawMap_["lower_point_height"];
  auto& lowerPointTimeLayer = rawMap_["lower_point_time"];
  auto& lowerPointCountLayer = rawMap_["lower_point_count"];
  auto& adaptiveLowerPointHeightLayer = rawMap_["adaptive_lower_point_height"];
  auto& adaptiveLowerPointTimeLayer = rawMap_["adaptive_lower_point_time"];
  auto& adaptiveLowerPointCountLayer = rawMap_["adaptive_lower_point_count"];

  grid_map::GridMap currentScanMap(
      {"elevation", "variance", "color", "lower_candidate",
       "adaptive_lower_support", "adaptive_lower_confirmed"});
  if (enableAdaptiveLowerSurface_) {
    currentScanMap.setGeometry(rawMap_.getLength(), rawMap_.getResolution(), rawMap_.getPosition());
    currentScanMap["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
    currentScanMap["variance"].setConstant(std::numeric_limits<float>::quiet_NaN());
    currentScanMap["color"].setConstant(std::numeric_limits<float>::quiet_NaN());
    currentScanMap["lower_candidate"].setZero();
    currentScanMap["adaptive_lower_support"].setConstant(std::numeric_limits<float>::quiet_NaN());
    currentScanMap["adaptive_lower_confirmed"].setZero();
    for (unsigned int i = 0; i < pointCloud->size(); ++i) {
      const auto& point = pointCloud->points[i];
      grid_map::Index scanIndex;
      if (!currentScanMap.getIndex(grid_map::Position(point.x, point.y), scanIndex)) {
        continue;
      }
      grid_map::Index rawIndex;
      if (!rawMap_.getIndex(grid_map::Position(point.x, point.y), rawIndex) ||
          isMultimodalHandled(rawIndex)) {
        continue;
      }
      auto& scanHeight = currentScanMap.at("elevation", scanIndex);
      if (!std::isfinite(scanHeight) || point.z < scanHeight) {
        scanHeight = point.z;
        currentScanMap.at("variance", scanIndex) =
            std::max(static_cast<float>(minVariance_),
                     static_cast<float>(pointCloudVariances(i)));
        grid_map::colorVectorToValue(
            point.getRGBVector3i(), currentScanMap.at("color", scanIndex));
      }
    }

    for (grid_map::GridMapIterator iterator(currentScanMap); !iterator.isPastEnd(); ++iterator) {
      const float scanHeight = currentScanMap.at("elevation", *iterator);
      if (!std::isfinite(scanHeight)) {
        continue;
      }
      grid_map::Position scanPosition;
      currentScanMap.getPosition(*iterator, scanPosition);
      grid_map::Index rawIndex;
      if (!rawMap_.getIndex(scanPosition, rawIndex)) {
        continue;
      }
      if (isMultimodalHandled(rawIndex)) {
        continue;
      }
      const float rawHeight = rawMap_.at("elevation", rawIndex);
      if (std::isfinite(rawHeight) &&
          rawHeight - scanHeight > lowerSurfaceHeightThreshold_) {
          currentScanMap.at("lower_candidate", *iterator) = 1.0f;
      }
    }

    for (grid_map::GridMapIterator iterator(currentScanMap); !iterator.isPastEnd(); ++iterator) {
      const float scanHeight = currentScanMap.at("elevation", *iterator);
      if (!std::isfinite(scanHeight)) {
        continue;
      }

      grid_map::Position scanPosition;
      currentScanMap.getPosition(*iterator, scanPosition);
      grid_map::Index rawIndex;
      if (!rawMap_.getIndex(scanPosition, rawIndex)) {
        continue;
      }
      if (isMultimodalHandled(rawIndex)) {
        continue;
      }

      const float rawHeight = rawMap_.at("elevation", rawIndex);
      const float rawVariance = rawMap_.at("variance", rawIndex);
      const bool isCandidate =
          currentScanMap.at("lower_candidate", *iterator) > 0.5f &&
          std::isfinite(rawHeight) && std::isfinite(rawVariance) &&
          std::fabs(scanHeight - rawHeight) / std::sqrt(rawVariance) >
              mahalanobisDistanceThreshold_;

      bool hasSurfaceSupport = false;
      if (isCandidate) {
        hasSurfaceSupport = hasConsistentLowerSurfaceSupport(
            currentScanMap, "elevation", "lower_candidate", *iterator, scanHeight,
            lowerSurfaceNeighborRadius_, lowerSurfaceHeightThreshold_,
            static_cast<std::size_t>(lowerSurfaceMinSupport_),
            static_cast<std::size_t>(lowerSurfaceMinCandidateSupport_));
        currentScanMap.at("adaptive_lower_support", *iterator) =
            hasSurfaceSupport ? 1.0f : 0.0f;
      }

      AdaptiveLowerSurfaceState adaptiveState;
      adaptiveState.height = adaptiveLowerPointHeightLayer(rawIndex(0), rawIndex(1));
      adaptiveState.timestamp = adaptiveLowerPointTimeLayer(rawIndex(0), rawIndex(1));
      const float storedCount = adaptiveLowerPointCountLayer(rawIndex(0), rawIndex(1));
      adaptiveState.count =
          std::isfinite(storedCount)
              ? std::max(0, static_cast<int>(std::lround(storedCount)))
              : 0;
      const bool confirmed = updateAdaptiveLowerSurfaceState(
          adaptiveState, scanHeight, scanTimeSinceInitialization,
          isCandidate && hasSurfaceSupport, lowerSurfaceHeightThreshold_,
          lowerSurfaceMaxTimeGap_, lowerSurfaceRecoveryCount_);
      adaptiveLowerPointHeightLayer(rawIndex(0), rawIndex(1)) = adaptiveState.height;
      adaptiveLowerPointTimeLayer(rawIndex(0), rawIndex(1)) = adaptiveState.timestamp;
      adaptiveLowerPointCountLayer(rawIndex(0), rawIndex(1)) =
          static_cast<float>(adaptiveState.count);
      if (confirmed) {
        currentScanMap.at("adaptive_lower_confirmed", *iterator) = 1.0f;
      }
    }
  }

  std::vector<Eigen::Ref<const grid_map::Matrix>> basicLayers_;
  for (const std::string& layer : rawMap_.getBasicLayers()) {
    basicLayers_.push_back(rawMap_.get(layer));
  }

  const auto selectedPointIndices =
      selectScanCellRepresentatives(rawMap_, *pointCloud);
  for (const auto i : selectedPointIndices) {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    // RCLCPP_INFO(nodeHandle_->get_logger(), "Position in grid map: %f, %f", position.x(), position.y());
    if (!rawMap_.getIndex(position, index)) {
      continue;  // Skip this point if it does not lie within the elevation map.
    }
    if (isMultimodalHandled(index)) {
      continue;
    }
    if (enableAdaptiveLowerSurface_) {
      grid_map::Index scanIndex;
      if (currentScanMap.getIndex(position, scanIndex) &&
          currentScanMap.at("adaptive_lower_confirmed", scanIndex) > 0.5f) {
        continue;
      }
    }

    auto& elevation = elevationLayer(index(0), index(1));
    auto& variance = varianceLayer(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceXLayer(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceYLayer(index(0), index(1));
    auto& horizontalVarianceXY = horizontalVarianceXYLayer(index(0), index(1));
    auto& color = colorLayer(index(0), index(1));
    auto& time = timeLayer(index(0), index(1));
    auto& dynamicTime = dynamicTimeLayer(index(0), index(1));
    auto& lowestScanPoint = lowestScanPointLayer(index(0), index(1));
    auto& sensorXatLowestScan = sensorXatLowestScanLayer(index(0), index(1));
    auto& sensorYatLowestScan = sensorYatLowestScanLayer(index(0), index(1));
    auto& sensorZatLowestScan = sensorZatLowestScanLayer(index(0), index(1));
    auto& lowerPointHeight = lowerPointHeightLayer(index(0), index(1));
    auto& lowerPointTime = lowerPointTimeLayer(index(0), index(1));
    auto& lowerPointCount = lowerPointCountLayer(index(0), index(1));
    auto& adaptiveLowerPointHeight = adaptiveLowerPointHeightLayer(index(0), index(1));
    auto& adaptiveLowerPointTime = adaptiveLowerPointTimeLayer(index(0), index(1));
    auto& adaptiveLowerPointCount = adaptiveLowerPointCountLayer(index(0), index(1));

    const float pointVariance =
        sanitizePointVariance(pointCloudVariances(i), minVariance_);
    bool isValid = std::all_of(basicLayers_.begin(), basicLayers_.end(),
                               [&](Eigen::Ref<const grid_map::Matrix> layer) { 
                                return std::isfinite(layer(index(0), index(1))); });
    if (!isValid) {
      // No prior information in elevation map, use measurement.
      // RCLCPP_INFO(nodeHandle_->get_logger(), "No prior information in elevation map, using measurement.");
      elevation = point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)
      // RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation is %f", elevation);
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      time = scanTimeSinceInitialization;
      dynamicTime = currentTimeSecondsPattern;
      lowestScanPoint = point.z + 3.0 * sqrt(pointVariance);
      const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
      lowerPointHeight = NAN;
      lowerPointTime = NAN;
      lowerPointCount = 0.0f;
      adaptiveLowerPointHeight = NAN;
      adaptiveLowerPointTime = NAN;
      adaptiveLowerPointCount = 0.0f;
      continue;
    }

    // RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation: %f, Point Z: %f, Variance: %f", elevation, point.z, variance);
    // Deal with multiple heights in one cell.
    const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    const double cellAge = scanTimeSinceInitialization - time;
    const bool isLowerPoint = elevation > point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    const auto recordLowerPoint = [&]() {
      const bool newLowerScan =
          std::isnan(lowerPointTime) || fabs(scanTimeSinceInitialization - lowerPointTime) > 0.0001;
      const bool sameLowerHeight =
          !std::isnan(lowerPointHeight) &&
          fabs(point.z - lowerPointHeight) <= fusionHeightDifferenceThreshold_;
      if (!sameLowerHeight) {
        lowerPointHeight = point.z;
        lowerPointCount = 1.0f;
      } else if (newLowerScan) {
        lowerPointHeight = 0.5f * (lowerPointHeight + point.z);
        lowerPointCount += 1.0f;
      }
      lowerPointTime = scanTimeSinceInitialization;
    };

    const auto acceptLowerPoint = [&](float acceptedHeight) {
      elevation = acceptedHeight;
      variance = pointVariance;
      time = scanTimeSinceInitialization;
      dynamicTime = currentTimeSecondsPattern;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      lowerPointHeight = NAN;
      lowerPointTime = NAN;
      lowerPointCount = 0.0f;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
    };

    // RCLCPP_INFO(nodeHandle_->get_logger(), "Mahalanobis Distance: %f", mahalanobisDistance);
    if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
      // RCLCPP_INFO(nodeHandle_->get_logger(), "Mahalanobis distance exceeds threshold.");
      if ((scanTimeSinceInitialization - time <= scanningDuration_ ||
           (enableSkipLowerPoints_ && cellAge <= skipLowerPointsDuration_)) &&
          isLowerPoint) {
          // RCLCPP_INFO(nodeHandle_->get_logger(), "Ignoring point, lower than existing elevation and within scanning duration.");
        // Ignore point if measurement is from the same point cloud (time comparison) and
        // if measurement is lower then the elevation in the map.
        const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance);  // 3 sigma.
        if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint) {
          lowestScanPoint = pointHeightPlusUncertainty;
          const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
          sensorXatLowestScan = sensorTranslation.x();
          sensorYatLowestScan = sensorTranslation.y();
          sensorZatLowestScan = sensorTranslation.z();
        }
        if (enableSkipLowerPoints_) {
          recordLowerPoint();
        }
      } else if (enableSkipLowerPoints_ && isLowerPoint) {
        recordLowerPoint();
        if (lowerPointCount >= std::max(1, lowerPointRecoveryCount_)) {
          acceptLowerPoint(lowerPointHeight);
        }
      } else if (scanTimeSinceInitialization - time <= scanningDuration_) {
        // RCLCPP_INFO(nodeHandle_->get_logger(), "Point is higher, updating elevation and variance.");
        // If point is higher.
        elevation = point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)
        variance = pointVariance;
        lowerPointHeight = NAN;
        lowerPointTime = NAN;
        lowerPointCount = 0.0f;
        
      } else {
        // RCLCPP_INFO(nodeHandle_->get_logger(), "Increasing variance due to multi-height noise.");
        variance += multiHeightNoise_;
      }
        continue;
    }

    // Store lowest points from scan for visibility checking.
    const float pointHeightPlusUncertainty =  point.z + 3.0 * sqrt(pointVariance);  // 3 sigma. // NOLINT(cppcoreguidelines-pro-type-union-access)
    if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint) {
      // RCLCPP_INFO(nodeHandle_->get_logger(), "Updating lowest scan point.");
      lowestScanPoint = pointHeightPlusUncertainty;
      const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
    }

    // RCLCPP_INFO(nodeHandle_->get_logger(), "Fusing measurement with elevation map data.");
    // Fuse measurement with elevation map data. aka kalman filter
    elevation =
        (variance * point.z + pointVariance * elevation) / (variance + pointVariance);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    variance = (pointVariance * variance) / (pointVariance + variance);
    // TODO(max): Add color fusion.
    grid_map::colorVectorToValue(point.getRGBVector3i(), color);
    time = scanTimeSinceInitialization;
    dynamicTime = currentTimeSecondsPattern;

    // Horizontal variances are reset.
    horizontalVarianceX = minHorizontalVariance_;
    horizontalVarianceY = minHorizontalVariance_;
    horizontalVarianceXY = 0.0;
    lowerPointHeight = NAN;
    lowerPointTime = NAN;
    lowerPointCount = 0.0f;
    // RCLCPP_INFO(nodeHandle_->get_logger(), "Updated map cell: Elevation = %f, Variance = %f", elevation, variance);
  }

  if (enableAdaptiveLowerSurface_) {
    const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
    for (grid_map::GridMapIterator iterator(currentScanMap); !iterator.isPastEnd(); ++iterator) {
      if (currentScanMap.at("adaptive_lower_confirmed", *iterator) <= 0.5f) {
        continue;
      }

      grid_map::Position scanPosition;
      currentScanMap.getPosition(*iterator, scanPosition);
      grid_map::Index rawIndex;
      if (!rawMap_.getIndex(scanPosition, rawIndex)) {
        continue;
      }
      if (isMultimodalHandled(rawIndex)) {
        continue;
      }

      const float acceptedHeight =
          adaptiveLowerPointHeightLayer(rawIndex(0), rawIndex(1));
      const float acceptedVariance = currentScanMap.at("variance", *iterator);
      elevationLayer(rawIndex(0), rawIndex(1)) = acceptedHeight;
      varianceLayer(rawIndex(0), rawIndex(1)) = acceptedVariance;
      horizontalVarianceXLayer(rawIndex(0), rawIndex(1)) = minHorizontalVariance_;
      horizontalVarianceYLayer(rawIndex(0), rawIndex(1)) = minHorizontalVariance_;
      horizontalVarianceXYLayer(rawIndex(0), rawIndex(1)) = 0.0;
      colorLayer(rawIndex(0), rawIndex(1)) = currentScanMap.at("color", *iterator);
      timeLayer(rawIndex(0), rawIndex(1)) = scanTimeSinceInitialization;
      dynamicTimeLayer(rawIndex(0), rawIndex(1)) = currentTimeSecondsPattern;
      lowestScanPointLayer(rawIndex(0), rawIndex(1)) =
          acceptedHeight + 3.0f * std::sqrt(acceptedVariance);
      sensorXatLowestScanLayer(rawIndex(0), rawIndex(1)) = sensorTranslation.x();
      sensorYatLowestScanLayer(rawIndex(0), rawIndex(1)) = sensorTranslation.y();
      sensorZatLowestScanLayer(rawIndex(0), rawIndex(1)) = sensorTranslation.z();
      lowerPointHeightLayer(rawIndex(0), rawIndex(1)) = NAN;
      lowerPointTimeLayer(rawIndex(0), rawIndex(1)) = NAN;
      lowerPointCountLayer(rawIndex(0), rawIndex(1)) = 0.0f;
      adaptiveLowerPointHeightLayer(rawIndex(0), rawIndex(1)) = NAN;
      adaptiveLowerPointTimeLayer(rawIndex(0), rawIndex(1)) = NAN;
      adaptiveLowerPointCountLayer(rawIndex(0), rawIndex(1)) = 0.0f;
    }
  }

  if (enableMultimodalCells_) {
    for (std::size_t key = 0; key < multimodalPrimaryUpdates.size(); ++key) {
      if (multimodalPrimaryUpdates[key] == 0) {
        continue;
      }

      const grid_map::Index index(
          static_cast<int>(key / static_cast<std::size_t>(rawMapSize(1))),
          static_cast<int>(key % static_cast<std::size_t>(rawMapSize(1))));
      applyMultimodalPrimary(
          rawMap_, index, multimodalResults[key].primary,
          static_cast<float>(minHorizontalVariance_),
          currentTimeSecondsPattern);
    }
  }

  //std::stringstream ss;
  //ss << sensorXatLowestScanLayer.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]"));
  //RCLCPP_INFO(nodeHandle_->get_logger(), "robotPoseCovariance: \n%s", ss.str().c_str() );
  
  clean();
  rawMap_.setTimestamp(timestamp.nanoseconds());  // Point cloud stores time in microseconds.

  const std::chrono::duration<double> duration  = std::chrono::system_clock::now() - methodStartTime;
  // RCLCPP_INFO(nodeHandle_->get_logger(), "Raw map has been updated with a new point cloud in %f s.", duration.count());
  return true;
}

bool ElevationMap::update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY, const grid_map::Matrix& horizontalVarianceUpdateXY,
                          const rclcpp::Time& time) {
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = rawMap_.getSize();

  if (!((grid_map::Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "The size of the update matrices does not match.");
    return false;
  }

  rawMap_.get("variance") += varianceUpdate;
  rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
  clean();
  rawMap_.setTimestamp(time.nanoseconds());

  return true;
}

bool ElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size) {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Fusing elevation map...");

  // Nothing to do.
  if ((size == 0).any()) {
    return false;
  }

  // Initializations.
  const auto methodStartTime = std::chrono::system_clock::now();  

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = rawMap_;
  scopedLockForRawData.unlock();

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  // More initializations.
  const double halfResolution = fusedMap_.getResolution() / 2.0;
  const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);
  // Conservative cell inclusion for ellipse iterator.
  const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) {
    resetFusedData();
  }

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) {
    fusedMap_.move(rawMapCopy.getPosition());
  }

  // For each cell in requested area.
  for (grid_map::SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {
    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) {
      continue;
    }

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      // TODO(max):
      continue;
    }

    const float centerElevation = rawMapCopy.at("elevation", *areaIterator);

    // Get size of error ellipse.
    const float& sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
    const float& sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
    const float& sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
    // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
    // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
    const double uncertaintyFactor = 2.486;  // sqrt(6.18)
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

    Eigen::Array2d::Index maxEigenvalueIndex;
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex;
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    const grid_map::Length ellipseLength =
        2.0 * uncertaintyFactor * grid_map::Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() +
        ellipseExtension;
    const double ellipseRotation(
        atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1), solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

    // Requested length and position (center) of submap in map.
    grid_map::Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
    grid_map::EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength, ellipseRotation);

    // Prepare data fusion.
    Eigen::ArrayXf means, weights;
    const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
    means.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);
    WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
    WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
    std::string maxEigenvalueLayer, minEigenvalueLayer;
    if (maxEigenvalueIndex == 0) {
      maxEigenvalueLayer = "horizontal_variance_x";
      minEigenvalueLayer = "horizontal_variance_y";
    } else {
      maxEigenvalueLayer = "horizontal_variance_y";
      minEigenvalueLayer = "horizontal_variance_x";
    }

    // For each cell in error ellipse.
    size_t i = 0;
    for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
      if (!rawMapCopy.isValid(*ellipseIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      const float candidateElevation = rawMapCopy.at("elevation", *ellipseIterator);
      if (edgeAwareFusion_ && std::fabs(candidateElevation - centerElevation) > fusionHeightDifferenceThreshold_) {
        continue;
      }

      means[i] = candidateElevation;

      // Compute weight from probability.
      grid_map::Position absolutePosition;
      rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
      Eigen::Vector2d distanceToCenter = (rotationMatrix * (absolutePosition - requestedSubmapPosition)).cwiseAbs();

      float probability1 = cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0, maxStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0, maxStandardDeviation);
      float probability2 = cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0, minStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0, minStandardDeviation);

      const float weight = std::max(minimalWeight, probability1 * probability2);
      weights[i] = weight;
      const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
      lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
      upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

      i++;
    }

    if (i == 0) {
      // Nothing to fuse.
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("lower_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) - 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("upper_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) + 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      for (const auto* layer : kMultimodalDiagnosticLayers) {
        fusedMap_.at(layer, *areaIterator) =
            rawMapCopy.at(layer, *areaIterator);
      }
      continue;
    }

    // Fuse.
    means.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();

    if (!std::isfinite(mean)) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Something went wrong when fusing the map: Mean = %f", mean);
      continue;
    }

    // Add to fused map.
    fusedMap_.at("elevation", *areaIterator) = mean;
    lowerBoundDistribution.compute();
    upperBoundDistribution.compute();
    fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01);  // TODO(max):
    fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99);  // TODO(max):
    // TODO(max): Add fusion of colors.
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
    for (const auto* layer : kMultimodalDiagnosticLayers) {
      fusedMap_.at(layer, *areaIterator) =
          rawMapCopy.at(layer, *areaIterator);
    }
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  const std::chrono::duration<double> duration = std::chrono::system_clock::now() - methodStartTime;
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map has been fused in %f s.", duration.count());

  return true;
}




void ElevationMap::visibilityCleanup(const rclcpp::Time& updatedTime) {
  // Get current time to compute calculation time.
  const auto methodStartTime = std::chrono::system_clock::now();
  const double timeSinceInitialization = (updatedTime - initialTime_).seconds();

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  visibilityCleanupMap_ = rawMap_;
  rawMap_.clear("lowest_scan_point");
  rawMap_.clear("sensor_x_at_lowest_scan");
  rawMap_.clear("sensor_y_at_lowest_scan");
  rawMap_.clear("sensor_z_at_lowest_scan");
  scopedLockForRawData.unlock();
  visibilityCleanupMap_.add("max_height");

  // Create max. height layer with ray tracing.
  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
    const auto& sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
    const auto& sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
    const auto& sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
    if (std::isnan(lowestScanPoint)) {
      continue;
    }
    grid_map::Index indexAtSensor;
    if (!visibilityCleanupMap_.getIndex(grid_map::Position(sensorXatLowestScan, sensorYatLowestScan), indexAtSensor)) {
      continue;
    }
    grid_map::Position point;
    visibilityCleanupMap_.getPosition(*iterator, point);
    float pointDiffX = point.x() - sensorXatLowestScan;
    float pointDiffY = point.y() - sensorYatLowestScan;
    float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
    if (distanceToPoint > 0.0) {
      for (grid_map::LineIterator iterator(visibilityCleanupMap_, indexAtSensor, *iterator); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position cellPosition;
        visibilityCleanupMap_.getPosition(*iterator, cellPosition);
        const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
        const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
        const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
        const float maxHeightPoint = lowestScanPoint + (sensorZatLowestScan - lowestScanPoint) / distanceToPoint * distanceToCell;
        auto& cellMaxHeight = visibilityCleanupMap_.at("max_height", *iterator);
        if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint) {
          cellMaxHeight = maxHeightPoint;
        }
      }
    }
  }

  // Vector of indices that will be removed.
  std::vector<grid_map::Position> cellPositionsToRemove;
  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& time = visibilityCleanupMap_.at("time", *iterator);
    if (timeSinceInitialization - time > scanningDuration_) {
      // Only remove cells that have not been updated during the last scan duration.
      // This prevents a.o. removal of overhanging objects.
      const auto& elevation = visibilityCleanupMap_.at("elevation", *iterator);
      const auto& variance = visibilityCleanupMap_.at("variance", *iterator);
      const auto& maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
      if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight) {
        grid_map::Position position;
        visibilityCleanupMap_.getPosition(*iterator, position);
        cellPositionsToRemove.push_back(position);
      }
    }
  }

  // Remove points in current raw map.
  scopedLockForRawData.lock();
  for (const auto& cellPosition : cellPositionsToRemove) {
    grid_map::Index index;
    if (!rawMap_.getIndex(cellPosition, index)) {
      continue;
    }
    if (rawMap_.isValid(index)) {
      rawMap_.at("elevation", index) = NAN;
      rawMap_.at("dynamic_time", index) = 0.0f;
    }
  }
  scopedLockForRawData.unlock();

  // Publish visibility cleanup map for debugging.
  publishVisibilityCleanupMap();

  const std::chrono::duration<double> duration = std::chrono::system_clock::now() - methodStartTime;
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Visibility cleanup has been performed in %f s (%d points).", duration.count(), (int)cellPositionsToRemove.size());
  if (duration.count() > visibilityCleanupDuration_) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Visibility cleanup duration is too high (current rate is %f).", 1.0 / duration.count());
  }
}

void ElevationMap::move(const Eigen::Vector2d& position) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  std::vector<grid_map::BufferRegion> newRegions;
  std::vector<grid_map::BufferRegion> newMultimodalRegions;

  if (rawMap_.move(position, newRegions)) {
    multimodalStateMap_.move(position, newMultimodalRegions);
    resetMultimodalState(multimodalStateMap_, newMultimodalRegions);
    resetRawDiagnostics(rawMap_, newRegions);
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(), rawMap_.getPosition().y());

    // The "dynamic_time" layer is meant to be interpreted as integer values, therefore nan:s need to be zeroed.
    grid_map::Matrix& dynTime{rawMap_.get("dynamic_time")};
    dynTime = dynTime.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynTime.array());

    if (hasUnderlyingMap_) {
      rawMap_.addDataFrom(underlyingMap_, false, false, true);
    }
  }
}






void ElevationMap::setRawSubmapHeight(const grid_map::Position& initPosition, float mapHeight, double lengthInXSubmap,
                                      double lengthInYSubmap, double margin) {
  // Set a submap area (lengthInYSubmap + margin, lengthInXSubmap + margin) with a constant height (mapHeight).
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  // Calculate submap iterator start index.
  const grid_map::Position topLeftPosition(initPosition(0) + lengthInXSubmap / 2, initPosition(1) + lengthInYSubmap / 2);
  grid_map::Index submapTopLeftIndex;
  rawMap_.getIndex(topLeftPosition, submapTopLeftIndex);

  // Calculate submap area.
  const double resolution = rawMap_.getResolution();
  const int lengthInXSubmapI = static_cast<int>(lengthInXSubmap / resolution + 2 * margin);
  const int lengthInYSubmapI = static_cast<int>(lengthInYSubmap / resolution + 2 * margin);
  const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);

  // Iterate through submap and fill height values.
  grid_map::Matrix& elevationData = rawMap_["elevation"];
  grid_map::Matrix& varianceData = rawMap_["variance"];
  for (grid_map::SubmapIterator iterator(rawMap_, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    elevationData(index(0), index(1)) = mapHeight;
    varianceData(index(0), index(1)) = 0.0;
  }
}


bool ElevationMap::postprocessAndPublishRawElevationMap() {
  if (!hasRawMapSubscribers()) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_; 
  scopedLock.unlock();
  return postprocessorPool_.runTask(rawMapCopy);
}

bool ElevationMap::publishFusedElevationMap() {
  if (!hasFusedMapSubscribers()) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  grid_map::GridMap fusedMapCopy = fusedMap_;
  const grid_map::GridMap planarSupportMap = fusedMapCopy;
  scopedLock.unlock();
  fillHolesForPublication(fusedMapCopy);
  fillSmallElevationHolesIfEnabled(
      fusedMapCopy, enableSmallHoleFilling_, smallHoleFillingParameters_);
  processPiecewisePlanarElevationIfEnabled(
      planarSupportMap, fusedMapCopy, enablePiecewisePlanarRegularization_,
      piecewisePlanarParameters_);
  fusedMapCopy.add("uncertainty_range", fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(fusedMapCopy);
  elevationMapFusedPublisher_->publish(std::move(message));
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map (fused) has been published.");
  return true;
}

void ElevationMap::fillHolesForPublication(grid_map::GridMap& map) const {
  if (!enableFusedMapHoleFilling_ || fusedMapHoleFillingRadius_ <= 0.0 || !map.exists("elevation")) {
    return;
  }

  const grid_map::GridMap originalMap = map;
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    if (std::isfinite(originalMap.at("elevation", index))) {
      continue;
    }

    grid_map::Index bestIndex;
    if (fusedMapHoleFillingMinSupport_ <= 0 ||
        !findEdgeSafeHoleFillSource(
            originalMap, "elevation", "height_mode_count",
            "height_mode_separation", index, fusedMapHoleFillingRadius_,
            fusedMapHoleFillingHeightThreshold_,
            static_cast<std::size_t>(fusedMapHoleFillingMinSupport_), bestIndex)) {
      continue;
    }

    map.at("elevation", index) = originalMap.at("elevation", bestIndex);
    if (map.exists("upper_bound") && originalMap.exists("upper_bound")) {
      map.at("upper_bound", index) = originalMap.at("upper_bound", bestIndex);
    }
    if (map.exists("lower_bound") && originalMap.exists("lower_bound")) {
      map.at("lower_bound", index) = originalMap.at("lower_bound", bestIndex);
    }
    if (map.exists("color") && originalMap.exists("color")) {
      map.at("color", index) = originalMap.at("color", bestIndex);
    }
  }
}

bool ElevationMap::publishVisibilityCleanupMap() {
  if (visibilityCleanupMapPublisher_->get_subscription_count() < 1) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(visibilityCleanupMapMutex_);
  grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
  scopedLock.unlock();
  visibilityCleanupMapCopy.erase("elevation");
  visibilityCleanupMapCopy.erase("variance");
  visibilityCleanupMapCopy.erase("horizontal_variance_x");
  visibilityCleanupMapCopy.erase("horizontal_variance_y");
  visibilityCleanupMapCopy.erase("horizontal_variance_xy");
  visibilityCleanupMapCopy.erase("color");
  visibilityCleanupMapCopy.erase("time");
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(visibilityCleanupMapCopy);
  visibilityCleanupMapPublisher_->publish(std::move(message));
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Visibility cleanup map has been published.");
  return true;
}

void ElevationMap::underlyingMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr underlyingMap) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Updating underlying map.");
  grid_map::GridMapRosConverter::fromMessage(*underlyingMap, underlyingMap_);
  if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
    RCLCPP_ERROR_STREAM(nodeHandle_->get_logger(), "The underlying map does not have the same map frame ('" << underlyingMap_.getFrameId() << "') as the elevation map ('"
                                                                              << rawMap_.getFrameId() << "').");
    return;
  }
  if (!underlyingMap_.exists("elevation")) {
    RCLCPP_ERROR_STREAM(nodeHandle_->get_logger(), "The underlying map does not have an 'elevation' layer.");
    return;
  }
  if (!underlyingMap_.exists("variance")) {
    underlyingMap_.add("variance", minVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_x")) {
    underlyingMap_.add("horizontal_variance_x", minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_y")) {
    underlyingMap_.add("horizontal_variance_y", minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("color")) {
    underlyingMap_.add("color", 0.0);
  }
  underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
  hasUnderlyingMap_ = true;
  rawMap_.addDataFrom(underlyingMap_, false, false, true);
}







bool ElevationMap::clear() {
  // Lock raw and fused map object in different scopes to prevent deadlock.
  {
    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    rawMap_.clearAll();
    rawMap_.resetTimestamp();
    rawMap_.get("dynamic_time").setZero();
    resetRawDiagnostics(rawMap_);
    multimodalStateMap_.clearAll();
    multimodalStateMap_.resetTimestamp();
    resetMultimodalState(multimodalStateMap_);
  }
  {
    boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
    fusedMap_.clearAll();
    fusedMap_.resetTimestamp();
  }
  return true;
} 

void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  resetRawDiagnostics(rawMap_);
  multimodalStateMap_.setGeometry(length, resolution, position);
  resetMultimodalState(multimodalStateMap_);
  fusedMap_.setGeometry(length, resolution, position);
  RCLCPP_INFO_STREAM(nodeHandle_->get_logger(), "Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and " << rawMap_.getSize()(1) << " columns.");
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

bool ElevationMap::hasFusedMapSubscribers() const {
  return elevationMapFusedPublisher_->get_subscription_count() >= 1;
}

//sets frame id for both maps.
void ElevationMap::setFrameId(const std::string& frameId) {
  rawMap_.setFrameId(frameId);
  fusedMap_.setFrameId(frameId);
}

//gives time in nanoseconds (of type RCL_TIME) to the maps.
void ElevationMap::setTimestamp(rclcpp::Time timestamp) {
  rawMap_.setTimestamp(timestamp.nanoseconds());
  fusedMap_.setTimestamp(timestamp.nanoseconds());
}

//gets frame id from map raw.
const std::string& ElevationMap::getFrameId() {
  return rawMap_.getFrameId();
}

bool ElevationMap::hasRawMapSubscribers() const {
  return postprocessorPool_.pipelineHasSubscribers();
}

bool ElevationMap::clean() {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));
  rawMap_.get("horizontal_variance_x") =
      rawMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  rawMap_.get("horizontal_variance_y") =
      rawMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData() {
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
}

bool ElevationMap::fuseAll() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(grid_map::Index(0, 0), fusedMap_.getSize());
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length) {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)", position[0], position[1],
            length[0], length[1]);

  grid_map::Index topLeftIndex;
  grid_map::Index submapBufferSize;

  // These parameters are not used in this function.
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  grid_map::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, position, length,
                                 rawMap_.getLength(), rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                                 rawMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize);
}

grid_map::GridMap& ElevationMap::getRawGridMap() {
  return rawMap_;
}

void ElevationMap::setRawGridMap(const grid_map::GridMap& map) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  if (!hasValidRawMapGeometry(map) || !map.exists("elevation") ||
      !map.exists("variance")) {
    RCLCPP_ERROR(
        nodeHandle_->get_logger(),
        "Refusing raw map replacement with invalid geometry or missing "
        "elevation/variance layers.");
    return;
  }

  grid_map::GridMap replacement = map;
  resetRawDiagnostics(replacement);

  grid_map::GridMap replacementState(multimodalStateMap_.getLayers());
  replacementState.setGeometry(
      replacement.getLength(), replacement.getResolution(),
      replacement.getPosition());
  if ((replacementState.getSize() != replacement.getSize()).any()) {
    RCLCPP_ERROR(
        nodeHandle_->get_logger(),
        "Refusing raw map replacement whose geometry cannot be represented by "
        "the multimodal state grid.");
    return;
  }
  replacementState.setFrameId(replacement.getFrameId());
  replacementState.setTimestamp(replacement.getTimestamp());
  replacementState.setStartIndex(replacement.getStartIndex());
  resetMultimodalState(replacementState);

  rawMap_ = replacement;
  multimodalStateMap_ = replacementState;
}

grid_map::GridMap& ElevationMap::getFusedGridMap() {
  return fusedMap_;
}

void ElevationMap::setFusedGridMap(const grid_map::GridMap& map) {
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_ = map;
}

rclcpp::Time ElevationMap::getTimeOfLastUpdate() {
  return rclcpp::Time(rawMap_.getTimestamp(), RCL_ROS_TIME);
}

rclcpp::Time ElevationMap::getTimeOfLastFusion() {
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return rclcpp::Time(fusedMap_.getTimestamp(), RCL_ROS_TIME);
}

const kindr::HomTransformQuatD& ElevationMap::getPose() {
  return pose_;
}

bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position) {
  kindr::Position3D positionInGridFrame;
  if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector())) {
    return false;
  }
  position = pose_.transform(positionInGridFrame);
  return true;
}

boost::recursive_mutex& ElevationMap::getFusedDataMutex() {
  return fusedMapMutex_;
}

boost::recursive_mutex& ElevationMap::getRawDataMutex() {
  return rawMapMutex_;
}












}  // namespace elevation_mapping
