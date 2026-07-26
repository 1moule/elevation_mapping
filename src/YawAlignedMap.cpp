#include "elevation_mapping/YawAlignedMap.hpp"

#include <Eigen/Geometry>

#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace elevation_mapping {

grid_map::GridMap createYawAlignedMap(
    const grid_map::GridMap& source,
    const grid_map::Position& bodyWorldPosition, const double bodyYaw,
    const std::string& frameId) {
  grid_map::GridMap output(source.getLayers());
  output.setBasicLayers(source.getBasicLayers());
  output.setFrameId(frameId);
  output.setTimestamp(source.getTimestamp());
  output.setGeometry(source.getLength(), source.getResolution(),
                     grid_map::Position::Zero());

  const Eigen::Rotation2Dd bodyToWorld(bodyYaw);
  for (grid_map::GridMapIterator iterator(output); !iterator.isPastEnd();
       ++iterator) {
    grid_map::Position localPosition;
    output.getPosition(*iterator, localPosition);

    grid_map::Index sourceIndex;
    const grid_map::Position worldPosition =
        bodyWorldPosition + bodyToWorld * localPosition;
    if (!source.getIndex(worldPosition, sourceIndex)) {
      continue;
    }

    for (const auto& layer : source.getLayers()) {
      output.at(layer, *iterator) = source.at(layer, sourceIndex);
    }
  }

  return output;
}

}  // namespace elevation_mapping
