#include "elevation_mapping/ElevationMapping.hpp"

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {
namespace {

TEST(ElevationMappingLifecycle, DestroysWhenVisibilityCleanupIsDisabled) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("enable_visibility_cleanup", false),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "elevation_mapping_lifecycle_test", options);
  node->declare_parameter("postprocessor_num_threads", 1);

  {
    ElevationMapping mapping(node);
  }

  SUCCEED();
}

}  // namespace
}  // namespace elevation_mapping
