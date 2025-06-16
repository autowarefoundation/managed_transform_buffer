// Copyright 2025 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc.

#ifndef MANAGED_TRANSFORM_BUFFER__TRANSFORM_PROVIDER_HPP_
#define MANAGED_TRANSFORM_BUFFER__TRANSFORM_PROVIDER_HPP_

#include "managed_transform_buffer/bypass_transform_provider.hpp"
#include "managed_transform_buffer/dynamic_transform_provider.hpp"
#include "managed_transform_buffer/static_transform_provider.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <optional>
#include <string>

namespace managed_transform_buffer
{
using geometry_msgs::msg::TransformStamped;

/**
 * @brief A unified transform provider that manages different types of transform providers.
 *
 * This class provides a unified interface for accessing transforms while internally managing
 * static, dynamic, and bypass transform providers. It automatically selects the appropriate
 * provider based on the configuration and transform characteristics.
 */
class TransformProvider
{
public:
  /**
   * @brief Construct a new Transform Provider object.
   *
   * @param[in] node the ROS node for logging and service access
   * @param[in] force_dynamic if true, forces the use of dynamic transform provider
   * @param[in] non_managed if true, uses bypass transform provider for non-managed operation
   * @param[in] cache_time how long to keep a history of transforms
   */
  TransformProvider(
    rclcpp::Node * node, const bool force_dynamic, const bool non_managed,
    tf2::Duration cache_time);

  ~TransformProvider() = default;

  /**
   * @brief Check if the provider is configured for static transforms only.
   *
   * @return true if only static transforms are being used
   */
  bool isStatic() const;

  /**
   * @brief Get the transform between two frames.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired
   * @param[in] timeout how long to block before failing
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout);

private:
  rclcpp::Node * node_{nullptr};
  std::unique_ptr<StaticTransformProvider> static_tf_provider_{nullptr};
  DynamicTransformProvider * dynamic_tf_provider_{nullptr};
  std::unique_ptr<BypassTransformProvider> bypass_tf_provider_{nullptr};
  tf2::Duration cache_time_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__TRANSFORM_PROVIDER_HPP_
