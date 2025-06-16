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

#ifndef MANAGED_TRANSFORM_BUFFER__BYPASS_TRANSFORM_PROVIDER_HPP_
#define MANAGED_TRANSFORM_BUFFER__BYPASS_TRANSFORM_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>

namespace managed_transform_buffer
{

using geometry_msgs::msg::TransformStamped;

/**
 * @brief A non-managed TF buffer provider that bypasses the managed transform system.
 *
 * This class provides a traditional TF buffer interface without the managed optimizations.
 * It creates its own TF listener and buffer, providing standard ROS transform lookup
 * behavior. This is used when the non_managed flag is set in the main transform buffer.
 */
class BypassTransformProvider
{
public:
  /**
   * @brief Construct a new Bypass Transform Provider object.
   *
   * @param[in] node the ROS node for the TF listener
   * @param[in] cache_time how long to keep a history of transforms
   */
  BypassTransformProvider(rclcpp::Node * node, tf2::Duration cache_time);

  /**
   * @brief Destroy the Bypass Transform Provider object.
   */
  ~BypassTransformProvider();

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
  rclcpp::Node * node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__BYPASS_TRANSFORM_PROVIDER_HPP_
