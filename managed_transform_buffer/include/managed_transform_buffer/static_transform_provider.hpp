// Copyright 2024 The Autoware Foundation
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

#ifndef MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_PROVIDER_HPP_
#define MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_PROVIDER_HPP_

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <managed_transform_buffer_msgs/srv/detail/get_static_transform__struct.hpp>
#include <managed_transform_buffer_msgs/srv/get_static_transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>

#include <cstddef>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace std
{
template <>
struct hash<std::pair<std::string, std::string>>
{
  size_t operator()(const std::pair<std::string, std::string> & p) const
  {
    size_t h1 = std::hash<std::string>{}(p.first);
    size_t h2 = std::hash<std::string>{}(p.second);
    return h1 ^ (h2 << 1u);
  }
};
}  // namespace std

namespace managed_transform_buffer
{
using Key = std::pair<std::string, std::string>;

/**
 * @brief Equality comparison for transform frame pairs.
 */
struct PairEqual
{
  bool operator()(const Key & p1, const Key & p2) const
  {
    return p1.first == p2.first && p1.second == p2.second;
  }
};
using geometry_msgs::msg::TransformStamped;
using TFMap = std::unordered_map<Key, TransformStamped, std::hash<Key>, PairEqual>;

/**
 * @brief Result of a static transform query.
 */
struct StaticTransformResult
{
  bool success;
  bool is_static;
  TransformStamped transform;
};

/**
 * @brief A provider for static transforms that caches transforms locally.
 *
 * This class manages static transforms by first checking a local cache, then falling back
 * to querying a static transform server. It maintains a local buffer of static transforms
 * to avoid repeated service calls for the same transform pairs.
 */
class StaticTransformProvider
{
public:
  /**
   * @brief Construct a new Static Transform Provider object.
   *
   * @param[in] node the ROS node for service client access
   */
  explicit StaticTransformProvider(rclcpp::Node * node);

  /**
   * @brief Destroy the Static Transform Provider object.
   */
  ~StaticTransformProvider();

  /**
   * @brief Get a static transform between two frames.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired
   * @return a result containing success status, static flag, and transform if available
   */
  StaticTransformResult getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time);

private:
  /**
   * @brief Get a static transform from the local buffer.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired
   * @return an optional containing the transform if available in the local buffer
   */
  std::optional<TransformStamped> getStaticTransformFromStaticBuffer(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time);

  rclcpp::Node * node_;
  rclcpp::Client<managed_transform_buffer_msgs::srv::GetStaticTransform>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr group_;
  TFMap static_tf_buffer_;

  // Service executor members for avoiding deadlock
  rclcpp::executors::SingleThreadedExecutor::SharedPtr service_executor_;
  std::thread executor_thread_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_PROVIDER_HPP_
