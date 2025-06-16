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

#ifndef MANAGED_TRANSFORM_BUFFER__DYNAMIC_TRANSFORM_PROVIDER_HPP_
#define MANAGED_TRANSFORM_BUFFER__DYNAMIC_TRANSFORM_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace managed_transform_buffer
{
using geometry_msgs::msg::TransformStamped;

/**
 * @brief A managed TF buffer provider with use of singleton pattern.
 *
 * This class provides dynamic transform lookups using a singleton pattern to ensure
 * only one instance manages the TF listener and buffer. It automatically handles
 * the lifecycle of the TF listener and provides thread-safe access to transforms.
 */
class DynamicTransformProvider
{
public:
  /** @brief Get the instance of the DynamicTransformProvider
   *
   * @param[in] clock_type type of the clock to use for timing
   * @param[in] cache_time how long to keep a history of transforms
   * @return the instance of the DynamicTransformProvider
   */
  static DynamicTransformProvider & getInstance(
    rcl_clock_type_t clock_type, tf2::Duration cache_time);

  DynamicTransformProvider(const DynamicTransformProvider &) = delete;
  DynamicTransformProvider & operator=(const DynamicTransformProvider &) = delete;

  /** @brief Destroy the Dynamic Transform Provider object */
  ~DynamicTransformProvider();

  /**
   * @brief Get the transform between two frames by frame ID.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger for diagnostic messages
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger);

  /** @brief Get clock.
   *
   * @return the clock used by this provider
   */
  rclcpp::Clock::SharedPtr getClock() const;

private:
  /**
   * @brief Construct a new Dynamic Transform Provider object
   *
   * @param[in] clock_type type of the clock
   * @param[in] cache_time how long to keep a history of transforms
   */
  explicit DynamicTransformProvider(rcl_clock_type_t clock_type, tf2::Duration cache_time);

  /** @brief Initialize TF listener */
  void activateListener();

  /** @brief Deactivate TF listener */
  void deactivateListener();

  /** @brief Callback for TF messages
   *
   * @param[in] msg the TF message
   * @param[in] is_static whether the TF topic refers to static transforms
   */
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static);

  /** @brief Default ROS-ish lookupTransform trigger.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger for diagnostic messages
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger) const;

  /** @brief Get a dynamic transform from the TF buffer.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger for diagnostic messages
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getDynamicTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger) const;

  static std::unique_ptr<DynamicTransformProvider> instance;
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_{nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_{nullptr};
  std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)> cb_;
  std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)> cb_static_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
  std::shared_ptr<std::thread> executor_thread_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Logger logger_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__DYNAMIC_TRANSFORM_PROVIDER_HPP_
