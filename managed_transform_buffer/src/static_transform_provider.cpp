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

#include "managed_transform_buffer/static_transform_provider.hpp"

#include "managed_transform_buffer/utils.hpp"

#include <rclcpp/executors.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_ros/qos.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rmw/qos_profiles.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace managed_transform_buffer
{

StaticTransformProvider::StaticTransformProvider(rclcpp::Node * node, uint32_t tf_server_timeout_ms)
: node_(node)
{
  // Create a separate callback group for the service client
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
  client_ = node_->create_client<managed_transform_buffer_msgs::srv::GetStaticTransform>(
    "/managed_transform_buffer/get_static_transform", rmw_qos_profile_services_default, group_);

  // Create a dedicated executor and thread for service calls
  // This prevents deadlock when the main thread is blocked waiting for service response
  // while the service response callback needs to execute on the same thread
  service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_callback_group(group_, node_->get_node_base_interface());

  // Start the executor in a separate thread
  executor_thread_ = std::thread([this]() { service_executor_->spin(); });

  // Override server delay if parameter exists
  if (node_->has_parameter("tf_server_timeout_ms")) {
    node_->get_parameter("tf_server_timeout_ms", tf_server_timeout_ms);
  } else {
    tf_server_timeout_ms =
      node_->declare_parameter<int64_t>("tf_server_timeout_ms", tf_server_timeout_ms);
  }

  // Wait for the service to become available
  const auto timeout = std::chrono::milliseconds(tf_server_timeout_ms);
  const auto interval = std::chrono::milliseconds(tf_server_timeout_ms / 5);
  auto start_time = std::chrono::steady_clock::now();

  while (!client_->wait_for_service(interval) && rclcpp::ok()) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      service_executor_->cancel();
      if (executor_thread_.joinable()) {
        executor_thread_.join();
      }
      throw std::runtime_error("managed_transform_buffer service is not available.");
    }
  }
}

StaticTransformProvider::~StaticTransformProvider()
{
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

StaticTransformResult StaticTransformProvider::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time)
{
  // Check if the transform is already in the buffer
  if (static_tf_buffer_.find({target_frame, source_frame}) != static_tf_buffer_.end()) {
    return {true, true, static_tf_buffer_.at({target_frame, source_frame})};
  }

  auto request =
    std::make_shared<managed_transform_buffer_msgs::srv::GetStaticTransform::Request>();
  request->target_frame = target_frame;
  request->source_frame = source_frame;

  auto result{client_->async_send_request(
    request,
    [](rclcpp::Client<managed_transform_buffer_msgs::srv::GetStaticTransform>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  bool skip_first_log{false};
  while (status != std::future_status::ready) {
    if (!skip_first_log) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Waiting for response from static_transform_server...\n");
      skip_first_log = true;
    }
    if (!rclcpp::ok()) {
      return {false, false, TransformStamped()};
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  if (status == std::future_status::ready) {
    StaticTransformResult out;
    out.success = result.get()->success;
    out.is_static = result.get()->is_static;
    out.transform.transform = result.get()->transform;
    out.transform.header.stamp = tf2_ros::toRclcpp(time);
    out.transform.header.frame_id = target_frame;
    out.transform.child_frame_id = source_frame;
    if (out.success && out.is_static) {
      static_tf_buffer_.emplace(std::make_pair(target_frame, source_frame), out.transform);
    }
    return out;
  }
  return {false, false, TransformStamped()};
}

std::optional<TransformStamped> StaticTransformProvider::getStaticTransformFromStaticBuffer(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time)
{
  // Check if the transform is already in the buffer
  auto key = std::make_pair(target_frame, source_frame);
  auto it = static_tf_buffer_.find(key);
  if (it != static_tf_buffer_.end()) {
    auto tf_msg = it->second;
    tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  // Check if the inverse transform is already in the buffer
  auto key_inv = std::make_pair(source_frame, target_frame);
  auto it_inv = static_tf_buffer_.find(key_inv);
  if (it_inv != static_tf_buffer_.end()) {
    auto tf_msg = it_inv->second;
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);
    tf2::Transform inv_tf = tf.inverse();
    TransformStamped inv_tf_msg;
    inv_tf_msg.transform = tf2::toMsg(inv_tf);
    inv_tf_msg.header.frame_id = tf_msg.child_frame_id;
    inv_tf_msg.child_frame_id = tf_msg.header.frame_id;
    inv_tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    static_tf_buffer_.emplace(key, inv_tf_msg);
    return std::make_optional<TransformStamped>(inv_tf_msg);
  }

  // Check if transform is needed
  if (target_frame == source_frame) {
    auto tf_identity = tf2::Transform::getIdentity();
    TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(tf_identity);
    tf_msg.header.frame_id = target_frame;
    tf_msg.child_frame_id = source_frame;
    tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    static_tf_buffer_.emplace(key, tf_msg);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  return std::nullopt;
}

}  // namespace managed_transform_buffer
