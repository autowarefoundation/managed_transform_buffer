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

#ifndef MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_SERVER_HPP_
#define MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <managed_transform_buffer_msgs/srv/get_static_transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace managed_transform_buffer
{

/**
 * @brief A node in the transform tree containing parent information and static flag.
 */
struct TreeNode
{
  TreeNode() : is_static(false) {}
  TreeNode(std::string p_parent, const bool p_is_static)
  : parent(std::move(p_parent)), is_static(p_is_static)
  {
  }
  std::string parent;
  bool is_static;
};

/**
 * @brief Result of traversing the transform tree.
 */
struct TraverseResult
{
  TraverseResult() : success(false), is_static(false) {}
  TraverseResult(const bool p_success, const bool p_is_static)
  : success(p_success), is_static(p_is_static)
  {
  }
  bool success;
  bool is_static;
};

using geometry_msgs::msg::TransformStamped;
using TreeMap = std::unordered_map<std::string, TreeNode>;

/**
 * @brief A ROS node that provides static transform information via service calls.
 *
 * This server maintains a local transform tree by listening to both static and dynamic
 * transform topics. It provides a service to determine if a transform between two frames
 * is static, which is used by the managed transform buffer system to optimize transform
 * lookups.
 */
class StaticTransformServer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Static Transform Server object.
   *
   * @param[in] options node options for ROS node configuration
   */
  explicit StaticTransformServer(const rclcpp::NodeOptions & options);

  /**
   * @brief Service callback to determine if a transform is static.
   *
   * @param[in] request the service request containing source and target frames
   * @param[out] response the service response containing the result
   */
  void getStaticTransformService(
    const std::shared_ptr<managed_transform_buffer_msgs::srv::GetStaticTransform::Request> request,
    std::shared_ptr<managed_transform_buffer_msgs::srv::GetStaticTransform::Response> response);

private:
  /** @brief Callback for TF messages
   *
   * @param[in] msg the TF message
   * @param[in] is_static whether the TF topic refers to static transforms
   */
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static);

  /** @brief Traverse TF tree built by local TF listener.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] logger logger, if not specified, default logger will be used
   * @return a traverse result indicating if the transform is possible and if it is static
   */
  TraverseResult traverseTree(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger);

  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<managed_transform_buffer_msgs::srv::GetStaticTransform>::SharedPtr
    get_static_transform_service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_{nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_{nullptr};
  std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)> cb_;
  std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)> cb_static_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
  std::shared_ptr<std::thread> executor_thread_{nullptr};
  std::shared_mutex buffer_mutex_;
  std::shared_mutex tree_mutex_;
  TreeMap tf_tree_;
};
}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__STATIC_TRANSFORM_SERVER_HPP_
