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

#include "managed_transform_buffer/static_transform_server.hpp"

#include <tf2/convert.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace managed_transform_buffer
{

StaticTransformServer::StaticTransformServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("static_transform_server", options)
{
  // Node
  {
    auto listener_options = rclcpp::NodeOptions();
    listener_options.start_parameter_event_publisher(false);
    listener_options.start_parameter_services(false);
    listener_options.arguments({"--ros-args", "-r", "__node:=managed_tf_listener_impl_server"});
    node_ = rclcpp::Node::make_unique("_", listener_options);
  }

  // TF buffer
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_buffer_->setUsingDedicatedThread(true);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
  }

  // TF subscriptions
  {
    callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto tf_options = tf2_ros::detail::get_default_transform_listener_sub_options();
    tf_options.callback_group = callback_group_;
    tf_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    auto tf_static_options = tf2_ros::detail::get_default_transform_listener_static_sub_options();
    tf_static_options.callback_group = callback_group_;
    cb_ = std::bind(&StaticTransformServer::tfCallback, this, std::placeholders::_1, false);
    cb_static_ = std::bind(&StaticTransformServer::tfCallback, this, std::placeholders::_1, true);
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(), cb_, tf_options);
    tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticListenerQoS(), cb_static_, tf_static_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_thread_ = std::make_shared<std::thread>(
      std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, executor_));
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
  }

  // Service
  get_static_transform_service_ =
    create_service<managed_transform_buffer_msgs::srv::GetStaticTransform>(
      "managed_transform_buffer/get_static_transform",
      std::bind(
        &StaticTransformServer::getStaticTransformService, this, std::placeholders::_1,
        std::placeholders::_2));
}

void StaticTransformServer::getStaticTransformService(
  const std::shared_ptr<managed_transform_buffer_msgs::srv::GetStaticTransform::Request> request,
  std::shared_ptr<managed_transform_buffer_msgs::srv::GetStaticTransform::Response> response)
{
  auto traverse_result =
    traverseTree(request->target_frame, request->source_frame, this->get_logger());
  response->success = traverse_result.success;
  response->is_static = traverse_result.is_static;

  if (!traverse_result.success || !traverse_result.is_static) {
    return;
  }

  // Exception shall not occur here as related TF exists in the buffer (traverseTree result)
  try {
    auto tf =
      tf_buffer_->lookupTransform(request->target_frame, request->source_frame, tf2::TimePoint());
    response->transform = tf.transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock_, 3000, "Failure to get transform from %s to %s with error: %s",
      request->target_frame.c_str(), request->source_frame.c_str(), ex.what());
    response->success = false;
  }
}

void StaticTransformServer::tfCallback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static)
{
  std::string authority = "Authority undetectable";
  for (const auto & tf : msg->transforms) {
    try {
      tf_buffer_->setTransform(tf, authority, is_static);
      std::shared_lock<std::shared_mutex> sh_tree_lock(tree_mutex_);
      auto tree_it = tf_tree_.find(tf.child_frame_id);
      if (tree_it == tf_tree_.end()) {
        sh_tree_lock.unlock();
        std::unique_lock<std::shared_mutex> unq_tree_lock(tree_mutex_);
        tf_tree_.emplace(tf.child_frame_id, TreeNode{tf.header.frame_id, is_static});
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Failure to set received transform from %s to %s with error: %s\n",
        tf.child_frame_id.c_str(), tf.header.frame_id.c_str(), ex.what());
    }
  }
}

TraverseResult StaticTransformServer::traverseTree(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  if (target_frame == source_frame) {
    return {true, true};
  }

  std::shared_lock<std::shared_mutex> sh_tree_lock(tree_mutex_);
  auto last_tf_tree = tf_tree_;
  sh_tree_lock.unlock();

  // Helper function to find path from a frame to root (following parent links)
  auto get_ancestor_chain = [&](const std::string & frame) -> std::vector<std::string> {
    std::vector<std::string> chain;
    std::string current_frame = frame;
    std::set<std::string> visited;  // To detect cycles

    chain.push_back(current_frame);

    while (true) {
      if (visited.count(current_frame)) {
        // Cycle detected - return empty to indicate error
        return {};
      }
      visited.insert(current_frame);

      auto it = last_tf_tree.find(current_frame);
      if (it == last_tf_tree.end()) {
        // Reached root or disconnected frame
        break;
      }

      current_frame = it->second.parent;
      chain.push_back(current_frame);
    }

    return chain;
  };

  // Get ancestor chains for both frames
  auto source_chain = get_ancestor_chain(source_frame);
  auto target_chain = get_ancestor_chain(target_frame);

  // Check for cycles
  if (source_chain.empty() || target_chain.empty()) {
    RCLCPP_ERROR(
      logger, "Cycle detected in TF tree for frames %s or %s", source_frame.c_str(),
      target_frame.c_str());
    return {false, false};
  }

  // Find the lowest common ancestor (LCA)
  std::set<std::string> source_ancestors(source_chain.begin(), source_chain.end());

  std::string lca;
  bool found_lca = false;

  // Find first common ancestor in target chain
  for (const auto & frame : target_chain) {
    if (source_ancestors.count(frame)) {
      lca = frame;
      found_lca = true;
      break;
    }
  }

  if (!found_lca) {
    RCLCPP_DEBUG(
      logger, "No path found between frames %s and %s", source_frame.c_str(), target_frame.c_str());
    return {false, false};
  }

  // Check if all transforms in the path are static
  bool all_static = true;

  // Check path from source to LCA
  for (const auto & frame : source_chain) {
    if (frame == lca) {
      break;  // Reached LCA
    }

    auto it = last_tf_tree.find(frame);
    if (it != last_tf_tree.end() && !it->second.is_static) {
      all_static = false;
      break;
    }
  }

  // Check path from target to LCA
  if (all_static) {
    for (const auto & frame : target_chain) {
      if (frame == lca) {
        break;  // Reached LCA
      }

      auto it = last_tf_tree.find(frame);
      if (it != last_tf_tree.end() && !it->second.is_static) {
        all_static = false;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    logger, "Found path between frames %s and %s via LCA %s, all_static: %s", source_frame.c_str(),
    target_frame.c_str(), lca.c_str(), all_static ? "true" : "false");

  return {true, all_static};
}

}  // namespace managed_transform_buffer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(managed_transform_buffer::StaticTransformServer)
